//! STM32H753ZI board support for the two GSE boards.
//!
//! This module is deliberately small and dependency-free: every register is
//! touched directly, there are no interrupts apart from the 1 ms SysTick, and
//! all UART traffic is polled. That makes the timing predictable, keeps the
//! safety logic in `gse-fw-core` in charge of every output, and lets the whole
//! module be type-checked on the host (see `crates/gse-fw-check`).
//!
//! Clocking: the chips run from the 64 MHz HSI exactly as they come out of
//! reset (PLL bypassed, all prescalers /1). That is ample for two 115200/9600
//! baud UARTs and keeps the bring-up short enough to audit.
//!
//! Pin map (NUCLEO-H753ZI):
//!
//! | Signal                | Pin            | Mode                       |
//! |-----------------------|----------------|----------------------------|
//! | USB VCP TX (ground)   | PD8            | USART3_TX, AF7             |
//! | USB VCP RX (ground)   | PD9            | USART3_RX, AF7             |
//! | Radio TX              | PG14           | USART6_TX, AF7             |
//! | Radio RX              | PG9            | USART6_RX, AF7             |
//! | LED green             | PB0            | push-pull output           |
//! | LED yellow            | PE1            | push-pull output           |
//! | LED red               | PB14           | push-pull output           |
//! | FILL servo            | PE9            | TIM1_CH1, 50 Hz PWM        |
//! | SRV servo             | PE11           | TIM1_CH2, 50 Hz PWM        |
//! | IGN output            | PE13           | push-pull output           |

#![allow(clippy::missing_safety_doc)]

use core::sync::atomic::{AtomicU32, Ordering};

use gse_fw_core::hal::{GroundHal, Leds, VehicleHal};

// ---------------------------------------------------------------------------
// Register map
// ---------------------------------------------------------------------------

pub const RCC_BASE: usize = 0x5802_4400;
const RCC_AHB4ENR: usize = RCC_BASE + 0xE0;
const RCC_APB1LENR: usize = RCC_BASE + 0xE8;
const RCC_APB2ENR: usize = RCC_BASE + 0xF0;

pub const GPIOB: usize = 0x5802_0400;
pub const GPIOD: usize = 0x5802_0C00;
pub const GPIOE: usize = 0x5802_1000;
pub const GPIOG: usize = 0x5802_1800;

const GPIO_MODER: usize = 0x00;
const GPIO_OSPEEDR: usize = 0x08;
const GPIO_BSRR: usize = 0x18;
const GPIO_AFR0: usize = 0x20;
const GPIO_AFR1: usize = 0x24;

pub const USART3: usize = 0x4000_4800;
pub const USART6: usize = 0x4001_1400;
const USART_CR1: usize = 0x00;
const USART_CR2: usize = 0x04;
const USART_CR3: usize = 0x08;
const USART_BRR: usize = 0x0C;
const USART_ISR: usize = 0x1C;
const USART_ICR: usize = 0x20;
const USART_RDR: usize = 0x24;
const USART_TDR: usize = 0x28;

const USART_CR1_RE: u32 = 1 << 2;
const USART_CR1_TE: u32 = 1 << 3;
const USART_CR1_UE: u32 = 1 << 0;
const USART_CR1_FIFOEN: u32 = 1 << 29;
const USART_ISR_RXNE: u32 = 1 << 5;
const USART_ISR_TXE: u32 = 1 << 7;
const USART_ISR_ORE: u32 = 1 << 3;
const USART_ICR_ORECF: u32 = 1 << 3;

pub const TIM1: usize = 0x4001_0000;
const TIM_CR1: usize = 0x00;
const TIM_EGR: usize = 0x14;
const TIM_CCMR1: usize = 0x18;
const TIM_CCER: usize = 0x20;
const TIM_PSC: usize = 0x28;
const TIM_ARR: usize = 0x2C;
const TIM_CCR1: usize = 0x34;
const TIM_CCR2: usize = 0x38;
const TIM_BDTR: usize = 0x44;

pub const IWDG1: usize = 0x5800_4800;
const IWDG_KR: usize = 0x00;
const IWDG_PR: usize = 0x04;
const IWDG_RLR: usize = 0x08;
const IWDG_SR: usize = 0x0C;
const IWDG_KEY_ENABLE: u32 = 0xCCCC;
const IWDG_KEY_ACCESS: u32 = 0x5555;
const IWDG_KEY_RELOAD: u32 = 0xAAAA;

const SCB_CPACR: usize = 0xE000_ED88;
const SYST_CSR: usize = 0xE000_E010;
const SYST_RVR: usize = 0xE000_E014;
const SYST_CVR: usize = 0xE000_E018;

/// Core clock after reset (HSI), in Hz.
pub const CORE_HZ: u32 = 64_000_000;

const TX_CAPACITY: usize = 2048;

// ---------------------------------------------------------------------------
// Raw register helpers
// ---------------------------------------------------------------------------

#[inline(always)]
unsafe fn rd(addr: usize) -> u32 {
    (addr as *const u32).read_volatile()
}

#[inline(always)]
unsafe fn wr(addr: usize, value: u32) {
    (addr as *mut u32).write_volatile(value);
}

#[inline(always)]
unsafe fn modify(addr: usize, clear: u32, set: u32) {
    wr(addr, (rd(addr) & !clear) | set);
}

#[inline(always)]
unsafe fn set_bits(addr: usize, bits: u32) {
    wr(addr, rd(addr) | bits);
}

// ---------------------------------------------------------------------------
// Time base
// ---------------------------------------------------------------------------

static MILLIS: AtomicU32 = AtomicU32::new(0);

/// Milliseconds since reset, wrapping after ~49 days.
pub fn now_ms() -> u32 {
    MILLIS.load(Ordering::Relaxed)
}

/// Start the 1 ms SysTick.
pub fn timebase_init() {
    unsafe {
        // FPU full access (cortex-m-rt normally does this; be explicit).
        wr(SCB_CPACR, rd(SCB_CPACR) | 0x00F0_0000);
        wr(SYST_RVR, CORE_HZ / 1000 - 1);
        wr(SYST_CVR, 0);
        wr(SYST_CSR, 0b111); // processor clock, interrupt, enable
    }
}

#[cfg(target_arch = "arm")]
#[no_mangle]
pub extern "C" fn SysTick() {
    MILLIS.fetch_add(1, Ordering::Relaxed);
}

// ---------------------------------------------------------------------------
// GPIO
// ---------------------------------------------------------------------------

#[derive(Clone, Copy)]
pub struct Pin {
    pub port: usize,
    pub pin: u8,
}

impl Pin {
    pub const fn new(port: usize, pin: u8) -> Self {
        Pin { port, pin }
    }

    fn mode_mask(&self) -> (usize, u32) {
        let shift = (self.pin as u32) * 2;
        (GPIO_MODER, 0b11 << shift)
    }

    fn set_mode(&self, mode: u32) {
        let (reg, mask) = self.mode_mask();
        unsafe { modify(self.port + reg, mask, mode << ((self.pin as u32) * 2)) }
    }

    fn set_speed_high(&self) {
        let shift = (self.pin as u32) * 2;
        unsafe {
            modify(
                self.port + GPIO_OSPEEDR,
                0b11 << shift,
                0b10 << shift, // high speed
            )
        }
    }

    pub fn as_output(&self) {
        self.set_speed_high();
        self.set_mode(0b01);
    }

    pub fn as_alternate(&self, af: u32) {
        self.set_speed_high();
        self.set_mode(0b10);
        let (reg, shift) = if self.pin < 8 {
            (GPIO_AFR0, (self.pin as u32) * 4)
        } else {
            (GPIO_AFR1, ((self.pin as u32) - 8) * 4)
        };
        unsafe { modify(self.port + reg, 0xF << shift, (af & 0xF) << shift) }
    }

    pub fn set(&self, high: bool) {
        unsafe {
            if high {
                wr(self.port + GPIO_BSRR, 1 << self.pin);
            } else {
                wr(self.port + GPIO_BSRR, 1 << (self.pin + 16));
            }
        }
    }
}

fn enable_gpio(bit: u32) {
    unsafe { set_bits(RCC_AHB4ENR, 1 << bit) }
}

// ---------------------------------------------------------------------------
// UART
// ---------------------------------------------------------------------------

/// Baud rate divider for the 16x oversampling mode.
pub fn baud_divider(fck: u32, baud: u32) -> u32 {
    if baud == 0 {
        return 0;
    }
    (fck + baud / 2) / baud
}

/// Lock-free (single-threaded) TX ring buffer.
pub struct TxRing {
    buf: [u8; TX_CAPACITY],
    head: usize,
    tail: usize,
}

impl TxRing {
    pub const fn new() -> Self {
        TxRing {
            buf: [0; TX_CAPACITY],
            head: 0,
            tail: 0,
        }
    }

    pub fn len(&self) -> usize {
        self.head.wrapping_sub(self.tail)
    }

    pub fn is_empty(&self) -> bool {
        self.len() == 0
    }

    pub fn has_space(&self, count: usize) -> bool {
        TX_CAPACITY - self.len() >= count
    }

    /// Enqueue a whole slice; returns false (and enqueues nothing) when it
    /// does not fit, so a frame is never transmitted half-written.
    pub fn push_all(&mut self, data: &[u8]) -> bool {
        if !self.has_space(data.len()) {
            return false;
        }
        for &byte in data {
            self.buf[self.head % TX_CAPACITY] = byte;
            self.head = self.head.wrapping_add(1);
        }
        true
    }

    pub fn pop(&mut self) -> Option<u8> {
        if self.is_empty() {
            return None;
        }
        let byte = self.buf[self.tail % TX_CAPACITY];
        self.tail = self.tail.wrapping_add(1);
        Some(byte)
    }
}

impl Default for TxRing {
    fn default() -> Self {
        Self::new()
    }
}

/// Polled USART with a software transmit queue.
pub struct Uart {
    base: usize,
    tx: TxRing,
}

impl Uart {
    pub const fn new(base: usize) -> Self {
        Uart {
            base,
            tx: TxRing::new(),
        }
    }

    /// Configure 8N1, RX/TX and the hardware FIFO. The peripheral clock is the
    /// 64 MHz APB clock; the kernel clock mux stays at its reset value.
    pub fn init(&mut self, baud: u32) {
        unsafe {
            let cr1 = self.base + USART_CR1;
            // UE must be cleared before changing the frame format.
            modify(cr1, USART_CR1_UE, 0);
            wr(self.base + USART_CR2, 0);
            wr(self.base + USART_CR3, 0);
            wr(self.base + USART_BRR, baud_divider(CORE_HZ, baud));
            // FIFO mode gives the polled loop eight bytes of RX slack.
            wr(
                cr1,
                USART_CR1_FIFOEN | USART_CR1_TE | USART_CR1_RE | USART_CR1_UE,
            );
        }
    }

    /// Drain received bytes. Returns 0 when the RX FIFO is empty.
    pub fn read(&mut self, out: &mut [u8]) -> usize {
        let mut count = 0;
        unsafe {
            let isr_addr = self.base + USART_ISR;
            if rd(isr_addr) & USART_ISR_ORE != 0 {
                // Reading ISR then ICR clears the overrun without losing sync:
                // COBS framing resynchronises on the next 0x00 delimiter.
                wr(self.base + USART_ICR, USART_ICR_ORECF);
            }
            while count < out.len() && rd(isr_addr) & USART_ISR_RXNE != 0 {
                out[count] = rd(self.base + USART_RDR) as u8;
                count += 1;
            }
        }
        count
    }

    pub fn write(&mut self, data: &[u8]) -> bool {
        self.tx.push_all(data)
    }

    /// Push queued bytes into the hardware as fast as it accepts them.
    pub fn pump(&mut self) {
        unsafe {
            while !self.tx.is_empty() {
                if rd(self.base + USART_ISR) & USART_ISR_TXE == 0 {
                    break;
                }
                if let Some(byte) = self.tx.pop() {
                    wr(self.base + USART_TDR, byte as u32);
                }
            }
        }
    }

    pub fn pending(&self) -> usize {
        self.tx.len()
    }

    pub fn has_space(&self, count: usize) -> bool {
        self.tx.has_space(count)
    }
}

// ---------------------------------------------------------------------------
// Servo PWM and ignition outputs
// ---------------------------------------------------------------------------

/// Servo pulse width for "off" and "on", in microseconds.
pub const SERVO_OFF_US: u16 = 1_000;
pub const SERVO_ON_US: u16 = 2_000;

pub const SERVO_MIN_US: u16 = 500;
pub const SERVO_MAX_US: u16 = 2_500;

/// Clamp a requested pulse to the safe mechanical range.
pub fn clamp_pulse(us: u16) -> u16 {
    us.clamp(SERVO_MIN_US, SERVO_MAX_US)
}

pub fn servo_pulse(on: bool) -> u16 {
    if on {
        SERVO_ON_US
    } else {
        SERVO_OFF_US
    }
}

/// TIM1 dual PWM at 50 Hz (1 MHz counter, 20 ms period).
pub struct ServoPair {
    tim: usize,
}

impl ServoPair {
    pub const fn new() -> Self {
        ServoPair { tim: TIM1 }
    }

    pub fn init(&mut self) {
        unsafe {
            set_bits(RCC_APB2ENR, 1 << 0); // TIM1EN
            let tim = self.tim;
            wr(tim + TIM_CR1, 0);
            wr(tim + TIM_PSC, 63); // 64 MHz / 64 = 1 MHz
            wr(tim + TIM_ARR, 19_999); // 20 ms period
                                       // OC1/OC2: PWM mode 1 with preload.
            wr(
                tim + TIM_CCMR1,
                (0b110 << 4) | (1 << 3) | (0b110 << 12) | (1 << 11),
            );
            wr(tim + TIM_CCER, (1 << 0) | (1 << 4));
            wr(tim + TIM_BDTR, 1 << 15); // main output enable
            wr(tim + TIM_CR1, (1 << 7) | (1 << 0)); // ARPE | CEN
            wr(tim + TIM_EGR, 1 << 0); // update
            self.set(SERVO_OFF_US, SERVO_OFF_US);
        }
    }

    pub fn set(&mut self, fill_us: u16, srv_us: u16) {
        unsafe {
            wr(self.tim + TIM_CCR1, clamp_pulse(fill_us) as u32);
            wr(self.tim + TIM_CCR2, clamp_pulse(srv_us) as u32);
        }
    }
}

impl Default for ServoPair {
    fn default() -> Self {
        Self::new()
    }
}

// ---------------------------------------------------------------------------
// Watchdog
// ---------------------------------------------------------------------------

/// LSI/64 with a 4 s timeout: long enough for the 9600 baud link, short
/// enough that a hung loop removes ignition power quickly.
pub fn watchdog_init() {
    unsafe {
        wr(IWDG1 + IWDG_KR, IWDG_KEY_ENABLE);
        wr(IWDG1 + IWDG_KR, IWDG_KEY_ACCESS);
        wr(IWDG1 + IWDG_PR, 4); // /64 -> 500 Hz
        wr(IWDG1 + IWDG_RLR, 1_999); // ~4 s
        while rd(IWDG1 + IWDG_SR) & 0b11 != 0 {}
        wr(IWDG1 + IWDG_KR, IWDG_KEY_RELOAD);
    }
}

pub fn watchdog_kick() {
    unsafe { wr(IWDG1 + IWDG_KR, IWDG_KEY_RELOAD) }
}

// ---------------------------------------------------------------------------
// LEDs
// ---------------------------------------------------------------------------

pub struct LedBank {
    pub green: Pin,
    pub yellow: Pin,
    pub red: Pin,
}

impl LedBank {
    pub const fn nucleo() -> Self {
        LedBank {
            green: Pin::new(GPIOB, 0),
            yellow: Pin::new(GPIOE, 1),
            red: Pin::new(GPIOB, 14),
        }
    }

    pub fn init(&self) {
        self.green.as_output();
        self.yellow.as_output();
        self.red.as_output();
        self.apply(Leds::default());
    }

    pub fn apply(&self, leds: Leds) {
        self.green.set(leds.link);
        self.yellow.set(leds.activity);
        self.red.set(leds.fault);
    }
}

// ---------------------------------------------------------------------------
// Fail-safe outputs
// ---------------------------------------------------------------------------

pub const IGN_PIN: Pin = Pin::new(GPIOE, 13);

/// Force every actuator into its de-energised state using raw registers.
///
/// Used by the panic and hard-fault handlers, where no HAL object is
/// guaranteed to exist. Safe to call more than once.
pub fn force_safe_outputs() {
    IGN_PIN.set(false);
    unsafe {
        // Servo pulses: 1 ms is the mechanical "closed/off" position.
        wr(TIM1 + TIM_CCR1, SERVO_OFF_US as u32);
        wr(TIM1 + TIM_CCR2, SERVO_OFF_US as u32);
    }
    LedBank::nucleo().apply(Leds {
        link: false,
        activity: false,
        fault: true,
    });
}

#[inline(always)]
pub fn wfi() {
    #[cfg(target_arch = "arm")]
    unsafe {
        core::arch::asm!("wfi", options(nomem, nostack, preserves_flags));
    }
    #[cfg(not(target_arch = "arm"))]
    core::hint::spin_loop();
}

#[cfg(target_arch = "arm")]
#[no_mangle]
pub unsafe extern "C" fn HardFault() -> ! {
    force_safe_outputs();
    loop {
        wfi();
    }
}

#[cfg(target_arch = "arm")]
#[panic_handler]
fn panic(_info: &core::panic::PanicInfo) -> ! {
    force_safe_outputs();
    loop {
        wfi();
    }
}

fn device_id(role_salt: u32) -> u32 {
    // 96-bit unique ID; one word is plenty for a bench identifier.
    let uid = unsafe { rd(0x1FF1_E800) };
    uid ^ role_salt
}

// ---------------------------------------------------------------------------
// Ground board (USB bridge)
// ---------------------------------------------------------------------------

pub struct GroundBoard {
    usb: Uart,
    radio: Uart,
    leds: LedBank,
    device: u32,
}

impl GroundBoard {
    /// Bring up the bridge board: USART3 on the ST-LINK VCP, USART6 to the
    /// SX1281 module, LEDs, watchdog.
    pub fn init() -> Self {
        // GPIO clocks: B (LEDs), D (VCP), G (radio).
        enable_gpio(1);
        enable_gpio(3);
        enable_gpio(6);
        unsafe {
            set_bits(RCC_APB1LENR, 1 << 18); // USART3EN
            set_bits(RCC_APB2ENR, 1 << 5); // USART6EN
        }

        Pin::new(GPIOD, 8).as_alternate(7);
        Pin::new(GPIOD, 9).as_alternate(7);
        Pin::new(GPIOG, 14).as_alternate(7);
        Pin::new(GPIOG, 9).as_alternate(7);

        let leds = LedBank::nucleo();
        leds.init();
        IGN_PIN.set(false);

        let mut usb = Uart::new(USART3);
        usb.init(115_200);
        let mut radio = Uart::new(USART6);
        radio.init(9_600);

        timebase_init();
        watchdog_init();

        GroundBoard {
            usb,
            radio,
            leds,
            device: device_id(0x4752_0000),
        }
    }

    pub fn pump(&mut self) {
        self.usb.pump();
        self.radio.pump();
    }
}

impl GroundHal for GroundBoard {
    fn now_ms(&self) -> u32 {
        now_ms()
    }

    fn read_usb(&mut self, buf: &mut [u8]) -> usize {
        self.usb.read(buf)
    }

    fn write_usb(&mut self, data: &[u8]) {
        let _ = self.usb.write(data);
    }

    fn usb_tx_pending(&self) -> usize {
        self.usb.pending()
    }

    fn read_radio(&mut self, buf: &mut [u8]) -> usize {
        self.radio.read(buf)
    }

    fn write_radio(&mut self, data: &[u8]) {
        let _ = self.radio.write(data);
    }

    fn radio_ready(&mut self) -> bool {
        self.radio.has_space(256)
    }

    fn set_leds(&mut self, leds: Leds) {
        self.leds.apply(leds);
    }

    fn kick_watchdog(&mut self) {
        watchdog_kick();
    }

    fn device_id(&self) -> u32 {
        self.device
    }
}

// ---------------------------------------------------------------------------
// Vehicle board (actuators)
// ---------------------------------------------------------------------------

pub struct VehicleBoard {
    radio: Uart,
    servos: ServoPair,
    ignition: Pin,
    leds: LedBank,
    fill_on: bool,
    srv_on: bool,
    ign_on: bool,
    device: u32,
}

impl VehicleBoard {
    pub fn init() -> Self {
        enable_gpio(1); // LEDs
        enable_gpio(4); // LEDs + ignition
        enable_gpio(6); // radio
        unsafe { set_bits(RCC_APB2ENR, 1 << 5) }; // USART6EN

        Pin::new(GPIOG, 14).as_alternate(7);
        Pin::new(GPIOG, 9).as_alternate(7);

        // Servo outputs on TIM1 CH1/CH2.
        Pin::new(GPIOE, 9).as_alternate(1);
        Pin::new(GPIOE, 11).as_alternate(1);
        let mut servos = ServoPair::new();
        servos.init();

        let ignition = IGN_PIN;
        ignition.as_output();
        ignition.set(false);

        let leds = LedBank::nucleo();
        leds.init();

        let mut radio = Uart::new(USART6);
        radio.init(9_600);

        timebase_init();
        watchdog_init();

        VehicleBoard {
            radio,
            servos,
            ignition,
            leds,
            fill_on: false,
            srv_on: false,
            ign_on: false,
            device: device_id(0x5645_0000),
        }
    }

    pub fn pump(&mut self) {
        self.radio.pump();
    }

    fn apply_outputs(&mut self) {
        self.servos
            .set(servo_pulse(self.fill_on), servo_pulse(self.srv_on));
        self.ignition.set(self.ign_on);
    }
}

impl VehicleHal for VehicleBoard {
    fn now_ms(&self) -> u32 {
        now_ms()
    }

    fn set_fill(&mut self, on: bool) {
        self.fill_on = on;
        self.apply_outputs();
    }

    fn set_srv(&mut self, on: bool) {
        self.srv_on = on;
        self.apply_outputs();
    }

    fn set_ign(&mut self, on: bool) {
        self.ign_on = on;
        self.apply_outputs();
    }

    fn read_radio(&mut self, buf: &mut [u8]) -> usize {
        self.radio.read(buf)
    }

    fn write_radio(&mut self, data: &[u8]) {
        let _ = self.radio.write(data);
    }

    fn radio_ready(&mut self) -> bool {
        self.radio.has_space(256)
    }

    fn set_leds(&mut self, leds: Leds) {
        self.leds.apply(leds);
    }

    fn kick_watchdog(&mut self) {
        watchdog_kick();
    }

    fn device_id(&self) -> u32 {
        self.device
    }
}

// ---------------------------------------------------------------------------
// Host-checkable helpers
// ---------------------------------------------------------------------------

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn baud_divider_matches_reference() {
        assert_eq!(baud_divider(64_000_000, 115_200), 556);
        assert_eq!(baud_divider(64_000_000, 9_600), 6_667);
        assert_eq!(baud_divider(64_000_000, 0), 0);
    }

    #[test]
    fn ring_buffer_is_atomic_and_wrapping() {
        let mut ring = TxRing::new();
        assert!(ring.is_empty());
        assert!(ring.push_all(b"hello"));
        assert_eq!(ring.len(), 5);
        let big = [0u8; TX_CAPACITY];
        assert!(!ring.push_all(&big), "must not partially enqueue");
        assert_eq!(ring.len(), 5);
        assert_eq!(ring.pop(), Some(b'h'));
        for _ in 0..4 {
            ring.pop();
        }
        assert!(ring.pop().is_none());
        // Fill, drain and wrap many times so head/tail cross the buffer end.
        for round in 0..50u32 {
            let mut chunk = [0u8; 200];
            for (i, b) in chunk.iter_mut().enumerate() {
                *b = (round as usize + i) as u8;
            }
            assert!(ring.push_all(&chunk));
            for (i, expected) in chunk.iter().enumerate() {
                let got = ring.pop().unwrap();
                assert_eq!(got, *expected, "round {round} byte {i}");
            }
        }
    }

    #[test]
    fn servo_pulses_are_clamped() {
        assert_eq!(servo_pulse(false), SERVO_OFF_US);
        assert_eq!(servo_pulse(true), SERVO_ON_US);
        assert_eq!(clamp_pulse(0), SERVO_MIN_US);
        assert_eq!(clamp_pulse(9_999), SERVO_MAX_US);
        assert_eq!(clamp_pulse(1_500), 1_500);
    }

    #[test]
    fn timebase_constant_is_one_kilohertz() {
        // SysTick reload value used by `timebase_init`.
        assert_eq!(CORE_HZ / 1000 - 1, 63_999);
    }
}
