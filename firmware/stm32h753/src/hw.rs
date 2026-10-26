//! STM32H753ZI board support for the two GSE boards.
//!
//! This module is deliberately small and dependency-free: every register is
//! touched directly, there are no interrupts apart from the 1 ms SysTick, and
//! all UART / SPI traffic is polled. That makes the timing predictable, keeps
//! the safety logic in `gse-fw-core` in charge of every output, and lets the
//! whole module be type-checked and unit-tested on the host (`gse-fw-check`).
//!
//! Clocking: the chips run from the 64 MHz HSI exactly as they come out of
//! reset (PLL bypassed, all prescalers /1). `RCC_D1CCIPR.CKPERSEL = 00` selects
//! `hsi_ker_ck` (`64 MHz`) as `per_ck`, and `RCC_D2CCIP1R.SPI123SEL = 100`
//! feeds `per_ck` into `SPI1` (`64 MHz / 16 = 4.000 MHz` Mode-0 SCK).
//!
//! ### Pin map — `NUCLEO-H753ZI` (`MB1364`)
//!
//! | Signal                                  | Pin   | Header      | Mode                                          |
//! |-----------------------------------------|-------|-------------|-----------------------------------------------|
//! | USB VCP TX (ground)                     | PD8   | ST-LINK VCP | `USART3_TX`, AF7 (`9600..460800 Bd 8N1`)      |
//! | USB VCP RX (ground)                     | PD9   | ST-LINK VCP | `USART3_RX`, AF7, internal pull-up            |
//! | `DX-LR42` Pin 3 (`RX`) <- MCU TX        | PG14  | CN10 pin 12 | `USART6_TX`, AF7 (`2400..115200 Bd 8N1`)      |
//! | `DX-LR42` Pin 4 (`TX`) -> MCU RX        | PG9   | CN9 pin 63  | `USART6_RX`, AF7, internal pull-up            |
//! | `DX-LR42` Pin 5 (`AUX`) / `E28` `BUSY`  | PG10  | CN9 pin 66  | GPIO input, internal pull-down                |
//! | `DX-LR42` Pin 1 (`M0`) / `E28` `RX_EN`  | PG12  | CN9 pin 65  | open-drain `M0` (`DX-LR42`) / push-pull LNA   |
//! | `DX-LR42` Pin 2 (`M1`) / `E28` `TX_EN`  | PG13  | CN9 pin 68  | open-drain `M1` (`DX-LR42`) / push-pull PA    |
//! | `E28-2G4M27S` Pin 5 (`SCK_RTSN`)        | PA5   | CN7 pin 10  | `SPI1_SCK`, AF5 (`4 MHz`, Mode 0)             |
//! | `E28-2G4M27S` Pin 3 (`MISO_TX`)         | PA6   | CN7 pin 12  | `SPI1_MISO`, AF5, internal pull-down          |
//! | `E28-2G4M27S` Pin 4 (`MOSI_RX`)         | PA7   | CN7 pin 14  | `SPI1_MOSI`, AF5                              |
//! | `E28-2G4M27S` Pin 6 (`NSS_CTS`)         | PD14  | CN7 pin 16  | GPIO push-pull output, active-LOW CS          |
//! | `E28-2G4M27S` Pin 11 (`NRESET`)         | PG11  | CN10 pin 30 | GPIO push-pull output, active-LOW reset       |
//! | `E28-2G4M27S` Pin 13 (`DIO1`)           | PF13  | CN10 pin 2  | GPIO input, internal pull-down (`SX128x` IRQ) |
//! | `DX-LR42` `VCC` (`5 V`) / `E28` (`3.3V`)| 5V/3V3| CN8 9 / 7   | **`E28-2G4M27S` is STRICTLY `3.3 V` MAX!**    |
//! | LED green / yellow / red                | PB0/PE1/PB14 | LD1..3 | push-pull outputs                          |
//! | FILL servo / SRV servo                  | PE9 / PE11 | CN10 4/6 | `TIM1_CH1` / `TIM1_CH2`, `50 Hz` PWM          |
//! | IGN output                              | PE13  | CN10 pin 10 | push-pull output (forced LOW on boot/fault)   |

#![allow(clippy::missing_safety_doc)]

use core::sync::atomic::{AtomicU32, Ordering};

use gse_fw_core::e28::{E28Driver, Sx128xBus};
use gse_fw_core::hal::{elapsed_ms, GroundHal, Leds, LinkState, VehicleHal};
use gse_proto::crc::crc16;
use gse_proto::radio::AUX_MAX_WAIT_MS;
use gse_proto::{DxLr42Baud, LinkConfig, RadioModule, Reason, UsbBaud};

// ---------------------------------------------------------------------------
// Register map
// ---------------------------------------------------------------------------

pub const RCC_BASE: usize = 0x5802_4400;
const RCC_D1CCIPR: usize = RCC_BASE + 0x4C;
const RCC_D2CCIP1R: usize = RCC_BASE + 0x50;
const RCC_AHB4ENR: usize = RCC_BASE + 0xE0;
const RCC_APB1LENR: usize = RCC_BASE + 0xE8;
const RCC_APB2ENR: usize = RCC_BASE + 0xF0;

const RCC_D1CCIPR_CKPERSEL_MSK: u32 = 0x3 << 28; // 00 = hsi_ker_ck (64 MHz)
const RCC_D2CCIP1R_SPI123SEL_MSK: u32 = 0x7 << 12;
const RCC_D2CCIP1R_SPI123SEL_PER: u32 = 0x4 << 12; // 100 = per_ck

pub const GPIOA: usize = 0x5802_0000;
pub const GPIOB: usize = 0x5802_0400;
pub const GPIOD: usize = 0x5802_0C00;
pub const GPIOE: usize = 0x5802_1000;
pub const GPIOF: usize = 0x5802_1400;
pub const GPIOG: usize = 0x5802_1800;

const GPIO_MODER: usize = 0x00;
const GPIO_OTYPER: usize = 0x04;
const GPIO_OSPEEDR: usize = 0x08;
const GPIO_PUPDR: usize = 0x0C;
const GPIO_IDR: usize = 0x10;
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
const USART_ISR_TC: u32 = 1 << 6;
const USART_ISR_TXE: u32 = 1 << 7;
const USART_ISR_ORE: u32 = 1 << 3;
const USART_ICR_ORECF: u32 = 1 << 3;

pub const SPI1: usize = 0x4001_3000;
const SPI_CR1: usize = 0x00;
const SPI_CR2: usize = 0x04;
const SPI_CFG1: usize = 0x08;
const SPI_CFG2: usize = 0x0C;
const SPI_IER: usize = 0x10;
const SPI_SR: usize = 0x14;
const SPI_IFCR: usize = 0x18;
const SPI_TXDR: usize = 0x20;
const SPI_RXDR: usize = 0x30;

const SPI_CR1_SPE: u32 = 1 << 0;
const SPI_CR1_CSTART: u32 = 1 << 9;
const SPI_CR1_SSI: u32 = 1 << 12;
const SPI_CFG1_8BIT_DIV16: u32 = (0b011 << 28) | 0x07; // 64 MHz / 16 = 4 MHz, 8-bit
const SPI_CFG2_MASTER_MODE0: u32 = (1 << 31) | (1 << 26) | (1 << 22); // AFCNTR | SSM | MASTER
const SPI_SR_RXP: u32 = 1 << 0;
const SPI_SR_TXP: u32 = 1 << 1;
const SPI_SR_EOT: u32 = 1 << 3;
const SPI_IFCR_ALL: u32 = 0x0FF8;

// STM32H753ZI Flash Bank 2 controller (`RM0433` §4.9, `stm32h753xx.h`).
// Firmware code runs exclusively in Bank 1 (`0x0800_0000..0x080F_FFFF`), so
// Bank 2 Sector 7 (`0x081E_0000..0x081F_FFFF`, 128 KB) never stalls instruction fetch.
pub const FLASH_R_BASE: usize = 0x5200_2000;
const FLASH_KEYR2: usize = FLASH_R_BASE + 0x104;
const FLASH_CR2: usize = FLASH_R_BASE + 0x10C;
const FLASH_SR2: usize = FLASH_R_BASE + 0x110;
const FLASH_CCR2: usize = FLASH_R_BASE + 0x114;

const FLASH_KEY1: u32 = 0x4567_0123;
const FLASH_KEY2: u32 = 0xCDEF_89AB;
const FLASH_CR_LOCK: u32 = 1 << 0;
const FLASH_CR_PG: u32 = 1 << 1;
const FLASH_CR_SER: u32 = 1 << 2;
const FLASH_CR_PSIZE_32: u32 = 0b10 << 4;
const FLASH_CR_START: u32 = 1 << 7;
const FLASH_CR_SNB_SECTOR7: u32 = 7 << 8;
const FLASH_SR_BUSY_MASK: u32 = (1 << 0) | (1 << 1) | (1 << 2); // BSY | WBNE | QW
const FLASH_CCR_ERR_MASK: u32 = 0x0FEF_0000;

pub const CONFIG_SECTOR_ADDR: usize = 0x081E_0000;
pub const CONFIG_SECTOR_BYTES: usize = 128 * 1024;
pub const FLASH_WORD_BYTES: usize = 32;
const CONFIG_MAGIC: u32 = 0x4753_4531; // "GSE1"
const CONFIG_RECORD_VERSION: u8 = 1;

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

/// Compile-time default radio module when Bank 2 Sector 7 flash is blank.
pub const fn compile_default_radio() -> RadioModule {
    #[cfg(feature = "radio-e28-2g4m27s")]
    {
        RadioModule::E28_2G4M27S
    }
    #[cfg(not(feature = "radio-e28-2g4m27s"))]
    {
        RadioModule::DxLr42
    }
}

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
unsafe fn rd8(addr: usize) -> u8 {
    (addr as *const u8).read_volatile()
}

#[inline(always)]
unsafe fn wr8(addr: usize, value: u8) {
    (addr as *mut u8).write_volatile(value);
}

#[inline(always)]
unsafe fn modify(addr: usize, clear: u32, set: u32) {
    wr(addr, (rd(addr) & !clear) | set);
}

#[inline(always)]
unsafe fn set_bits(addr: usize, bits: u32) {
    wr(addr, rd(addr) | bits);
}

#[inline(always)]
fn data_sync_barrier() {
    #[cfg(target_arch = "arm")]
    unsafe {
        core::arch::asm!("dsb sy", options(nostack, preserves_flags));
    }
    #[cfg(not(target_arch = "arm"))]
    core::sync::atomic::fence(Ordering::SeqCst);
}

/// Bounded microsecond spin delay calibrated to 64 MHz Cortex-M7 (`~16` loop cycles/us).
#[inline(never)]
pub fn spin_delay_us(us: u32) {
    let iters = us.saturating_mul(16);
    for _ in 0..iters {
        core::hint::spin_loop();
    }
}

// ---------------------------------------------------------------------------
// Non-volatile link config log in STM32H753ZI Flash Bank 2 Sector 7
// ---------------------------------------------------------------------------

/// Encode a 32-byte (`256-bit`) STM32H7 flash word holding `LinkState` + sequence + CRC-16.
pub fn encode_flash_record(state: LinkState, seq: u32) -> [u8; FLASH_WORD_BYTES] {
    let mut raw = [0xFFu8; FLASH_WORD_BYTES];
    raw[0..4].copy_from_slice(&CONFIG_MAGIC.to_le_bytes());
    raw[4..8].copy_from_slice(&seq.to_le_bytes());
    raw[8] = state.usb_baud.code();
    raw[9] = state.uart_baud.code();
    raw[10] = state.radio.code();
    raw[11] = CONFIG_RECORD_VERSION;
    let crc = crc16(&raw[0..12]);
    raw[12..14].copy_from_slice(&crc.to_le_bytes());
    raw
}

/// Decode and CRC-verify a single 32-byte flash slot.
pub fn decode_flash_record(slot: &[u8]) -> Option<(LinkState, u32)> {
    if slot.len() < FLASH_WORD_BYTES {
        return None;
    }
    let magic = u32::from_le_bytes([slot[0], slot[1], slot[2], slot[3]]);
    if magic != CONFIG_MAGIC || slot[11] != CONFIG_RECORD_VERSION {
        return None;
    }
    let expected = crc16(&slot[0..12]);
    let actual = u16::from_le_bytes([slot[12], slot[13]]);
    if expected != actual {
        return None;
    }
    let seq = u32::from_le_bytes([slot[4], slot[5], slot[6], slot[7]]);
    Some((
        LinkState {
            usb_baud: UsbBaud::from_code(slot[8])?,
            uart_baud: DxLr42Baud::from_code(slot[9])?,
            radio: RadioModule::from_code(slot[10])?,
            flash_saved: true,
        },
        seq,
    ))
}

/// Scan a flash sector image, returning `(latest_valid_record, next_free_slot_index)`.
///
/// Appending a 32-byte flash word per save avoids erasing the 128 KB sector on normal
/// updates (`< 0.5 ms` write vs `~1 s` erase) and provides `4,096x` wear-levelling;
/// interrupted or corrupted slots are skipped cleanly in favour of the last valid `seq`.
pub fn scan_config_sector(sector: &[u8]) -> (Option<(LinkState, u32)>, Option<usize>) {
    let slots = sector.len() / FLASH_WORD_BYTES;
    let mut best: Option<(LinkState, u32)> = None;
    let mut last_non_erased: Option<usize> = None;

    for idx in 0..slots {
        let start = idx * FLASH_WORD_BYTES;
        let word = &sector[start..start + FLASH_WORD_BYTES];
        if word.iter().all(|&b| b == 0xFF) {
            continue;
        }
        last_non_erased = Some(idx);
        if let Some((state, seq)) = decode_flash_record(word) {
            if best.map(|(_, prev_seq)| seq >= prev_seq).unwrap_or(true) {
                best = Some((state, seq));
            }
        }
    }

    let next_free = match last_non_erased {
        None if slots > 0 => Some(0),
        Some(last) if last + 1 < slots => Some(last + 1),
        _ => None,
    };
    (best, next_free)
}

unsafe fn flash_bank2_wait_ready() -> bool {
    for _ in 0..8_000_000 {
        if rd(FLASH_SR2) & FLASH_SR_BUSY_MASK == 0 {
            return true;
        }
    }
    false
}

unsafe fn flash_bank2_unlock() -> bool {
    if !flash_bank2_wait_ready() {
        return false;
    }
    wr(FLASH_CCR2, FLASH_CCR_ERR_MASK);
    if rd(FLASH_CR2) & FLASH_CR_LOCK != 0 {
        wr(FLASH_KEYR2, FLASH_KEY1);
        wr(FLASH_KEYR2, FLASH_KEY2);
    }
    rd(FLASH_CR2) & FLASH_CR_LOCK == 0
}

unsafe fn flash_bank2_lock() {
    set_bits(FLASH_CR2, FLASH_CR_LOCK);
}

unsafe fn flash_bank2_erase_sector7() -> bool {
    watchdog_kick();
    wr(FLASH_CCR2, FLASH_CCR_ERR_MASK);
    wr(
        FLASH_CR2,
        FLASH_CR_PSIZE_32 | FLASH_CR_SNB_SECTOR7 | FLASH_CR_SER,
    );
    set_bits(FLASH_CR2, FLASH_CR_START);
    data_sync_barrier();
    let ok = flash_bank2_wait_ready() && (rd(FLASH_SR2) & FLASH_CCR_ERR_MASK == 0);
    wr(FLASH_CR2, 0);
    watchdog_kick();
    ok
}

unsafe fn flash_bank2_write_word(slot_addr: usize, bytes: &[u8; FLASH_WORD_BYTES]) -> bool {
    if !flash_bank2_wait_ready() {
        return false;
    }
    wr(FLASH_CCR2, FLASH_CCR_ERR_MASK);
    wr(FLASH_CR2, FLASH_CR_PSIZE_32 | FLASH_CR_PG);
    data_sync_barrier();
    for i in 0..8 {
        let off = i * 4;
        let word = u32::from_le_bytes([bytes[off], bytes[off + 1], bytes[off + 2], bytes[off + 3]]);
        wr(slot_addr + off, word);
    }
    data_sync_barrier();
    let ok = flash_bank2_wait_ready() && (rd(FLASH_SR2) & FLASH_CCR_ERR_MASK == 0);
    wr(FLASH_CR2, 0);
    ok
}

/// Load persisted link configuration from STM32H753ZI Flash Bank 2 Sector 7, or
/// fall back to Cargo compile-time defaults when flash is blank.
pub fn boot_link_state() -> LinkState {
    #[cfg(target_arch = "arm")]
    {
        let sector = unsafe {
            core::slice::from_raw_parts(CONFIG_SECTOR_ADDR as *const u8, CONFIG_SECTOR_BYTES)
        };
        if let (Some((saved, _)), _) = scan_config_sector(sector) {
            return saved;
        }
    }
    LinkState {
        usb_baud: UsbBaud::default(),
        uart_baud: DxLr42Baud::default(),
        radio: compile_default_radio(),
        flash_saved: false,
    }
}

/// Save `state` to STM32H753ZI Flash Bank 2 Sector 7 (`0x081E_0000`).
pub fn persist_link_state(state: LinkState) -> bool {
    #[cfg(target_arch = "arm")]
    unsafe {
        let sector =
            core::slice::from_raw_parts(CONFIG_SECTOR_ADDR as *const u8, CONFIG_SECTOR_BYTES);
        let (latest, next_slot) = scan_config_sector(sector);
        if let Some((cur, _)) = latest {
            if cur.usb_baud == state.usb_baud
                && cur.uart_baud == state.uart_baud
                && cur.radio == state.radio
            {
                return true;
            }
        }
        let next_seq = latest.map(|(_, s)| s.wrapping_add(1)).unwrap_or(1);
        let record = encode_flash_record(state, next_seq);

        if !flash_bank2_unlock() {
            return false;
        }
        let slot_idx = match next_slot {
            Some(idx) => idx,
            None => {
                if !flash_bank2_erase_sector7() {
                    flash_bank2_lock();
                    return false;
                }
                0
            }
        };
        let addr = CONFIG_SECTOR_ADDR + slot_idx * FLASH_WORD_BYTES;
        let ok = flash_bank2_write_word(addr, &record);
        flash_bank2_lock();
        ok && decode_flash_record(core::slice::from_raw_parts(
            addr as *const u8,
            FLASH_WORD_BYTES,
        ))
        .is_some()
    }
    #[cfg(not(target_arch = "arm"))]
    {
        let _ = state;
        true
    }
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

#[derive(Clone, Copy, Debug, PartialEq, Eq)]
#[repr(u32)]
pub enum Pull {
    None = 0b00,
    Up = 0b01,
    Down = 0b10,
}

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

    fn set_open_drain(&self, open_drain: bool) {
        let bit = 1u32 << self.pin;
        unsafe {
            modify(
                self.port + GPIO_OTYPER,
                bit,
                if open_drain { bit } else { 0 },
            )
        }
    }

    pub fn set_pull(&self, pull: Pull) {
        let shift = (self.pin as u32) * 2;
        unsafe {
            modify(
                self.port + GPIO_PUPDR,
                0b11 << shift,
                (pull as u32) << shift,
            )
        }
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

    pub fn as_input(&self, pull: Pull) {
        self.set_pull(pull);
        self.set_mode(0b00);
    }

    pub fn as_output(&self) {
        self.set_open_drain(false);
        self.set_speed_high();
        self.set_mode(0b01);
    }

    /// Configure as push-pull output with initial output state latched before enabling driver.
    pub fn as_output_level(&self, initial_high: bool) {
        self.set(initial_high);
        self.set_open_drain(false);
        self.set_pull(Pull::None);
        self.set_speed_high();
        self.set_mode(0b01);
    }

    /// Configure as open-drain output with preloaded state so no transition glitch occurs.
    pub fn as_open_drain_output(&self, pull: Pull, initial_high: bool) {
        self.set(initial_high);
        self.set_open_drain(true);
        self.set_pull(pull);
        self.set_speed_high();
        self.set_mode(0b01);
    }

    pub fn as_alternate(&self, af: u32) {
        self.as_alternate_pull(af, Pull::None);
    }

    pub fn as_alternate_pull(&self, af: u32, pull: Pull) {
        self.set_open_drain(false);
        self.set_pull(pull);
        self.set_speed_high();
        let (reg, shift) = if self.pin < 8 {
            (GPIO_AFR0, (self.pin as u32) * 4)
        } else {
            (GPIO_AFR1, ((self.pin as u32) - 8) * 4)
        };
        unsafe { modify(self.port + reg, 0xF << shift, (af & 0xF) << shift) };
        self.set_mode(0b10);
    }

    pub fn is_high(&self) -> bool {
        unsafe { rd(self.port + GPIO_IDR) & (1u32 << self.pin) != 0 }
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

    pub fn clear(&mut self) {
        self.head = 0;
        self.tail = 0;
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

    pub fn peek(&self) -> Option<u8> {
        if self.is_empty() {
            None
        } else {
            Some(self.buf[self.tail % TX_CAPACITY])
        }
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

/// Polled USART with a software transmit queue and runtime baud reconfiguration.
pub struct Uart {
    base: usize,
    baud: u32,
    tx: TxRing,
}

impl Uart {
    pub const fn new(base: usize) -> Self {
        Uart {
            base,
            baud: 0,
            tx: TxRing::new(),
        }
    }

    pub fn baud(&self) -> u32 {
        self.baud
    }

    /// Configure 8N1, RX/TX and the hardware FIFO at `baud` bps.
    pub fn init(&mut self, baud: u32) {
        self.baud = baud;
        self.tx.clear();
        unsafe {
            let cr1 = self.base + USART_CR1;
            // UE must be cleared before changing BRR or frame format.
            modify(cr1, USART_CR1_UE, 0);
            wr(self.base + USART_CR2, 0);
            wr(self.base + USART_CR3, 0);
            wr(self.base + USART_BRR, baud_divider(CORE_HZ, baud));
            wr(
                cr1,
                USART_CR1_FIFOEN | USART_CR1_TE | USART_CR1_RE | USART_CR1_UE,
            );
        }
    }

    pub fn disable(&mut self) {
        self.tx.clear();
        unsafe {
            modify(self.base + USART_CR1, USART_CR1_UE, 0);
        }
    }

    /// True while either the software ring or hardware shift register (`ISR.TC == 0`) is busy.
    pub fn is_transmitting(&self) -> bool {
        if !self.tx.is_empty() {
            return true;
        }
        unsafe { rd(self.base + USART_ISR) & USART_ISR_TC == 0 }
    }

    /// Synchronously transmit a short AT configuration command and wait until the last stop bit
    /// has left the shift register (`ISR.TC == 1`), bounded so a faulted peripheral cannot hang.
    pub fn write_blocking_bounded(&mut self, bytes: &[u8]) {
        unsafe {
            for &b in bytes {
                for _ in 0..200_000 {
                    if rd(self.base + USART_ISR) & USART_ISR_TXE != 0 {
                        break;
                    }
                }
                wr(self.base + USART_TDR, b as u32);
            }
            for _ in 0..400_000 {
                if rd(self.base + USART_ISR) & USART_ISR_TC != 0 {
                    break;
                }
            }
        }
    }

    /// Drain received bytes. Returns 0 when the RX FIFO is empty.
    pub fn read(&mut self, out: &mut [u8]) -> usize {
        let mut count = 0;
        unsafe {
            let isr_addr = self.base + USART_ISR;
            if rd(isr_addr) & USART_ISR_ORE != 0 {
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

    pub fn pump_framed(&mut self, in_frame: &mut bool, can_start_frame: bool) {
        unsafe {
            while !self.tx.is_empty() {
                if !*in_frame && !can_start_frame {
                    break;
                }
                if rd(self.base + USART_ISR) & USART_ISR_TXE == 0 {
                    break;
                }
                if let Some(byte) = self.tx.pop() {
                    if byte != 0 {
                        *in_frame = true;
                    } else if *in_frame {
                        *in_frame = false;
                    }
                    wr(self.base + USART_TDR, byte as u32);
                }
            }
        }
    }
}

// ---------------------------------------------------------------------------
// Radio A: DX-LR42-24T12D (Semtech SX1281 UART transceiver on USART6)
// ---------------------------------------------------------------------------

pub const RADIO_TX_PIN: Pin = Pin::new(GPIOG, 14); // CN10 pin 12 -> DX-LR42 Pin 3 (RX)
pub const RADIO_RX_PIN: Pin = Pin::new(GPIOG, 9); // CN9 pin 63  <- DX-LR42 Pin 4 (TX)
pub const RADIO_AUX_PIN: Pin = Pin::new(GPIOG, 10); // CN9 pin 66  <- DX-LR42 Pin 5 (AUX) / E28 BUSY
pub const RADIO_M0_PIN: Pin = Pin::new(GPIOG, 12); // CN9 pin 65  -> DX-LR42 M0 / E28 RX_EN
pub const RADIO_M1_PIN: Pin = Pin::new(GPIOG, 13); // CN9 pin 68  -> DX-LR42 M1 / E28 TX_EN

#[derive(Clone, Copy, Debug, Default)]
pub struct AuxTracker {
    busy_since_ms: Option<u32>,
}

impl AuxTracker {
    pub const fn new() -> Self {
        AuxTracker {
            busy_since_ms: None,
        }
    }

    pub fn can_transmit(&mut self, aux_high: bool, now_ms: u32) -> bool {
        if !aux_high {
            self.busy_since_ms = None;
            return true;
        }
        match self.busy_since_ms {
            None => {
                self.busy_since_ms = Some(now_ms);
                false
            }
            Some(since) => elapsed_ms(now_ms, since) >= AUX_MAX_WAIT_MS,
        }
    }
}

/// Hardware port driving the `DX-LR42-24T12D` UART transceiver on `USART6` + `GPIOG`.
pub struct DxLr42Port {
    uart: Uart,
    aux: Pin,
    m0: Pin,
    m1: Pin,
    tracker: AuxTracker,
    in_frame: bool,
}

impl DxLr42Port {
    pub fn init_with_baud(baud: DxLr42Baud) -> Self {
        enable_gpio(6); // GPIOG
        unsafe { set_bits(RCC_APB2ENR, 1 << 5) }; // USART6EN

        let mut port = DxLr42Port {
            uart: Uart::new(USART6),
            aux: RADIO_AUX_PIN,
            m0: RADIO_M0_PIN,
            m1: RADIO_M1_PIN,
            tracker: AuxTracker::new(),
            in_frame: false,
        };
        port.activate(baud);
        port
    }

    /// Claim shared `GPIOG` control pins (`PG10`, `PG12`, `PG13`) and start `USART6` at `baud`.
    pub fn activate(&mut self, baud: DxLr42Baud) {
        self.m0.as_open_drain_output(Pull::Up, false);
        self.m1.as_open_drain_output(Pull::Up, false);
        self.aux.as_input(Pull::Down);
        RADIO_TX_PIN.as_alternate(7);
        RADIO_RX_PIN.as_alternate_pull(7, Pull::Up);
        self.in_frame = false;
        self.tracker = AuxTracker::new();
        self.uart.init(baud.bps());
    }

    /// Provision the attached `DX-LR42-24T12D` module to `new_baud` (`AT+BAUDx` + `AT+RESET`)
    /// at the previous baud rate before switching STM32 `USART6->BRR` to `new_baud`.
    pub fn set_baud_live(&mut self, new_baud: DxLr42Baud) {
        if self.uart.baud() != 0 && self.uart.baud() != new_baud.bps() {
            watchdog_kick();
            // Support both hardware AT pin switching (`AT+SWITCH=1`: M0=1, M1=0)
            // and software `"+++"` AT switching (`AT+SWITCH=0`, factory default).
            self.set_at_pin_mode(true);
            spin_delay_us(20_000);
            self.uart.write_blocking_bounded(b"+++");
            spin_delay_us(35_000);
            self.uart.write_blocking_bounded(new_baud.at_command());
            spin_delay_us(25_000);
            self.uart.write_blocking_bounded(b"AT+RESET\r\n");
            spin_delay_us(20_000);
            watchdog_kick();
        }
        self.set_transparent_mode();
        self.in_frame = false;
        self.uart.init(new_baud.bps());
        // Flush any local `"Exit AT\r\nPower On\r\n"` bytes still queued in the FIFO.
        let mut discard = [0u8; 32];
        let _ = self.uart.read(&mut discard);
    }

    pub fn deactivate(&mut self) {
        self.in_frame = false;
        self.uart.disable();
    }

    pub fn set_transparent_mode(&self) {
        self.m0.set(false);
        self.m1.set(false);
    }

    pub fn set_at_pin_mode(&self, at_mode: bool) {
        self.m0.set(at_mode);
        self.m1.set(false);
    }

    pub fn read(&mut self, buf: &mut [u8]) -> usize {
        self.uart.read(buf)
    }

    pub fn write(&mut self, data: &[u8]) -> bool {
        self.uart.write(data)
    }

    pub fn ready(&mut self, now_ms: u32) -> bool {
        self.uart.has_space(256) && self.tracker.can_transmit(self.aux.is_high(), now_ms)
    }

    pub fn tx_busy(&self) -> bool {
        self.uart.is_transmitting()
    }

    pub fn pump(&mut self, now_ms: u32) {
        let can_start = self.tracker.can_transmit(self.aux.is_high(), now_ms);
        self.uart.pump_framed(&mut self.in_frame, can_start);
    }
}

// ---------------------------------------------------------------------------
// Radio B: CDEBYTE E28-2G4M27S (SX1280/SX1281 + 500 mW PA/LNA on SPI1)
// ---------------------------------------------------------------------------

pub const E28_SCK_PIN: Pin = Pin::new(GPIOA, 5); // CN7 pin 10 (D13) -> E28 Pin 5 (SCK)
pub const E28_MISO_PIN: Pin = Pin::new(GPIOA, 6); // CN7 pin 12 (D12) <- E28 Pin 3 (MISO)
pub const E28_MOSI_PIN: Pin = Pin::new(GPIOA, 7); // CN7 pin 14 (D11) -> E28 Pin 4 (MOSI)
pub const E28_NSS_PIN: Pin = Pin::new(GPIOD, 14); // CN7 pin 16 (D10) -> E28 Pin 6 (NSS)
pub const E28_BUSY_PIN: Pin = Pin::new(GPIOG, 10); // CN9 pin 66       <- E28 Pin 12 (BUSY)
pub const E28_NRESET_PIN: Pin = Pin::new(GPIOG, 11); // CN10 pin 30      -> E28 Pin 11 (NRESET)
pub const E28_DIO1_PIN: Pin = Pin::new(GPIOF, 13); // CN10 pin 2 (D7)  <- E28 Pin 13 (DIO1)
pub const E28_RX_EN_PIN: Pin = Pin::new(GPIOG, 12); // CN9 pin 65       -> E28 Pin 8 (RX_EN)
pub const E28_TX_EN_PIN: Pin = Pin::new(GPIOG, 13); // CN9 pin 68       -> E28 Pin 9 (TX_EN)

/// Bare-metal STM32H753ZI `SPI1` (`4 MHz` Mode 0) + GPIO bus for [`E28Driver`].
pub struct Stm32Spi1Sx128x {
    control_pins_claimed: bool,
}

impl Stm32Spi1Sx128x {
    pub const fn new() -> Self {
        Stm32Spi1Sx128x {
            control_pins_claimed: false,
        }
    }

    /// Enable clocks and configure dedicated `SPI1` (`PA5/PA6/PA7/PD14`), `NRESET` (`PG11`),
    /// and `DIO1` (`PF13`). Safe to call while `DX-LR42` owns `PG12`/`PG13` so hardware
    /// probing over `SPI1` never glitches `DX-LR42` `M0`/`M1`.
    pub fn init_spi_bus(&mut self) {
        enable_gpio(0); // GPIOA (PA5/PA6/PA7)
        enable_gpio(3); // GPIOD (PD14 NSS)
        enable_gpio(5); // GPIOF (PF13 DIO1)
        enable_gpio(6); // GPIOG (PG10 BUSY, PG11 NRESET, PG12/13 PA/LNA)

        unsafe {
            // Route `hsi_ker_ck` (64 MHz) -> `per_ck` -> `SPI1` kernel clock!
            modify(RCC_D1CCIPR, RCC_D1CCIPR_CKPERSEL_MSK, 0);
            modify(
                RCC_D2CCIP1R,
                RCC_D2CCIP1R_SPI123SEL_MSK,
                RCC_D2CCIP1R_SPI123SEL_PER,
            );
            set_bits(RCC_APB2ENR, 1 << 12); // SPI1EN
        }

        E28_NSS_PIN.as_output_level(true);
        E28_NRESET_PIN.as_output_level(true);
        E28_DIO1_PIN.as_input(Pull::Down);
        E28_BUSY_PIN.as_input(Pull::Down);

        E28_SCK_PIN.as_alternate(5);
        E28_MISO_PIN.as_alternate_pull(5, Pull::Down);
        E28_MOSI_PIN.as_alternate(5);

        unsafe {
            wr(SPI1 + SPI_CR1, SPI_CR1_SSI);
            wr(SPI1 + SPI_IER, 0);
            wr(SPI1 + SPI_CFG1, SPI_CFG1_8BIT_DIV16);
            wr(SPI1 + SPI_CFG2, SPI_CFG2_MASTER_MODE0);
            wr(SPI1 + SPI_IFCR, SPI_IFCR_ALL);
        }
    }

    /// Claim shared front-end control pins `PG12` (`RX_EN`) and `PG13` (`TX_EN`) as push-pull outputs.
    pub fn claim_rf_switch_pins(&mut self, claim: bool) {
        self.control_pins_claimed = claim;
        if claim {
            E28_RX_EN_PIN.as_output_level(false);
            E28_TX_EN_PIN.as_output_level(false);
        }
    }
}

impl Default for Stm32Spi1Sx128x {
    fn default() -> Self {
        Self::new()
    }
}

impl Sx128xBus for Stm32Spi1Sx128x {
    fn set_nss(&mut self, high: bool) {
        E28_NSS_PIN.set(high);
    }

    fn set_nreset(&mut self, high: bool) {
        E28_NRESET_PIN.set(high);
    }

    fn set_rf_switch(&mut self, rx_en: bool, tx_en: bool) {
        if !self.control_pins_claimed {
            return;
        }
        // Strict break-before-make: never assert RX_EN and TX_EN together.
        if tx_en {
            E28_RX_EN_PIN.set(false);
            E28_TX_EN_PIN.set(true);
        } else if rx_en {
            E28_TX_EN_PIN.set(false);
            E28_RX_EN_PIN.set(true);
        } else {
            E28_TX_EN_PIN.set(false);
            E28_RX_EN_PIN.set(false);
        }
    }

    fn is_busy(&self) -> bool {
        E28_BUSY_PIN.is_high()
    }

    fn dio1_high(&self) -> bool {
        E28_DIO1_PIN.is_high()
    }

    fn spi_transfer(&mut self, tx: &[u8], rx: &mut [u8]) -> bool {
        let len = tx.len().min(rx.len());
        if len == 0 {
            return true;
        }
        unsafe {
            wr(SPI1 + SPI_CR1, SPI_CR1_SSI);
            wr(SPI1 + SPI_IFCR, SPI_IFCR_ALL);
            wr(SPI1 + SPI_CR2, len as u32);
            wr(SPI1 + SPI_CR1, SPI_CR1_SSI | SPI_CR1_SPE);
            wr(SPI1 + SPI_CR1, SPI_CR1_SSI | SPI_CR1_SPE | SPI_CR1_CSTART);

            let mut tx_idx = 0usize;
            let mut rx_idx = 0usize;
            let mut guard = 0u32;
            while rx_idx < len {
                let sr = rd(SPI1 + SPI_SR);
                if tx_idx < len && (sr & SPI_SR_TXP != 0) {
                    wr8(SPI1 + SPI_TXDR, tx[tx_idx]);
                    tx_idx += 1;
                }
                if sr & SPI_SR_RXP != 0 {
                    rx[rx_idx] = rd8(SPI1 + SPI_RXDR);
                    rx_idx += 1;
                }
                guard += 1;
                if guard > 300_000 {
                    wr(SPI1 + SPI_IFCR, SPI_IFCR_ALL);
                    wr(SPI1 + SPI_CR1, SPI_CR1_SSI);
                    return false;
                }
            }
            for _ in 0..20_000 {
                if rd(SPI1 + SPI_SR) & SPI_SR_EOT != 0 {
                    break;
                }
            }
            wr(SPI1 + SPI_IFCR, SPI_IFCR_ALL);
            wr(SPI1 + SPI_CR1, SPI_CR1_SSI);
        }
        true
    }

    fn delay_us(&mut self, us: u32) {
        spin_delay_us(us);
    }
}

/// Dual-transceiver runtime port switching cleanly between `DX-LR42-24T12D` (`USART6`)
/// and `E28-2G4M27S` (`SPI1` + PA/LNA).
pub struct DualRadioPort {
    active: RadioModule,
    uart_baud: DxLr42Baud,
    dx: DxLr42Port,
    e28: E28Driver<Stm32Spi1Sx128x>,
}

impl DualRadioPort {
    pub fn init(initial_radio: RadioModule, uart_baud: DxLr42Baud) -> Self {
        let mut spi = Stm32Spi1Sx128x::new();
        spi.init_spi_bus();
        let e28 = E28Driver::new(spi);
        let dx = DxLr42Port::init_with_baud(uart_baud);

        let mut port = DualRadioPort {
            active: RadioModule::DxLr42,
            uart_baud,
            dx,
            e28,
        };
        if initial_radio == RadioModule::E28_2G4M27S && !port.select_radio(RadioModule::E28_2G4M27S)
        {
            // Safe hardware fallback: if an image defaults to E28 or had E28 saved,
            // fall back automatically to DX-LR42 if no E28 responds on SPI1.
            port.dx.activate(uart_baud);
            port.active = RadioModule::DxLr42;
        }
        port
    }

    pub fn active_radio(&self) -> RadioModule {
        self.active
    }

    pub fn uart_baud(&self) -> DxLr42Baud {
        self.uart_baud
    }

    /// Non-destructively probe `radio` without dropping an active `DX-LR42` link.
    pub fn probe(&mut self, radio: RadioModule) -> bool {
        match radio {
            RadioModule::DxLr42 => true,
            RadioModule::E28_2G4M27S => {
                if self.active == RadioModule::E28_2G4M27S && self.e28.is_present() {
                    return true;
                }
                watchdog_kick();
                self.e28.bus_mut().init_spi_bus();
                let ok = self.e28.init();
                if self.active == RadioModule::DxLr42 {
                    self.e28.shutdown();
                }
                watchdog_kick();
                ok
            }
        }
    }

    pub fn select_radio(&mut self, radio: RadioModule) -> bool {
        match radio {
            RadioModule::DxLr42 => {
                self.e28.shutdown();
                self.e28.bus_mut().claim_rf_switch_pins(false);
                self.dx.activate(self.uart_baud);
                self.active = RadioModule::DxLr42;
                true
            }
            RadioModule::E28_2G4M27S => {
                self.dx.deactivate();
                self.e28.bus_mut().init_spi_bus();
                self.e28.bus_mut().claim_rf_switch_pins(true);
                watchdog_kick();
                let ok = self.e28.init();
                watchdog_kick();
                if ok {
                    self.active = RadioModule::E28_2G4M27S;
                    true
                } else {
                    self.e28.bus_mut().claim_rf_switch_pins(false);
                    self.dx.activate(self.uart_baud);
                    self.active = RadioModule::DxLr42;
                    false
                }
            }
        }
    }

    pub fn apply(&mut self, radio: RadioModule, uart_baud: DxLr42Baud) -> Result<(), Reason> {
        let baud_changed = uart_baud != self.uart_baud;
        self.uart_baud = uart_baud;
        match (self.active, radio) {
            (RadioModule::DxLr42, RadioModule::DxLr42) => {
                if baud_changed {
                    self.dx.set_baud_live(uart_baud);
                }
                Ok(())
            }
            (RadioModule::DxLr42, RadioModule::E28_2G4M27S) => {
                if baud_changed {
                    self.dx.set_baud_live(uart_baud);
                }
                if self.select_radio(RadioModule::E28_2G4M27S) {
                    Ok(())
                } else {
                    Err(Reason::HardwareFault)
                }
            }
            (RadioModule::E28_2G4M27S, RadioModule::DxLr42) => {
                let _ = self.select_radio(RadioModule::DxLr42);
                if baud_changed {
                    self.dx.set_baud_live(uart_baud);
                }
                Ok(())
            }
            (RadioModule::E28_2G4M27S, RadioModule::E28_2G4M27S) => Ok(()),
        }
    }

    pub fn set_transparent_mode(&self) {
        if self.active == RadioModule::DxLr42 {
            self.dx.set_transparent_mode();
        }
    }

    pub fn read(&mut self, buf: &mut [u8]) -> usize {
        match self.active {
            RadioModule::DxLr42 => self.dx.read(buf),
            RadioModule::E28_2G4M27S => self.e28.read(buf),
        }
    }

    pub fn write(&mut self, data: &[u8]) -> bool {
        match self.active {
            RadioModule::DxLr42 => self.dx.write(data),
            RadioModule::E28_2G4M27S => self.e28.write(data),
        }
    }

    pub fn ready(&mut self, now_ms: u32) -> bool {
        match self.active {
            RadioModule::DxLr42 => self.dx.ready(now_ms),
            RadioModule::E28_2G4M27S => self.e28.ready(),
        }
    }

    pub fn tx_busy(&self) -> bool {
        match self.active {
            RadioModule::DxLr42 => self.dx.tx_busy(),
            RadioModule::E28_2G4M27S => self.e28.tx_busy(),
        }
    }

    pub fn pump(&mut self, now_ms: u32) {
        match self.active {
            RadioModule::DxLr42 => self.dx.pump(now_ms),
            RadioModule::E28_2G4M27S => self.e28.pump(now_ms),
        }
    }
}

// ---------------------------------------------------------------------------
// Servo PWM and ignition outputs
// ---------------------------------------------------------------------------

pub const SERVO_OFF_US: u16 = 1_000;
pub const SERVO_ON_US: u16 = 2_000;

pub const SERVO_MIN_US: u16 = 500;
pub const SERVO_MAX_US: u16 = 2_500;

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

pub fn force_safe_outputs() {
    IGN_PIN.set(false);
    // Ensure E28 500 mW PA is never left keyed on fault.
    E28_TX_EN_PIN.set(false);
    unsafe {
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
    let uid = unsafe { rd(0x1FF1_E800) };
    uid ^ role_salt
}

// ---------------------------------------------------------------------------
// Ground board (USB bridge)
// ---------------------------------------------------------------------------

pub struct GroundBoard {
    usb: Uart,
    usb_baud: UsbBaud,
    radio: DualRadioPort,
    flash_saved: bool,
    leds: LedBank,
    device: u32,
}

impl GroundBoard {
    /// Bring up the bridge board using saved Flash Bank 2 Sector 7 link config
    /// (or compile-time Cargo feature defaults on a blank board).
    pub fn init() -> Self {
        enable_gpio(1);
        enable_gpio(3);
        enable_gpio(4);
        unsafe {
            set_bits(RCC_APB1LENR, 1 << 18); // USART3EN
        }

        Pin::new(GPIOD, 8).as_alternate(7);
        Pin::new(GPIOD, 9).as_alternate_pull(7, Pull::Up);

        let leds = LedBank::nucleo();
        leds.init();
        IGN_PIN.set(false);

        timebase_init();
        watchdog_init();

        let boot_cfg = boot_link_state();
        let mut usb = Uart::new(USART3);
        usb.init(boot_cfg.usb_baud.bps());
        let radio = DualRadioPort::init(boot_cfg.radio, boot_cfg.uart_baud);

        GroundBoard {
            usb,
            usb_baud: boot_cfg.usb_baud,
            radio,
            flash_saved: boot_cfg.flash_saved,
            leds,
            device: device_id(0x4752_0000),
        }
    }

    pub fn pump(&mut self) {
        self.usb.pump();
        self.radio.pump(now_ms());
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

    fn usb_tx_busy(&self) -> bool {
        self.usb.is_transmitting()
    }

    fn read_radio(&mut self, buf: &mut [u8]) -> usize {
        self.radio.read(buf)
    }

    fn write_radio(&mut self, data: &[u8]) {
        let _ = self.radio.write(data);
    }

    fn radio_ready(&mut self) -> bool {
        self.radio.ready(now_ms())
    }

    fn radio_tx_busy(&self) -> bool {
        self.radio.tx_busy()
    }

    fn set_radio_transparent(&mut self) {
        self.radio.set_transparent_mode();
    }

    fn link_state(&self) -> LinkState {
        LinkState {
            usb_baud: self.usb_baud,
            uart_baud: self.radio.uart_baud(),
            radio: self.radio.active_radio(),
            flash_saved: self.flash_saved,
        }
    }

    fn probe_radio_module(&mut self, radio: RadioModule) -> bool {
        self.radio.probe(radio)
    }

    fn apply_link_config(&mut self, cfg: LinkConfig) -> Result<bool, Reason> {
        self.radio.apply(cfg.radio, cfg.uart_baud)?;
        if cfg.usb_baud != self.usb_baud {
            self.usb_baud = cfg.usb_baud;
            self.usb.init(cfg.usb_baud.bps());
        }
        let persisted = if cfg.persist() {
            let saved = persist_link_state(LinkState {
                usb_baud: self.usb_baud,
                uart_baud: self.radio.uart_baud(),
                radio: self.radio.active_radio(),
                flash_saved: true,
            });
            self.flash_saved = saved;
            saved
        } else {
            false
        };
        Ok(persisted)
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
    usb_baud_shadow: UsbBaud,
    radio: DualRadioPort,
    flash_saved: bool,
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
        enable_gpio(4); // LEDs + servos + ignition

        Pin::new(GPIOE, 9).as_alternate(1);
        Pin::new(GPIOE, 11).as_alternate(1);
        let mut servos = ServoPair::new();
        servos.init();

        let ignition = IGN_PIN;
        ignition.as_output();
        ignition.set(false);

        let leds = LedBank::nucleo();
        leds.init();

        timebase_init();
        watchdog_init();

        let boot_cfg = boot_link_state();
        let radio = DualRadioPort::init(boot_cfg.radio, boot_cfg.uart_baud);

        VehicleBoard {
            usb_baud_shadow: boot_cfg.usb_baud,
            radio,
            flash_saved: boot_cfg.flash_saved,
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
        self.radio.pump(now_ms());
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
        self.radio.ready(now_ms())
    }

    fn radio_tx_busy(&self) -> bool {
        self.radio.tx_busy()
    }

    fn set_radio_transparent(&mut self) {
        self.radio.set_transparent_mode();
    }

    fn link_state(&self) -> LinkState {
        LinkState {
            usb_baud: self.usb_baud_shadow,
            uart_baud: self.radio.uart_baud(),
            radio: self.radio.active_radio(),
            flash_saved: self.flash_saved,
        }
    }

    fn probe_radio_module(&mut self, radio: RadioModule) -> bool {
        self.radio.probe(radio)
    }

    fn apply_link_config(&mut self, cfg: LinkConfig) -> Result<bool, Reason> {
        self.radio.apply(cfg.radio, cfg.uart_baud)?;
        self.usb_baud_shadow = cfg.usb_baud;
        let persisted = if cfg.persist() {
            let saved = persist_link_state(LinkState {
                usb_baud: self.usb_baud_shadow,
                uart_baud: self.radio.uart_baud(),
                radio: self.radio.active_radio(),
                flash_saved: true,
            });
            self.flash_saved = saved;
            saved
        } else {
            false
        };
        Ok(persisted)
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
    fn baud_divider_matches_reference_across_all_presets() {
        assert_eq!(baud_divider(64_000_000, 460_800), 139);
        assert_eq!(baud_divider(64_000_000, 230_400), 278);
        assert_eq!(baud_divider(64_000_000, 115_200), 556);
        assert_eq!(baud_divider(64_000_000, 57_600), 1_111);
        assert_eq!(baud_divider(64_000_000, 38_400), 1_667);
        assert_eq!(baud_divider(64_000_000, 19_200), 3_333);
        assert_eq!(baud_divider(64_000_000, 9_600), 6_667);
        assert_eq!(baud_divider(64_000_000, 4_800), 13_333);
        assert_eq!(baud_divider(64_000_000, 2_400), 26_667);
        assert_eq!(baud_divider(64_000_000, 0), 0);
    }

    #[test]
    fn flash_append_log_scans_latest_valid_and_survives_torn_write() {
        let mut sector = [0xFFu8; FLASH_WORD_BYTES * 8];
        assert_eq!(scan_config_sector(&sector), (None, Some(0)));

        let first = LinkState {
            usb_baud: UsbBaud::Baud115200,
            uart_baud: DxLr42Baud::Baud57600,
            radio: RadioModule::DxLr42,
            flash_saved: true,
        };
        sector[0..32].copy_from_slice(&encode_flash_record(first, 1));
        assert_eq!(scan_config_sector(&sector), (Some((first, 1)), Some(1)));

        let second = LinkState {
            usb_baud: UsbBaud::Baud230400,
            uart_baud: DxLr42Baud::Baud115200,
            radio: RadioModule::E28_2G4M27S,
            flash_saved: true,
        };
        sector[32..64].copy_from_slice(&encode_flash_record(second, 2));
        assert_eq!(scan_config_sector(&sector), (Some((second, 2)), Some(2)));

        // Simulate power-loss half-way through writing slot 2: scanner falls back cleanly
        // to `second` (slot 1) and picks slot 3 as the next clean erased flash word!
        sector[64..70].copy_from_slice(b"GSE1\x03\x00");
        assert_eq!(scan_config_sector(&sector), (Some((second, 2)), Some(3)));
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
        assert_eq!(CORE_HZ / 1000 - 1, 63_999);
    }

    #[test]
    fn aux_tracker_holds_off_while_busy_and_releases_on_failsafe_or_low() {
        let mut aux = AuxTracker::new();
        assert!(aux.can_transmit(false, 1_000));
        assert!(!aux.can_transmit(true, 1_000));
        assert!(!aux.can_transmit(true, 1_000 + AUX_MAX_WAIT_MS - 1));
        assert!(aux.can_transmit(true, 1_000 + AUX_MAX_WAIT_MS));
        assert!(aux.can_transmit(false, u32::MAX - 10));
        assert!(!aux.can_transmit(true, u32::MAX - 5));
        assert!(!aux.can_transmit(true, 10));
        assert!(aux.can_transmit(true, (u32::MAX - 5).wrapping_add(AUX_MAX_WAIT_MS)));
    }
}
