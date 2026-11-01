#![no_std]
#![no_main]

use core::{fmt::{self, Write}, ptr::{read_volatile, write_volatile}, sync::atomic::{AtomicU32, Ordering}};
use cortex_m_rt::{entry, exception};
use panic_halt as _;

// The H753 starts from HSI at 64 MHz with all bus prescalers at /1.
// This standalone image intentionally does not change the reset clock tree.
const RCC: usize = 0x5802_4400;
const GPIOD: usize = 0x5802_0c00;
const USART3: usize = 0x4000_4800;
static MILLIS: AtomicU32 = AtomicU32::new(0);

unsafe fn read(addr: usize) -> u32 { read_volatile(addr as *const u32) }
unsafe fn write(addr: usize, value: u32) { write_volatile(addr as *mut u32, value) }
unsafe fn update(addr: usize, clear: u32, set: u32) { write(addr, (read(addr) & !clear) | set); }

#[exception]
fn SysTick() { MILLIS.fetch_add(1, Ordering::Relaxed); }
fn now() -> u32 { MILLIS.load(Ordering::Relaxed) }
fn elapsed(since: u32, duration: u32) -> bool { now().wrapping_sub(since) >= duration }

fn init() {
    unsafe {
        // Enable GPIO D and USART3, then read back to allow clocks to propagate.
        update(RCC + 0xe0, 0, 1 << 3);
        update(RCC + 0xe8, 0, 1 << 18);
        let _ = read(RCC + 0xe8);
        // PD8/PD9: AF7 USART3 TX/RX, push-pull, pull-up on RX.
        update(GPIOD, (3 << 16) | (3 << 18), (2 << 16) | (2 << 18));
        update(GPIOD + 0x04, (1 << 8) | (1 << 9), 0);
        update(GPIOD + 0x0c, (3 << 16) | (3 << 18), 1 << 18);
        update(GPIOD + 0x24, 0xff, 0x77); // AFRH, pins 8 and 9
        // Default USART3 kernel clock is PCLK1, 64 MHz. 8N1, no flow control.
        write(USART3, 0);
        write(USART3 + 0x04, 0);
        write(USART3 + 0x08, 0);
        write(USART3 + 0x0c, 556); // round(64 MHz / 115200)
        write(USART3, (1 << 0) | (1 << 2) | (1 << 3));
        let mut p = cortex_m::Peripherals::steal();
        p.SYST.set_clock_source(cortex_m::peripheral::syst::SystClkSource::Core);
        p.SYST.set_reload(64_000 - 1);
        p.SYST.clear_current();
        p.SYST.enable_interrupt();
        p.SYST.enable_counter();
    }
}

struct Serial;
impl Write for Serial {
    fn write_str(&mut self, s: &str) -> fmt::Result {
        for b in s.bytes() {
            unsafe {
                while read(USART3 + 0x1c) & (1 << 7) == 0 {}
                write(USART3 + 0x28, b as u32);
            }
        }
        Ok(())
    }
}
fn receive() -> Option<u8> {
    unsafe {
        let flags = read(USART3 + 0x1c);
        if flags & 0x0f != 0 { write(USART3 + 0x20, flags & 0x0f); }
        if flags & (1 << 5) != 0 { Some(read(USART3 + 0x24) as u8) } else { None }
    }
}
fn parse_pong(line: &[u8]) -> Option<u32> {
    let s = core::str::from_utf8(line).ok()?.trim();
    let value = s.strip_prefix("PONG ")?;
    if value.is_empty() || !value.bytes().all(|b| b.is_ascii_digit()) { return None; }
    value.parse().ok()
}

#[entry]
fn main() -> ! {
    init();
    let mut ack = 0u32;
    let mut last_send = now();
    let _ = write!(Serial, "PING {}\n", ack);
    let mut line = [0u8; 48];
    let mut len = 0usize;
    let mut overflow = false;
    loop {
        if let Some(b) = receive() {
            if b == b'\n' {
                if !overflow {
                    if let Some(accepted) = parse_pong(&line[..len]) {
                        // Always trust the host's last accepted count, including a
                        // mismatch after a host/board reset or a lost response.
                        ack = accepted;
                        // Modest pacing avoids flooding the terminal.
                        let pause = now();
                        while !elapsed(pause, 250) {}
                        let _ = write!(Serial, "PING {}\n", ack);
                        last_send = now();
                    }
                }
                len = 0;
                overflow = false;
            } else if len < line.len() && !overflow {
                line[len] = b;
                len += 1;
            } else { overflow = true; }
        }
        if elapsed(last_send, 5_000) {
            let _ = write!(Serial, "PING {}\n", ack);
            last_send = now();
        }
    }
}
