//! Ground board image. Normally this is the USB-to-radio GSE bridge. With the
//! `pingpong-regression` feature, the normal bridge main loop is replaced by
//! the original minimal ASCII USART3 heartbeat from the `pingpong` branch.

#![no_std]
#![no_main]

use cortex_m_rt::entry;

#[cfg(not(feature = "pingpong-regression"))]
use gse_board::hw::GroundBoard;
#[cfg(not(feature = "pingpong-regression"))]
use gse_fw_core::ground::Ground;

#[cfg(feature = "pingpong-regression")]
use core::{
    fmt::{self, Write},
    ptr::{read_volatile, write_volatile},
};

#[cfg(feature = "pingpong-regression")]
const RCC: usize = 0x5802_4400;
#[cfg(feature = "pingpong-regression")]
const GPIOD: usize = 0x5802_0c00;
#[cfg(feature = "pingpong-regression")]
const USART3: usize = 0x4000_4800;

#[cfg(feature = "pingpong-regression")]
unsafe fn read(addr: usize) -> u32 {
    read_volatile(addr as *const u32)
}

#[cfg(feature = "pingpong-regression")]
unsafe fn write(addr: usize, value: u32) {
    write_volatile(addr as *mut u32, value)
}

#[cfg(feature = "pingpong-regression")]
unsafe fn update(addr: usize, clear: u32, set: u32) {
    write(addr, (read(addr) & !clear) | set);
}

#[cfg(feature = "pingpong-regression")]
fn now() -> u32 {
    gse_board::hw::now_ms()
}

#[cfg(feature = "pingpong-regression")]
fn elapsed(since: u32, duration: u32) -> bool {
    now().wrapping_sub(since) >= duration
}

#[cfg(feature = "pingpong-regression")]
fn init_pingpong() {
    unsafe {
        // Match the standalone pingpong image: HSI 64 MHz, USART3 on PD8/PD9,
        // 115200 8N1, with no radio, bridge, protocol, or control setup.
        update(RCC + 0xe0, 0, 1 << 3);
        update(RCC + 0xe8, 0, 1 << 18);
        let _ = read(RCC + 0xe8);
        update(GPIOD, (3 << 16) | (3 << 18), (2 << 16) | (2 << 18));
        update(GPIOD + 0x04, (1 << 8) | (1 << 9), 0);
        update(GPIOD + 0x0c, (3 << 16) | (3 << 18), 1 << 18);
        update(GPIOD + 0x24, 0xff, 0x77);
        write(USART3, 0);
        write(USART3 + 0x04, 0);
        write(USART3 + 0x08, 0);
        write(USART3 + 0x0c, 556);
        write(USART3, (1 << 0) | (1 << 2) | (1 << 3));
    }
    // This is the same 64 MHz / 1 ms SysTick setup used by the known-good
    // pingpong image, via the GSE board's shared timebase helper.
    gse_board::hw::timebase_init();
}

#[cfg(feature = "pingpong-regression")]
struct Serial;

#[cfg(feature = "pingpong-regression")]
impl Write for Serial {
    fn write_str(&mut self, text: &str) -> fmt::Result {
        for byte in text.bytes() {
            unsafe {
                while read(USART3 + 0x1c) & (1 << 7) == 0 {}
                write(USART3 + 0x28, byte as u32);
            }
        }
        Ok(())
    }
}

#[cfg(feature = "pingpong-regression")]
fn receive() -> Option<u8> {
    unsafe {
        let flags = read(USART3 + 0x1c);
        if flags & 0x0f != 0 {
            write(USART3 + 0x20, flags & 0x0f);
        }
        if flags & (1 << 5) != 0 {
            Some(read(USART3 + 0x24) as u8)
        } else {
            None
        }
    }
}

#[cfg(feature = "pingpong-regression")]
fn parse_pong(line: &[u8]) -> Option<u32> {
    let text = core::str::from_utf8(line).ok()?.trim();
    let value = text.strip_prefix("PONG ")?;
    if value.is_empty() || !value.bytes().all(|byte| byte.is_ascii_digit()) {
        return None;
    }
    value.parse().ok()
}

#[cfg(feature = "pingpong-regression")]
#[entry]
fn main() -> ! {
    init_pingpong();
    let mut accepted = 0u32;
    let mut last_send = now();
    let _ = write!(Serial, "PING {}\n", accepted);
    let mut line = [0u8; 48];
    let mut len = 0usize;
    let mut overflow = false;

    loop {
        if let Some(byte) = receive() {
            if byte == b'\n' {
                if !overflow {
                    if let Some(host_accepted) = parse_pong(&line[..len]) {
                        // Adopt the host count to recover from reset or lost data.
                        accepted = host_accepted;
                        let pause = now();
                        while !elapsed(pause, 250) {}
                        let _ = write!(Serial, "PING {}\n", accepted);
                        last_send = now();
                    }
                }
                len = 0;
                overflow = false;
            } else if len < line.len() && !overflow {
                line[len] = byte;
                len += 1;
            } else {
                overflow = true;
            }
        }

        if elapsed(last_send, 5_000) {
            let _ = write!(Serial, "PING {}\n", accepted);
            last_send = now();
        }
    }
}

#[cfg(not(feature = "pingpong-regression"))]
#[entry]
fn main() -> ! {
    let mut bridge = Ground::new(GroundBoard::init());
    loop {
        // Poll continuously: the USB side runs at 115200 baud and a burst of
        // host frames must not wait for a timer tick.
        bridge.poll();
        bridge.hal_mut().pump();
    }
}
