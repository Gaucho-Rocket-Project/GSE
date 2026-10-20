//! Ground board image: USB CDC (USART3) <-> SX1281 (USART6) bridge.
//!
//! The bridge never interprets operator commands; it forwards frames and
//! answers PING/HELLO/LINKSTAT so the host can detect it. All safety decisions
//! happen on the vehicle.

#![no_std]
#![no_main]

use cortex_m_rt::entry;
use gse_board::hw::GroundBoard;
use gse_fw_core::ground::Ground;

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
