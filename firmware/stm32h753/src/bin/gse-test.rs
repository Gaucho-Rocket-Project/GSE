//! Single-board servo/ignition test firmware.
//!
//! The connected NUCLEO acts as both the GSE USB bridge and the local Vehicle
//! endpoint, so the host console can exercise its servos and IGN output without
//! a second vehicle board or a radio module.

#![no_std]
#![no_main]

use cortex_m_rt::entry;
use gse_board::hw::GseTestBoard;
use gse_fw_core::vehicle::Vehicle;

#[entry]
fn main() -> ! {
    let mut vehicle = Vehicle::new(GseTestBoard::init());
    loop {
        vehicle.poll();
        vehicle.hal_mut().pump();
    }
}
