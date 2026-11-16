//! Vehicle board image running the GSE-simple pad node over LoRa.
//!
//! Same NUCLEO-H753ZI vehicle board as `gse-vehicle`, but with the ESP32
//! pad's control algorithm (`gse_fw_core::pad`): it follows the host's
//! 10 Hz stream of complete desired states, fails safe after the link
//! watchdog (FILL and PURGE closed, both VEHICLE servos OPEN, IGN off) and
//! fires the igniter once per armed press. The GSE fill timer, interlocks,
//! countdown and E-STOP latches run app-side.
//!
//! Outputs: FILL `PE9`, VEH `PE11`, VEH2 `PD12`, PURGE `PD13`, IGN `PE13`,
//! heartbeat `PE14` (see `gse_board::hw::pad_servo_route`). Flash
//! `gse-vehicle` instead to go back to the original vehicle state machine.

#![no_std]
#![no_main]

use cortex_m_rt::entry;
use gse_board::hw::{PadVehicleBoard, PAD_SERVO_SETUP};
use gse_fw_core::pad::Pad;

#[entry]
fn main() -> ! {
    let mut pad = Pad::with_board(PadVehicleBoard::init(), PAD_SERVO_SETUP);
    loop {
        pad.poll();
        pad.hal_mut().pump();
    }
}
