//! Vehicle board image: `DX-LR42-24T12D` SX1281 (`USART6`) receiver and actuator driver.
//!
//! FILL and SRV are 50 Hz servos on TIM1, IGN is a push-pull output. Every
//! command is re-checked here even if the ground station already gated it;
//! the vehicle is the last line of defence.

#![no_std]
#![no_main]

use cortex_m_rt::entry;
use gse_board::hw::VehicleBoard;
use gse_fw_core::vehicle::Vehicle;

#[entry]
fn main() -> ! {
    let mut vehicle = Vehicle::new(VehicleBoard::init());
    loop {
        vehicle.poll();
        vehicle.hal_mut().pump();
    }
}
