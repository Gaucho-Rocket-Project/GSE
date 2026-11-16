#![allow(clippy::missing_safety_doc)]

#[path = "platform.rs"]
mod platform;

use esp_idf_hal::peripherals::Peripherals;
use gse_fw_core::vehicle::Vehicle;

fn main() {
    esp_idf_sys::link_patches();
    let peripherals = Peripherals::take().expect("ESP32 peripherals already taken");
    let mut vehicle = Vehicle::new(platform::vehicle_hal(&peripherals));
    loop {
        vehicle.poll();
    }
}
