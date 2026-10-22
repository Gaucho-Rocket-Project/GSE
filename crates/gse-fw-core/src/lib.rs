//! Platform independent firmware logic for the two NUCLEO-H753ZI boards.
//!
//! The crate is `no_std` (it is linked into the flight firmware) but all of its
//! logic is exercised on the host by unit tests and by the `gse-sim` desktop
//! simulator, which pumps the two state machines over a byte pipe.

#![cfg_attr(not(any(test, feature = "mock")), no_std)]
#![forbid(unsafe_code)]

pub mod ground;
pub mod hal;
pub mod radio;
pub mod vehicle;

#[cfg(any(test, feature = "mock"))]
pub mod mock;

pub use ground::Ground;
pub use hal::{GroundHal, Leds, VehicleHal};
pub use radio::{DxLr42Receiver, DxLr42Stats};
pub use vehicle::{Vehicle, VehicleState};
