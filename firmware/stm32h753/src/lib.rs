//! GSE board support library.
//!
//! The hardware layer lives in [`hw`]; the two binaries in `src/bin` are thin
//! wrappers that hand a board object to the corresponding state machine in
//! `gse-fw-core`.
//!
//! The same `hw.rs` file is compiled and unit-tested on the host through
//! `crates/gse-fw-check`, which keeps the register math and the transmit ring
//! buffer covered by tests even though this sandbox has no Cortex-M target.

#![no_std]

pub mod hw;

/// Firmware version reported in HELLO frames comes from `gse-proto`.
pub use gse_proto::FIRMWARE_VERSION;
