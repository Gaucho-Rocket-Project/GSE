//! Host-side compile and test harness for `firmware/stm32h753/src/hw.rs`.
//!
//! The firmware package is excluded from the workspace because it needs a
//! Cortex-M target and `cortex-m-rt`. This crate pulls in the same hardware
//! module with `#[path]` and builds it for the host, which catches type and
//! borrow errors in the board layer and runs its portable unit tests. Nothing
//! here touches real hardware: the tests only exercise pure helpers such as
//! the UART ring buffer and the servo pulse maths.

#![allow(dead_code)]

#[path = "../../../firmware/stm32h753/src/hw.rs"]
pub mod hw;
