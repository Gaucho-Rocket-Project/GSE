//! Hardware abstraction used by the firmware logic.
//!
//! The real implementations live in `firmware/stm32h753`; tests and the desktop
//! simulator use the mock in [`crate::mock`]. Keeping this trait tiny makes the
//! safety-critical logic testable on a normal PC.

use gse_proto::{DxLr42Baud, LinkConfig, RadioModule, Reason, UsbBaud};

/// Front-panel LEDs (NUCLEO-H753ZI: green, yellow, red).
#[derive(Clone, Copy, Debug, Default, PartialEq, Eq)]
pub struct Leds {
    /// Green: link to the ground station (vehicle) / link to the PC (ground).
    pub link: bool,
    /// Yellow: traffic activity (radio or USB frames).
    pub activity: bool,
    /// Red: fault or ignition powered.
    pub fault: bool,
}

/// Active physical link configuration reported by a board.
#[derive(Clone, Copy, Debug, Default, PartialEq, Eq)]
pub struct LinkState {
    pub usb_baud: UsbBaud,
    pub uart_baud: DxLr42Baud,
    pub radio: RadioModule,
    pub flash_saved: bool,
}

/// Vehicle (flight) board hardware.
pub trait VehicleHal {
    /// Free-running millisecond counter. Wrapping is expected and handled.
    fn now_ms(&self) -> u32;

    /// Drive the FILL valve servo (true = open).
    fn set_fill(&mut self, on: bool);
    /// Drive the vehicle servo power output (true = powered).
    fn set_srv(&mut self, on: bool);
    /// Drive the ignition output (true = PWR).
    fn set_ign(&mut self, on: bool);

    /// Non-blocking read of bytes received from the active RF module.
    /// Returns the number of bytes written into `buf` (0 when nothing pending).
    fn read_radio(&mut self, buf: &mut [u8]) -> usize;
    /// Write bytes to the active RF module (`USART6` `DX-LR42` or `SPI1` `E28-2G4M27S`).
    fn write_radio(&mut self, data: &[u8]);
    /// True when the TX queue has space and the transceiver can accept a frame.
    fn radio_ready(&mut self) -> bool {
        true
    }
    /// True while previously queued radio TX bytes are still draining onto the air.
    /// Used so `LinkConfigAck` completes on the old link before switching hardware.
    fn radio_tx_busy(&self) -> bool {
        false
    }
    /// Drive the `DX-LR42-24T12D` hardware mode pins (`PG12` `M0`, `PG13` `M1`)
    /// to `(0, 0)` when operating in UART radio mode.
    fn set_radio_transparent(&mut self) {}

    /// Current active radio / UART configuration on this board.
    fn link_state(&self) -> LinkState {
        LinkState::default()
    }
    /// Non-destructively verify that `radio` hardware is present before acknowledging
    /// a live switchover request.
    fn probe_radio_module(&mut self, _radio: RadioModule) -> bool {
        true
    }
    /// Apply `cfg` (radio module + UART baud + optional flash persistence) after
    /// the acknowledgement frame has drained from TX. Returns `Ok(persisted)`.
    fn apply_link_config(&mut self, cfg: LinkConfig) -> Result<bool, Reason> {
        Ok(cfg.persist())
    }

    fn set_leds(&mut self, leds: Leds);
    fn kick_watchdog(&mut self);
    /// Unique-ish 32-bit board id reported in HELLO frames.
    fn device_id(&self) -> u32;
}

/// Ground (bridge) board hardware.
pub trait GroundHal {
    fn now_ms(&self) -> u32;

    fn read_usb(&mut self, buf: &mut [u8]) -> usize;
    fn write_usb(&mut self, data: &[u8]);
    /// Bytes currently queued for USB TX (for back-pressure).
    fn usb_tx_pending(&self) -> usize {
        0
    }
    /// True while queued USB TX bytes or shift register (`USART3` `TC`) are still busy.
    fn usb_tx_busy(&self) -> bool {
        self.usb_tx_pending() > 0
    }

    fn read_radio(&mut self, buf: &mut [u8]) -> usize;
    fn write_radio(&mut self, data: &[u8]);
    fn radio_ready(&mut self) -> bool {
        true
    }
    fn radio_tx_busy(&self) -> bool {
        false
    }
    fn set_radio_transparent(&mut self) {}

    fn link_state(&self) -> LinkState {
        LinkState::default()
    }
    fn probe_radio_module(&mut self, _radio: RadioModule) -> bool {
        true
    }
    fn apply_link_config(&mut self, cfg: LinkConfig) -> Result<bool, Reason> {
        Ok(cfg.persist())
    }

    fn set_leds(&mut self, leds: Leds);
    fn kick_watchdog(&mut self);
    fn device_id(&self) -> u32;
}

/// Elapsed milliseconds with wrap-around safety.
#[inline]
pub const fn elapsed_ms(now: u32, since: u32) -> u32 {
    now.wrapping_sub(since)
}

/// True when `now` is at or past `deadline` (both wrapping millisecond stamps).
#[inline]
pub const fn deadline_passed(now: u32, deadline: u32) -> bool {
    now.wrapping_sub(deadline) < 0x8000_0000
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn elapsed_handles_wrap() {
        assert_eq!(elapsed_ms(10, 5), 5);
        assert_eq!(elapsed_ms(3, u32::MAX - 2), 6);
        assert_eq!(elapsed_ms(u32::MAX, 0), u32::MAX);
    }

    #[test]
    fn deadline_handles_wrap() {
        assert!(deadline_passed(10, 5));
        assert!(!deadline_passed(4, 5));
        assert!(deadline_passed(2, u32::MAX - 2));
        assert!(!deadline_passed(u32::MAX - 3, u32::MAX - 2));
    }
}
