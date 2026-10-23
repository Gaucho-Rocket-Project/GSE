//! Compact vehicle state snapshot (16 bytes) carried by STATE/PONG/ACK/EVENT.
//!
//! The snapshot is deliberately small: it is broadcast four times per second
//! over a 9600 baud radio link, and the flight firmware has no allocator.

use core::fmt;

/// State flag bits.
pub mod flags {
    /// FILL valve is open (servo in the ON position).
    pub const FILL_OPEN: u16 = 1 << 0;
    /// Vehicle servos are powered.
    pub const SRV_ON: u16 = 1 << 1;
    /// Ignition output is in PWR.
    pub const IGN_PWR: u16 = 1 << 2;
    /// The fill countdown is running.
    pub const FILL_TIMER_ACTIVE: u16 = 1 << 3;
    /// A complete fill cycle (ON for the full duration, then OFF) has finished.
    pub const FILL_SEQ_COMPLETE: u16 = 1 << 4;
    /// The last fill cycle ended before the configured duration elapsed.
    pub const FILL_SEQ_ABORTED: u16 = 1 << 5;
    /// Outputs were forced safe because the command link timed out.
    pub const LINK_LOSS_SAFE: u16 = 1 << 6;
    /// The last command was rejected by a safety interlock.
    pub const LAST_REJECTED: u16 = 1 << 7;
    /// FILL was closed automatically by the vehicle's fill timer.
    pub const FILL_AUTO_OFF: u16 = 1 << 8;
    /// Firmware booted with all outputs safe (cleared on first command).
    pub const BOOT_SAFE: u16 = 1 << 9;
    /// Emergency-stop has been asserted since boot.
    pub const ESTOP: u16 = 1 << 10;
    /// The vehicle is inhibited (e.g. watchdog reset recovery); operator must
    /// reset the fill sequence before ignition.
    pub const INHIBITED: u16 = 1 << 11;
    /// FILL was manually turned OFF prior to timer expiry; remaining duration
    /// is preserved for resume and downstream channels (SRV / SRV+IGN) unlock.
    pub const FILL_PAUSED: u16 = 1 << 12;
}

/// Fault bits (sticky until cleared with the SYSTEM/CLEAR_FAULTS command).
pub mod faults {
    pub const UART_OVERRUN: u16 = 1 << 0;
    pub const CRC_ERROR: u16 = 1 << 1;
    pub const LINK_TIMEOUT: u16 = 1 << 2;
    pub const WATCHDOG_RESET: u16 = 1 << 3;
    pub const RADIO_ERROR: u16 = 1 << 4;
    pub const CONFIG_ERROR: u16 = 1 << 5;
    pub const OUTPUT_FAULT: u16 = 1 << 6;
}

#[derive(Clone, Copy, PartialEq, Eq, Default)]
pub struct StateSnapshot {
    pub flags: u16,
    pub faults: u16,
    pub fill_remaining_ms: u32,
    pub fill_duration_s: u16,
    pub last_cmd_id: u16,
    pub uptime_ms: u32,
}

impl StateSnapshot {
    pub const WIRE_LEN: usize = 16;

    pub const fn all_safe() -> Self {
        StateSnapshot {
            flags: flags::BOOT_SAFE,
            faults: 0,
            fill_remaining_ms: 0,
            fill_duration_s: 0,
            last_cmd_id: 0,
            uptime_ms: 0,
        }
    }

    pub fn encode(&self, out: &mut [u8]) -> Option<usize> {
        if out.len() < Self::WIRE_LEN {
            return None;
        }
        out[0..2].copy_from_slice(&self.flags.to_le_bytes());
        out[2..4].copy_from_slice(&self.faults.to_le_bytes());
        out[4..8].copy_from_slice(&self.fill_remaining_ms.to_le_bytes());
        out[8..10].copy_from_slice(&self.fill_duration_s.to_le_bytes());
        out[10..12].copy_from_slice(&self.last_cmd_id.to_le_bytes());
        out[12..16].copy_from_slice(&self.uptime_ms.to_le_bytes());
        Some(Self::WIRE_LEN)
    }

    pub fn decode(data: &[u8]) -> Option<Self> {
        if data.len() < Self::WIRE_LEN {
            return None;
        }
        Some(StateSnapshot {
            flags: u16::from_le_bytes([data[0], data[1]]),
            faults: u16::from_le_bytes([data[2], data[3]]),
            fill_remaining_ms: u32::from_le_bytes([data[4], data[5], data[6], data[7]]),
            fill_duration_s: u16::from_le_bytes([data[8], data[9]]),
            last_cmd_id: u16::from_le_bytes([data[10], data[11]]),
            uptime_ms: u32::from_le_bytes([data[12], data[13], data[14], data[15]]),
        })
    }

    #[inline]
    pub const fn has(&self, flag: u16) -> bool {
        self.flags & flag != 0
    }

    pub const fn fill_open(&self) -> bool {
        self.has(flags::FILL_OPEN)
    }

    pub const fn srv_on(&self) -> bool {
        self.has(flags::SRV_ON)
    }

    pub const fn ign_pwr(&self) -> bool {
        self.has(flags::IGN_PWR)
    }

    pub const fn fill_timer_active(&self) -> bool {
        self.has(flags::FILL_TIMER_ACTIVE)
    }

    pub const fn fill_seq_complete(&self) -> bool {
        self.has(flags::FILL_SEQ_COMPLETE)
    }

    pub const fn fill_paused(&self) -> bool {
        self.has(flags::FILL_PAUSED)
    }

    pub const fn fill_seq_aborted(&self) -> bool {
        self.has(flags::FILL_SEQ_ABORTED)
    }

    /// True once `FILL` has completed automatically OR been manually paused
    /// (`FILL -> OFF`) without an uncommanded abort.
    pub const fn fill_ready(&self) -> bool {
        (self.fill_seq_complete() || self.fill_paused()) && !self.fill_seq_aborted()
    }

    pub const fn estop(&self) -> bool {
        self.has(flags::ESTOP)
    }

    pub const fn inhibited(&self) -> bool {
        self.has(flags::INHIBITED)
    }

    pub const fn outputs_safe(&self) -> bool {
        !self.fill_open() && !self.srv_on() && !self.ign_pwr()
    }

    pub const fn has_fault(&self) -> bool {
        self.faults != 0
    }

    /// Short human readable state, e.g. `FILL=ON SRV=OFF IGN=SAFE`.
    pub fn summary(&self) -> StateSummary {
        StateSummary {
            fill: self.fill_open(),
            srv: self.srv_on(),
            ign: self.ign_pwr(),
        }
    }
}

impl fmt::Debug for StateSnapshot {
    fn fmt(&self, f: &mut fmt::Formatter<'_>) -> fmt::Result {
        write!(
            f,
            "State({} fill={} srv={} ign={} flags=0x{:04X} faults=0x{:04X} rem={}ms dur={}s up={}ms)",
            if self.fill_open() { "FILL=ON" } else { "FILL=OFF" },
            self.fill_open(),
            self.srv_on(),
            self.ign_pwr(),
            self.flags,
            self.faults,
            self.fill_remaining_ms,
            self.fill_duration_s,
            self.uptime_ms
        )
    }
}

#[derive(Clone, Copy, PartialEq, Eq, Debug)]
pub struct StateSummary {
    pub fill: bool,
    pub srv: bool,
    pub ign: bool,
}

impl StateSummary {
    pub const fn is_safe(&self) -> bool {
        !self.fill && !self.srv && !self.ign
    }
}

impl fmt::Display for StateSummary {
    fn fmt(&self, f: &mut fmt::Formatter<'_>) -> fmt::Result {
        write!(
            f,
            "FILL={} SRV={} IGN={}",
            if self.fill { "ON" } else { "OFF" },
            if self.srv { "ON" } else { "OFF" },
            if self.ign { "PWR" } else { "SAFE" }
        )
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn wire_roundtrip() {
        let state = StateSnapshot {
            flags: flags::FILL_OPEN | flags::FILL_TIMER_ACTIVE | flags::ESTOP,
            faults: faults::CRC_ERROR | faults::LINK_TIMEOUT,
            fill_remaining_ms: 59_123,
            fill_duration_s: 60,
            last_cmd_id: 0xBEEF,
            uptime_ms: 123_456,
        };
        let mut buf = [0u8; 32];
        let n = state.encode(&mut buf).unwrap();
        assert_eq!(n, StateSnapshot::WIRE_LEN);
        assert_eq!(StateSnapshot::decode(&buf[..n]).unwrap(), state);
        assert!(state.fill_open());
        assert!(state.fill_timer_active());
        assert!(state.estop());
        assert!(!state.srv_on());
        assert!(!state.ign_pwr());
        assert!(!state.outputs_safe());
    }

    #[test]
    fn short_buffer_rejected() {
        assert!(StateSnapshot::decode(&[0u8; 15]).is_none());
        let mut out = [0u8; 8];
        assert!(StateSnapshot::all_safe().encode(&mut out).is_none());
    }

    #[test]
    fn all_safe() {
        let state = StateSnapshot::all_safe();
        assert!(state.outputs_safe());
        assert!(!state.fill_open());
        assert!(!state.fill_paused());
        assert!(!state.fill_ready());
        assert!(state.has(flags::BOOT_SAFE));
        assert_eq!(state.summary().to_string(), "FILL=OFF SRV=OFF IGN=SAFE");
    }

    #[test]
    fn fill_paused_preserves_remaining_ms_and_marks_fill_ready() {
        let paused = StateSnapshot {
            flags: flags::FILL_PAUSED,
            faults: 0,
            fill_remaining_ms: 3_450,
            fill_duration_s: 10,
            last_cmd_id: 7,
            uptime_ms: 9_000,
        };
        let mut buf = [0u8; 32];
        let n = paused.encode(&mut buf).unwrap();
        let decoded = StateSnapshot::decode(&buf[..n]).unwrap();
        assert_eq!(decoded, paused);
        assert!(decoded.outputs_safe());
        assert!(decoded.fill_paused());
        assert!(!decoded.fill_seq_complete());
        assert!(decoded.fill_ready());
        assert_eq!(decoded.fill_remaining_ms, 3_450);

        let aborted = StateSnapshot {
            flags: flags::FILL_PAUSED | flags::FILL_SEQ_ABORTED,
            ..paused
        };
        assert!(!aborted.fill_ready());
    }
}
