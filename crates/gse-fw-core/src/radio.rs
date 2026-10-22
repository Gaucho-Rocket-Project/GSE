//! Portable `DX-LR42-24T12D` (Semtech SX1281) UART stream adapter.
//!
//! Both NUCLEO-H753ZI boards talk to their local `DX-LR42-24T12D` module over a
//! 3.3 V UART in transparent mode (`AT+MODE0`, `AT+SLEEP2`, `9600 Bd 8N1`). On real
//! hardware the module's onboard MCU shares that single UART for two things:
//!
//! 1. Over-the-air transparent payload bytes (COBS frames delimited by `0x00`).
//! 2. Local `\r\n`-terminated status lines:
//!    * `"Power On\r\n"` on initial power-up or `AT+RESET`,
//!    * `"Entry AT\r\n"` / `"ERROR=104\r\n"` if the module was left in AT mode
//!      (for example after bench configuration or if `AT+SWITCH0` ever saw `"+++"`),
//!    * `"Exit AT\r\nPower On\r\n"` when leaving AT mode back to transparent mode.
//!
//! [`DxLr42Receiver`] sits directly ahead of [`FrameDecoder`] on both boards so:
//! * `"Power On\r\n"` boot banners never increment `uart_errors` or pretend a remote
//!   peer is transmitting,
//! * accidental AT command mode is detected automatically and healed without a power
//!   cycle by quiet-guarding the UART, forcing `M0=0, M1=0`, and sending `"+++"`,
//! * every real binary frame (valid, CRC-damaged, truncated, or oversized) reaches
//!   [`FrameDecoder`] byte-for-byte unchanged.

use gse_proto::radio::AT_TOGGLE_SEQ;
use gse_proto::{DecodeStats, Frame, FrameDecoder};

use crate::hal::deadline_passed;

/// Quiet guard (`ms`) enforced before sending `"+++"` to exit accidental AT mode.
pub const AT_PRE_GUARD_MS: u32 = 80;

/// Quiet settle window (`ms`) after `"+++"` while `"Exit AT\r\nPower On\r\n"` completes.
pub const AT_POST_SETTLE_MS: u32 = 120;

#[derive(Clone, Copy, Debug, PartialEq, Eq)]
enum Recovery {
    Idle,
    PreGuard { send_at_ms: u32 },
    PostSettle { resume_at_ms: u32 },
}

#[derive(Clone, Copy, Debug, Default, PartialEq, Eq)]
pub struct DxLr42Stats {
    /// Local `"Power On\r\n"` / `"Exit AT\r\n"` banners absorbed cleanly.
    pub power_on_banners: u16,
    /// Times local AT command mode was detected and exited automatically.
    pub at_recoveries: u16,
}

/// Non-blocking stream wrapper combining [`FrameDecoder`] with `DX-LR42-24T12D`
/// local banner filtering and automatic AT-mode recovery.
#[derive(Debug)]
pub struct DxLr42Receiver {
    decoder: FrameDecoder,
    pending: [u8; Frame::MAX_ENCODED],
    pending_len: usize,
    recovery: Recovery,
    radio_stats: DxLr42Stats,
}

impl Default for DxLr42Receiver {
    fn default() -> Self {
        Self::new()
    }
}

impl DxLr42Receiver {
    pub fn new() -> Self {
        DxLr42Receiver {
            decoder: FrameDecoder::new(),
            pending: [0u8; Frame::MAX_ENCODED],
            pending_len: 0,
            recovery: Recovery::Idle,
            radio_stats: DxLr42Stats::default(),
        }
    }

    pub fn stats(&self) -> DecodeStats {
        self.decoder.stats()
    }

    pub fn radio_stats(&self) -> DxLr42Stats {
        self.radio_stats
    }

    /// True while the UART must remain silent for an in-progress AT-mode exit.
    pub fn tx_inhibited(&self, now_ms: u32) -> bool {
        match self.recovery {
            Recovery::Idle => false,
            Recovery::PreGuard { .. } => true,
            Recovery::PostSettle { resume_at_ms } => !deadline_passed(now_ms, resume_at_ms),
        }
    }

    /// Advance the non-blocking AT recovery state machine.
    ///
    /// Returns `Some(b"+++")` at the exact moment the escape sequence should be
    /// written to the radio UART to return the module to transparent mode.
    pub fn poll_recovery_tx(&mut self, now_ms: u32) -> Option<&'static [u8]> {
        match self.recovery {
            Recovery::Idle => None,
            Recovery::PreGuard { send_at_ms } => {
                if deadline_passed(now_ms, send_at_ms) {
                    self.recovery = Recovery::PostSettle {
                        resume_at_ms: now_ms.wrapping_add(AT_POST_SETTLE_MS),
                    };
                    Some(AT_TOGGLE_SEQ)
                } else {
                    None
                }
            }
            Recovery::PostSettle { resume_at_ms } => {
                if deadline_passed(now_ms, resume_at_ms) {
                    self.recovery = Recovery::Idle;
                }
                None
            }
        }
    }

    /// Push one byte received from the local `DX-LR42-24T12D` UART.
    pub fn push(&mut self, byte: u8, now_ms: u32) -> Option<Frame> {
        if byte == 0 {
            self.flush_pending_to_decoder();
            return self.decoder.push(0);
        }

        if self.pending_len == self.pending.len() {
            // Pre-delimiter run fills `MAX_ENCODED`: not a short ASCII status line,
            // forward everything untouched so `FrameDecoder` records the oversize block.
            self.flush_pending_to_decoder();
            return self.decoder.push(byte);
        }

        self.pending[self.pending_len] = byte;
        self.pending_len += 1;

        if byte == b'\n' {
            if let Some(is_at_trap) = classify_local_line(&self.pending[..self.pending_len]) {
                self.pending_len = 0;
                if is_at_trap {
                    self.radio_stats.at_recoveries =
                        self.radio_stats.at_recoveries.saturating_add(1);
                    if matches!(self.recovery, Recovery::Idle) {
                        self.recovery = Recovery::PreGuard {
                            send_at_ms: now_ms.wrapping_add(AT_PRE_GUARD_MS),
                        };
                    }
                } else {
                    self.radio_stats.power_on_banners =
                        self.radio_stats.power_on_banners.saturating_add(1);
                }
            }
        }

        None
    }

    fn flush_pending_to_decoder(&mut self) {
        let len = self.pending_len;
        self.pending_len = 0;
        for i in 0..len {
            let _ = self.decoder.push(self.pending[i]);
        }
    }
}

/// Returns:
/// * `Some(false)` for benign local status lines (`Power On\r\n`, `Exit AT\r\n`, `OK\r\n`),
/// * `Some(true)` when the local module is stuck in AT command mode (`Entry AT\r\n`, `ERROR=10x\r\n`),
/// * `None` for anything else (ordinary binary frame bytes that happen to contain `0x0A`).
fn classify_local_line(buf: &[u8]) -> Option<bool> {
    let trimmed = trim_ascii_crlf(buf);
    if trimmed.is_empty() {
        return None;
    }
    if ends_with(trimmed, b"Power On") || ends_with(trimmed, b"Exit AT") || trimmed == b"OK" {
        return Some(false);
    }
    if ends_with(trimmed, b"Entry AT")
        || ends_with(trimmed, b"ERROR=104")
        || ends_with(trimmed, b"ERROR=105")
        || ends_with(trimmed, b"ERROR=106")
    {
        return Some(true);
    }
    None
}

fn trim_ascii_crlf(mut s: &[u8]) -> &[u8] {
    while let Some((&first, rest)) = s.split_first() {
        if first == b'\r' || first == b'\n' {
            s = rest;
        } else {
            break;
        }
    }
    while let Some((&last, rest)) = s.split_last() {
        if last == b'\r' || last == b'\n' {
            s = rest;
        } else {
            break;
        }
    }
    s
}

fn ends_with(haystack: &[u8], suffix: &[u8]) -> bool {
    haystack.len() >= suffix.len() && &haystack[haystack.len() - suffix.len()..] == suffix
}

#[cfg(test)]
mod tests {
    use super::*;
    use gse_proto::{encode_message, Endpoint, Message, StateSnapshot};

    #[test]
    fn absorbs_power_on_banner_without_uart_errors() {
        let mut rx = DxLr42Receiver::new();
        for &b in b"Power On\r\n" {
            assert!(rx.push(b, 0).is_none());
        }
        assert_eq!(rx.radio_stats().power_on_banners, 1);
        assert_eq!(rx.stats().malformed, 0);

        let msg = Message::State(StateSnapshot::all_safe());
        let mut wire = [0u8; Frame::MAX_ENCODED];
        let n = encode_message(&msg, Endpoint::Vehicle, Endpoint::Host, 3, &mut wire).unwrap();

        let mut decoded = None;
        for &b in &wire[..n] {
            if let Some(frame) = rx.push(b, 10) {
                decoded = Some(frame);
            }
        }
        assert_eq!(decoded.unwrap().message(), Some(msg));
        assert_eq!(rx.stats().frames_ok, 1);
        assert_eq!(rx.stats().malformed, 0);
        assert_eq!(rx.stats().crc_errors, 0);
    }

    #[test]
    fn recovers_automatically_from_accidental_at_mode() {
        let mut rx = DxLr42Receiver::new();
        for &b in b"ERROR=104\r\n" {
            assert!(rx.push(b, 100).is_none());
        }
        assert_eq!(rx.radio_stats().at_recoveries, 1);
        assert!(rx.tx_inhibited(150));
        assert_eq!(rx.poll_recovery_tx(150), None);

        // After quiet guard, emits bare "+++" to exit AT mode.
        assert_eq!(
            rx.poll_recovery_tx(100 + AT_PRE_GUARD_MS),
            Some(b"+++" as &[u8])
        );
        assert!(rx.tx_inhibited(100 + AT_PRE_GUARD_MS + 50));

        // Simulate module printing "Exit AT\r\nPower On\r\n" as it reboots to transparent mode.
        for &b in b"Exit AT\r\nPower On\r\n" {
            assert!(rx.push(b, 220).is_none());
        }
        assert_eq!(rx.radio_stats().power_on_banners, 2);

        // Once settle window completes, normal TX is unblocked cleanly.
        let done_ms = 100 + AT_PRE_GUARD_MS + AT_POST_SETTLE_MS;
        assert!(!rx.tx_inhibited(done_ms));
        assert_eq!(rx.poll_recovery_tx(done_ms), None);
        assert_eq!(rx.stats().malformed, 0);
    }

    #[test]
    fn preserves_exact_stats_on_corrupted_and_garbage_binary_frames() {
        let mut rx = DxLr42Receiver::new();
        let mut bare = FrameDecoder::new();

        let msg = Message::State(StateSnapshot::all_safe());
        let mut bad_crc = [0u8; Frame::MAX_ENCODED];
        let n = encode_message(&msg, Endpoint::Vehicle, Endpoint::Host, 1, &mut bad_crc).unwrap();
        bad_crc[n / 2] ^= 0x04;

        let stream = [
            &bad_crc[..n],
            &[0x00, 0x01, 0x02, 0x00],
            &[0xAA; 80],
            &[0x00],
        ]
        .concat();

        for &b in &stream {
            assert_eq!(rx.push(b, 0), bare.push(b));
        }
        assert_eq!(rx.stats(), bare.stats());
    }
}
