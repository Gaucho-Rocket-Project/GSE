//! `gse-proto` — the shared wire protocol for the GSE ground station.
//!
//! Three consumers use this crate:
//!
//! * the ground-station desktop app (`gse-app`), running on Windows/Linux/macOS,
//! * the ground NUCLEO-H753ZI firmware (USB bridge to the radio),
//! * the vehicle NUCLEO-H753ZI firmware (servo/igniter control).
//!
//! The crate is `no_std` and allocation-free so the identical code runs on the
//! STM32H753 and on the host (`cargo test` exercises it with the host std).
//!
//! See `docs/PROTOCOL.md` for the byte-level specification.

#![cfg_attr(not(test), no_std)]
#![forbid(unsafe_code)]

pub mod cobs;
pub mod control;
pub mod crc;
pub mod frame;
pub mod message;
pub mod state;

pub use control::{
    action_label, is_valid_action, AckResult, Control, Endpoint, EventCode, MsgType, Reason,
    ACTION_OFF, ACTION_ON, SYSTEM_ACTION_CLEAR_FAULTS, SYSTEM_ACTION_RESET_FILL,
    SYSTEM_ACTION_SAFE_ALL,
};
pub use control::cmd_flags;
pub use frame::{DecodeError, DecodeStats, Frame, FrameDecoder};
pub use message::{
    capabilities, Ack, Command, Event, FwVersion, Heartbeat, Hello, LinkStat, Message, Ping, Pong,
};
pub use state::{faults, flags, StateSnapshot, StateSummary};

/// Increment when the wire format changes in a way older nodes cannot parse.
pub const PROTOCOL_VERSION: u8 = 1;

/// Frame magic (`0xA5 0x5A`).
pub const MAGIC: [u8; 2] = [0xA5, 0x5A];

/// Maximum message body length carried by one frame.
pub const MAX_BODY: usize = 48;

/// Firmware version reported by both boards (keep in sync with
/// `firmware/stm32h753/Cargo.toml`).
pub const FIRMWARE_VERSION: (u8, u8, u8) = (1, 0, 0);

/// Default fill duration. The operator can change it in the app (and it is sent
/// with every FILL ON command, so the vehicle never needs to store settings).
pub const DEFAULT_FILL_SECONDS: u16 = 60;

/// Hard limits accepted by the firmware for the fill duration.
pub const MIN_FILL_SECONDS: u16 = 1;
pub const MAX_FILL_SECONDS: u16 = 600;

/// USB CDC / ST-LINK virtual COM port baud rate. Both ends use 8N1.
pub const USB_BAUD: u32 = 115_200;

/// Default UART baud rate between the NUCLEO and the SX1281 module.
pub const RADIO_BAUD: u32 = 9_600;

/// Encode a message into a ready-to-write wire frame.
///
/// Returns the number of bytes written to `out` (including the two `0x00`
/// delimiters), or `None` if `out` is too small.
pub fn encode_message(
    message: &Message,
    src: Endpoint,
    dst: Endpoint,
    seq: u8,
    out: &mut [u8],
) -> Option<usize> {
    let frame = Frame::from_message(message, src, dst, seq)?;
    frame.encode(out)
}

/// Decode a single complete COBS block (without delimiters) into a frame.
pub fn decode_block(block: &[u8]) -> Result<Frame, DecodeError> {
    let mut decoded = [0u8; frame::MAX_WIRE_LEN];
    let n = cobs::decode(block, &mut decoded).ok_or(DecodeError::BadCobs)?;
    Frame::decode_payload(&decoded[..n])
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn encode_message_helper() {
        let msg = Message::State(StateSnapshot::all_safe());
        let mut buf = [0u8; 96];
        let n = encode_message(&msg, Endpoint::Vehicle, Endpoint::Host, 1, &mut buf).unwrap();
        assert_eq!(buf[0], 0);
        assert_eq!(buf[n - 1], 0);
        let block = &buf[1..n - 1];
        let frame = decode_block(block).unwrap();
        assert_eq!(frame.message(), Some(msg));
    }

    #[test]
    fn tiny_buffer_returns_none() {
        let msg = Message::State(StateSnapshot::all_safe());
        let mut buf = [0u8; 8];
        assert!(encode_message(&msg, Endpoint::Vehicle, Endpoint::Host, 1, &mut buf).is_none());
    }

    #[test]
    fn defaults_are_sane() {
        assert!(MIN_FILL_SECONDS <= DEFAULT_FILL_SECONDS);
        assert!(DEFAULT_FILL_SECONDS <= MAX_FILL_SECONDS);
        assert_eq!(USB_BAUD, 115_200);
    }
}
