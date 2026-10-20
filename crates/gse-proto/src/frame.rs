//! Framing layer.
//!
//! ```text
//! 0x00 | COBS( MAGIC(2) | VER(1) | TYPE(1) | SRC(1) | DST(1) | SEQ(1) | LEN(1) | BODY(LEN) | CRC16(2) ) | 0x00
//! ```
//!
//! * CRC-16/CCITT-FALSE over everything from MAGIC to the last body byte.
//! * A leading delimiter is emitted so a receiver that just powered up (or
//!   resynchronised after noise) does not need any preamble.
//! * `SEQ` is a free-running per-sender frame counter (statistics/debug only).
//!   Command de-duplication uses the `cmd_id` field inside the body instead, so
//!   retries are safe even if a frame is repeated on the radio.

use core::fmt;

use crate::cobs;
use crate::control::{Endpoint, MsgType};
use crate::crc::crc16;
use crate::message::Message;
use crate::{MAGIC, MAX_BODY, PROTOCOL_VERSION};

pub const HEADER_LEN: usize = 8;
pub const CRC_LEN: usize = 2;
pub const MAX_WIRE_LEN: usize = HEADER_LEN + MAX_BODY + CRC_LEN;
pub const MAX_ENCODED_LEN: usize = cobs::max_encoded_len(MAX_WIRE_LEN) + 2;

#[derive(Clone, Copy, PartialEq, Eq)]
pub struct Frame {
    pub msg_type: MsgType,
    pub src: Endpoint,
    pub dst: Endpoint,
    pub seq: u8,
    body: [u8; MAX_BODY],
    body_len: u8,
}

impl Frame {
    pub fn new(
        msg_type: MsgType,
        src: Endpoint,
        dst: Endpoint,
        seq: u8,
        body: &[u8],
    ) -> Option<Self> {
        if body.len() > MAX_BODY {
            return None;
        }
        let mut frame = Frame {
            msg_type,
            src,
            dst,
            seq,
            body: [0u8; MAX_BODY],
            body_len: body.len() as u8,
        };
        frame.body[..body.len()].copy_from_slice(body);
        Some(frame)
    }

    pub fn from_message(message: &Message, src: Endpoint, dst: Endpoint, seq: u8) -> Option<Self> {
        let mut body = [0u8; MAX_BODY];
        let len = message.encode(&mut body)?;
        Self::new(message.msg_type(), src, dst, seq, &body[..len])
    }

    pub fn body(&self) -> &[u8] {
        &self.body[..self.body_len as usize]
    }

    pub fn body_len(&self) -> usize {
        self.body_len as usize
    }

    pub fn message(&self) -> Option<Message> {
        Message::decode(self.msg_type, self.body())
    }

    /// Worst-case encoded size, including the two delimiters.
    pub const MAX_ENCODED: usize = MAX_ENCODED_LEN;

    pub const fn max_encoded_len() -> usize {
        MAX_ENCODED_LEN
    }

    /// Serialise the frame, using at most `Frame::max_encoded_len()` bytes.
    pub fn encode(&self, out: &mut [u8]) -> Option<usize> {
        if out.len() < MAX_ENCODED_LEN {
            return None;
        }
        let mut raw = [0u8; MAX_WIRE_LEN];
        raw[0] = MAGIC[0];
        raw[1] = MAGIC[1];
        raw[2] = PROTOCOL_VERSION;
        raw[3] = self.msg_type.as_u8();
        raw[4] = self.src.as_u8();
        raw[5] = self.dst.as_u8();
        raw[6] = self.seq;
        raw[7] = self.body_len;
        raw[8..8 + self.body_len as usize].copy_from_slice(self.body());
        let crc = crc16(&raw[..8 + self.body_len as usize]);
        raw[8 + self.body_len as usize] = (crc & 0xFF) as u8;
        raw[9 + self.body_len as usize] = (crc >> 8) as u8;
        let raw_len = HEADER_LEN + self.body_len as usize + CRC_LEN;

        out[0] = 0;
        let encoded = cobs::encode(&raw[..raw_len], &mut out[1..])?;
        out[1 + encoded] = 0;
        Some(encoded + 2)
    }

    /// Parse a de-COBS'd payload.
    pub fn decode_payload(payload: &[u8]) -> Result<Self, DecodeError> {
        if payload.len() < HEADER_LEN + CRC_LEN {
            return Err(DecodeError::TooShort);
        }
        if payload[0] != MAGIC[0] || payload[1] != MAGIC[1] {
            return Err(DecodeError::BadMagic);
        }
        if payload[2] != PROTOCOL_VERSION {
            return Err(DecodeError::BadVersion(payload[2]));
        }
        let msg_type = MsgType::from_u8(payload[3]).ok_or(DecodeError::UnknownType(payload[3]))?;
        let src = Endpoint::from_u8(payload[4]).ok_or(DecodeError::BadEndpoint(payload[4]))?;
        let dst = Endpoint::from_u8(payload[5]).ok_or(DecodeError::BadEndpoint(payload[5]))?;
        let seq = payload[6];
        let len = payload[7] as usize;
        if len > MAX_BODY {
            return Err(DecodeError::BodyTooLong(len));
        }
        let expected = HEADER_LEN + len + CRC_LEN;
        if payload.len() < expected {
            return Err(DecodeError::TooShort);
        }
        let want = u16::from_le_bytes([payload[8 + len], payload[9 + len]]);
        let got = crc16(&payload[..8 + len]);
        if want != got {
            return Err(DecodeError::BadCrc { want, got });
        }
        Frame::new(msg_type, src, dst, seq, &payload[8..8 + len]).ok_or(DecodeError::TooShort)
    }
}

impl fmt::Debug for Frame {
    fn fmt(&self, f: &mut fmt::Formatter<'_>) -> fmt::Result {
        write!(
            f,
            "Frame {{ {} {}->{} seq={} len={} }}",
            self.msg_type, self.src, self.dst, self.seq, self.body_len
        )
    }
}

#[derive(Clone, Copy, Debug, PartialEq, Eq)]
pub enum DecodeError {
    TooShort,
    BadMagic,
    BadVersion(u8),
    UnknownType(u8),
    BadEndpoint(u8),
    BodyTooLong(usize),
    BadCrc { want: u16, got: u16 },
    BadCobs,
}

impl fmt::Display for DecodeError {
    fn fmt(&self, f: &mut fmt::Formatter<'_>) -> fmt::Result {
        match self {
            DecodeError::TooShort => f.write_str("frame too short"),
            DecodeError::BadMagic => f.write_str("bad magic"),
            DecodeError::BadVersion(v) => write!(f, "unsupported protocol version {v}"),
            DecodeError::UnknownType(t) => write!(f, "unknown message type 0x{t:02X}"),
            DecodeError::BadEndpoint(e) => write!(f, "unknown endpoint 0x{e:02X}"),
            DecodeError::BodyTooLong(n) => write!(f, "body too long ({n} bytes)"),
            DecodeError::BadCrc { want, got } => {
                write!(f, "CRC mismatch (want 0x{want:04X}, got 0x{got:04X})")
            }
            DecodeError::BadCobs => f.write_str("COBS decode error"),
        }
    }
}

/// Error counters, useful for the link-status panel and flight logs.
#[derive(Clone, Copy, Debug, Default, PartialEq, Eq)]
pub struct DecodeStats {
    pub frames_ok: u32,
    pub crc_errors: u32,
    pub malformed: u32,
    pub bytes_in: u32,
}

/// Streaming decoder: push bytes as they arrive from the serial port / radio.
#[derive(Debug)]
pub struct FrameDecoder {
    block: [u8; MAX_ENCODED_LEN],
    block_len: usize,
    stats: DecodeStats,
}

impl Default for FrameDecoder {
    fn default() -> Self {
        Self::new()
    }
}

impl FrameDecoder {
    pub fn new() -> Self {
        FrameDecoder {
            block: [0u8; MAX_ENCODED_LEN],
            block_len: 0,
            stats: DecodeStats::default(),
        }
    }

    pub fn stats(&self) -> DecodeStats {
        self.stats
    }

    pub fn reset_stats(&mut self) {
        self.stats = DecodeStats::default();
    }

    pub fn reset(&mut self) {
        self.block_len = 0;
    }

    /// Feed one byte. Returns a frame when a full, valid frame was received.
    ///
    /// Invalid frames are counted in [`DecodeStats`] and discarded; the decoder
    /// stays synchronised because it always re-aligns on the next `0x00`.
    pub fn push(&mut self, byte: u8) -> Option<Frame> {
        self.stats.bytes_in = self.stats.bytes_in.wrapping_add(1);
        if byte == 0 {
            if self.block_len == 0 {
                return None;
            }
            let len = self.block_len;
            self.block_len = 0;
            let mut decoded = [0u8; MAX_WIRE_LEN];
            let decoded_len = match cobs::decode(&self.block[..len], &mut decoded) {
                Some(n) => n,
                None => {
                    self.stats.malformed = self.stats.malformed.wrapping_add(1);
                    return None;
                }
            };
            match Frame::decode_payload(&decoded[..decoded_len]) {
                Ok(frame) => {
                    self.stats.frames_ok = self.stats.frames_ok.wrapping_add(1);
                    Some(frame)
                }
                Err(DecodeError::BadCrc { .. }) => {
                    self.stats.crc_errors = self.stats.crc_errors.wrapping_add(1);
                    None
                }
                Err(_) => {
                    self.stats.malformed = self.stats.malformed.wrapping_add(1);
                    None
                }
            }
        } else if self.block_len < self.block.len() {
            self.block[self.block_len] = byte;
            self.block_len += 1;
            None
        } else {
            // Oversized block: drop it and wait for the next delimiter.
            self.block_len = 0;
            self.stats.malformed = self.stats.malformed.wrapping_add(1);
            None
        }
    }
}

#[cfg(test)]
mod tests {
    use super::*;
    use crate::control::Control;
    use crate::message::capabilities;
    use crate::message::{Ack, Command, Hello, Ping};
    use crate::state::{flags, StateSnapshot};

    fn sample_frame() -> Frame {
        let cmd = Command {
            cmd_id: 42,
            control: Control::Fill,
            action: 1,
            arg_s: 60,
            flags: 0,
        };
        Frame::from_message(&Message::Command(cmd), Endpoint::Host, Endpoint::Vehicle, 7).unwrap()
    }

    #[test]
    fn encode_decode_roundtrip() {
        let frame = sample_frame();
        let mut buf = [0u8; MAX_ENCODED_LEN];
        let n = frame.encode(&mut buf).unwrap();
        assert!(n <= MAX_ENCODED_LEN);
        assert_eq!(buf[0], 0);
        assert_eq!(buf[n - 1], 0);
        let mut dec = FrameDecoder::new();
        let mut found = None;
        for &b in &buf[..n] {
            if let Some(f) = dec.push(b) {
                found = Some(f);
            }
        }
        assert_eq!(found, Some(frame));
        assert_eq!(dec.stats().frames_ok, 1);
    }

    #[test]
    fn message_roundtrip_through_wire() {
        let ack = Ack {
            cmd_id: 42,
            control: Control::Fill,
            action: 1,
            result: crate::control::AckResult::Ok,
            reason: crate::control::Reason::None,
            exec_ms: 310,
            state: StateSnapshot {
                flags: flags::FILL_OPEN | flags::FILL_TIMER_ACTIVE,
                faults: 0,
                fill_remaining_ms: 60_000,
                fill_duration_s: 60,
                last_cmd_id: 42,
                uptime_ms: 1000,
            },
        };
        let frame =
            Frame::from_message(&Message::Ack(ack), Endpoint::Vehicle, Endpoint::Host, 9).unwrap();
        let mut buf = [0u8; MAX_ENCODED_LEN];
        let n = frame.encode(&mut buf).unwrap();
        let mut dec = FrameDecoder::new();
        let mut found = None;
        for &b in &buf[..n] {
            found = dec.push(b).or(found);
        }
        assert_eq!(found.unwrap().message(), Some(Message::Ack(ack)));
    }

    #[test]
    fn rejects_bad_crc() {
        let frame = sample_frame();
        let mut buf = [0u8; MAX_ENCODED_LEN];
        let n = frame.encode(&mut buf).unwrap();
        // Flip a byte in the middle of the encoded block (still a valid COBS block).
        let idx = n / 2;
        buf[idx] ^= 0x01;
        let mut dec = FrameDecoder::new();
        let mut count = 0;
        for &b in &buf[..n] {
            if dec.push(b).is_some() {
                count += 1;
            }
        }
        assert_eq!(count, 0);
        assert_eq!(dec.stats().crc_errors + dec.stats().malformed, 1);
    }

    #[test]
    fn recovers_after_garbage() {
        let frame = sample_frame();
        let mut buf = [0u8; MAX_ENCODED_LEN];
        let n = frame.encode(&mut buf).unwrap();
        let mut dec = FrameDecoder::new();
        for &b in &[0xFF, 0x00, 0x12, 0x34, 0x00, 0xAA] {
            dec.push(b);
        }
        let mut found = None;
        for &b in &buf[..n] {
            found = dec.push(b).or(found);
        }
        assert_eq!(found.unwrap().msg_type, MsgType::Command);
    }

    #[test]
    fn consecutive_frames() {
        let msg = Message::Ping(Ping {
            nonce: 1,
            host_ms: 2,
        });
        let a = Frame::from_message(&msg, Endpoint::Host, Endpoint::Vehicle, 1).unwrap();
        let b = Frame::from_message(&msg, Endpoint::Host, Endpoint::Vehicle, 2).unwrap();
        let mut buf = [0u8; MAX_ENCODED_LEN * 2];
        let n1 = a.encode(&mut buf).unwrap();
        let n2 = b.encode(&mut buf[n1..]).unwrap();
        let mut dec = FrameDecoder::new();
        let mut frames = 0;
        for &byte in &buf[..n1 + n2] {
            if dec.push(byte).is_some() {
                frames += 1;
            }
        }
        assert_eq!(frames, 2);
    }

    #[test]
    fn hello_roundtrip_through_wire() {
        let hello = Hello {
            protocol: PROTOCOL_VERSION,
            fw: [1, 2, 3],
            role: Endpoint::Ground,
            uptime_ms: 1234,
            device_id: 0xAABBCCDD,
            capabilities: capabilities::RADIO,
        };
        let frame = Frame::from_message(
            &Message::Hello(hello),
            Endpoint::Ground,
            Endpoint::Broadcast,
            0,
        )
        .unwrap();
        let mut buf = [0u8; MAX_ENCODED_LEN];
        let n = frame.encode(&mut buf).unwrap();
        let mut dec = FrameDecoder::new();
        let mut out = None;
        for &b in &buf[..n] {
            out = dec.push(b).or(out);
        }
        assert_eq!(out.unwrap().message(), Some(Message::Hello(hello)));
    }

    #[test]
    fn oversize_body_rejected() {
        let body = [0u8; MAX_BODY + 1];
        assert!(Frame::new(MsgType::State, Endpoint::Vehicle, Endpoint::Host, 0, &body).is_none());
    }

    #[test]
    fn max_size_body_roundtrip() {
        let body = [0x5A; MAX_BODY];
        let frame = Frame::new(
            MsgType::State,
            Endpoint::Vehicle,
            Endpoint::Host,
            255,
            &body,
        )
        .unwrap();
        let mut buf = [0u8; MAX_ENCODED_LEN];
        let n = frame.encode(&mut buf).unwrap();
        let mut dec = FrameDecoder::new();
        let mut out = None;
        for &b in &buf[..n] {
            out = dec.push(b).or(out);
        }
        assert_eq!(out.unwrap().body(), &body);
    }

    #[test]
    fn bad_magic_is_malformed() {
        let mut payload = [0u8; 16];
        payload[0] = 0x00;
        assert_eq!(Frame::decode_payload(&payload), Err(DecodeError::BadMagic));
    }
}
