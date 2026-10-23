//! Fixed-size message bodies.
//!
//! Every message has a fixed layout (little endian) so the flight firmware can
//! encode/decode without an allocator. Decoders accept trailing bytes so a
//! future protocol revision can extend a body without breaking older nodes;
//! the protocol version in the frame header gates incompatible changes.

use crate::control::{AckResult, Control, Endpoint, EventCode, MsgType, Reason};
use crate::radio::{DxLr42Baud, RadioModule, UsbBaud};
use crate::state::StateSnapshot;

/// Capability bits advertised in `HELLO`.
pub mod capabilities {
    pub const RADIO: u16 = 1 << 0;
    pub const SERVOS: u16 = 1 << 1;
    pub const IGNITER: u16 = 1 << 2;
    pub const WATCHDOG: u16 = 1 << 3;
    pub const FILL_TIMER: u16 = 1 << 4;
    pub const USB_BRIDGE: u16 = 1 << 5;
    /// Runtime USB/UART baud + radio module reconfiguration & flash storage supported.
    pub const LINK_CONFIG: u16 = 1 << 6;
    /// CDEBYTE `E28-2G4M27S` (`SPI1` + PA/LNA `+27 dBm`) driver compiled in.
    pub const RADIO_E28_2G4M27S: u16 = 1 << 7;
}

/// Bit flags carried in [`LinkConfig::flags`], [`LinkConfigAck::flags`], and [`LinkStat::link_flags`].
pub mod link_cfg_flags {
    /// Save (in `LinkConfig`) / saved (in `LinkConfigAck` & `LinkStat`) to non-volatile MCU flash.
    pub const PERSIST: u8 = 1 << 0;
    pub const FLASH_SAVED: u8 = PERSIST;
}

#[derive(Clone, Copy, Debug, PartialEq, Eq)]
pub struct Hello {
    pub protocol: u8,
    pub fw: [u8; 3],
    pub role: Endpoint,
    pub uptime_ms: u32,
    /// Stable per-board identifier (96-bit UID hashed, or a compile-time id).
    pub device_id: u32,
    pub capabilities: u16,
}

impl Hello {
    pub const WIRE_LEN: usize = 15;

    pub fn encode(&self, out: &mut [u8]) -> Option<usize> {
        if out.len() < Self::WIRE_LEN {
            return None;
        }
        out[0] = self.protocol;
        out[1] = self.fw[0];
        out[2] = self.fw[1];
        out[3] = self.fw[2];
        out[4] = self.role.as_u8();
        out[5..9].copy_from_slice(&self.uptime_ms.to_le_bytes());
        out[9..13].copy_from_slice(&self.device_id.to_le_bytes());
        out[13..15].copy_from_slice(&self.capabilities.to_le_bytes());
        Some(Self::WIRE_LEN)
    }

    pub fn decode(body: &[u8]) -> Option<Self> {
        if body.len() < Self::WIRE_LEN {
            return None;
        }
        Some(Hello {
            protocol: body[0],
            fw: [body[1], body[2], body[3]],
            role: Endpoint::from_u8(body[4])?,
            uptime_ms: u32::from_le_bytes([body[5], body[6], body[7], body[8]]),
            device_id: u32::from_le_bytes([body[9], body[10], body[11], body[12]]),
            capabilities: u16::from_le_bytes([body[13], body[14]]),
        })
    }

    pub fn fw_version(&self) -> FwVersion {
        FwVersion {
            major: self.fw[0],
            minor: self.fw[1],
            patch: self.fw[2],
        }
    }
}

#[derive(Clone, Copy, Debug, PartialEq, Eq)]
pub struct Ping {
    pub nonce: u32,
    pub host_ms: u32,
}

impl Ping {
    pub const WIRE_LEN: usize = 8;

    pub fn encode(&self, out: &mut [u8]) -> Option<usize> {
        if out.len() < Self::WIRE_LEN {
            return None;
        }
        out[0..4].copy_from_slice(&self.nonce.to_le_bytes());
        out[4..8].copy_from_slice(&self.host_ms.to_le_bytes());
        Some(Self::WIRE_LEN)
    }

    pub fn decode(body: &[u8]) -> Option<Self> {
        if body.len() < Self::WIRE_LEN {
            return None;
        }
        Some(Ping {
            nonce: u32::from_le_bytes([body[0], body[1], body[2], body[3]]),
            host_ms: u32::from_le_bytes([body[4], body[5], body[6], body[7]]),
        })
    }
}

#[derive(Clone, Copy, Debug, PartialEq, Eq)]
pub struct Pong {
    pub nonce: u32,
    pub uptime_ms: u32,
    pub state: StateSnapshot,
}

impl Pong {
    pub const WIRE_LEN: usize = 24;

    pub fn encode(&self, out: &mut [u8]) -> Option<usize> {
        if out.len() < Self::WIRE_LEN {
            return None;
        }
        out[0..4].copy_from_slice(&self.nonce.to_le_bytes());
        out[4..8].copy_from_slice(&self.uptime_ms.to_le_bytes());
        self.state.encode(&mut out[8..24])?;
        Some(Self::WIRE_LEN)
    }

    pub fn decode(body: &[u8]) -> Option<Self> {
        if body.len() < Self::WIRE_LEN {
            return None;
        }
        Some(Pong {
            nonce: u32::from_le_bytes([body[0], body[1], body[2], body[3]]),
            uptime_ms: u32::from_le_bytes([body[4], body[5], body[6], body[7]]),
            state: StateSnapshot::decode(&body[8..24])?,
        })
    }
}

#[derive(Clone, Copy, Debug, PartialEq, Eq)]
pub struct Command {
    /// Rolling command id used for de-duplication across radio retries.
    pub cmd_id: u16,
    pub control: Control,
    pub action: u8,
    /// Argument in seconds (FILL ON: fill duration; ignored otherwise).
    pub arg_s: u16,
    pub flags: u8,
}

impl Command {
    pub const WIRE_LEN: usize = 7;

    pub fn encode(&self, out: &mut [u8]) -> Option<usize> {
        if out.len() < Self::WIRE_LEN {
            return None;
        }
        out[0..2].copy_from_slice(&self.cmd_id.to_le_bytes());
        out[2] = self.control.as_u8();
        out[3] = self.action;
        out[4..6].copy_from_slice(&self.arg_s.to_le_bytes());
        out[6] = self.flags;
        Some(Self::WIRE_LEN)
    }

    pub fn decode(body: &[u8]) -> Option<Self> {
        if body.len() < Self::WIRE_LEN {
            return None;
        }
        Some(Command {
            cmd_id: u16::from_le_bytes([body[0], body[1]]),
            control: Control::from_u8(body[2])?,
            action: body[3],
            arg_s: u16::from_le_bytes([body[4], body[5]]),
            flags: body[6],
        })
    }
}

#[derive(Clone, Copy, Debug, PartialEq, Eq)]
pub struct Ack {
    pub cmd_id: u16,
    pub control: Control,
    pub action: u8,
    pub result: AckResult,
    pub reason: Reason,
    pub exec_ms: u16,
    pub state: StateSnapshot,
}

impl Ack {
    pub const WIRE_LEN: usize = 24;

    pub fn encode(&self, out: &mut [u8]) -> Option<usize> {
        if out.len() < Self::WIRE_LEN {
            return None;
        }
        out[0..2].copy_from_slice(&self.cmd_id.to_le_bytes());
        out[2] = self.control.as_u8();
        out[3] = self.action;
        out[4] = self.result.as_u8();
        out[5] = self.reason.as_u8();
        out[6..8].copy_from_slice(&self.exec_ms.to_le_bytes());
        self.state.encode(&mut out[8..24])?;
        Some(Self::WIRE_LEN)
    }

    pub fn decode(body: &[u8]) -> Option<Self> {
        if body.len() < Self::WIRE_LEN {
            return None;
        }
        Some(Ack {
            cmd_id: u16::from_le_bytes([body[0], body[1]]),
            control: Control::from_u8(body[2])?,
            action: body[3],
            result: AckResult::from_u8(body[4])?,
            reason: Reason::from_u8(body[5])?,
            exec_ms: u16::from_le_bytes([body[6], body[7]]),
            state: StateSnapshot::decode(&body[8..24])?,
        })
    }
}

#[derive(Clone, Copy, Debug, PartialEq, Eq)]
pub struct Event {
    pub code: EventCode,
    /// Free-form detail: for `SafetyReject` this is the `Reason` value, for
    /// `FillAutoOff` the fill duration in seconds, etc.
    pub detail: u8,
    pub t_ms: u32,
    pub state: StateSnapshot,
}

impl Event {
    pub const WIRE_LEN: usize = 22;

    pub fn encode(&self, out: &mut [u8]) -> Option<usize> {
        if out.len() < Self::WIRE_LEN {
            return None;
        }
        out[0] = self.code.as_u8();
        out[1] = self.detail;
        out[2..6].copy_from_slice(&self.t_ms.to_le_bytes());
        self.state.encode(&mut out[6..22])?;
        Some(Self::WIRE_LEN)
    }

    pub fn decode(body: &[u8]) -> Option<Self> {
        if body.len() < Self::WIRE_LEN {
            return None;
        }
        Some(Event {
            code: EventCode::from_u8(body[0])?,
            detail: body[1],
            t_ms: u32::from_le_bytes([body[2], body[3], body[4], body[5]]),
            state: StateSnapshot::decode(&body[6..22])?,
        })
    }
}

/// Telemetry from the ground bridge about the radio and USB links it maintains.
#[derive(Clone, Copy, Debug, PartialEq, Eq, Default)]
pub struct LinkStat {
    pub frames_from_radio: u32,
    pub frames_to_radio: u32,
    pub crc_errors: u16,
    pub uart_errors: u16,
    pub bridge_uptime_ms: u32,
    pub last_radio_age_ms: u16,
    pub drops: u16,
    pub usb_baud: UsbBaud,
    pub uart_baud: DxLr42Baud,
    pub radio: RadioModule,
    pub link_flags: u8,
}

impl LinkStat {
    /// Legacy v1 minimum wire length (accepted when decoding older nodes).
    pub const MIN_WIRE_LEN: usize = 20;
    pub const WIRE_LEN: usize = 24;

    pub const fn flash_saved(&self) -> bool {
        self.link_flags & link_cfg_flags::PERSIST != 0
    }

    pub fn encode(&self, out: &mut [u8]) -> Option<usize> {
        if out.len() < Self::WIRE_LEN {
            return None;
        }
        out[0..4].copy_from_slice(&self.frames_from_radio.to_le_bytes());
        out[4..8].copy_from_slice(&self.frames_to_radio.to_le_bytes());
        out[8..10].copy_from_slice(&self.crc_errors.to_le_bytes());
        out[10..12].copy_from_slice(&self.uart_errors.to_le_bytes());
        out[12..16].copy_from_slice(&self.bridge_uptime_ms.to_le_bytes());
        out[16..18].copy_from_slice(&self.last_radio_age_ms.to_le_bytes());
        out[18..20].copy_from_slice(&self.drops.to_le_bytes());
        out[20] = self.usb_baud.code();
        out[21] = self.uart_baud.code();
        out[22] = self.radio.code();
        out[23] = self.link_flags;
        Some(Self::WIRE_LEN)
    }

    pub fn decode(body: &[u8]) -> Option<Self> {
        if body.len() < Self::MIN_WIRE_LEN {
            return None;
        }
        let (usb_baud, uart_baud, radio, link_flags) = if body.len() >= Self::WIRE_LEN {
            (
                UsbBaud::from_code(body[20]).unwrap_or_default(),
                DxLr42Baud::from_code(body[21]).unwrap_or_default(),
                RadioModule::from_code(body[22]).unwrap_or_default(),
                body[23],
            )
        } else {
            (
                UsbBaud::default(),
                DxLr42Baud::default(),
                RadioModule::default(),
                0,
            )
        };
        Some(LinkStat {
            frames_from_radio: u32::from_le_bytes([body[0], body[1], body[2], body[3]]),
            frames_to_radio: u32::from_le_bytes([body[4], body[5], body[6], body[7]]),
            crc_errors: u16::from_le_bytes([body[8], body[9]]),
            uart_errors: u16::from_le_bytes([body[10], body[11]]),
            bridge_uptime_ms: u32::from_le_bytes([body[12], body[13], body[14], body[15]]),
            last_radio_age_ms: u16::from_le_bytes([body[16], body[17]]),
            drops: u16::from_le_bytes([body[18], body[19]]),
            usb_baud,
            uart_baud,
            radio,
            link_flags,
        })
    }
}

/// Request from host to reconfigure a board's USB CDC baud rate, UART radio baud
/// rate, and active radio module (`DX-LR42-24T12D` vs `E28-2G4M27S`), optionally
/// persisting the configuration to non-volatile MCU flash across resets.
#[derive(Clone, Copy, Debug, Default, PartialEq, Eq)]
pub struct LinkConfig {
    pub req_id: u16,
    pub usb_baud: UsbBaud,
    pub uart_baud: DxLr42Baud,
    pub radio: RadioModule,
    pub flags: u8,
}

impl LinkConfig {
    pub const WIRE_LEN: usize = 6;

    pub const fn persist(&self) -> bool {
        self.flags & link_cfg_flags::PERSIST != 0
    }

    pub fn encode(&self, out: &mut [u8]) -> Option<usize> {
        if out.len() < Self::WIRE_LEN {
            return None;
        }
        out[0..2].copy_from_slice(&self.req_id.to_le_bytes());
        out[2] = self.usb_baud.code();
        out[3] = self.uart_baud.code();
        out[4] = self.radio.code();
        out[5] = self.flags;
        Some(Self::WIRE_LEN)
    }

    pub fn decode(body: &[u8]) -> Option<Self> {
        if body.len() < Self::WIRE_LEN {
            return None;
        }
        Some(LinkConfig {
            req_id: u16::from_le_bytes([body[0], body[1]]),
            usb_baud: UsbBaud::from_code(body[2])?,
            uart_baud: DxLr42Baud::from_code(body[3])?,
            radio: RadioModule::from_code(body[4])?,
            flags: body[5],
        })
    }
}

/// Acknowledgement emitted by Vehicle or Ground **before** switching hardware,
/// guaranteeing the host receives confirmation on the existing link first.
#[derive(Clone, Copy, Debug, PartialEq, Eq)]
pub struct LinkConfigAck {
    pub req_id: u16,
    pub result: AckResult,
    pub reason: Reason,
    pub usb_baud: UsbBaud,
    pub uart_baud: DxLr42Baud,
    pub radio: RadioModule,
    pub flags: u8,
}

impl LinkConfigAck {
    pub const WIRE_LEN: usize = 8;

    pub const fn persisted(&self) -> bool {
        self.flags & link_cfg_flags::PERSIST != 0
    }

    pub fn encode(&self, out: &mut [u8]) -> Option<usize> {
        if out.len() < Self::WIRE_LEN {
            return None;
        }
        out[0..2].copy_from_slice(&self.req_id.to_le_bytes());
        out[2] = self.result.as_u8();
        out[3] = self.reason.as_u8();
        out[4] = self.usb_baud.code();
        out[5] = self.uart_baud.code();
        out[6] = self.radio.code();
        out[7] = self.flags;
        Some(Self::WIRE_LEN)
    }

    pub fn decode(body: &[u8]) -> Option<Self> {
        if body.len() < Self::WIRE_LEN {
            return None;
        }
        Some(LinkConfigAck {
            req_id: u16::from_le_bytes([body[0], body[1]]),
            result: AckResult::from_u8(body[2])?,
            reason: Reason::from_u8(body[3])?,
            usb_baud: UsbBaud::from_code(body[4])?,
            uart_baud: DxLr42Baud::from_code(body[5])?,
            radio: RadioModule::from_code(body[6])?,
            flags: body[7],
        })
    }
}

#[derive(Clone, Copy, Debug, PartialEq, Eq)]
pub struct Heartbeat {
    pub uptime_ms: u32,
    pub mode: u8,
}

impl Heartbeat {
    pub const WIRE_LEN: usize = 5;

    pub fn encode(&self, out: &mut [u8]) -> Option<usize> {
        if out.len() < Self::WIRE_LEN {
            return None;
        }
        out[0..4].copy_from_slice(&self.uptime_ms.to_le_bytes());
        out[4] = self.mode;
        Some(Self::WIRE_LEN)
    }

    pub fn decode(body: &[u8]) -> Option<Self> {
        if body.len() < Self::WIRE_LEN {
            return None;
        }
        Some(Heartbeat {
            uptime_ms: u32::from_le_bytes([body[0], body[1], body[2], body[3]]),
            mode: body[4],
        })
    }
}

#[derive(Clone, Copy, Debug, PartialEq, Eq)]
pub struct FwVersion {
    pub major: u8,
    pub minor: u8,
    pub patch: u8,
}

impl core::fmt::Display for FwVersion {
    fn fmt(&self, f: &mut core::fmt::Formatter<'_>) -> core::fmt::Result {
        write!(f, "{}.{}.{}", self.major, self.minor, self.patch)
    }
}

#[derive(Clone, Copy, Debug, PartialEq, Eq)]
pub enum Message {
    Hello(Hello),
    Ping(Ping),
    Pong(Pong),
    Command(Command),
    Ack(Ack),
    State(StateSnapshot),
    Event(Event),
    LinkStat(LinkStat),
    Heartbeat(Heartbeat),
    LinkConfig(LinkConfig),
    LinkConfigAck(LinkConfigAck),
}

impl Message {
    pub fn msg_type(&self) -> MsgType {
        match self {
            Message::Hello(_) => MsgType::Hello,
            Message::Ping(_) => MsgType::Ping,
            Message::Pong(_) => MsgType::Pong,
            Message::Command(_) => MsgType::Command,
            Message::Ack(_) => MsgType::Ack,
            Message::State(_) => MsgType::State,
            Message::Event(_) => MsgType::Event,
            Message::LinkStat(_) => MsgType::LinkStat,
            Message::Heartbeat(_) => MsgType::Heartbeat,
            Message::LinkConfig(_) => MsgType::LinkConfig,
            Message::LinkConfigAck(_) => MsgType::LinkConfigAck,
        }
    }

    pub fn body_len(&self) -> usize {
        match self {
            Message::Hello(_) => Hello::WIRE_LEN,
            Message::Ping(_) => Ping::WIRE_LEN,
            Message::Pong(_) => Pong::WIRE_LEN,
            Message::Command(_) => Command::WIRE_LEN,
            Message::Ack(_) => Ack::WIRE_LEN,
            Message::State(_) => StateSnapshot::WIRE_LEN,
            Message::Event(_) => Event::WIRE_LEN,
            Message::LinkStat(_) => LinkStat::WIRE_LEN,
            Message::Heartbeat(_) => Heartbeat::WIRE_LEN,
            Message::LinkConfig(_) => LinkConfig::WIRE_LEN,
            Message::LinkConfigAck(_) => LinkConfigAck::WIRE_LEN,
        }
    }

    pub fn encode(&self, out: &mut [u8]) -> Option<usize> {
        match self {
            Message::Hello(m) => m.encode(out),
            Message::Ping(m) => m.encode(out),
            Message::Pong(m) => m.encode(out),
            Message::Command(m) => m.encode(out),
            Message::Ack(m) => m.encode(out),
            Message::State(m) => m.encode(out),
            Message::Event(m) => m.encode(out),
            Message::LinkStat(m) => m.encode(out),
            Message::Heartbeat(m) => m.encode(out),
            Message::LinkConfig(m) => m.encode(out),
            Message::LinkConfigAck(m) => m.encode(out),
        }
    }

    pub fn decode(msg_type: MsgType, body: &[u8]) -> Option<Message> {
        match msg_type {
            MsgType::Hello => Some(Message::Hello(Hello::decode(body)?)),
            MsgType::Ping => Some(Message::Ping(Ping::decode(body)?)),
            MsgType::Pong => Some(Message::Pong(Pong::decode(body)?)),
            MsgType::Command => Some(Message::Command(Command::decode(body)?)),
            MsgType::Ack => Some(Message::Ack(Ack::decode(body)?)),
            MsgType::State => Some(Message::State(StateSnapshot::decode(body)?)),
            MsgType::Event => Some(Message::Event(Event::decode(body)?)),
            MsgType::LinkStat => Some(Message::LinkStat(LinkStat::decode(body)?)),
            MsgType::Heartbeat => Some(Message::Heartbeat(Heartbeat::decode(body)?)),
            MsgType::LinkConfig => Some(Message::LinkConfig(LinkConfig::decode(body)?)),
            MsgType::LinkConfigAck => Some(Message::LinkConfigAck(LinkConfigAck::decode(body)?)),
        }
    }

    /// Compact one-line description for logs.
    pub fn describe(&self) -> MessageDescription {
        match self {
            Message::Hello(h) => MessageDescription::Hello {
                role: h.role,
                fw: h.fw_version(),
                device_id: h.device_id,
            },
            Message::Ping(p) => MessageDescription::Ping { nonce: p.nonce },
            Message::Pong(p) => MessageDescription::Pong {
                nonce: p.nonce,
                state: p.state.summary(),
                uptime_ms: p.uptime_ms,
            },
            Message::Command(c) => MessageDescription::Command {
                cmd_id: c.cmd_id,
                control: c.control,
                action: c.action,
                arg_s: c.arg_s,
            },
            Message::Ack(a) => MessageDescription::Ack {
                cmd_id: a.cmd_id,
                control: a.control,
                action: a.action,
                result: a.result,
                reason: a.reason,
            },
            Message::State(s) => MessageDescription::State { state: s.summary() },
            Message::Event(e) => MessageDescription::Event {
                code: e.code,
                detail: e.detail,
            },
            Message::LinkStat(l) => MessageDescription::LinkStat {
                crc_errors: l.crc_errors,
                last_radio_age_ms: l.last_radio_age_ms,
                radio: l.radio,
            },
            Message::Heartbeat(h) => MessageDescription::Heartbeat {
                uptime_ms: h.uptime_ms,
            },
            Message::LinkConfig(c) => MessageDescription::LinkConfig {
                req_id: c.req_id,
                usb_baud: c.usb_baud,
                uart_baud: c.uart_baud,
                radio: c.radio,
            },
            Message::LinkConfigAck(a) => MessageDescription::LinkConfigAck {
                req_id: a.req_id,
                result: a.result,
                radio: a.radio,
            },
        }
    }
}

/// Borrow-free, formatted description used by the ground app.
#[derive(Clone, Copy, Debug, PartialEq, Eq)]
pub enum MessageDescription {
    Hello {
        role: Endpoint,
        fw: FwVersion,
        device_id: u32,
    },
    Ping {
        nonce: u32,
    },
    Pong {
        nonce: u32,
        state: crate::state::StateSummary,
        uptime_ms: u32,
    },
    Command {
        cmd_id: u16,
        control: Control,
        action: u8,
        arg_s: u16,
    },
    Ack {
        cmd_id: u16,
        control: Control,
        action: u8,
        result: AckResult,
        reason: Reason,
    },
    State {
        state: crate::state::StateSummary,
    },
    Event {
        code: EventCode,
        detail: u8,
    },
    LinkStat {
        crc_errors: u16,
        last_radio_age_ms: u16,
        radio: RadioModule,
    },
    Heartbeat {
        uptime_ms: u32,
    },
    LinkConfig {
        req_id: u16,
        usb_baud: UsbBaud,
        uart_baud: DxLr42Baud,
        radio: RadioModule,
    },
    LinkConfigAck {
        req_id: u16,
        result: AckResult,
        radio: RadioModule,
    },
}

#[cfg(test)]
mod tests {
    use super::*;
    use crate::state::flags;

    fn state(flags: u16) -> StateSnapshot {
        StateSnapshot {
            flags,
            faults: 3,
            fill_remaining_ms: 1234,
            fill_duration_s: 60,
            last_cmd_id: 77,
            uptime_ms: 9999,
        }
    }

    #[test]
    fn hello_roundtrip() {
        let msg = Hello {
            protocol: crate::PROTOCOL_VERSION,
            fw: [1, 2, 3],
            role: Endpoint::Ground,
            uptime_ms: 123_456,
            device_id: 0xDEADBEEF,
            capabilities: capabilities::RADIO | capabilities::USB_BRIDGE,
        };
        let mut buf = [0u8; 32];
        let n = msg.encode(&mut buf).unwrap();
        assert_eq!(n, Hello::WIRE_LEN);
        assert_eq!(Hello::decode(&buf[..n]).unwrap(), msg);
    }

    #[test]
    fn ping_pong_roundtrip() {
        let ping = Ping {
            nonce: 0xCAFEBABE,
            host_ms: 42,
        };
        let mut buf = [0u8; 32];
        let n = ping.encode(&mut buf).unwrap();
        assert_eq!(Ping::decode(&buf[..n]).unwrap(), ping);

        let pong = Pong {
            nonce: ping.nonce,
            uptime_ms: 777,
            state: state(flags::FILL_OPEN),
        };
        let n = pong.encode(&mut buf).unwrap();
        assert_eq!(Pong::decode(&buf[..n]).unwrap(), pong);
    }

    #[test]
    fn command_ack_roundtrip() {
        let cmd = Command {
            cmd_id: 513,
            control: Control::Fill,
            action: 1,
            arg_s: 60,
            flags: 0,
        };
        let mut buf = [0u8; 32];
        let n = cmd.encode(&mut buf).unwrap();
        assert_eq!(Command::decode(&buf[..n]).unwrap(), cmd);

        let ack = Ack {
            cmd_id: 513,
            control: Control::Fill,
            action: 1,
            result: AckResult::Ok,
            reason: Reason::None,
            exec_ms: 300,
            state: state(flags::FILL_OPEN | flags::FILL_TIMER_ACTIVE),
        };
        let n = ack.encode(&mut buf).unwrap();
        assert_eq!(Ack::decode(&buf[..n]).unwrap(), ack);
    }

    #[test]
    fn event_linkstat_heartbeat_roundtrip() {
        let mut buf = [0u8; 48];
        let ev = Event {
            code: EventCode::FillAutoOff,
            detail: 60,
            t_ms: 123,
            state: state(flags::FILL_SEQ_COMPLETE),
        };
        let n = ev.encode(&mut buf).unwrap();
        assert_eq!(Event::decode(&buf[..n]).unwrap(), ev);

        let ls = LinkStat {
            frames_from_radio: 1,
            frames_to_radio: 2,
            crc_errors: 3,
            uart_errors: 4,
            bridge_uptime_ms: 5,
            last_radio_age_ms: 6,
            drops: 7,
            usb_baud: UsbBaud::Baud230400,
            uart_baud: DxLr42Baud::Baud57600,
            radio: RadioModule::E28_2G4M27S,
            link_flags: link_cfg_flags::PERSIST,
        };
        let n = ls.encode(&mut buf).unwrap();
        assert_eq!(LinkStat::decode(&buf[..n]).unwrap(), ls);
        // Backwards-compatible 20-byte legacy LinkStat body still decodes with defaults.
        let legacy = LinkStat::decode(&buf[..LinkStat::MIN_WIRE_LEN]).unwrap();
        assert_eq!(legacy.usb_baud, UsbBaud::Baud115200);
        assert_eq!(legacy.uart_baud, DxLr42Baud::Baud9600);
        assert_eq!(legacy.radio, RadioModule::DxLr42);

        let hb = Heartbeat {
            uptime_ms: 4242,
            mode: 1,
        };
        let n = hb.encode(&mut buf).unwrap();
        assert_eq!(Heartbeat::decode(&buf[..n]).unwrap(), hb);

        let lc = LinkConfig {
            req_id: 91,
            usb_baud: UsbBaud::Baud230400,
            uart_baud: DxLr42Baud::Baud115200,
            radio: RadioModule::E28_2G4M27S,
            flags: link_cfg_flags::PERSIST,
        };
        let n = lc.encode(&mut buf).unwrap();
        assert_eq!(LinkConfig::decode(&buf[..n]).unwrap(), lc);
        assert!(lc.persist());

        let lca = LinkConfigAck {
            req_id: 91,
            result: AckResult::Ok,
            reason: Reason::None,
            usb_baud: UsbBaud::Baud230400,
            uart_baud: DxLr42Baud::Baud115200,
            radio: RadioModule::E28_2G4M27S,
            flags: link_cfg_flags::PERSIST,
        };
        let n = lca.encode(&mut buf).unwrap();
        assert_eq!(LinkConfigAck::decode(&buf[..n]).unwrap(), lca);
        assert!(lca.persisted());
    }

    #[test]
    fn short_bodies_rejected() {
        assert!(Hello::decode(&[0u8; 14]).is_none());
        assert!(Ping::decode(&[0u8; 7]).is_none());
        assert!(Pong::decode(&[0u8; 23]).is_none());
        assert!(Command::decode(&[0u8; 6]).is_none());
        assert!(Ack::decode(&[0u8; 23]).is_none());
        assert!(Event::decode(&[0u8; 21]).is_none());
        assert!(LinkStat::decode(&[0u8; 19]).is_none());
        assert!(Heartbeat::decode(&[0u8; 4]).is_none());
        assert!(StateSnapshot::decode(&[0u8; 15]).is_none());
    }

    #[test]
    fn unknown_enum_values_rejected() {
        let mut body = [0u8; 32];
        let cmd = Command {
            cmd_id: 1,
            control: Control::Fill,
            action: 1,
            arg_s: 1,
            flags: 0,
        };
        let n = cmd.encode(&mut body).unwrap();
        body[2] = 9; // invalid control id
        assert!(Command::decode(&body[..n]).is_none());

        let ack = Ack {
            cmd_id: 1,
            control: Control::Fill,
            action: 1,
            result: AckResult::Ok,
            reason: Reason::None,
            exec_ms: 0,
            state: StateSnapshot::all_safe(),
        };
        let n = ack.encode(&mut body).unwrap();
        body[4] = 42;
        assert!(Ack::decode(&body[..n]).is_none());
    }

    #[test]
    fn message_enum_roundtrip_all_variants() {
        let messages = [
            Message::Hello(Hello {
                protocol: 1,
                fw: [1, 0, 0],
                role: Endpoint::Vehicle,
                uptime_ms: 1,
                device_id: 2,
                capabilities: 3,
            }),
            Message::Ping(Ping {
                nonce: 4,
                host_ms: 5,
            }),
            Message::Pong(Pong {
                nonce: 4,
                uptime_ms: 5,
                state: StateSnapshot::all_safe(),
            }),
            Message::Command(Command {
                cmd_id: 6,
                control: Control::Srv,
                action: 1,
                arg_s: 0,
                flags: 0,
            }),
            Message::Ack(Ack {
                cmd_id: 6,
                control: Control::Srv,
                action: 1,
                result: AckResult::Ok,
                reason: Reason::None,
                exec_ms: 300,
                state: StateSnapshot::all_safe(),
            }),
            Message::State(StateSnapshot::all_safe()),
            Message::Event(Event {
                code: EventCode::Boot,
                detail: 0,
                t_ms: 1,
                state: StateSnapshot::all_safe(),
            }),
            Message::LinkStat(LinkStat::default()),
            Message::Heartbeat(Heartbeat {
                uptime_ms: 1,
                mode: 0,
            }),
            Message::LinkConfig(LinkConfig {
                req_id: 7,
                usb_baud: UsbBaud::Baud57600,
                uart_baud: DxLr42Baud::Baud38400,
                radio: RadioModule::E28_2G4M27S,
                flags: link_cfg_flags::PERSIST,
            }),
            Message::LinkConfigAck(LinkConfigAck {
                req_id: 7,
                result: AckResult::Ok,
                reason: Reason::None,
                usb_baud: UsbBaud::Baud57600,
                uart_baud: DxLr42Baud::Baud38400,
                radio: RadioModule::E28_2G4M27S,
                flags: link_cfg_flags::PERSIST,
            }),
        ];
        for msg in messages {
            let mut buf = [0u8; 64];
            let n = msg.encode(&mut buf).expect("encode");
            let decoded = Message::decode(msg.msg_type(), &buf[..n]).expect("decode");
            assert_eq!(decoded, msg);
        }
    }

    #[test]
    fn trailing_bytes_tolerated() {
        let hello = Hello {
            protocol: 1,
            fw: [1, 2, 3],
            role: Endpoint::Ground,
            uptime_ms: 1,
            device_id: 2,
            capabilities: 3,
        };
        let mut buf = [0u8; 20];
        let n = hello.encode(&mut buf).unwrap();
        buf[n] = 0xAA;
        assert_eq!(Hello::decode(&buf[..n + 1]).unwrap(), hello);
    }
}
