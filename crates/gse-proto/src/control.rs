//! Control identifiers, actions, results and event codes used on the wire.
//!
//! All enums are `#[repr(u8)]` and have total conversions from `u8` so a
//! corrupted-but-CRC-valid frame can be rejected cleanly rather than causing a
//! panic (important: the flight firmware has no unwinding).

use core::fmt;

/// Which node a frame is addressed to or originates from.
#[repr(u8)]
#[derive(Clone, Copy, Debug, PartialEq, Eq, Hash)]
pub enum Endpoint {
    Host = 0x01,
    Ground = 0x10,
    Vehicle = 0x20,
    Broadcast = 0xFF,
}

impl Endpoint {
    pub const fn as_u8(self) -> u8 {
        self as u8
    }

    pub const fn from_u8(value: u8) -> Option<Self> {
        match value {
            0x01 => Some(Endpoint::Host),
            0x10 => Some(Endpoint::Ground),
            0x20 => Some(Endpoint::Vehicle),
            0xFF => Some(Endpoint::Broadcast),
            _ => None,
        }
    }

    pub const fn label(self) -> &'static str {
        match self {
            Endpoint::Host => "HOST",
            Endpoint::Ground => "GROUND",
            Endpoint::Vehicle => "VEHICLE",
            Endpoint::Broadcast => "BROADCAST",
        }
    }

    /// True when this endpoint is a physical board (not the PC, not broadcast).
    pub const fn is_board(self) -> bool {
        matches!(self, Endpoint::Ground | Endpoint::Vehicle)
    }
}

impl fmt::Display for Endpoint {
    fn fmt(&self, f: &mut fmt::Formatter<'_>) -> fmt::Result {
        f.write_str(self.label())
    }
}

/// Message type field (byte 3 of the frame header).
#[repr(u8)]
#[derive(Clone, Copy, Debug, PartialEq, Eq, Hash)]
pub enum MsgType {
    Hello = 0x01,
    Ping = 0x02,
    Pong = 0x03,
    Command = 0x04,
    Ack = 0x05,
    State = 0x06,
    Event = 0x07,
    LinkStat = 0x08,
    Heartbeat = 0x09,
    LinkConfig = 0x0A,
    LinkConfigAck = 0x0B,
}

impl MsgType {
    pub const fn as_u8(self) -> u8 {
        self as u8
    }

    pub const fn from_u8(value: u8) -> Option<Self> {
        match value {
            0x01 => Some(MsgType::Hello),
            0x02 => Some(MsgType::Ping),
            0x03 => Some(MsgType::Pong),
            0x04 => Some(MsgType::Command),
            0x05 => Some(MsgType::Ack),
            0x06 => Some(MsgType::State),
            0x07 => Some(MsgType::Event),
            0x08 => Some(MsgType::LinkStat),
            0x09 => Some(MsgType::Heartbeat),
            0x0A => Some(MsgType::LinkConfig),
            0x0B => Some(MsgType::LinkConfigAck),
            _ => None,
        }
    }

    pub const fn label(self) -> &'static str {
        match self {
            MsgType::Hello => "HELLO",
            MsgType::Ping => "PING",
            MsgType::Pong => "PONG",
            MsgType::Command => "CMD",
            MsgType::Ack => "ACK",
            MsgType::State => "STATE",
            MsgType::Event => "EVENT",
            MsgType::LinkStat => "LINKSTAT",
            MsgType::Heartbeat => "HEARTBEAT",
            MsgType::LinkConfig => "LINKCFG",
            MsgType::LinkConfigAck => "LINKCFG_ACK",
        }
    }
}

impl fmt::Display for MsgType {
    fn fmt(&self, f: &mut fmt::Formatter<'_>) -> fmt::Result {
        f.write_str(self.label())
    }
}

/// The vehicle controls (three individual channels, the combined live-launch
/// `SRV+IGN` pair, plus a system pseudo-control).
#[repr(u8)]
#[derive(Clone, Copy, Debug, PartialEq, Eq, Hash)]
pub enum Control {
    System = 0,
    Fill = 1,
    Srv = 2,
    Ign = 3,
    SrvIgn = 4,
}

impl Control {
    pub const fn as_u8(self) -> u8 {
        self as u8
    }

    pub const fn from_u8(value: u8) -> Option<Self> {
        match value {
            0 => Some(Control::System),
            1 => Some(Control::Fill),
            2 => Some(Control::Srv),
            3 => Some(Control::Ign),
            4 => Some(Control::SrvIgn),
            _ => None,
        }
    }

    pub const fn label(self) -> &'static str {
        match self {
            Control::System => "SYSTEM",
            Control::Fill => "FILL",
            Control::Srv => "SRV",
            Control::Ign => "IGN",
            Control::SrvIgn => "SRV+IGN",
        }
    }
}

impl fmt::Display for Control {
    fn fmt(&self, f: &mut fmt::Formatter<'_>) -> fmt::Result {
        f.write_str(self.label())
    }
}

/// Action byte inside a `CMD` frame.
///
/// For `FILL` / `SRV` the value is OFF (0) / ON (1); for `IGN` it is
/// SAFE (0) / PWR (1). `SYSTEM` actions use the `system_action` constants.
pub const ACTION_OFF: u8 = 0;
pub const ACTION_ON: u8 = 1;

pub const SYSTEM_ACTION_SAFE_ALL: u8 = 0;
pub const SYSTEM_ACTION_RESET_FILL: u8 = 1;
pub const SYSTEM_ACTION_CLEAR_FAULTS: u8 = 2;

pub const fn action_label(control: Control, action: u8) -> &'static str {
    match control {
        Control::System => match action {
            SYSTEM_ACTION_SAFE_ALL => "SAFE ALL",
            SYSTEM_ACTION_RESET_FILL => "RESET FILL SEQ",
            SYSTEM_ACTION_CLEAR_FAULTS => "CLEAR FAULTS",
            _ => "UNKNOWN SYSTEM ACTION",
        },
        Control::Ign => {
            if action == ACTION_ON {
                "PWR"
            } else {
                "SAFE"
            }
        }
        _ => {
            if action == ACTION_ON {
                "ON"
            } else {
                "OFF"
            }
        }
    }
}

pub const fn is_valid_action(control: Control, action: u8) -> bool {
    match control {
        Control::System => matches!(
            action,
            SYSTEM_ACTION_SAFE_ALL | SYSTEM_ACTION_RESET_FILL | SYSTEM_ACTION_CLEAR_FAULTS
        ),
        _ => action <= 1,
    }
}

/// Command flags.
pub mod cmd_flags {
    /// Command was generated by the automatic fill timeout, not by the operator.
    pub const AUTO: u8 = 1 << 0;
    /// Command was generated by the emergency-stop path.
    pub const ESTOP: u8 = 1 << 1;
    /// Do not wait for the servo settle time before acknowledging.
    pub const IMMEDIATE: u8 = 1 << 2;
}

/// Acks are only ever emitted once the action has completed.
#[repr(u8)]
#[derive(Clone, Copy, Debug, PartialEq, Eq)]
pub enum AckResult {
    Ok = 0,
    RejectedSafety = 1,
    RejectedState = 2,
    Busy = 3,
    UnknownControl = 4,
    UnknownAction = 5,
    BadArgument = 6,
    HardwareFault = 7,
}

impl AckResult {
    pub const fn from_u8(value: u8) -> Option<Self> {
        match value {
            0 => Some(AckResult::Ok),
            1 => Some(AckResult::RejectedSafety),
            2 => Some(AckResult::RejectedState),
            3 => Some(AckResult::Busy),
            4 => Some(AckResult::UnknownControl),
            5 => Some(AckResult::UnknownAction),
            6 => Some(AckResult::BadArgument),
            7 => Some(AckResult::HardwareFault),
            _ => None,
        }
    }

    pub const fn as_u8(self) -> u8 {
        self as u8
    }

    pub const fn is_ok(self) -> bool {
        matches!(self, AckResult::Ok)
    }

    pub const fn label(self) -> &'static str {
        match self {
            AckResult::Ok => "OK",
            AckResult::RejectedSafety => "REJECTED (SAFETY)",
            AckResult::RejectedState => "REJECTED (STATE)",
            AckResult::Busy => "BUSY",
            AckResult::UnknownControl => "UNKNOWN CONTROL",
            AckResult::UnknownAction => "UNKNOWN ACTION",
            AckResult::BadArgument => "BAD ARGUMENT",
            AckResult::HardwareFault => "HARDWARE FAULT",
        }
    }
}

impl fmt::Display for AckResult {
    fn fmt(&self, f: &mut fmt::Formatter<'_>) -> fmt::Result {
        f.write_str(self.label())
    }
}

/// Machine-readable reason attached to an ack. The app turns these into
/// operator-facing text, but they are also included in the flight log.
#[repr(u8)]
#[derive(Clone, Copy, Debug, PartialEq, Eq)]
pub enum Reason {
    None = 0,
    FillSrvExclusive = 1,
    FillActive = 2,
    SrvRequired = 3,
    FillSeqIncomplete = 4,
    IgnPowered = 5,
    IgnRequiredSafe = 6,
    LinkLost = 7,
    AlreadyInState = 8,
    NotActive = 9,
    Duplicate = 10,
    HardwareFault = 11,
    InvalidFillTime = 12,
    Busy = 13,
    NotPermitted = 14,
    BadFrame = 15,
}

impl Reason {
    pub const fn from_u8(value: u8) -> Option<Self> {
        match value {
            0 => Some(Reason::None),
            1 => Some(Reason::FillSrvExclusive),
            2 => Some(Reason::FillActive),
            3 => Some(Reason::SrvRequired),
            4 => Some(Reason::FillSeqIncomplete),
            5 => Some(Reason::IgnPowered),
            6 => Some(Reason::IgnRequiredSafe),
            7 => Some(Reason::LinkLost),
            8 => Some(Reason::AlreadyInState),
            9 => Some(Reason::NotActive),
            10 => Some(Reason::Duplicate),
            11 => Some(Reason::HardwareFault),
            12 => Some(Reason::InvalidFillTime),
            13 => Some(Reason::Busy),
            14 => Some(Reason::NotPermitted),
            15 => Some(Reason::BadFrame),
            _ => None,
        }
    }

    pub const fn as_u8(self) -> u8 {
        self as u8
    }

    pub const fn label(self) -> &'static str {
        match self {
            Reason::None => "",
            Reason::FillSrvExclusive => {
                "FILL and SRV are interlocked: both cannot be on at the same time"
            }
            Reason::FillActive => "SRV cannot be enabled while FILL is open",
            Reason::SrvRequired => "IGN PWR requires SRV ON",
            Reason::FillSeqIncomplete => "IGN PWR requires a completed FILL sequence",
            Reason::IgnPowered => "IGN is in PWR; SAFE the igniter first",
            Reason::IgnRequiredSafe => "IGN must be SAFE for this action",
            Reason::LinkLost => "Vehicle link lost - outputs forced safe",
            Reason::AlreadyInState => "Already in the requested state",
            Reason::NotActive => "Control is not active",
            Reason::Duplicate => "Duplicate command (already acknowledged)",
            Reason::HardwareFault => "Vehicle reported a hardware fault",
            Reason::InvalidFillTime => "Fill duration out of range",
            Reason::Busy => "Another command is still being executed",
            Reason::NotPermitted => "Action not permitted in the current state",
            Reason::BadFrame => "Malformed command frame",
        }
    }
}

impl fmt::Display for Reason {
    fn fmt(&self, f: &mut fmt::Formatter<'_>) -> fmt::Result {
        f.write_str(self.label())
    }
}

/// Unsolicited event emitted by the vehicle (or bridge).
#[repr(u8)]
#[derive(Clone, Copy, Debug, PartialEq, Eq)]
pub enum EventCode {
    Boot = 1,
    FillAutoOff = 2,
    FillAborted = 3,
    LinkTimeoutSafe = 4,
    SafetyReject = 5,
    FillOn = 6,
    FillOff = 7,
    SrvOn = 8,
    SrvOff = 9,
    IgnPwr = 10,
    IgnSafe = 11,
    Estop = 12,
    FillSeqReset = 13,
    WatchdogReset = 14,
    RadioFault = 15,
    GroundDetected = 16,
    GroundLost = 17,
    BridgeReady = 18,
    FaultsCleared = 19,
    LinkReconfigured = 20,
}

impl EventCode {
    pub const fn from_u8(value: u8) -> Option<Self> {
        match value {
            1 => Some(EventCode::Boot),
            2 => Some(EventCode::FillAutoOff),
            3 => Some(EventCode::FillAborted),
            4 => Some(EventCode::LinkTimeoutSafe),
            5 => Some(EventCode::SafetyReject),
            6 => Some(EventCode::FillOn),
            7 => Some(EventCode::FillOff),
            8 => Some(EventCode::SrvOn),
            9 => Some(EventCode::SrvOff),
            10 => Some(EventCode::IgnPwr),
            11 => Some(EventCode::IgnSafe),
            12 => Some(EventCode::Estop),
            13 => Some(EventCode::FillSeqReset),
            14 => Some(EventCode::WatchdogReset),
            15 => Some(EventCode::RadioFault),
            16 => Some(EventCode::GroundDetected),
            17 => Some(EventCode::GroundLost),
            18 => Some(EventCode::BridgeReady),
            19 => Some(EventCode::FaultsCleared),
            20 => Some(EventCode::LinkReconfigured),
            _ => None,
        }
    }

    pub const fn as_u8(self) -> u8 {
        self as u8
    }

    /// Stable snake_case code for logs and the JSON API.
    pub const fn code(self) -> &'static str {
        match self {
            EventCode::Boot => "boot",
            EventCode::FillAutoOff => "fill_auto_off",
            EventCode::FillAborted => "fill_aborted",
            EventCode::LinkTimeoutSafe => "link_timeout_safe",
            EventCode::SafetyReject => "safety_reject",
            EventCode::FillOn => "fill_on",
            EventCode::FillOff => "fill_off",
            EventCode::SrvOn => "srv_on",
            EventCode::SrvOff => "srv_off",
            EventCode::IgnPwr => "ign_pwr",
            EventCode::IgnSafe => "ign_safe",
            EventCode::Estop => "estop",
            EventCode::FillSeqReset => "fill_seq_reset",
            EventCode::WatchdogReset => "watchdog_reset",
            EventCode::RadioFault => "radio_fault",
            EventCode::GroundDetected => "ground_detected",
            EventCode::GroundLost => "ground_lost",
            EventCode::BridgeReady => "bridge_ready",
            EventCode::FaultsCleared => "faults_cleared",
            EventCode::LinkReconfigured => "link_reconfigured",
        }
    }

    pub const fn label(self) -> &'static str {
        match self {
            EventCode::Boot => "Vehicle booted",
            EventCode::FillAutoOff => "FILL auto-off (timer expired)",
            EventCode::FillAborted => "FILL cycle aborted",
            EventCode::LinkTimeoutSafe => "Link timeout: outputs forced safe",
            EventCode::SafetyReject => "Safety interlock rejected a command",
            EventCode::FillOn => "FILL opened",
            EventCode::FillOff => "FILL closed",
            EventCode::SrvOn => "SRV powered",
            EventCode::SrvOff => "SRV unpowered",
            EventCode::IgnPwr => "IGN PWR",
            EventCode::IgnSafe => "IGN SAFE",
            EventCode::Estop => "Emergency stop",
            EventCode::FillSeqReset => "Fill sequence reset",
            EventCode::WatchdogReset => "Watchdog reset detected",
            EventCode::RadioFault => "Radio link fault",
            EventCode::GroundDetected => "Ground board detected",
            EventCode::GroundLost => "Ground board lost",
            EventCode::BridgeReady => "Ground board ready",
            EventCode::FaultsCleared => "Faults cleared",
            EventCode::LinkReconfigured => "Link configuration applied",
        }
    }
}

impl fmt::Display for EventCode {
    fn fmt(&self, f: &mut fmt::Formatter<'_>) -> fmt::Result {
        f.write_str(self.label())
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn control_roundtrip() {
        for value in 0..=4u8 {
            let c = Control::from_u8(value).unwrap();
            assert_eq!(c.as_u8(), value);
        }
        assert!(Control::from_u8(5).is_none());
        assert_eq!(Control::SrvIgn.label(), "SRV+IGN");
    }

    #[test]
    fn endpoint_roundtrip() {
        for value in [1u8, 0x10, 0x20, 0xFF] {
            assert_eq!(Endpoint::from_u8(value).unwrap().as_u8(), value);
        }
        assert!(Endpoint::from_u8(2).is_none());
    }

    #[test]
    fn action_validation() {
        assert!(is_valid_action(Control::Fill, 0));
        assert!(is_valid_action(Control::Srv, 1));
        assert!(!is_valid_action(Control::Ign, 2));
        assert!(is_valid_action(Control::System, SYSTEM_ACTION_SAFE_ALL));
        assert!(is_valid_action(Control::System, SYSTEM_ACTION_RESET_FILL));
        assert!(is_valid_action(Control::System, SYSTEM_ACTION_CLEAR_FAULTS));
        assert!(!is_valid_action(Control::System, 9));
    }

    #[test]
    fn reason_labels_are_non_empty_except_none() {
        for value in 0..=15u8 {
            let reason = Reason::from_u8(value).unwrap();
            if reason == Reason::None {
                assert!(reason.label().is_empty());
            } else {
                assert!(!reason.label().is_empty());
            }
        }
    }
}
