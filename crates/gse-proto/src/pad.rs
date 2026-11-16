//! GSE-simple pad control set for the ESP32 + ESP-NOW boards.
//!
//! The ESP-NOW variant keeps the control algorithm of the GSE-simple pad node
//! (`GSE-simple/firmware/src/common/gse_protocol.h`) and carries its messages
//! inside the normal GSE frame (COBS + CRC-16, [`crate::frame`]):
//!
//! * the host streams the *complete desired output state* ([`PadCommand`]) at
//!   20 Hz instead of sending one command per change,
//! * the pad recomputes its outputs every 20 ms from the last command and its
//!   age ([`failsafe`]) and reports what it is actually driving ([`PadState`])
//!   at 10 Hz,
//! * the igniter needs [`ARM_CODE`] in every command and fires once per press
//!   for at most `ign_max_ds`,
//! * no valid command for the watchdog period -> FILL closed, PURGE closed,
//!   VEHICLE open (vented), IGN off ([`PadOutputs::SAFE`]).
//!
//! Payload layouts are byte-identical to GSE-simple's `gse_cmd_t`,
//! `gse_state_t`, `gse_cfgs_t` and `gse_event_t`, and the message type numbers
//! (`0x10..=0x13`) match its frame types. The only extension is the
//! [`state_flags::IGN_SPENT`] bit.
//!
//! "VEHICLE" is the fuel and oxidizer valve pair: both servos follow the one
//! `veh` command, each on its own pin with its own direction and trim.

/// Arm code that must be present in *every* command for the igniter to fire.
pub const ARM_CODE: u8 = 0xA5;

/// Host command stream period on ESP-NOW (20 Hz). See [`PadTiming`].
pub const CMD_PERIOD_MS: u32 = 50;
/// Pad telemetry period on ESP-NOW (10 Hz). See [`PadTiming`].
pub const STATE_PERIOD_MS: u32 = 100;

/// Command-stream and telemetry rates for one radio link.
///
/// ESP-NOW runs GSE-simple's 20 Hz / 10 Hz. The LoRa modules are half-duplex
/// and far slower: at `DX-LR42` LEVEL 3 (10 kbps) a 25-byte `PAD_CMD` is
/// ~26 ms of air time and a 37-byte `PAD_STATE` ~36 ms, so 20 Hz + 10 Hz
/// would occupy ~90 % of the channel. LoRa therefore uses 10 Hz commands and
/// 4 Hz telemetry (~41 % at LEVEL 3; the 1 s watchdog still spans 10 command
/// periods). LEVEL 1 and 2 are too slow for the pad stream at any rate.
#[derive(Clone, Copy, Debug, PartialEq, Eq)]
pub struct PadTiming {
    /// Host (and ground SAFE-stream) command period.
    pub cmd_period_ms: u32,
    /// Pad telemetry period.
    pub state_period_ms: u32,
}

impl PadTiming {
    /// ESP-NOW: GSE-simple's 20 Hz commands / 10 Hz telemetry.
    pub const ESPNOW: PadTiming = PadTiming {
        cmd_period_ms: CMD_PERIOD_MS,
        state_period_ms: STATE_PERIOD_MS,
    };

    /// DX-LR42 (LEVEL >= 3) and E28 LoRa: 10 Hz commands / 4 Hz telemetry.
    pub const LORA: PadTiming = PadTiming {
        cmd_period_ms: 100,
        state_period_ms: 250,
    };

    pub const fn for_radio(radio: crate::RadioModule) -> PadTiming {
        if radio.is_esp_now() {
            PadTiming::ESPNOW
        } else {
            PadTiming::LORA
        }
    }
}

// Every timing must leave the host's stale-telemetry window and the default
// watchdog several periods of margin.
const _: () = {
    assert!(PadTiming::LORA.state_period_ms * 4 <= STATE_STALE_MS);
    assert!(PadTiming::LORA.cmd_period_ms * 5 <= DEFAULT_WDG_DS as u32 * 100);
    assert!(PadTiming::LORA.cmd_period_ms < HOST_STALE_MS);
    assert!(PadTiming::LORA.cmd_period_ms * 2 <= HOST_SILENT_MS);
};
/// Pad output recompute / servo refresh period (50 Hz).
pub const OUTPUT_TICK_MS: u32 = 20;
/// Host side: pad telemetry older than this is "stale" (no blind arming).
pub const STATE_STALE_MS: u32 = 1_500;
/// Pad HELLO (GSE [`crate::Hello`]) period, for firmware/device identification.
pub const HELLO_PERIOD_MS: u32 = 5_000;

/// Ground box: a command arriving more than this after the previous host frame
/// is stale and dropped (GSE-simple `CMD_MAX_AGE_MS`).
pub const HOST_STALE_MS: u32 = 200;
/// Ground box: after this much host silence it streams SAFE itself
/// (GSE-simple `LINK_TIMEOUT_MS`).
pub const HOST_SILENT_MS: u32 = 300;

/// Link watchdog in deciseconds: default 1.0 s, range 0.3..=3.0 s.
pub const DEFAULT_WDG_DS: u8 = 10;
pub const MIN_WDG_DS: u8 = 3;
pub const MAX_WDG_DS: u8 = 30;

/// Igniter on-time limit in deciseconds: default 0.5 s, range 0.1..=2.0 s.
pub const DEFAULT_IGN_MAX_DS: u8 = 5;
pub const MIN_IGN_MAX_DS: u8 = 1;
pub const MAX_IGN_MAX_DS: u8 = 20;

/// Servo travel, degrees each side of neutral: default 45, range 5..=90.
pub const DEFAULT_TRAVEL_DEG: u8 = 45;
pub const MIN_TRAVEL_DEG: u8 = 5;
pub const MAX_TRAVEL_DEG: u8 = 90;

/// Servo pulse geometry (50 Hz): 1500 us neutral, 500 us per 45 deg, clamped.
pub const SERVO_NEUTRAL_US: u16 = 1_500;
pub const SERVO_US_PER_45DEG: u16 = 500;
pub const SERVO_MIN_US: u16 = 500;
pub const SERVO_MAX_US: u16 = 2_500;

/// `age_ms` value meaning "no valid command has ever arrived".
pub const AGE_NEVER: u16 = 0xFFFF;
/// `rssi_dbm` value meaning "not measured".
pub const RSSI_UNKNOWN: i8 = 0x7F;

/// [`PadState::flags`] bits.
pub mod state_flags {
    pub const LINK_OK: u8 = 0x01;
    /// No valid command yet (or fault latched).
    pub const LATCHED: u8 = 0x02;
    /// Arm interlock satisfied (arm code present and link fresh).
    pub const ARMED: u8 = 0x04;
    /// Igniter relay actually energised.
    pub const IGN_ON: u8 = 0x08;
    /// Watchdog expired: outputs forced SAFE.
    pub const COMMS_LOSS: u8 = 0x10;
    /// First command received after boot.
    pub const HELLO_OK: u8 = 0x20;
    /// GSE extension: the one-shot igniter pulse ended on the on-time limit and
    /// stays off until the request is released.
    pub const IGN_SPENT: u8 = 0x40;
}

/// [`PadState::ign_src`] bits.
pub mod ign_src {
    pub const CMD: u8 = 0x01;
    pub const WDG: u8 = 0x02;
}

/// [`PadCommand::flags`] bits.
pub mod cmd_flags {
    pub const REQ_ACK: u8 = 0x01;
}

/// Pad run state reported in [`PadState::pad_state`].
#[repr(u8)]
#[derive(Clone, Copy, Debug, PartialEq, Eq, Hash)]
pub enum PadRunState {
    Boot = 0,
    WaitHello = 1,
    Run = 2,
    Latched = 3,
}

impl PadRunState {
    pub const fn from_u8(value: u8) -> Option<Self> {
        match value {
            0 => Some(PadRunState::Boot),
            1 => Some(PadRunState::WaitHello),
            2 => Some(PadRunState::Run),
            3 => Some(PadRunState::Latched),
            _ => None,
        }
    }

    pub const fn label(self) -> &'static str {
        match self {
            PadRunState::Boot => "BOOT",
            PadRunState::WaitHello => "WAIT",
            PadRunState::Run => "RUN",
            PadRunState::Latched => "LATCHED",
        }
    }
}

/// Pad event codes ([`PadEvent::code`]).
#[repr(u8)]
#[derive(Clone, Copy, Debug, PartialEq, Eq, Hash)]
pub enum PadEventCode {
    Boot = 1,
    HelloOk = 2,
    CommsLoss = 3,
    Arm = 4,
    Disarm = 5,
    IgnOn = 6,
    /// `arg` = 1 when the on-time limit cut the relay, 0 on release.
    IgnOff = 7,
    /// `arg` = the echoed `cfg_seq`.
    CfgChanged = 8,
    BadCmd = 9,
}

impl PadEventCode {
    pub const fn from_u8(value: u8) -> Option<Self> {
        match value {
            1 => Some(PadEventCode::Boot),
            2 => Some(PadEventCode::HelloOk),
            3 => Some(PadEventCode::CommsLoss),
            4 => Some(PadEventCode::Arm),
            5 => Some(PadEventCode::Disarm),
            6 => Some(PadEventCode::IgnOn),
            7 => Some(PadEventCode::IgnOff),
            8 => Some(PadEventCode::CfgChanged),
            9 => Some(PadEventCode::BadCmd),
            _ => None,
        }
    }

    pub const fn label(self) -> &'static str {
        match self {
            PadEventCode::Boot => "BOOT",
            PadEventCode::HelloOk => "HELLO_OK",
            PadEventCode::CommsLoss => "COMMS_LOSS",
            PadEventCode::Arm => "ARM",
            PadEventCode::Disarm => "DISARM",
            PadEventCode::IgnOn => "IGN_ON",
            PadEventCode::IgnOff => "IGN_OFF",
            PadEventCode::CfgChanged => "CFG_CHANGED",
            PadEventCode::BadCmd => "BAD_CMD",
        }
    }
}

pub const fn clamp_wdg_ds(value: u8) -> u8 {
    clamp_u8(value, MIN_WDG_DS, MAX_WDG_DS)
}

pub const fn clamp_ign_max_ds(value: u8) -> u8 {
    clamp_u8(value, MIN_IGN_MAX_DS, MAX_IGN_MAX_DS)
}

pub const fn clamp_travel(value: u8) -> u8 {
    clamp_u8(value, MIN_TRAVEL_DEG, MAX_TRAVEL_DEG)
}

const fn clamp_u8(value: u8, lo: u8, hi: u8) -> u8 {
    if value < lo {
        lo
    } else if value > hi {
        hi
    } else {
        value
    }
}

/// Servo pulse for one channel (GSE-simple `servoUs`).
///
/// `open` is the logical valve state; `reversed` mirrors a servo mounted the
/// other way round. The travel is clamped to `5..=90` deg and the pulse to
/// `500..=2500` us.
pub const fn servo_pulse_us(open: bool, travel_deg: u8, reversed: bool, trim_us: i16) -> u16 {
    let deg = clamp_travel(travel_deg) as i32;
    let off = SERVO_US_PER_45DEG as i32 * deg / 45;
    let open = open != reversed;
    let mut us = SERVO_NEUTRAL_US as i32 + trim_us as i32 + if open { off } else { -off };
    if us < SERVO_MIN_US as i32 {
        us = SERVO_MIN_US as i32;
    }
    if us > SERVO_MAX_US as i32 {
        us = SERVO_MAX_US as i32;
    }
    us as u16
}

/// Host -> pad command (GSE-simple `gse_cmd_t`, 12 bytes), sent at 20 Hz.
#[derive(Clone, Copy, Debug, PartialEq, Eq)]
pub struct PadCommand {
    /// [`ARM_CODE`] while arming, otherwise 0.
    pub arm_code: u8,
    pub fill: bool,
    /// Vehicle valves (fuel + oxidizer): `true` = OPEN.
    pub veh: bool,
    pub prg: bool,
    /// Igniter request (the pad decides; see [`failsafe`]).
    pub ign: bool,
    /// Watchdog in force on the pad, deciseconds (0 = keep current).
    pub wdg_ds: u8,
    /// Igniter on-time limit, deciseconds (0 = keep current).
    pub ign_max_ds: u8,
    pub flags: u8,
    /// Sender millisecond counter (wraps), for staleness diagnostics.
    pub tx_ms: u16,
    /// Allow the watchdog to fire the igniter. Never set by GSE.
    pub ign_wdg: bool,
}

impl PadCommand {
    pub const WIRE_LEN: usize = 12;

    /// The SAFE command (FILL closed, VEHICLE open, PURGE closed, IGN off).
    pub const fn safe(wdg_ds: u8, ign_max_ds: u8) -> Self {
        PadCommand {
            arm_code: 0,
            fill: false,
            veh: true,
            prg: false,
            ign: false,
            wdg_ds,
            ign_max_ds,
            flags: 0,
            tx_ms: 0,
            ign_wdg: false,
        }
    }

    pub const fn armed(&self) -> bool {
        self.arm_code == ARM_CODE
    }

    pub fn encode(&self, out: &mut [u8]) -> Option<usize> {
        if out.len() < Self::WIRE_LEN {
            return None;
        }
        out[0] = self.arm_code;
        out[1] = self.fill as u8;
        out[2] = self.veh as u8;
        out[3] = self.prg as u8;
        out[4] = self.ign as u8;
        out[5] = self.wdg_ds;
        out[6] = self.ign_max_ds;
        out[7] = self.flags;
        out[8..10].copy_from_slice(&self.tx_ms.to_le_bytes());
        out[10] = self.ign_wdg as u8;
        out[11] = 0;
        Some(Self::WIRE_LEN)
    }

    pub fn decode(body: &[u8]) -> Option<Self> {
        if body.len() < Self::WIRE_LEN {
            return None;
        }
        Some(PadCommand {
            arm_code: body[0],
            fill: body[1] != 0,
            veh: body[2] != 0,
            prg: body[3] != 0,
            ign: body[4] != 0,
            wdg_ds: body[5],
            ign_max_ds: body[6],
            flags: body[7],
            tx_ms: u16::from_le_bytes([body[8], body[9]]),
            ign_wdg: body[10] != 0,
        })
    }
}

/// Pad -> host telemetry (GSE-simple `gse_state_t`, 24 bytes), sent at 10 Hz.
///
/// The `out_*` fields are the outputs the pad is actually driving.
#[derive(Clone, Copy, Debug, PartialEq, Eq)]
pub struct PadState {
    pub flags: u8,
    pub out_fill: bool,
    pub out_veh: bool,
    pub out_prg: bool,
    pub out_ign: bool,
    pub ign_src: u8,
    /// Milliseconds since the last valid command ([`AGE_NEVER`] = never).
    pub age_ms: u16,
    /// Valid commands received (lower 16 bits).
    pub rx_count: u16,
    /// Bad frames (CRC / length), lower 16 bits.
    pub crc_err: u16,
    /// Last received signal strength ([`RSSI_UNKNOWN`] = not measured).
    pub rssi_dbm: i8,
    pub pad_state: u8,
    pub uptime_s: u16,
    pub wdg_ds: u8,
    pub ign_max_ds: u8,
    pub fw_major: u8,
    pub fw_minor: u8,
    pub travel_fill: u8,
    pub travel_veh: u8,
    pub travel_prg: u8,
    pub allow_purge: bool,
}

impl Default for PadState {
    fn default() -> Self {
        PadState {
            flags: state_flags::LATCHED | state_flags::COMMS_LOSS,
            out_fill: false,
            out_veh: true,
            out_prg: false,
            out_ign: false,
            ign_src: 0,
            age_ms: AGE_NEVER,
            rx_count: 0,
            crc_err: 0,
            rssi_dbm: RSSI_UNKNOWN,
            pad_state: PadRunState::WaitHello as u8,
            uptime_s: 0,
            wdg_ds: DEFAULT_WDG_DS,
            ign_max_ds: DEFAULT_IGN_MAX_DS,
            fw_major: 0,
            fw_minor: 0,
            travel_fill: DEFAULT_TRAVEL_DEG,
            travel_veh: DEFAULT_TRAVEL_DEG,
            travel_prg: DEFAULT_TRAVEL_DEG,
            allow_purge: false,
        }
    }
}

impl PadState {
    pub const WIRE_LEN: usize = 24;

    pub const fn has(&self, flag: u8) -> bool {
        self.flags & flag != 0
    }

    pub const fn link_ok(&self) -> bool {
        self.has(state_flags::LINK_OK)
    }

    pub const fn latched(&self) -> bool {
        self.has(state_flags::LATCHED)
    }

    pub const fn armed(&self) -> bool {
        self.has(state_flags::ARMED)
    }

    pub const fn ign_on(&self) -> bool {
        self.has(state_flags::IGN_ON)
    }

    pub const fn comms_loss(&self) -> bool {
        self.has(state_flags::COMMS_LOSS)
    }

    pub const fn hello_ok(&self) -> bool {
        self.has(state_flags::HELLO_OK)
    }

    pub const fn ign_spent(&self) -> bool {
        self.has(state_flags::IGN_SPENT)
    }

    pub const fn run_state(&self) -> Option<PadRunState> {
        PadRunState::from_u8(self.pad_state)
    }

    /// The config the pad has in force, in [`PadConfig`] form (for comparison).
    pub const fn config_in_force(&self) -> PadConfig {
        PadConfig {
            travel_fill: self.travel_fill,
            travel_veh: self.travel_veh,
            travel_prg: self.travel_prg,
            wdg_ds: self.wdg_ds,
            ign_max_ds: self.ign_max_ds,
            allow_purge: self.allow_purge,
            cfg_seq: 0,
        }
    }

    pub fn encode(&self, out: &mut [u8]) -> Option<usize> {
        if out.len() < Self::WIRE_LEN {
            return None;
        }
        out[0] = self.flags;
        out[1] = self.out_fill as u8;
        out[2] = self.out_veh as u8;
        out[3] = self.out_prg as u8;
        out[4] = self.out_ign as u8;
        out[5] = self.ign_src;
        out[6..8].copy_from_slice(&self.age_ms.to_le_bytes());
        out[8..10].copy_from_slice(&self.rx_count.to_le_bytes());
        out[10..12].copy_from_slice(&self.crc_err.to_le_bytes());
        out[12] = self.rssi_dbm as u8;
        out[13] = self.pad_state;
        out[14..16].copy_from_slice(&self.uptime_s.to_le_bytes());
        out[16] = self.wdg_ds;
        out[17] = self.ign_max_ds;
        out[18] = self.fw_major;
        out[19] = self.fw_minor;
        out[20] = self.travel_fill;
        out[21] = self.travel_veh;
        out[22] = self.travel_prg;
        out[23] = self.allow_purge as u8;
        Some(Self::WIRE_LEN)
    }

    pub fn decode(body: &[u8]) -> Option<Self> {
        if body.len() < Self::WIRE_LEN {
            return None;
        }
        Some(PadState {
            flags: body[0],
            out_fill: body[1] != 0,
            out_veh: body[2] != 0,
            out_prg: body[3] != 0,
            out_ign: body[4] != 0,
            ign_src: body[5],
            age_ms: u16::from_le_bytes([body[6], body[7]]),
            rx_count: u16::from_le_bytes([body[8], body[9]]),
            crc_err: u16::from_le_bytes([body[10], body[11]]),
            rssi_dbm: body[12] as i8,
            pad_state: body[13],
            uptime_s: u16::from_le_bytes([body[14], body[15]]),
            wdg_ds: body[16],
            ign_max_ds: body[17],
            fw_major: body[18],
            fw_minor: body[19],
            travel_fill: body[20],
            travel_veh: body[21],
            travel_prg: body[22],
            allow_purge: body[23] != 0,
        })
    }
}

/// Host -> pad configuration (GSE-simple `gse_cfgs_t`, 8 bytes), sent on
/// change; the pad echoes the values in force in every [`PadState`].
#[derive(Clone, Copy, Debug, PartialEq, Eq)]
pub struct PadConfig {
    pub travel_fill: u8,
    pub travel_veh: u8,
    pub travel_prg: u8,
    pub wdg_ds: u8,
    pub ign_max_ds: u8,
    pub allow_purge: bool,
    /// Incrementing; echoed in the `CfgChanged` event.
    pub cfg_seq: u8,
}

impl Default for PadConfig {
    fn default() -> Self {
        PadConfig {
            travel_fill: DEFAULT_TRAVEL_DEG,
            travel_veh: DEFAULT_TRAVEL_DEG,
            travel_prg: DEFAULT_TRAVEL_DEG,
            wdg_ds: DEFAULT_WDG_DS,
            ign_max_ds: DEFAULT_IGN_MAX_DS,
            allow_purge: false,
            cfg_seq: 0,
        }
    }
}

impl PadConfig {
    pub const WIRE_LEN: usize = 8;

    /// Same values, ignoring `cfg_seq`.
    pub const fn same_values(&self, other: &PadConfig) -> bool {
        self.travel_fill == other.travel_fill
            && self.travel_veh == other.travel_veh
            && self.travel_prg == other.travel_prg
            && self.wdg_ds == other.wdg_ds
            && self.ign_max_ds == other.ign_max_ds
            && self.allow_purge == other.allow_purge
    }

    /// Clamp every field to the range the pad accepts.
    pub const fn clamped(self) -> Self {
        PadConfig {
            travel_fill: clamp_travel(self.travel_fill),
            travel_veh: clamp_travel(self.travel_veh),
            travel_prg: clamp_travel(self.travel_prg),
            wdg_ds: clamp_wdg_ds(self.wdg_ds),
            ign_max_ds: clamp_ign_max_ds(self.ign_max_ds),
            allow_purge: self.allow_purge,
            cfg_seq: self.cfg_seq,
        }
    }

    pub fn encode(&self, out: &mut [u8]) -> Option<usize> {
        if out.len() < Self::WIRE_LEN {
            return None;
        }
        out[0] = self.travel_fill;
        out[1] = self.travel_veh;
        out[2] = self.travel_prg;
        out[3] = self.wdg_ds;
        out[4] = self.ign_max_ds;
        out[5] = self.allow_purge as u8;
        out[6] = self.cfg_seq;
        out[7] = 0;
        Some(Self::WIRE_LEN)
    }

    pub fn decode(body: &[u8]) -> Option<Self> {
        if body.len() < Self::WIRE_LEN {
            return None;
        }
        Some(PadConfig {
            travel_fill: body[0],
            travel_veh: body[1],
            travel_prg: body[2],
            wdg_ds: body[3],
            ign_max_ds: body[4],
            allow_purge: body[5] != 0,
            cfg_seq: body[6],
        })
    }
}

/// Pad -> host event (GSE-simple `gse_event_t`, 3 bytes), sent on change.
#[derive(Clone, Copy, Debug, PartialEq, Eq)]
pub struct PadEvent {
    /// Raw code; see [`PadEventCode`] (unknown codes are kept, not rejected).
    pub code: u8,
    pub arg: u8,
}

impl PadEvent {
    pub const WIRE_LEN: usize = 3;

    pub const fn new(code: PadEventCode, arg: u8) -> Self {
        PadEvent {
            code: code as u8,
            arg,
        }
    }

    pub const fn kind(&self) -> Option<PadEventCode> {
        PadEventCode::from_u8(self.code)
    }

    pub fn encode(&self, out: &mut [u8]) -> Option<usize> {
        if out.len() < Self::WIRE_LEN {
            return None;
        }
        out[0] = self.code;
        out[1] = self.arg;
        out[2] = 0;
        Some(Self::WIRE_LEN)
    }

    pub fn decode(body: &[u8]) -> Option<Self> {
        if body.len() < Self::WIRE_LEN {
            return None;
        }
        Some(PadEvent {
            code: body[0],
            arg: body[1],
        })
    }
}

/// Outputs the pad must drive (result of [`failsafe`]).
#[derive(Clone, Copy, Debug, PartialEq, Eq)]
pub struct PadOutputs {
    pub fill: bool,
    pub veh: bool,
    pub prg: bool,
    /// Igniter *request* after the arm interlock; the one-shot limit is
    /// applied by the pad's igniter sequencer on top of this.
    pub ign: bool,
    pub armed: bool,
    pub comms_loss: bool,
    pub ign_src: u8,
}

impl PadOutputs {
    /// COMMS LOSS / never connected: FILL closed, PURGE closed, VEHICLE open
    /// (dumps propellant), IGN off.
    pub const SAFE: PadOutputs = PadOutputs {
        fill: false,
        veh: true,
        prg: false,
        ign: false,
        armed: false,
        comms_loss: true,
        ign_src: 0,
    };
}

/// GSE-simple `gse_failsafe()`: given the last accepted command (if any), its
/// age in ms (`None` = never), and the watchdog in force (deciseconds), return
/// the outputs the pad must drive. Pure and allocation-free.
pub fn failsafe(last: Option<&PadCommand>, age_ms: Option<u32>, wdg_ds: u8) -> PadOutputs {
    let wdg_ms = wdg_ds as u32 * 100;
    let fresh = match (last, age_ms) {
        (Some(_), Some(age)) => age <= wdg_ms,
        _ => false,
    };
    let Some(cmd) = last.filter(|_| fresh) else {
        return PadOutputs::SAFE;
    };
    let armed = cmd.armed();
    let ign = cmd.ign && armed;
    PadOutputs {
        fill: cmd.fill,
        veh: cmd.veh,
        prg: cmd.prg,
        ign,
        armed,
        comms_loss: false,
        ign_src: if ign { ign_src::CMD } else { 0 },
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    fn cmd() -> PadCommand {
        PadCommand {
            arm_code: 0,
            fill: true,
            veh: false,
            prg: true,
            ign: false,
            wdg_ds: 12,
            ign_max_ds: 6,
            flags: 0,
            tx_ms: 0xBEEF,
            ign_wdg: false,
        }
    }

    #[test]
    fn command_layout_matches_gse_simple() {
        let mut c = cmd();
        c.arm_code = ARM_CODE;
        let mut buf = [0u8; 16];
        assert_eq!(c.encode(&mut buf), Some(12));
        assert_eq!(&buf[..12], &[0xA5, 1, 0, 1, 0, 12, 6, 0, 0xEF, 0xBE, 0, 0]);
        assert_eq!(PadCommand::decode(&buf[..12]), Some(c));
        assert!(PadCommand::decode(&buf[..11]).is_none());
    }

    #[test]
    fn state_and_config_roundtrip() {
        let mut st = PadState {
            out_fill: true,
            rssi_dbm: -61,
            uptime_s: 900,
            ..PadState::default()
        };
        st.flags = state_flags::LINK_OK | state_flags::ARMED;
        let mut buf = [0u8; 32];
        assert_eq!(st.encode(&mut buf), Some(24));
        assert_eq!(PadState::decode(&buf[..24]), Some(st));
        assert_eq!(buf[12], (-61i8) as u8);

        let cfg = PadConfig {
            travel_fill: 90,
            allow_purge: true,
            cfg_seq: 7,
            ..PadConfig::default()
        };
        assert_eq!(cfg.encode(&mut buf), Some(8));
        assert_eq!(PadConfig::decode(&buf[..8]), Some(cfg));

        let ev = PadEvent::new(PadEventCode::IgnOff, 1);
        assert_eq!(ev.encode(&mut buf), Some(3));
        assert_eq!(PadEvent::decode(&buf[..3]), Some(ev));
        assert_eq!(ev.kind(), Some(PadEventCode::IgnOff));
    }

    #[test]
    fn failsafe_matches_gse_simple_contract() {
        // Never connected / stale: FILL closed, PURGE closed, VEHICLE open, IGN off.
        assert_eq!(failsafe(None, None, 10), PadOutputs::SAFE);
        let c = cmd();
        assert_eq!(failsafe(Some(&c), Some(1_001), 10), PadOutputs::SAFE);
        assert_eq!(failsafe(Some(&c), None, 10), PadOutputs::SAFE);

        // Fresh: pass-through, no interlocks between the valves.
        let o = failsafe(Some(&c), Some(1_000), 10);
        assert!(o.fill && !o.veh && o.prg && !o.ign && !o.armed && !o.comms_loss);

        // Igniter needs the arm code in this very command.
        let mut fire = c;
        fire.ign = true;
        assert!(!failsafe(Some(&fire), Some(10), 10).ign);
        fire.arm_code = ARM_CODE;
        let o = failsafe(Some(&fire), Some(10), 10);
        assert!(o.ign && o.armed);
        assert_eq!(o.ign_src, ign_src::CMD);
        // ... and a fresh link.
        assert_eq!(failsafe(Some(&fire), Some(1_001), 10), PadOutputs::SAFE);
    }

    #[test]
    fn servo_pulse_follows_travel_reverse_and_trim() {
        assert_eq!(servo_pulse_us(true, 45, false, 0), 2_000);
        assert_eq!(servo_pulse_us(false, 45, false, 0), 1_000);
        assert_eq!(servo_pulse_us(true, 45, true, 0), 1_000);
        assert_eq!(servo_pulse_us(true, 90, false, 0), 2_500);
        assert_eq!(servo_pulse_us(false, 90, false, 0), 500);
        assert_eq!(servo_pulse_us(false, 90, false, -20), 500);
        assert_eq!(servo_pulse_us(true, 60, false, 0), 2_166);
        assert_eq!(servo_pulse_us(true, 1, false, 30), 1_585);
    }

    #[test]
    fn timing_follows_the_radio_and_fits_lora_airtime() {
        use crate::radio::DxLr42Level;
        assert_eq!(
            PadTiming::for_radio(crate::RadioModule::EspNow),
            PadTiming::ESPNOW
        );
        assert_eq!(
            PadTiming::for_radio(crate::RadioModule::DxLr42),
            PadTiming::LORA
        );
        assert_eq!(
            PadTiming::for_radio(crate::RadioModule::E28_2G4M27S),
            PadTiming::LORA
        );
        // Air time per frame at DX-LR42 LEVEL 3, duty cycle of the LoRa timing.
        let air_ms = |bytes: u64| (bytes + 8) * 8 * 1000 / DxLr42Level::Level3.air_bps() as u64;
        let cmd = 25; // 0x00 + COBS(8 + 12 + 2) + 0x00
        let state = 37; // 0x00 + COBS(8 + 24 + 2) + 0x00
        let t = PadTiming::LORA;
        let busy = air_ms(cmd) * 1000 / t.cmd_period_ms as u64
            + air_ms(state) * 1000 / t.state_period_ms as u64;
        assert!(busy < 500, "LoRa channel {busy}/1000 busy at LEVEL 3");
        let e = PadTiming::ESPNOW;
        let busy_fast = air_ms(cmd) * 1000 / e.cmd_period_ms as u64
            + air_ms(state) * 1000 / e.state_period_ms as u64;
        assert!(busy_fast > 800, "the ESP-NOW rates would saturate LoRa");
    }

    #[test]
    fn config_clamps_to_pad_limits() {
        let c = PadConfig {
            travel_fill: 0,
            travel_veh: 200,
            travel_prg: 45,
            wdg_ds: 1,
            ign_max_ds: 99,
            allow_purge: true,
            cfg_seq: 3,
        }
        .clamped();
        assert_eq!(
            (c.travel_fill, c.travel_veh, c.wdg_ds, c.ign_max_ds),
            (5, 90, 3, 20)
        );
        assert!(c.same_values(&PadConfig { cfg_seq: 9, ..c }));
    }
}
