//! Shared application state and its JSON projection for the browser UI.

use std::collections::VecDeque;
use std::path::PathBuf;
use std::sync::{Arc, Mutex, MutexGuard};
use std::time::{SystemTime, UNIX_EPOCH};

use gse_proto::{faults, flags, ControlState, LinkStat, StateSnapshot};

use crate::broadcaster::Broadcaster;
use crate::clock;
use crate::config::Config;
use crate::json;
use crate::logging::{Level, LogEntry, SessionLog, Source};
use crate::safety;

pub const MAX_LOG: usize = 1000;

#[derive(Clone, Debug, Default, PartialEq)]
pub struct ActionState {
    pub enabled: bool,
    pub reason: String,
}

#[derive(Clone, Debug, Default, PartialEq)]
pub struct Actions {
    pub fill_on: ActionState,
    pub fill_off: ActionState,
    pub srv_on: ActionState,
    pub srv_off: ActionState,
    pub ign_pwr: ActionState,
    pub ign_safe: ActionState,
    pub srv_ign_on: ActionState,
    pub srv_ign_off: ActionState,
    pub reset_controls: ActionState,
    pub reset_fill: ActionState,
    pub clear_faults: ActionState,
    pub estop: ActionState,
}

impl Actions {
    pub fn get(&self, action: &str) -> Option<&ActionState> {
        Some(match action {
            "fill_on" => &self.fill_on,
            "fill_off" => &self.fill_off,
            "srv_on" => &self.srv_on,
            "srv_off" => &self.srv_off,
            "ign_pwr" => &self.ign_pwr,
            "ign_safe" => &self.ign_safe,
            "srv_ign_on" => &self.srv_ign_on,
            "srv_ign_off" => &self.srv_ign_off,
            "reset_controls" => &self.reset_controls,
            "reset_fill" => &self.reset_fill,
            "clear_faults" => &self.clear_faults,
            "estop" => &self.estop,
            _ => return None,
        })
    }

    pub fn to_json(&self) -> String {
        let mut out = String::from("{");
        for (key, value) in [
            ("fill_on", &self.fill_on),
            ("fill_off", &self.fill_off),
            ("srv_on", &self.srv_on),
            ("srv_off", &self.srv_off),
            ("ign_pwr", &self.ign_pwr),
            ("ign_safe", &self.ign_safe),
            ("srv_ign_on", &self.srv_ign_on),
            ("srv_ign_off", &self.srv_ign_off),
            ("reset_controls", &self.reset_controls),
            ("reset_fill", &self.reset_fill),
            ("clear_faults", &self.clear_faults),
            ("estop", &self.estop),
        ] {
            let mut obj = String::from("{");
            json::field(&mut obj, "enabled", json::bool_json(value.enabled));
            json::field(&mut obj, "reason", &json::quoted(&value.reason));
            obj.push('}');
            json::field(&mut out, key, &obj);
        }
        out.push('}');
        out
    }
}

#[derive(Clone, Copy, Debug, PartialEq, Eq)]
pub enum GroundStatus {
    Disconnected,
    Scanning,
    Connecting,
    Online,
    Error,
}

impl GroundStatus {
    pub fn as_str(self) -> &'static str {
        match self {
            GroundStatus::Disconnected => "disconnected",
            GroundStatus::Scanning => "scanning",
            GroundStatus::Connecting => "connecting",
            GroundStatus::Online => "online",
            GroundStatus::Error => "error",
        }
    }
}

#[derive(Clone, Copy, Debug, PartialEq, Eq)]
pub enum VehicleStatus {
    Offline,
    Pinging,
    Online,
}

impl VehicleStatus {
    pub fn as_str(self) -> &'static str {
        match self {
            VehicleStatus::Offline => "offline",
            VehicleStatus::Pinging => "pinging",
            VehicleStatus::Online => "online",
        }
    }
}

pub struct GroundInfo {
    pub status: GroundStatus,
    pub port: String,
    pub fw: Option<String>,
    pub device_id: Option<u32>,
    pub last_seen_epoch_ms: u64,
    pub link: LinkStat,
    pub have_link_stat: bool,
    pub protocol: u8,
    pub error: Option<String>,
}

impl Default for GroundInfo {
    fn default() -> Self {
        GroundInfo {
            status: GroundStatus::Disconnected,
            port: "auto".to_string(),
            fw: None,
            device_id: None,
            last_seen_epoch_ms: 0,
            link: LinkStat::default(),
            have_link_stat: false,
            protocol: 0,
            error: None,
        }
    }
}

pub struct VehicleInfo {
    pub status: VehicleStatus,
    pub fw: Option<String>,
    pub device_id: Option<u32>,
    pub state: StateSnapshot,
    pub last_seen_epoch_ms: u64,
    pub rtt_ms: Option<u32>,
}

impl Default for VehicleInfo {
    fn default() -> Self {
        VehicleInfo {
            status: VehicleStatus::Offline,
            fw: None,
            device_id: None,
            state: StateSnapshot::default(),
            last_seen_epoch_ms: 0,
            rtt_ms: None,
        }
    }
}

#[derive(Clone, Default)]
pub struct FillUi {
    pub duration_s: u16,
    pub remaining_ms: u32,
    /// Wall clock time at which the app will send the automatic FILL OFF.
    pub ends_at_epoch_ms: u64,
    pub active: bool,
    pub seq_complete: bool,
    pub seq_aborted: bool,
    pub auto_off_pending: bool,
    pub completed_at_epoch_ms: u64,
}

/// Live-mode 5 s `SRV+IGN` countdown projection for the UI and SSE stream.
#[derive(Clone, Default)]
pub struct CountdownUi {
    pub active: bool,
    pub duration_ms: u32,
    pub remaining_ms: u32,
    pub ends_at_epoch_ms: u64,
}

/// Projection of the deterministic control machine for the UI.
#[derive(Clone, Debug)]
pub struct ControlsUi {
    /// Last state confirmed by the vehicle (ACK/STATE), never a guess.
    pub state: ControlState,
    /// State the in-flight command will reach once acknowledged.
    pub target: Option<ControlState>,
    /// Startup handshake finished: every output verified OFF/SAFE.
    pub ready: bool,
    /// Last event pushed into the machine, e.g. `fill_on`.
    pub last_event: Option<&'static str>,
    /// `pending` after send, then `acknowledged` / `rejected` / `timeout`.
    pub last_result: Option<&'static str>,
}

impl Default for ControlsUi {
    fn default() -> Self {
        ControlsUi {
            state: ControlState::SAFE,
            target: None,
            ready: false,
            last_event: None,
            last_result: None,
        }
    }
}

impl ControlsUi {
    pub fn to_json(&self) -> String {
        let mut out = String::from("{");
        json::field(&mut out, "fill", &json::quoted(self.state.fill.label()));
        json::field(&mut out, "srv", &json::quoted(self.state.srv.label()));
        json::field(&mut out, "ign", &json::quoted(self.state.ign.label()));
        json::field(
            &mut out,
            "summary",
            &json::quoted(&format!(
                "FILL={} SRV={} IGN={}",
                self.state.fill.label(),
                self.state.srv.label(),
                self.state.ign.label()
            )),
        );
        json::field(&mut out, "safe", json::bool_json(self.state.is_safe()));
        json::field(&mut out, "ready", json::bool_json(self.ready));
        match self.target {
            Some(target) => {
                let mut t = String::from("{");
                json::field(&mut t, "fill", &json::quoted(target.fill.label()));
                json::field(&mut t, "srv", &json::quoted(target.srv.label()));
                json::field(&mut t, "ign", &json::quoted(target.ign.label()));
                t.push('}');
                json::field(&mut out, "target", &t);
            }
            None => json::field(&mut out, "target", "null"),
        }
        json::field(&mut out, "last_event", &json::opt_str_json(self.last_event));
        json::field(
            &mut out,
            "last_result",
            &json::opt_str_json(self.last_result),
        );
        out.push('}');
        out
    }
}

#[derive(Clone, Debug)]
pub struct PendingInfo {
    pub cmd_id: u16,
    pub action: String,
    pub label: String,
    pub attempts: u32,
    pub sent_epoch_ms: u64,
    pub timeout_ms: u64,
}

#[derive(Clone, Debug, Default)]
pub struct Stats {
    pub rx_frames: u64,
    pub tx_frames: u64,
    pub crc_errors: u64,
    pub malformed: u64,
    pub retries: u64,
    pub reconnects: u64,
    pub acks_ok: u64,
    pub acks_rejected: u64,
    pub commands_sent: u64,
}

pub struct AppState {
    pub config: Config,
    pub ground: GroundInfo,
    pub vehicle: VehicleInfo,
    pub actions: Actions,
    pub controls: ControlsUi,
    pub fill: FillUi,
    pub countdown: CountdownUi,
    pub pending: Option<PendingInfo>,
    pub log: VecDeque<LogEntry>,
    pub next_log_id: u64,
    pub stats: Stats,
    pub started_epoch_ms: u64,
    pub now_epoch_ms: u64,
    pub revision: u64,
    pub sim: bool,
    pub url: String,
    pub port_hint: String,
    pub scan_note: Option<String>,
    pub log_path: Option<String>,
    pub host_uptime_ms: u64,
}

impl AppState {
    pub fn new(config: Config, sim: bool, url: String, port_hint: String) -> Self {
        let now = clock::unix_ms();
        AppState {
            fill: FillUi {
                duration_s: config.fill_seconds,
                ..FillUi::default()
            },
            config,
            ground: GroundInfo::default(),
            vehicle: VehicleInfo::default(),
            actions: Actions::default(),
            controls: ControlsUi::default(),
            countdown: CountdownUi::default(),
            pending: None,
            log: VecDeque::new(),
            next_log_id: 1,
            stats: Stats::default(),
            started_epoch_ms: now,
            now_epoch_ms: now,
            revision: 0,
            sim,
            url,
            port_hint,
            scan_note: None,
            log_path: None,
            host_uptime_ms: 0,
        }
    }

    pub fn push_log(
        &mut self,
        level: Level,
        source: Source,
        message: impl Into<String>,
        code: Option<&'static str>,
    ) -> LogEntry {
        let entry = LogEntry {
            id: self.next_log_id,
            epoch_ms: clock::unix_ms(),
            level,
            source,
            message: message.into(),
            code,
        };
        self.next_log_id = self.next_log_id.wrapping_add(1);
        if self.log.len() >= MAX_LOG {
            self.log.pop_front();
        }
        self.log.push_back(entry.clone());
        self.revision = self.revision.wrapping_add(1);
        entry
    }

    pub fn vehicle_online(&self) -> bool {
        self.vehicle.status == VehicleStatus::Online
    }

    /// Recompute derived UI state. Called every engine tick.
    pub fn refresh(&mut self) {
        let now = clock::unix_ms();
        self.now_epoch_ms = now;
        self.host_uptime_ms = now.saturating_sub(self.started_epoch_ms);
        if self.countdown.active {
            self.countdown.remaining_ms = self
                .countdown
                .ends_at_epoch_ms
                .saturating_sub(now)
                .min(u32::MAX as u64) as u32;
        } else {
            self.countdown.remaining_ms = 0;
        }
        self.actions = safety::evaluate(&safety::Inputs {
            mode: self.config.op_mode,
            ground_online: self.ground.status == GroundStatus::Online,
            vehicle_online: self.vehicle.status == VehicleStatus::Online,
            pending: self.pending.is_some(),
            countdown_active: self.countdown.active,
            controls_ready: self.controls.ready,
            state: self.vehicle.state,
        });
        // The machine follows the vehicle, never the operator's intent, and
        // it only moves once the command has been acknowledged. A STATE frame
        // can already report the output moving during the settle time; until
        // the ACK lands that is the *target*, not the confirmed state.
        if self.pending.is_none() {
            self.controls.state = ControlState::from_snapshot(&self.vehicle.state);
            self.controls.target = None;
        }
        let snap = self.vehicle.state;
        self.fill.active = snap.fill_open();
        self.fill.remaining_ms = snap.fill_remaining_ms;
        self.fill.seq_complete = snap.fill_seq_complete();
        self.fill.seq_aborted = snap.fill_seq_aborted();
        if snap.fill_open() {
            self.fill.duration_s = snap.fill_duration_s;
        } else if self.fill.duration_s == 0 {
            self.fill.duration_s = self.config.fill_seconds;
        }
        if self.pending.is_some() {
            // Auto-off is "pending" while the automatic command is in flight.
        }
    }

    pub fn state_json(snapshot: &StateSnapshot) -> String {
        let mut out = String::from("{");
        json::field(&mut out, "flags", &snapshot.flags.to_string());
        json::field(&mut out, "faults", &snapshot.faults.to_string());
        json::field(&mut out, "fill", json::bool_json(snapshot.fill_open()));
        json::field(&mut out, "srv", json::bool_json(snapshot.srv_on()));
        json::field(&mut out, "ign", json::bool_json(snapshot.ign_pwr()));
        json::field(
            &mut out,
            "fill_timer",
            json::bool_json(snapshot.fill_timer_active()),
        );
        json::field(
            &mut out,
            "fill_seq_complete",
            json::bool_json(snapshot.fill_seq_complete()),
        );
        json::field(
            &mut out,
            "fill_seq_aborted",
            json::bool_json(snapshot.fill_seq_aborted()),
        );
        json::field(
            &mut out,
            "fill_auto_off",
            json::bool_json(snapshot.has(flags::FILL_AUTO_OFF)),
        );
        json::field(
            &mut out,
            "link_loss_safe",
            json::bool_json(snapshot.has(flags::LINK_LOSS_SAFE)),
        );
        json::field(
            &mut out,
            "last_rejected",
            json::bool_json(snapshot.has(flags::LAST_REJECTED)),
        );
        json::field(&mut out, "inhibited", json::bool_json(snapshot.inhibited()));
        json::field(&mut out, "estop", json::bool_json(snapshot.estop()));
        json::field(
            &mut out,
            "boot_safe",
            json::bool_json(snapshot.has(flags::BOOT_SAFE)),
        );
        json::field(
            &mut out,
            "fill_remaining_ms",
            &snapshot.fill_remaining_ms.to_string(),
        );
        json::field(
            &mut out,
            "fill_duration_s",
            &snapshot.fill_duration_s.to_string(),
        );
        json::field(&mut out, "uptime_ms", &snapshot.uptime_ms.to_string());
        json::field(&mut out, "last_cmd_id", &snapshot.last_cmd_id.to_string());
        out.push('}');
        out
    }

    fn flags_json(snapshot: &StateSnapshot) -> String {
        let mut names: Vec<&str> = Vec::new();
        if snapshot.fill_open() {
            names.push("FILL OPEN");
        }
        if snapshot.srv_on() {
            names.push("SRV ON");
        }
        if snapshot.ign_pwr() {
            names.push("IGN PWR");
        }
        if snapshot.fill_timer_active() {
            names.push("FILL TIMER");
        }
        if snapshot.fill_seq_complete() {
            names.push("FILL SEQ DONE");
        }
        if snapshot.fill_seq_aborted() {
            names.push("FILL ABORTED");
        }
        if snapshot.estop() {
            names.push("E-STOP");
        }
        if snapshot.inhibited() {
            names.push("INHIBITED");
        }
        if snapshot.has(flags::LINK_LOSS_SAFE) {
            names.push("LINK SAFE");
        }
        if snapshot.has(flags::FILL_AUTO_OFF) {
            names.push("AUTO OFF");
        }
        if snapshot.has(flags::BOOT_SAFE) {
            names.push("BOOT SAFE");
        }
        if snapshot.has(flags::LAST_REJECTED) {
            names.push("LAST REJECTED");
        }
        let mut out = String::from("[");
        for (i, name) in names.iter().enumerate() {
            if i > 0 {
                out.push(',');
            }
            out.push_str(&json::quoted(name));
        }
        out.push(']');
        out
    }

    fn faults_json(snapshot: &StateSnapshot) -> String {
        let mut names: Vec<&str> = Vec::new();
        for (bit, name) in [
            (faults::UART_OVERRUN, "UART OVERRUN"),
            (faults::CRC_ERROR, "CRC ERROR"),
            (faults::LINK_TIMEOUT, "LINK TIMEOUT"),
            (faults::WATCHDOG_RESET, "WATCHDOG RESET"),
            (faults::RADIO_ERROR, "RADIO ERROR"),
            (faults::CONFIG_ERROR, "CONFIG ERROR"),
            (faults::OUTPUT_FAULT, "OUTPUT FAULT"),
        ] {
            if snapshot.faults & bit != 0 {
                names.push(name);
            }
        }
        let mut out = String::from("[");
        for (i, name) in names.iter().enumerate() {
            if i > 0 {
                out.push(',');
            }
            out.push_str(&json::quoted(name));
        }
        out.push(']');
        out
    }

    fn ground_json(&self) -> String {
        let age = if self.ground.last_seen_epoch_ms > 0 {
            self.now_epoch_ms
                .saturating_sub(self.ground.last_seen_epoch_ms)
        } else {
            0
        };
        let mut link = String::from("{");
        json::field(
            &mut link,
            "frames_from_radio",
            &self.ground.link.frames_from_radio.to_string(),
        );
        json::field(
            &mut link,
            "frames_to_radio",
            &self.ground.link.frames_to_radio.to_string(),
        );
        json::field(
            &mut link,
            "crc_errors",
            &self.ground.link.crc_errors.to_string(),
        );
        json::field(
            &mut link,
            "uart_errors",
            &self.ground.link.uart_errors.to_string(),
        );
        json::field(
            &mut link,
            "radio_age_ms",
            &self.ground.link.last_radio_age_ms.to_string(),
        );
        json::field(&mut link, "drops", &self.ground.link.drops.to_string());
        json::field(
            &mut link,
            "bridge_uptime_ms",
            &self.ground.link.bridge_uptime_ms.to_string(),
        );
        link.push('}');

        let mut out = String::from("{");
        json::field(
            &mut out,
            "status",
            &json::quoted(self.ground.status.as_str()),
        );
        json::field(&mut out, "port", &json::quoted(&self.ground.port));
        json::field(
            &mut out,
            "fw",
            &json::opt_str_json(self.ground.fw.as_deref()),
        );
        json::field(
            &mut out,
            "device_id",
            &self
                .ground
                .device_id
                .map(|v| v.to_string())
                .unwrap_or_else(|| "null".to_string()),
        );
        json::field(&mut out, "age_ms", &age.to_string());
        json::field(&mut out, "protocol", &self.ground.protocol.to_string());
        json::field(&mut out, "link", &link);
        json::field(
            &mut out,
            "error",
            &json::opt_str_json(self.ground.error.as_deref()),
        );
        out.push('}');
        out
    }

    fn vehicle_json(&self) -> String {
        let age = if self.vehicle.last_seen_epoch_ms > 0 {
            self.now_epoch_ms
                .saturating_sub(self.vehicle.last_seen_epoch_ms)
        } else {
            0
        };
        let mut out = String::from("{");
        json::field(
            &mut out,
            "status",
            &json::quoted(self.vehicle.status.as_str()),
        );
        json::field(
            &mut out,
            "fw",
            &json::opt_str_json(self.vehicle.fw.as_deref()),
        );
        json::field(
            &mut out,
            "device_id",
            &self
                .vehicle
                .device_id
                .map(|v| v.to_string())
                .unwrap_or_else(|| "null".to_string()),
        );
        json::field(
            &mut out,
            "rtt_ms",
            &self
                .vehicle
                .rtt_ms
                .map(|v| v.to_string())
                .unwrap_or_else(|| "null".to_string()),
        );
        json::field(&mut out, "age_ms", &age.to_string());
        json::field(&mut out, "state", &Self::state_json(&self.vehicle.state));
        json::field(&mut out, "flags", &Self::flags_json(&self.vehicle.state));
        json::field(
            &mut out,
            "fault_names",
            &Self::faults_json(&self.vehicle.state),
        );
        json::field(
            &mut out,
            "summary",
            &json::quoted(&format!(
                "FILL={} SRV={} IGN={}",
                if self.vehicle.state.fill_open() {
                    "ON"
                } else {
                    "OFF"
                },
                if self.vehicle.state.srv_on() {
                    "ON"
                } else {
                    "OFF"
                },
                if self.vehicle.state.ign_pwr() {
                    "PWR"
                } else {
                    "SAFE"
                }
            )),
        );
        out.push('}');
        out
    }

    pub fn snapshot_json(&self) -> String {
        let mut out = String::from("{");
        json::field(&mut out, "rev", &self.revision.to_string());
        json::field(&mut out, "now", &self.now_epoch_ms.to_string());
        json::field(&mut out, "uptime_ms", &self.host_uptime_ms.to_string());
        json::field(&mut out, "sim", json::bool_json(self.sim));
        json::field(&mut out, "url", &json::quoted(&self.url));
        json::field(&mut out, "started", &self.started_epoch_ms.to_string());
        json::field(
            &mut out,
            "log_path",
            &json::opt_str_json(self.log_path.as_deref()),
        );
        json::field(
            &mut out,
            "scan_note",
            &json::opt_str_json(self.scan_note.as_deref()),
        );
        json::field(&mut out, "port_hint", &json::quoted(&self.port_hint));
        json::field(&mut out, "ground", &self.ground_json());
        json::field(&mut out, "vehicle", &self.vehicle_json());
        json::field(&mut out, "controls", &self.controls.to_json());

        let mut fill = String::from("{");
        json::field(&mut fill, "duration_s", &self.fill.duration_s.to_string());
        json::field(
            &mut fill,
            "remaining_ms",
            &self.fill.remaining_ms.to_string(),
        );
        json::field(
            &mut fill,
            "ends_at",
            &self.fill.ends_at_epoch_ms.to_string(),
        );
        json::field(&mut fill, "active", json::bool_json(self.fill.active));
        json::field(
            &mut fill,
            "seq_complete",
            json::bool_json(self.fill.seq_complete),
        );
        json::field(
            &mut fill,
            "seq_aborted",
            json::bool_json(self.fill.seq_aborted),
        );
        json::field(
            &mut fill,
            "auto_off_pending",
            json::bool_json(self.fill.auto_off_pending),
        );
        json::field(
            &mut fill,
            "completed_at",
            &self.fill.completed_at_epoch_ms.to_string(),
        );
        fill.push('}');
        json::field(&mut out, "fill", &fill);

        let mut countdown = String::from("{");
        json::field(
            &mut countdown,
            "active",
            json::bool_json(self.countdown.active),
        );
        json::field(
            &mut countdown,
            "duration_ms",
            &self.countdown.duration_ms.to_string(),
        );
        json::field(
            &mut countdown,
            "remaining_ms",
            &self.countdown.remaining_ms.to_string(),
        );
        json::field(
            &mut countdown,
            "ends_at",
            &self.countdown.ends_at_epoch_ms.to_string(),
        );
        countdown.push('}');
        json::field(&mut out, "countdown", &countdown);

        json::field(&mut out, "actions", &self.actions.to_json());

        match &self.pending {
            Some(p) => {
                let mut pending = String::from("{");
                json::field(&mut pending, "cmd_id", &p.cmd_id.to_string());
                json::field(&mut pending, "action", &json::quoted(&p.action));
                json::field(&mut pending, "label", &json::quoted(&p.label));
                json::field(&mut pending, "attempts", &p.attempts.to_string());
                json::field(
                    &mut pending,
                    "age_ms",
                    &self
                        .now_epoch_ms
                        .saturating_sub(p.sent_epoch_ms)
                        .to_string(),
                );
                json::field(&mut pending, "timeout_ms", &p.timeout_ms.to_string());
                pending.push('}');
                json::field(&mut out, "pending", &pending);
            }
            None => json::field(&mut out, "pending", "null"),
        }

        let mut stats = String::from("{");
        json::field(&mut stats, "rx_frames", &self.stats.rx_frames.to_string());
        json::field(&mut stats, "tx_frames", &self.stats.tx_frames.to_string());
        json::field(&mut stats, "crc_errors", &self.stats.crc_errors.to_string());
        json::field(&mut stats, "malformed", &self.stats.malformed.to_string());
        json::field(&mut stats, "retries", &self.stats.retries.to_string());
        json::field(&mut stats, "reconnects", &self.stats.reconnects.to_string());
        json::field(&mut stats, "acks_ok", &self.stats.acks_ok.to_string());
        json::field(
            &mut stats,
            "acks_rejected",
            &self.stats.acks_rejected.to_string(),
        );
        json::field(
            &mut stats,
            "commands_sent",
            &self.stats.commands_sent.to_string(),
        );
        stats.push('}');
        json::field(&mut out, "stats", &stats);

        json::field(&mut out, "config", &self.config.to_json());
        out.push('}');
        out
    }
}

/// Shared state handle used by the engine, HTTP server and UI.
pub struct Shared {
    pub state: Mutex<AppState>,
    pub events: Broadcaster,
    pub session: Mutex<Option<SessionLog>>,
    pub token: String,
    pub config_path: PathBuf,
    pub verbose: bool,
}

impl Shared {
    pub fn new(
        state: AppState,
        session: Option<SessionLog>,
        token: String,
        config_path: PathBuf,
        verbose: bool,
    ) -> Arc<Self> {
        let initial = state.snapshot_json();
        Arc::new(Shared {
            state: Mutex::new(state),
            events: Broadcaster::new(initial),
            session: Mutex::new(session),
            token,
            config_path,
            verbose,
        })
    }

    pub fn lock(&self) -> MutexGuard<'_, AppState> {
        self.state.lock().unwrap_or_else(|e| e.into_inner())
    }

    pub fn snapshot(&self) -> String {
        self.lock().snapshot_json()
    }

    pub fn publish(&self) {
        let json = self.snapshot();
        self.events.state(&json);
    }

    pub fn log(
        &self,
        level: Level,
        source: Source,
        message: impl Into<String>,
        code: Option<&'static str>,
    ) {
        let entry = {
            let mut st = self.lock();
            st.push_log(level, source, message, code)
        };
        if let Some(session) = self
            .session
            .lock()
            .unwrap_or_else(|e| e.into_inner())
            .as_mut()
        {
            session.write(&entry);
        }
        self.events.log(&entry);
    }

    pub fn save_config(&self) {
        let cfg = self.lock().config.clone();
        if let Err(e) = cfg.save(&self.config_path) {
            self.log(
                Level::Warn,
                Source::App,
                format!(
                    "Could not save settings to {}: {e}",
                    self.config_path.display()
                ),
                None,
            );
        }
    }
}

/// Session token used to authenticate state-changing HTTP requests. The page
/// embeds it in a meta tag; cross-origin scripts cannot read it, and no CORS
/// headers are served, so a malicious web page cannot drive the GSE.
pub fn make_token() -> String {
    let local = 0u8;
    let seed = SystemTime::now()
        .duration_since(UNIX_EPOCH)
        .map(|d| d.as_nanos() as u64)
        .unwrap_or(0x9E37_79B9_7F4A_7C15)
        ^ (std::process::id() as u64).wrapping_mul(0x1000_0001)
        ^ (&local as *const u8 as usize as u64);
    let mut state = seed | 1;
    let mut out = String::with_capacity(32);
    for _ in 0..4 {
        state ^= state << 13;
        state ^= state >> 7;
        state ^= state << 17;
        out.push_str(&format!("{:08x}", (state & 0xFFFF_FFFF) as u32));
    }
    out
}

#[cfg(test)]
mod tests {
    use super::*;
    use crate::json;

    fn state() -> AppState {
        AppState::new(
            Config::default(),
            false,
            "http://127.0.0.1:8765/".into(),
            "/dev/ttyACM0".into(),
        )
    }

    #[test]
    fn snapshot_is_valid_json_with_expected_fields() {
        let mut st = state();
        st.refresh();
        let text = st.snapshot_json();
        let value = json::parse(&text).expect("valid json");
        assert_eq!(value.get("sim").unwrap().as_bool(), Some(false));
        assert!(value.get("ground").is_some());
        assert!(value.get("vehicle").is_some());
        assert!(value.get("actions").unwrap().get("fill_on").is_some());
        assert_eq!(
            value
                .get("config")
                .unwrap()
                .get("fill_seconds")
                .unwrap()
                .as_u64(),
            Some(60)
        );
        assert_eq!(
            value
                .get("fill")
                .unwrap()
                .get("duration_s")
                .unwrap()
                .as_u64(),
            Some(60)
        );
    }

    #[test]
    fn actions_are_disabled_without_vehicle() {
        let mut st = state();
        st.refresh();
        let value = json::parse(&st.snapshot_json()).unwrap();
        assert_eq!(
            value
                .get("actions")
                .unwrap()
                .get("fill_on")
                .unwrap()
                .get("enabled")
                .unwrap()
                .as_bool(),
            Some(false)
        );
    }

    #[test]
    fn log_is_bounded_and_ordered() {
        let mut st = state();
        for i in 0..MAX_LOG + 10 {
            st.push_log(Level::Info, Source::App, format!("msg {i}"), None);
        }
        assert_eq!(st.log.len(), MAX_LOG);
        assert_eq!(st.log.front().unwrap().message, "msg 10");
        assert!(st.log.back().unwrap().id > st.log.front().unwrap().id);
    }

    #[test]
    fn log_entry_serializes() {
        let entry = LogEntry {
            id: 1,
            epoch_ms: 0,
            level: Level::Error,
            source: Source::Vehicle,
            message: "x".into(),
            code: Some("link_timeout_safe"),
        };
        let value = json::parse(&entry.to_json()).unwrap();
        assert_eq!(value.get("level").unwrap().as_str(), Some("error"));
    }

    #[test]
    fn tokens_are_hex_and_unique() {
        let a = make_token();
        let b = make_token();
        assert_eq!(a.len(), 32);
        assert!(a.chars().all(|c| c.is_ascii_hexdigit()));
        assert_ne!(a, b);
    }

    #[test]
    fn controls_start_off_and_safe() {
        let mut st = state();
        st.refresh();
        let value = json::parse(&st.snapshot_json()).unwrap();
        let ctl = value.get("controls").unwrap();
        assert_eq!(ctl.get("fill").unwrap().as_str(), Some("OFF"));
        assert_eq!(ctl.get("srv").unwrap().as_str(), Some("OFF"));
        assert_eq!(ctl.get("ign").unwrap().as_str(), Some("SAFE"));
        assert_eq!(ctl.get("safe").unwrap().as_bool(), Some(true));
        assert_eq!(ctl.get("ready").unwrap().as_bool(), Some(false));
        assert_eq!(ctl.get("target"), Some(&json::Value::Null));
    }

    #[test]
    fn controls_follow_the_vehicle_only() {
        let mut st = state();
        st.vehicle.state = StateSnapshot {
            flags: flags::SRV_ON | flags::FILL_SEQ_COMPLETE,
            ..StateSnapshot::default()
        };
        st.controls.ready = true;
        st.refresh();
        assert_eq!(st.controls.state.srv.label(), "ON");
        assert_eq!(st.controls.state.fill.label(), "OFF");
        assert_eq!(st.controls.state.ign.label(), "SAFE");
        let value = json::parse(&st.snapshot_json()).unwrap();
        assert_eq!(
            value
                .get("controls")
                .unwrap()
                .get("summary")
                .unwrap()
                .as_str(),
            Some("FILL=OFF SRV=ON IGN=SAFE")
        );
        // The operator's desired state never overwrites the confirmed one.
        st.controls.target = Some(ControlState {
            fill: gse_proto::FillState::On,
            ..st.controls.state
        });
        st.refresh();
        assert!(
            st.controls.target.is_none(),
            "target clears with no pending"
        );
        assert_eq!(st.controls.state.srv.label(), "ON");
        assert!(!st.actions.fill_on.enabled);
    }

    #[test]
    fn confirmed_state_waits_for_the_ack() {
        let mut st = state();
        st.controls.ready = true;
        st.refresh();
        assert_eq!(st.controls.state, ControlState::SAFE);

        // The board has already driven the valve open and reports it, but the
        // command is still waiting for its ACK: the machine records only the
        // target and keeps the confirmed state at OFF.
        st.vehicle.state = StateSnapshot {
            flags: flags::FILL_OPEN,
            ..StateSnapshot::default()
        };
        st.controls.target = Some(ControlState {
            fill: gse_proto::FillState::On,
            ..ControlState::SAFE
        });
        st.pending = Some(PendingInfo {
            cmd_id: 1,
            action: "fill_on".into(),
            label: "FILL ON".into(),
            attempts: 1,
            sent_epoch_ms: 0,
            timeout_ms: 800,
        });
        st.refresh();
        assert_eq!(st.controls.state.fill.label(), "OFF");
        assert_eq!(st.controls.target.unwrap().fill.label(), "ON");
        // The gates already see the true output, so nothing unsafe is offered.
        assert!(!st.actions.srv_on.enabled);
        assert!(!st.actions.fill_on.enabled);

        // ACK: the command is no longer pending and the transition is confirmed.
        st.pending = None;
        st.refresh();
        assert_eq!(st.controls.state.fill.label(), "ON");
        assert!(st.controls.target.is_none());
    }

    #[test]
    fn state_json_contains_flags() {
        let snap = StateSnapshot {
            flags: flags::FILL_OPEN | flags::FILL_SEQ_COMPLETE,
            faults: faults::CRC_ERROR,
            ..StateSnapshot::default()
        };
        let value = json::parse(&AppState::state_json(&snap)).unwrap();
        assert_eq!(value.get("fill").unwrap().as_bool(), Some(true));
        assert_eq!(
            value.get("fill_seq_complete").unwrap().as_bool(),
            Some(true)
        );
        let flags = json::parse(&AppState::flags_json(&snap)).unwrap();
        assert!(format!("{flags:?}").contains("FILL OPEN"));
        let faults = json::parse(&AppState::faults_json(&snap)).unwrap();
        assert!(format!("{faults:?}").contains("CRC ERROR"));
    }
}
