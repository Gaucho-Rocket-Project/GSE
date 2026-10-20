//! The link engine: serial/TCP connection management, protocol handling,
//! command pacing, retries, fill timer and safety supervision.
//!
//! Runs on its own thread. All UI state lives in [`Shared`]; the browser gets
//! snapshots over SSE and sends [`Request`]s through a channel.

use std::io;
use std::sync::mpsc::Receiver;
use std::sync::Arc;
use std::thread;
use std::time::{Duration, Instant};

use gse_proto::{
    cmd_flags, encode_message, Ack, Command, Control, DecodeStats, Endpoint, Event, EventCode,
    Frame, FrameDecoder, Heartbeat, Hello, LinkStat, Message, Ping, Pong, Reason, StateSnapshot,
    ACTION_OFF, ACTION_ON,
};

use crate::clock;
use crate::logging::{Level, Source};
use crate::safety;
use crate::serial;
use crate::state::{GroundStatus, PendingInfo, Shared, VehicleStatus};
use crate::transport::{is_tcp_spec, open_link, ByteLink};

const TICK_MS: u64 = 10;
const PING_ONLINE_MS: u64 = 2_000;
const PING_FAST_MS: u64 = 500;
const HEARTBEAT_MS: u64 = 1_000;
const GROUND_TIMEOUT_MS: u64 = 3_000;
const VEHICLE_TIMEOUT_MS: u64 = 2_000;
const ACK_RETRY_MS: u64 = 800;
const ACK_MAX_ATTEMPTS: u32 = 5;
const RECONNECT_DELAY_MS: u64 = 1_500;
const SCAN_RETRY_MS: u64 = 2_500;
const PUBLISH_MS: u64 = 100;
const PROBE_TIMEOUT_MS: u64 = 700;

#[derive(Clone, Copy, Debug, PartialEq, Eq)]
pub enum UserAction {
    FillOn,
    FillOff,
    SrvOn,
    SrvOff,
    IgnPwr,
    IgnSafe,
    ResetFill,
    ClearFaults,
    Estop,
}

impl UserAction {
    pub fn from_id(id: &str) -> Option<Self> {
        Some(match id {
            "fill_on" => UserAction::FillOn,
            "fill_off" => UserAction::FillOff,
            "srv_on" => UserAction::SrvOn,
            "srv_off" => UserAction::SrvOff,
            "ign_pwr" => UserAction::IgnPwr,
            "ign_safe" => UserAction::IgnSafe,
            "reset_fill" => UserAction::ResetFill,
            "clear_faults" => UserAction::ClearFaults,
            "estop" => UserAction::Estop,
            _ => return None,
        })
    }

    pub fn id(self) -> &'static str {
        match self {
            UserAction::FillOn => "fill_on",
            UserAction::FillOff => "fill_off",
            UserAction::SrvOn => "srv_on",
            UserAction::SrvOff => "srv_off",
            UserAction::IgnPwr => "ign_pwr",
            UserAction::IgnSafe => "ign_safe",
            UserAction::ResetFill => "reset_fill",
            UserAction::ClearFaults => "clear_faults",
            UserAction::Estop => "estop",
        }
    }

    pub fn label(self) -> &'static str {
        match self {
            UserAction::FillOn => "FILL ON",
            UserAction::FillOff => "FILL OFF",
            UserAction::SrvOn => "SRV ON",
            UserAction::SrvOff => "SRV OFF",
            UserAction::IgnPwr => "IGN PWR",
            UserAction::IgnSafe => "IGN SAFE",
            UserAction::ResetFill => "RESET FILL SEQ",
            UserAction::ClearFaults => "CLEAR FAULTS",
            UserAction::Estop => "EMERGENCY STOP",
        }
    }

    fn command(self, cmd_id: u16, fill_seconds: u16, flags: u8) -> Command {
        match self {
            UserAction::FillOn => Command {
                cmd_id,
                control: Control::Fill,
                action: ACTION_ON,
                arg_s: fill_seconds,
                flags,
            },
            UserAction::FillOff => Command {
                cmd_id,
                control: Control::Fill,
                action: ACTION_OFF,
                arg_s: 0,
                flags,
            },
            UserAction::SrvOn => Command {
                cmd_id,
                control: Control::Srv,
                action: ACTION_ON,
                arg_s: 0,
                flags,
            },
            UserAction::SrvOff => Command {
                cmd_id,
                control: Control::Srv,
                action: ACTION_OFF,
                arg_s: 0,
                flags,
            },
            UserAction::IgnPwr => Command {
                cmd_id,
                control: Control::Ign,
                action: ACTION_ON,
                arg_s: 0,
                flags,
            },
            UserAction::IgnSafe => Command {
                cmd_id,
                control: Control::Ign,
                action: ACTION_OFF,
                arg_s: 0,
                flags,
            },
            UserAction::ResetFill => Command {
                cmd_id,
                control: Control::System,
                action: gse_proto::SYSTEM_ACTION_RESET_FILL,
                arg_s: 0,
                flags,
            },
            UserAction::ClearFaults => Command {
                cmd_id,
                control: Control::System,
                action: gse_proto::SYSTEM_ACTION_CLEAR_FAULTS,
                arg_s: 0,
                flags,
            },
            UserAction::Estop => Command {
                cmd_id,
                control: Control::System,
                action: gse_proto::SYSTEM_ACTION_SAFE_ALL,
                arg_s: 0,
                flags: flags | cmd_flags::ESTOP,
            },
        }
    }
}

#[derive(Clone, Debug, Default)]
pub struct ConfigPatch {
    pub fill_seconds: Option<u16>,
    pub port: Option<String>,
    pub baud: Option<u32>,
    pub confirm_ign: Option<bool>,
    pub sound: Option<bool>,
}

#[derive(Debug)]
pub enum Request {
    Action(UserAction),
    SetConfig(ConfigPatch),
    SetShortcut { action: String, combo: String },
    ResetShortcuts,
    Connect,
    Disconnect,
    ClearLog,
    Shutdown,
}

#[derive(Debug)]
struct PendingCmd {
    info: PendingInfo,
    command: Command,
    due: Instant,
    auto: bool,
}

pub struct Engine {
    shared: Arc<Shared>,
    requests: Receiver<Request>,
    link: Option<Box<dyn ByteLink>>,
    decoder: FrameDecoder,
    tx_seq: u8,
    nonce: u32,
    next_cmd_id: u16,
    pending: Option<PendingCmd>,
    started: Instant,
    last_connect_attempt: Option<Instant>,
    last_ground_rx: Option<Instant>,
    last_vehicle_rx: Option<Instant>,
    last_ping: Option<Instant>,
    vehicle_ping: Option<(u32, Instant)>,
    last_heartbeat: Option<Instant>,
    last_publish: Option<Instant>,
    last_scan_log: Option<Instant>,
    last_decoder_stats: DecodeStats,
    fill_deadline: Option<Instant>,
    fill_started: Option<Instant>,
    auto_off_sent: bool,
    prev_state: StateSnapshot,
    prev_vehicle_uptime: Option<u32>,
    ground_announced: bool,
    vehicle_announced: bool,
    was_vehicle_online: bool,
    radio_warned: bool,
    port_override: Option<String>,
    verbose: bool,
    shutting_down: bool,
}

impl Engine {
    pub fn new(
        shared: Arc<Shared>,
        requests: Receiver<Request>,
        port_override: Option<String>,
        verbose: bool,
    ) -> Self {
        let started = Instant::now();
        Engine {
            shared,
            requests,
            link: None,
            decoder: FrameDecoder::new(),
            tx_seq: 0,
            nonce: 1,
            next_cmd_id: 1,
            pending: None,
            started,
            last_connect_attempt: None,
            last_ground_rx: None,
            last_vehicle_rx: None,
            last_ping: None,
            vehicle_ping: None,
            last_heartbeat: None,
            last_publish: None,
            last_scan_log: None,
            last_decoder_stats: DecodeStats::default(),
            fill_deadline: None,
            fill_started: None,
            auto_off_sent: false,
            prev_state: StateSnapshot::default(),
            prev_vehicle_uptime: None,
            ground_announced: false,
            vehicle_announced: false,
            was_vehicle_online: false,
            radio_warned: false,
            port_override,
            verbose,
            shutting_down: false,
        }
    }

    pub fn run(mut self) {
        self.shared.log(
            Level::Info,
            Source::App,
            "Ground station ready - scanning for the ground NUCLEO-H753ZI".to_string(),
            Some("boot"),
        );
        loop {
            let tick_start = Instant::now();
            while let Ok(request) = self.requests.try_recv() {
                self.handle_request(request);
            }
            if self.shutting_down {
                break;
            }
            if self.link.is_none() {
                self.maybe_connect(tick_start);
            } else {
                self.service_link(tick_start);
            }
            self.update_ui();
            self.maybe_publish();
            let elapsed = tick_start.elapsed();
            if elapsed < Duration::from_millis(TICK_MS) {
                thread::sleep(Duration::from_millis(TICK_MS) - elapsed);
            }
        }
        if let Some(link) = self.link.as_mut() {
            let _ = link.write_all(&[]);
        }
    }

    // ---------------------------------------------------------------------
    // Requests
    // ---------------------------------------------------------------------

    fn handle_request(&mut self, request: Request) {
        match request {
            Request::Action(action) => self.user_action(action),
            Request::SetConfig(patch) => self.apply_config(patch),
            Request::SetShortcut { action, combo } => self.set_shortcut(&action, &combo),
            Request::ResetShortcuts => {
                {
                    let mut st = self.shared.lock();
                    st.config.reset_shortcuts();
                }
                self.shared.save_config();
                self.shared.log(
                    Level::Ok,
                    Source::Operator,
                    "Keyboard shortcuts reset to defaults".to_string(),
                    Some("shortcuts"),
                );
            }
            Request::Connect => {
                self.last_connect_attempt = None;
                self.shared.log(
                    Level::Info,
                    Source::Operator,
                    "Manual reconnect requested".to_string(),
                    None,
                );
            }
            Request::Disconnect => {
                self.disconnect("Disconnected by operator", Level::Warn);
            }
            Request::ClearLog => {
                let mut st = self.shared.lock();
                st.log.clear();
                st.revision = st.revision.wrapping_add(1);
                drop(st);
                self.shared.log(
                    Level::Info,
                    Source::Operator,
                    "Event log cleared".to_string(),
                    None,
                );
            }
            Request::Shutdown => {
                self.shutting_down = true;
            }
        }
    }

    fn apply_config(&mut self, patch: ConfigPatch) {
        let mut reconnect = false;
        let mut notes = Vec::new();
        {
            let mut st = self.shared.lock();
            let old = st.config.clone();
            if let Some(secs) = patch.fill_seconds {
                let secs = secs.clamp(gse_proto::MIN_FILL_SECONDS, gse_proto::MAX_FILL_SECONDS);
                st.config.fill_seconds = secs;
                if !st.fill.active {
                    st.fill.duration_s = secs;
                }
                notes.push(format!("fill time set to {secs} s"));
            }
            if let Some(port) = patch.port {
                if port != st.config.port {
                    st.config.port = port;
                    reconnect = true;
                }
            }
            if let Some(baud) = patch.baud {
                if baud != st.config.baud {
                    st.config.baud = baud;
                    reconnect = true;
                }
            }
            if let Some(confirm) = patch.confirm_ign {
                st.config.confirm_ign = confirm;
                notes.push(format!(
                    "ignition confirmation {}",
                    if confirm { "enabled" } else { "disabled" }
                ));
            }
            if let Some(sound) = patch.sound {
                st.config.sound = sound;
                notes.push(format!(
                    "audio alerts {}",
                    if sound { "enabled" } else { "disabled" }
                ));
            }
            st.config.sanitize();
            if st.config != old {
                st.revision = st.revision.wrapping_add(1);
            }
        }
        self.shared.save_config();
        if !notes.is_empty() {
            self.shared.log(
                Level::Ok,
                Source::Operator,
                format!("Settings updated: {}", notes.join(", ")),
                Some("config"),
            );
        }
        if reconnect {
            let (port, baud) = {
                let st = self.shared.lock();
                (st.config.port.clone(), st.config.baud)
            };
            self.disconnect(
                &format!("Port changed to {port} @ {baud} - reconnecting"),
                Level::Info,
            );
            self.last_connect_attempt = None;
        }
    }

    fn set_shortcut(&mut self, action: &str, combo: &str) {
        let mut st = self.shared.lock();
        match st.config.set_shortcut(action, combo) {
            Ok(()) => {
                st.revision = st.revision.wrapping_add(1);
                drop(st);
                self.shared.save_config();
                self.shared.log(
                    Level::Ok,
                    Source::Operator,
                    format!("Shortcut for {action} set to {combo}"),
                    Some("shortcut"),
                );
            }
            Err(e) => {
                drop(st);
                self.shared.log(Level::Warn, Source::Operator, e, Some("shortcut"));
            }
        }
    }

    fn user_action(&mut self, action: UserAction) {
        let (gate, fill_seconds) = {
            let st = self.shared.lock();
            let inputs = safety::Inputs {
                ground_online: st.ground.status == GroundStatus::Online,
                vehicle_online: st.vehicle.status == VehicleStatus::Online,
                pending: st.pending.is_some(),
                state: st.vehicle.state,
            };
            let gates = safety::evaluate(&inputs);
            (
                gates
                    .get(action.id())
                    .cloned()
                    .unwrap_or_default(),
                st.config.fill_seconds,
            )
        };

        if action == UserAction::Estop {
            // Never blocked by a pending command, but requires a live link.
            if self.link.is_none() || !gate.enabled {
                self.shared.log(
                    Level::Error,
                    Source::Operator,
                    format!(
                        "E-STOP NOT TRANSMITTED: {}",
                        if gate.reason.is_empty() {
                            "vehicle unreachable".to_string()
                        } else {
                            gate.reason.clone()
                        }
                    ),
                    Some("estop_failed"),
                );
                return;
            }
            if let Some(pending) = self.pending.take() {
                let mut st = self.shared.lock();
                st.pending = None;
                drop(st);
                self.shared.log(
                    Level::Warn,
                    Source::App,
                    format!("{} superseded by E-STOP", pending.info.label),
                    None,
                );
            }
            self.fill_deadline = None;
            self.fill_started = None;
            self.auto_off_sent = false;
            self.send_action(action, fill_seconds, cmd_flags::ESTOP, false);
            return;
        }

        if !gate.enabled {
            self.shared.log(
                Level::Warn,
                Source::Operator,
                format!("{} blocked: {}", action.label(), gate.reason),
                Some("blocked"),
            );
            return;
        }
        if self.link.is_none() {
            self.shared.log(
                Level::Warn,
                Source::Operator,
                format!("{} blocked: no link", action.label()),
                None,
            );
            return;
        }
        self.send_action(action, fill_seconds, 0, false);
    }

    fn send_action(&mut self, action: UserAction, fill_seconds: u16, flags: u8, auto: bool) {
        let cmd_id = self.next_cmd_id;
        self.next_cmd_id = self.next_cmd_id.wrapping_add(1);
        let command = action.command(cmd_id, fill_seconds, flags);
        let now = Instant::now();
        let label = if auto {
            format!("{} (auto)", action.label())
        } else {
            action.label().to_string()
        };
        if !self.send(Message::Command(command), Endpoint::Vehicle) {
            self.shared.log(
                Level::Error,
                Source::App,
                "Could not send command - link is down".to_string(),
                Some("send_failed"),
            );
            return;
        }
        let info = PendingInfo {
            cmd_id,
            action: action.id().to_string(),
            label: label.clone(),
            attempts: 1,
            sent_epoch_ms: clock::unix_ms(),
            timeout_ms: ACK_RETRY_MS,
        };
        {
            let mut st = self.shared.lock();
            st.pending = Some(info.clone());
            st.stats.commands_sent = st.stats.commands_sent.wrapping_add(1);
            if action == UserAction::FillOff && auto {
                st.fill.auto_off_pending = true;
            }
        }
        self.pending = Some(PendingCmd {
            info,
            command,
            due: now + Duration::from_millis(ACK_RETRY_MS),
            auto,
        });
        let message = match action {
            UserAction::FillOn => {
                format!("{} requested ({} s fill)", label, fill_seconds)
            }
            UserAction::Estop => "EMERGENCY STOP - all outputs SAFE".to_string(),
            _ => format!("{label} requested"),
        };
        let level = if action == UserAction::Estop {
            Level::Warn
        } else {
            Level::Info
        };
        self.shared
            .log(level, Source::Operator, message, Some(action.id()));
        if action == UserAction::FillOff && auto {
            self.shared.log(
                Level::Warn,
                Source::App,
                "FILL timer expired - automatic FILL OFF sent".to_string(),
                Some("fill_auto_off"),
            );
        }
    }

    // ---------------------------------------------------------------------
    // Connection management
    // ---------------------------------------------------------------------

    fn connection_target(&self) -> String {
        if let Some(port) = &self.port_override {
            return port.clone();
        }
        self.shared.lock().config.port.clone()
    }

    fn maybe_connect(&mut self, now: Instant) {
        let last = match self.last_connect_attempt {
            Some(t) => t,
            None => self.started - Duration::from_secs(10),
        };
        let target = self.connection_target();
        let auto = target == "auto";
        let delay = if auto { SCAN_RETRY_MS } else { RECONNECT_DELAY_MS };
        if now.duration_since(last) < Duration::from_millis(delay) {
            return;
        }
        self.last_connect_attempt = Some(now);
        let baud = self.shared.lock().config.baud;
        {
            let mut st = self.shared.lock();
            st.ground.status = if auto {
                GroundStatus::Scanning
            } else {
                GroundStatus::Connecting
            };
            st.ground.port = if auto { "auto".into() } else { target.clone() };
            st.ground.error = None;
            st.scan_note = None;
        }
        self.shared.publish();

        if auto {
            let ports = serial::list_ports();
            if ports.is_empty() {
                self.scan_failed("No serial ports found. Connect the ground board with a USB data cable.");
                return;
            }
            let mut tried = Vec::new();
            for port in &ports {
                tried.push(port.label.clone());
                if let Some(link) = probe_port(&port.path, baud, Endpoint::Ground, self.verbose) {
                    self.on_connected(link, &port.path);
                    return;
                }
            }
            self.scan_failed(&format!(
                "No ground board responded on {} port(s): {}",
                tried.len(),
                tried.join(", ")
            ));
        } else if is_tcp_spec(&target) {
            match open_link(&target, baud) {
                Ok(mut link) => {
                    if probe_existing(&mut *link, Endpoint::Ground, PROBE_TIMEOUT_MS) {
                        self.on_connected(link, &target);
                    } else {
                        self.scan_failed(&format!("No ground board responded on {target}"));
                    }
                }
                Err(e) => self.scan_failed(&format!("Cannot connect to {target}: {e}")),
            }
        } else {
            match probe_port(&target, baud, Endpoint::Ground, self.verbose) {
                Some(link) => self.on_connected(link, &target),
                None => self.scan_failed(&format!(
                    "No response from {target}. Check the cable, baud rate and permissions."
                )),
            }
        }
    }

    fn scan_failed(&mut self, note: &str) {
        {
            let mut st = self.shared.lock();
            st.ground.status = GroundStatus::Disconnected;
            st.ground.error = Some(note.to_string());
            st.scan_note = Some(note.to_string());
            st.vehicle.status = VehicleStatus::Offline;
        }
        let should_log = self
            .last_scan_log
            .map(|t| t.elapsed() >= Duration::from_secs(20))
            .unwrap_or(true);
        if should_log {
            self.last_scan_log = Some(Instant::now());
            self.shared
                .log(Level::Warn, Source::App, note.to_string(), Some("scan"));
        }
        self.shared.publish();
    }

    fn on_connected(&mut self, link: Box<dyn ByteLink>, port: &str) {
        self.link = Some(link);
        self.decoder = FrameDecoder::new();
        self.last_decoder_stats = DecodeStats::default();
        self.tx_seq = 0;
        self.last_ground_rx = Some(Instant::now());
        self.last_vehicle_rx = None;
        self.ground_announced = false;
        self.vehicle_announced = false;
        self.was_vehicle_online = false;
        self.pending = None;
        self.fill_deadline = None;
        self.fill_started = None;
        self.auto_off_sent = false;
        self.prev_state = StateSnapshot::default();
        self.prev_vehicle_uptime = None;
        {
            let mut st = self.shared.lock();
            st.ground.status = GroundStatus::Connecting;
            st.ground.port = port.to_string();
            st.ground.error = None;
            st.ground.fw = None;
            st.ground.device_id = None;
            st.ground.link = LinkStat::default();
            st.ground.have_link_stat = false;
            st.vehicle.status = VehicleStatus::Offline;
            st.pending = None;
            st.fill.auto_off_pending = false;
            st.fill.ends_at_epoch_ms = 0;
            st.stats.reconnects = st.stats.reconnects.wrapping_add(1);
            st.revision = st.revision.wrapping_add(1);
        }
        self.shared.log(
            Level::Ok,
            Source::App,
            format!("Port {port} opened - detecting ground board"),
            Some("port_open"),
        );
        self.shared.publish();
        // Requirement: as soon as the ground NUCLEO is detected, ping the
        // onboard NUCLEO.
        self.send_ping(Endpoint::Ground);
        self.send_ping(Endpoint::Vehicle);
        self.last_ping = Some(Instant::now());
    }

    fn disconnect(&mut self, reason: &str, level: Level) {
        let had_link = self.link.is_some();
        let was_fill = {
            let st = self.shared.lock();
            st.vehicle.state.fill_open()
        };
        self.link = None;
        self.pending = None;
        self.fill_deadline = None;
        self.auto_off_sent = false;
        self.fill_started = None;
        self.decoder.reset();
        {
            let mut st = self.shared.lock();
            st.ground.status = GroundStatus::Disconnected;
            st.ground.error = Some(reason.to_string());
            st.vehicle.status = VehicleStatus::Offline;
            st.pending = None;
            st.fill.auto_off_pending = false;
            st.fill.ends_at_epoch_ms = 0;
            st.stats.reconnects = st.stats.reconnects.wrapping_add(1);
            st.revision = st.revision.wrapping_add(1);
        }
        if had_link {
            self.shared
                .log(level, Source::App, reason.to_string(), Some("link_lost"));
            if was_fill {
                self.shared.log(
                    Level::Error,
                    Source::App,
                    "FILL was open when the link dropped. The vehicle firmware closes the valve 2 s after the fill deadline without hearing from the ground station.".to_string(),
                    Some("fill_link_lost"),
                );
            }
        }
        self.shared.publish();
    }

    // ---------------------------------------------------------------------
    // Link servicing
    // ---------------------------------------------------------------------

    fn service_link(&mut self, now: Instant) {
        let mut buf = [0u8; 1024];
        let mut frames: Vec<Frame> = Vec::new();
        // Drain the transport without holding a borrow across disconnect().
        let mut read_error: Option<io::Error> = None;
        {
            let link = match self.link.as_mut() {
                Some(l) => l,
                None => return,
            };
            for _ in 0..8 {
                match link.read(&mut buf) {
                    Ok(0) => break,
                    Ok(n) => {
                        for &byte in &buf[..n] {
                            if let Some(frame) = self.decoder.push(byte) {
                                frames.push(frame);
                            }
                        }
                    }
                    Err(e) if e.kind() == io::ErrorKind::Interrupted => break,
                    Err(e) => {
                        read_error = Some(e);
                        break;
                    }
                }
            }
        }
        if let Some(e) = read_error {
            self.disconnect(&format!("Serial link lost: {e}"), Level::Error);
            return;
        }

        for frame in frames {
            self.handle_frame(frame, now);
        }

        let stats = self.decoder.stats();
        let crc_delta = stats.crc_errors.wrapping_sub(self.last_decoder_stats.crc_errors);
        let malformed_delta = stats
            .malformed
            .wrapping_sub(self.last_decoder_stats.malformed);
        self.last_decoder_stats = stats;
        if crc_delta > 0 || malformed_delta > 0 {
            let mut st = self.shared.lock();
            st.stats.crc_errors = st.stats.crc_errors.wrapping_add(crc_delta as u64);
            st.stats.malformed = st.stats.malformed.wrapping_add(malformed_delta as u64);
        }

        // Heartbeat keeps the vehicle's 5 s link-loss failsafe quiet.
        if self
            .last_heartbeat
            .map(|t| now.duration_since(t) >= Duration::from_millis(HEARTBEAT_MS))
            .unwrap_or(true)
        {
            self.last_heartbeat = Some(now);
            let uptime = self.started.elapsed().as_millis() as u32;
            self.send(
                Message::Heartbeat(Heartbeat {
                    uptime_ms: uptime,
                    mode: 0,
                }),
                Endpoint::Vehicle,
            );
        }

        // Ping cadence: fast while waiting for the first PONG.
        let vehicle_online = self.shared.lock().vehicle.status == VehicleStatus::Online;
        let period = if vehicle_online {
            PING_ONLINE_MS
        } else {
            PING_FAST_MS
        };
        if self
            .last_ping
            .map(|t| now.duration_since(t) >= Duration::from_millis(period))
            .unwrap_or(true)
        {
            self.last_ping = Some(now);
            self.send_ping(Endpoint::Vehicle);
        }

        // Command retries.
        let retry_now = self
            .pending
            .as_ref()
            .map(|p| (p.due, p.info.attempts, p.command, p.info.label.clone()))
            .filter(|(due, _, _, _)| now >= *due);
        if let Some((_, attempts, command, label)) = retry_now {
            if attempts >= ACK_MAX_ATTEMPTS {
                self.pending = None;
                {
                    let mut st = self.shared.lock();
                    st.pending = None;
                    st.revision = st.revision.wrapping_add(1);
                }
                self.shared.log(
                    Level::Error,
                    Source::App,
                    format!("No ACK from vehicle for {label} after {ACK_MAX_ATTEMPTS} attempts - command state unknown"),
                    Some("ack_timeout"),
                );
            } else if let Some(pending) = self.pending.as_mut() {
                pending.info.attempts += 1;
                pending.info.sent_epoch_ms = clock::unix_ms();
                pending.due = now + Duration::from_millis(ACK_RETRY_MS);
                let info = pending.info.clone();
                if self.send(Message::Command(command), Endpoint::Vehicle) {
                    let mut st = self.shared.lock();
                    st.stats.retries = st.stats.retries.wrapping_add(1);
                    st.pending = Some(info);
                }
            }
        }

        // Fill auto-off (host side, authoritative at the configured deadline).
        self.service_fill_auto_off(now);

        // Ground board liveness.
        if let Some(last) = self.last_ground_rx {
            if now.duration_since(last) > Duration::from_millis(GROUND_TIMEOUT_MS) {
                self.disconnect(
                    "Ground board stopped responding (no HELLO for 3 s)",
                    Level::Error,
                );
                return;
            }
        }

        // Vehicle liveness.
        let vehicle_age = self.last_vehicle_rx.map(|t| now.duration_since(t));
        let vehicle_online = self.shared.lock().vehicle.status == VehicleStatus::Online;
        if vehicle_online
            && vehicle_age
                .map(|age| age > Duration::from_millis(VEHICLE_TIMEOUT_MS))
                .unwrap_or(true)
        {
            let pending_label = {
                let mut st = self.shared.lock();
                st.vehicle.status = VehicleStatus::Pinging;
                st.revision = st.revision.wrapping_add(1);
                let label = st.pending.as_ref().map(|p| p.label.clone());
                st.pending = None;
                label
            };
            self.pending = None;
            if let Some(label) = pending_label {
                self.shared.log(
                    Level::Warn,
                    Source::App,
                    format!("Vehicle link lost while waiting for ACK of {label}"),
                    None,
                );
            }
            self.shared.log(
                Level::Warn,
                Source::Vehicle,
                "Vehicle link lost - re-pinging the onboard board".to_string(),
                Some("vehicle_lost"),
            );
        }

        // Radio health from the bridge telemetry (warn once per outage).
        let (link_stat, have_stat) = {
            let st = self.shared.lock();
            (st.ground.link, st.ground.have_link_stat)
        };
        if have_stat {
            let age = link_stat.last_radio_age_ms;
            if age > 2_500 && vehicle_online && !self.radio_warned {
                self.radio_warned = true;
                self.shared.log(
                    Level::Warn,
                    Source::Ground,
                    format!("Radio link quiet for {:.1} s - check the SX1281 modules", age as f64 / 1000.0),
                    Some("radio_warn"),
                );
            } else if age < 1_000 {
                self.radio_warned = false;
            }
        }
    }

    fn service_fill_auto_off(&mut self, now: Instant) {
        let (fill_open, timer_active, remaining, vehicle_online, pending, auto_off_sent) = {
            let st = self.shared.lock();
            (
                st.vehicle.state.fill_open(),
                st.vehicle.state.fill_timer_active(),
                st.vehicle.state.fill_remaining_ms,
                st.vehicle.status == VehicleStatus::Online,
                st.pending.is_some(),
                self.auto_off_sent,
            )
        };
        if !fill_open || !vehicle_online || pending || auto_off_sent || !timer_active {
            return;
        }
        let deadline_reached = self
            .fill_deadline
            .map(|d| now >= d)
            .unwrap_or(false)
            || remaining == 0;
        if !deadline_reached {
            return;
        }
        self.auto_off_sent = true;
        let fill_seconds = self.shared.lock().config.fill_seconds;
        let mut st = self.shared.lock();
        st.fill.auto_off_pending = true;
        st.revision = st.revision.wrapping_add(1);
        drop(st);
        let _ = fill_seconds;
        self.send_action(UserAction::FillOff, 0, cmd_flags::AUTO, true);
    }

    fn send_ping(&mut self, dst: Endpoint) {
        self.nonce = self.nonce.wrapping_add(1);
        let uptime = self.started.elapsed().as_millis() as u32;
        if dst == Endpoint::Vehicle {
            self.vehicle_ping = Some((self.nonce, Instant::now()));
        }
        self.send(
            Message::Ping(Ping {
                nonce: self.nonce,
                host_ms: uptime,
            }),
            dst,
        );
    }

    fn send(&mut self, message: Message, dst: Endpoint) -> bool {
        let mut buf = [0u8; Frame::MAX_ENCODED];
        let seq = self.tx_seq;
        let n = match encode_message(&message, Endpoint::Host, dst, seq, &mut buf) {
            Some(n) => n,
            None => return false,
        };
        let result = {
            match self.link.as_mut() {
                Some(link) => link.write_all(&buf[..n]),
                None => return false,
            }
        };
        match result {
            Ok(()) => {
                self.tx_seq = seq.wrapping_add(1);
                {
                    let mut st = self.shared.lock();
                    st.stats.tx_frames = st.stats.tx_frames.wrapping_add(1);
                }
                if self.verbose
                    && !matches!(
                        message,
                        Message::State(_) | Message::Heartbeat(_) | Message::Ping(_)
                    )
                {
                    self.shared.log(
                        Level::Info,
                        Source::App,
                        format!("TX {} -> {dst}", message.msg_type()),
                        Some("tx"),
                    );
                }
                true
            }
            Err(e) => {
                self.disconnect(&format!("Write failed: {e}"), Level::Error);
                false
            }
        }
    }

    // ---------------------------------------------------------------------
    // Frame handling
    // ---------------------------------------------------------------------

    fn handle_frame(&mut self, frame: Frame, now: Instant) {
        if frame.dst != Endpoint::Host && frame.dst != Endpoint::Broadcast {
            return;
        }
        {
            let mut st = self.shared.lock();
            st.stats.rx_frames = st.stats.rx_frames.wrapping_add(1);
        }
        let message = match frame.message() {
            Some(m) => m,
            None => {
                if self.verbose {
                    self.shared.log(
                        Level::Info,
                        Source::App,
                        format!("Invalid body for {}", frame.msg_type),
                        None,
                    );
                }
                return;
            }
        };
        if self.verbose && !matches!(message, Message::State(_) | Message::Heartbeat(_)) {
            self.shared.log(
                Level::Info,
                Source::App,
                format!("RX {:?}", message.describe()),
                Some("rx"),
            );
        }
        match message {
            Message::Hello(hello) => self.handle_hello(hello),
            Message::Pong(pong) => self.handle_pong(frame.src, pong, now),
            Message::State(state) => {
                if frame.src == Endpoint::Vehicle {
                    self.note_vehicle_frame(now);
                    self.apply_state(state);
                }
            }
            Message::Ack(ack) => {
                if frame.src == Endpoint::Vehicle {
                    self.note_vehicle_frame(now);
                    self.handle_ack(ack);
                }
            }
            Message::Event(event) => {
                if frame.src == Endpoint::Vehicle {
                    self.note_vehicle_frame(now);
                    self.handle_event(event);
                }
            }
            Message::LinkStat(stat) => {
                if frame.src != Endpoint::Ground {
                    return;
                }
                self.last_ground_rx = Some(now);
                let mut st = self.shared.lock();
                st.ground.link = stat;
                st.ground.have_link_stat = true;
                if st.ground.status != GroundStatus::Online {
                    st.ground.status = GroundStatus::Online;
                }
                st.ground.last_seen_epoch_ms = clock::unix_ms();
            }
            Message::Heartbeat(_) => {}
            Message::Ping(_) => {}
            Message::Command(_) => {}
        }
    }

    fn handle_hello(&mut self, hello: Hello) {
        let fw = hello.fw_version().to_string();
        match hello.role {
            Endpoint::Ground => {
                self.last_ground_rx = Some(Instant::now());
                let first = {
                    let mut st = self.shared.lock();
                    let first = !st.ground.have_link_stat && st.ground.fw.is_none();
                    st.ground.status = GroundStatus::Online;
                    st.ground.protocol = hello.protocol;
                    st.ground.fw = Some(fw.clone());
                    st.ground.device_id = Some(hello.device_id);
                    st.ground.last_seen_epoch_ms = clock::unix_ms();
                    st.ground.error = None;
                    st.revision = st.revision.wrapping_add(1);
                    first
                };
                if first && !self.ground_announced {
                    self.ground_announced = true;
                    self.shared.log(
                        Level::Ok,
                        Source::Ground,
                        format!(
                            "Ground board detected (fw {fw}, id {:08X}) - pinging vehicle",
                            hello.device_id
                        ),
                        Some("ground_detected"),
                    );
                }
            }
            Endpoint::Vehicle => {
                self.note_vehicle_frame(Instant::now());
                {
                    let mut st = self.shared.lock();
                    st.vehicle.fw = Some(fw.clone());
                    st.vehicle.device_id = Some(hello.device_id);
                }
                if !self.vehicle_announced {
                    self.vehicle_announced = true;
                    self.shared.log(
                        Level::Ok,
                        Source::Vehicle,
                        format!("Vehicle firmware {fw} online (id {:08X})", hello.device_id),
                        Some("vehicle_hello"),
                    );
                }
            }
            _ => {}
        }
    }

    fn handle_pong(&mut self, source: Endpoint, pong: Pong, now: Instant) {
        if source == Endpoint::Ground {
            self.last_ground_rx = Some(now);
            let mut st = self.shared.lock();
            st.ground.status = GroundStatus::Online;
            st.ground.last_seen_epoch_ms = clock::unix_ms();
            st.ground.error = None;
            return;
        }
        self.note_vehicle_frame(now);
        let rtt = match self.vehicle_ping {
            Some((nonce, sent)) if nonce == pong.nonce => {
                Some(now.duration_since(sent).as_millis() as u32)
            }
            _ => None,
        };
        {
            let mut st = self.shared.lock();
            st.vehicle.rtt_ms = rtt;
            st.vehicle.status = VehicleStatus::Online;
            st.revision = st.revision.wrapping_add(1);
        }
        if !self.was_vehicle_online {
            self.was_vehicle_online = true;
            self.shared.log(
                Level::Ok,
                Source::Vehicle,
                format!(
                    "Vehicle online - PONG received{}",
                    rtt.map(|r| format!(" ({r} ms round trip)")).unwrap_or_default()
                ),
                Some("vehicle_online"),
            );
        }
        self.apply_state(pong.state);
    }

    fn note_vehicle_frame(&mut self, now: Instant) {
        self.last_vehicle_rx = Some(now);
        let mut st = self.shared.lock();
        st.vehicle.last_seen_epoch_ms = clock::unix_ms();
        if st.vehicle.status != VehicleStatus::Online {
            st.vehicle.status = VehicleStatus::Online;
        }
        st.revision = st.revision.wrapping_add(1);
    }

    fn apply_state(&mut self, state: StateSnapshot) {
        let previous = self.prev_state;
        let mut fill_closed = false;
        let mut fill_opened = false;
        {
            let mut st = self.shared.lock();
            st.vehicle.state = state;
            st.vehicle.last_seen_epoch_ms = clock::unix_ms();
            st.revision = st.revision.wrapping_add(1);
            if !previous.fill_open() && state.fill_open() {
                fill_opened = true;
            }
            if previous.fill_open() && !state.fill_open() {
                fill_closed = true;
            }
        }
        if fill_opened {
            self.fill_started = Some(Instant::now());
            if let Some(deadline) = self.fill_deadline {
                let remaining = deadline
                    .saturating_duration_since(Instant::now())
                    .as_millis() as u64;
                let mut st = self.shared.lock();
                st.fill.ends_at_epoch_ms = clock::unix_ms() + remaining;
            }
        }
        if fill_closed {
            self.fill_deadline = None;
            self.fill_started = None;
            self.auto_off_sent = false;
            {
                let mut st = self.shared.lock();
                st.fill.auto_off_pending = false;
                st.fill.ends_at_epoch_ms = 0;
                st.fill.completed_at_epoch_ms = clock::unix_ms();
            }
            if state.fill_seq_complete() {
                let duration = state.fill_duration_s;
                self.shared.log(
                    Level::Ok,
                    Source::Vehicle,
                    format!("FILL sequence COMPLETE ({duration} s) - ignition interlock satisfied"),
                    Some("fill_complete"),
                );
            } else if state.fill_seq_aborted() {
                self.shared.log(
                    Level::Warn,
                    Source::Vehicle,
                    "FILL sequence ABORTED before the configured duration - ignition remains inhibited".to_string(),
                    Some("fill_aborted"),
                );
            }
        }
        // Detect a vehicle reboot (uptime went backwards significantly).
        if let Some(last) = self.prev_vehicle_uptime {
            if state.uptime_ms.wrapping_add(2_000) < last {
                self.shared.log(
                    Level::Warn,
                    Source::Vehicle,
                    "Vehicle uptime reset - board rebooted with outputs SAFE".to_string(),
                    Some("vehicle_reboot"),
                );
                self.fill_deadline = None;
                self.fill_started = None;
                self.auto_off_sent = false;
            }
        }
        self.prev_vehicle_uptime = Some(state.uptime_ms);
        self.prev_state = state;
    }

    fn handle_ack(&mut self, ack: Ack) {
        let matches_pending = self
            .pending
            .as_ref()
            .map(|p| p.info.cmd_id == ack.cmd_id)
            .unwrap_or(false);
        if !matches_pending {
            if self.verbose {
                self.shared.log(
                    Level::Info,
                    Source::App,
                    format!(
                        "Ignoring ACK for unknown command id {} ({})",
                        ack.cmd_id,
                        ack.result.label()
                    ),
                    None,
                );
            }
            return;
        }
        let pending = self.pending.take();
        let auto = pending.as_ref().map(|p| p.auto).unwrap_or(false);
        let action = pending
            .as_ref()
            .and_then(|p| UserAction::from_id(&p.info.action));
        let label = pending
            .as_ref()
            .map(|p| p.info.label.clone())
            .unwrap_or_else(|| "command".to_string());
        {
            let mut st = self.shared.lock();
            st.pending = None;
            st.revision = st.revision.wrapping_add(1);
            if ack.result.is_ok() {
                st.stats.acks_ok = st.stats.acks_ok.wrapping_add(1);
            } else {
                st.stats.acks_rejected = st.stats.acks_rejected.wrapping_add(1);
            }
        }

        if ack.result.is_ok() {
            self.shared.log(
                Level::Ok,
                Source::Vehicle,
                format!("{label} completed in {} ms", ack.exec_ms),
                Some(action.map(|a| a.id()).unwrap_or("ack")),
            );
            match action {
                Some(UserAction::FillOn) => {
                    let duration = if ack.state.fill_duration_s > 0 {
                        ack.state.fill_duration_s as u32 * 1000
                    } else {
                        ack.state.fill_remaining_ms
                    };
                    self.fill_deadline =
                        Some(Instant::now() + Duration::from_millis(duration as u64));
                    self.fill_started = Some(Instant::now());
                    self.auto_off_sent = false;
                    {
                        let mut st = self.shared.lock();
                        st.fill.ends_at_epoch_ms = clock::unix_ms() + duration as u64;
                        st.fill.duration_s = ack.state.fill_duration_s;
                        st.fill.auto_off_pending = false;
                    }
                }
                Some(UserAction::FillOff) => {
                    self.fill_deadline = None;
                    self.fill_started = None;
                    self.auto_off_sent = false;
                    {
                        let mut st = self.shared.lock();
                        st.fill.ends_at_epoch_ms = 0;
                        st.fill.auto_off_pending = false;
                        st.fill.completed_at_epoch_ms = clock::unix_ms();
                    }
                    let _ = auto;
                }
                Some(UserAction::Estop) => {
                    self.fill_deadline = None;
                    self.fill_started = None;
                    self.auto_off_sent = false;
                }
                _ => {}
            }
        } else {
            let reason = ack.reason.label();
            self.shared.log(
                Level::Warn,
                Source::Vehicle,
                format!(
                    "{label} rejected by vehicle: {} ({})",
                    ack.result.label(),
                    reason
                ),
                Some("ack_rejected"),
            );
            if action == Some(UserAction::FillOn) {
                let mut st = self.shared.lock();
                st.fill.auto_off_pending = false;
            }
            if action == Some(UserAction::FillOff) && auto {
                self.auto_off_sent = false;
            }
        }
        self.apply_state(ack.state);
    }

    fn handle_event(&mut self, event: Event) {
        let code = event.code;
        match code {
            EventCode::Boot => {
                self.shared.log(
                    Level::Info,
                    Source::Vehicle,
                    "Vehicle booted - all outputs SAFE".to_string(),
                    Some(code.code()),
                );
                self.fill_deadline = None;
                self.fill_started = None;
                self.auto_off_sent = false;
            }
            EventCode::FillAutoOff => {
                let duration = event.detail;
                let already_auto = {
                    let st = self.shared.lock();
                    st.fill.auto_off_pending
                };
                if self.auto_off_sent || already_auto {
                    self.shared.log(
                        Level::Info,
                        Source::Vehicle,
                        format!("Vehicle confirmed FILL auto-off ({duration} s)"),
                        Some(code.code()),
                    );
                } else {
                    self.shared.log(
                        Level::Warn,
                        Source::Vehicle,
                        format!(
                            "Vehicle auto-closed FILL after {duration} s (ground timer fallback)"
                        ),
                        Some(code.code()),
                    );
                }
                self.fill_deadline = None;
                self.fill_started = None;
                self.auto_off_sent = true;
                let mut st = self.shared.lock();
                st.fill.auto_off_pending = false;
                st.fill.ends_at_epoch_ms = 0;
            }
            EventCode::FillAborted => {
                self.shared.log(
                    Level::Warn,
                    Source::Vehicle,
                    "FILL sequence aborted before the configured duration".to_string(),
                    Some(code.code()),
                );
            }
            EventCode::FillOn => {
                self.shared.log(Level::Info, Source::Vehicle, "FILL valve open".to_string(), Some(code.code()));
            }
            EventCode::FillOff => {
                self.shared.log(Level::Info, Source::Vehicle, "FILL valve closed".to_string(), Some(code.code()));
            }
            EventCode::SrvOn => {
                self.shared.log(Level::Info, Source::Vehicle, "SRV power ON".to_string(), Some(code.code()));
            }
            EventCode::SrvOff => {
                self.shared.log(Level::Info, Source::Vehicle, "SRV power OFF".to_string(), Some(code.code()));
            }
            EventCode::IgnPwr => {
                self.shared.log(Level::Warn, Source::Vehicle, "IGN PWR asserted".to_string(), Some(code.code()));
            }
            EventCode::IgnSafe => {
                self.shared.log(Level::Ok, Source::Vehicle, "IGN SAFE".to_string(), Some(code.code()));
            }
            EventCode::LinkTimeoutSafe => {
                self.shared.log(
                    Level::Error,
                    Source::Vehicle,
                    "VEHICLE LINK TIMEOUT - all outputs forced SAFE, ignition inhibited".to_string(),
                    Some(code.code()),
                );
                self.fill_deadline = None;
                self.fill_started = None;
                self.auto_off_sent = false;
            }
            EventCode::SafetyReject => {
                let reason = Reason::from_u8(event.detail)
                    .map(|r| r.label().to_string())
                    .unwrap_or_else(|| format!("reason {}", event.detail));
                self.shared.log(
                    Level::Warn,
                    Source::Vehicle,
                    format!("Vehicle safety interlock rejected a command: {reason}"),
                    Some(code.code()),
                );
            }
            EventCode::Estop => {
                self.shared.log(
                    Level::Warn,
                    Source::Vehicle,
                    "E-STOP asserted on vehicle - all outputs SAFE".to_string(),
                    Some(code.code()),
                );
            }
            EventCode::FillSeqReset => {
                self.shared.log(
                    Level::Info,
                    Source::Vehicle,
                    "Fill sequence latch reset".to_string(),
                    Some(code.code()),
                );
            }
            EventCode::FaultsCleared => {
                self.shared.log(
                    Level::Ok,
                    Source::Vehicle,
                    "Latched faults cleared".to_string(),
                    Some(code.code()),
                );
            }
            EventCode::WatchdogReset => {
                self.shared.log(
                    Level::Error,
                    Source::Vehicle,
                    "Vehicle watchdog reset - outputs were forced SAFE".to_string(),
                    Some(code.code()),
                );
            }
            EventCode::RadioFault => {
                self.shared.log(
                    Level::Warn,
                    Source::Vehicle,
                    "Vehicle reported a radio fault".to_string(),
                    Some(code.code()),
                );
            }
            EventCode::GroundDetected => {
                self.shared.log(
                    Level::Info,
                    Source::Ground,
                    "Ground board detected".to_string(),
                    Some(code.code()),
                );
            }
            EventCode::GroundLost => {
                self.shared.log(
                    Level::Info,
                    Source::Ground,
                    "Ground board reported host loss".to_string(),
                    Some(code.code()),
                );
            }
            EventCode::BridgeReady => {
                self.shared.log(
                    Level::Info,
                    Source::Ground,
                    "Ground bridge ready".to_string(),
                    Some(code.code()),
                );
            }
        }
        self.apply_state(event.state);
    }

    // ---------------------------------------------------------------------
    // UI
    // ---------------------------------------------------------------------

    fn update_ui(&mut self) {
        let now = Instant::now();
        {
            let mut st = self.shared.lock();
            st.refresh();
            // Derive ground status from the last HELLO.
            if self.link.is_some() {
                if let Some(last) = self.last_ground_rx {
                    if now.duration_since(last) > Duration::from_millis(GROUND_TIMEOUT_MS) {
                        st.ground.status = GroundStatus::Disconnected;
                    }
                }
            } else if st.ground.status == GroundStatus::Online {
                st.ground.status = GroundStatus::Disconnected;
            }
            // Vehicle status from the last frame.
            if self.link.is_some() {
                match self.last_vehicle_rx {
                    Some(last)
                        if now.duration_since(last) <= Duration::from_millis(VEHICLE_TIMEOUT_MS) =>
                    {
                        st.vehicle.status = VehicleStatus::Online;
                        st.vehicle.last_seen_epoch_ms = clock::unix_ms();
                    }
                    _ => {
                        if st.ground.status == GroundStatus::Online {
                            st.vehicle.status = VehicleStatus::Pinging;
                        } else {
                            st.vehicle.status = VehicleStatus::Offline;
                        }
                    }
                }
            } else {
                st.vehicle.status = VehicleStatus::Offline;
            }
            st.host_uptime_ms = clock::unix_ms().saturating_sub(st.started_epoch_ms);
            st.revision = st.revision.wrapping_add(1);
        }
    }

    fn maybe_publish(&mut self) {
        let now = Instant::now();
        if self
            .last_publish
            .map(|t| now.duration_since(t) >= Duration::from_millis(PUBLISH_MS))
            .unwrap_or(true)
        {
            self.last_publish = Some(now);
            self.shared.publish();
        }
    }
}

/// Probe a freshly opened port by sending a PING to `target` and waiting for a
/// valid frame from that endpoint. Returns the link if the board answered.
fn probe_port(
    path: &str,
    baud: u32,
    target: Endpoint,
    verbose: bool,
) -> Option<Box<dyn ByteLink>> {
    let mut link = match open_link(path, baud) {
        Ok(link) => link,
        Err(e) => {
            if verbose {
                eprintln!("[gse] cannot open {path}: {e}");
            }
            return None;
        }
    };
    if probe_existing(&mut *link, target, PROBE_TIMEOUT_MS) {
        Some(link)
    } else {
        None
    }
}

fn probe_existing(link: &mut dyn ByteLink, target: Endpoint, timeout_ms: u64) -> bool {
    let mut buf = [0u8; Frame::MAX_ENCODED];
    let nonce = clock::unix_ms() as u32;
    let n = match encode_message(
        &Message::Ping(Ping {
            nonce,
            host_ms: 0,
        }),
        Endpoint::Host,
        target,
        0,
        &mut buf,
    ) {
        Some(n) => n,
        None => return false,
    };
    if link.write_all(&buf[..n]).is_err() {
        return false;
    }
    let deadline = Instant::now() + Duration::from_millis(timeout_ms);
    let mut decoder = FrameDecoder::new();
    let mut buf = [0u8; 512];
    while Instant::now() < deadline {
        match link.read(&mut buf) {
            Ok(n) => {
                for &byte in &buf[..n] {
                    if let Some(frame) = decoder.push(byte) {
                        if frame.src == target {
                            return true;
                        }
                    }
                }
                if n == 0 {
                    thread::sleep(Duration::from_millis(3));
                }
            }
            Err(_) => return false,
        }
    }
    false
}
