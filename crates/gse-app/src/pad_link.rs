//! ESP-NOW pad link: the GSE-simple control set on the wire, with the GSE
//! extras supervised app-side.
//!
//! The ESP32 pad runs GSE-simple's algorithm ([`gse_proto::pad`]): it follows
//! a 20 Hz stream of complete desired output states, fails safe (FILL closed,
//! VEHICLE open, PURGE closed, IGN off) when the stream stops, and fires the
//! igniter once per armed press.
//!
//! Everything GSE adds on top of that - LIVE/TEST modes, the fill timer with
//! pause/resume, FILL/VEH interlocks, the ignition countdown with T-0 HOLD,
//! E-STOP and link-loss latches - is an app-only extra. It runs here by
//! hosting the very same GSE vehicle state machine
//! ([`gse_fw_core::vehicle::Vehicle`]) in-process as a *supervisor*:
//!
//! ```text
//! engine --COMMAND/HEARTBEAT--> supervisor (GSE FSM) --outputs--> PAD_CMD 20 Hz --> pad
//! engine <--ACK/STATE/EVENT---- supervisor            <--PAD_STATE readback------- pad
//! ```
//!
//! The engine keeps talking its normal command/ACK protocol, so its fill
//! timer, gates and countdown are unchanged. Two rules keep the operator
//! looking at the *pad's* truth, as GSE-simple requires:
//!
//! * an ACK is only released to the engine once a fresh `PAD_STATE` shows
//!   the pad driving the acknowledged outputs (or, after a timeout, released
//!   with a loud readback-mismatch error),
//! * supervisor traffic only reaches the engine while the pad telemetry is
//!   fresh, so the vehicle shows offline - and every control locks - as soon
//!   as the pad goes quiet; after 5 s the supervisor latches its own
//!   link-loss state exactly as the vehicle firmware would.
//!
//! Mapping of the supervisor outputs onto the pad:
//! `FILL -> fill`, `SRV -> veh` (both vehicle valves: fuel + oxidizer),
//! `IGN -> ign` (only with the operator's ARM), plus the app-side PURGE.
//! While E-STOP, link-loss or INHIBITED is latched the pad is held in the
//! GSE-simple SAFE set (VEHICLE open) until RESET CONTROLS.

use std::collections::VecDeque;
use std::time::{Duration, Instant};

use gse_fw_core::hal::{Leds, LinkState, VehicleHal};
use gse_fw_core::vehicle::Vehicle;
use gse_proto::pad::{self, PadCommand, PadConfig, PadEventCode, PadState, ARM_CODE};
use gse_proto::{
    encode_message, flags, Ack, DxLr42Baud, Endpoint, EventCode, Frame, FrameDecoder, LinkConfig,
    Message, RadioModule, Reason, StateSnapshot, UsbBaud,
};

use crate::logging::Level;

/// How long an ACK may wait for matching pad readback before it is released
/// anyway with a readback-mismatch error.
const ACK_CONFIRM_TIMEOUT: Duration = Duration::from_millis(1_500);
/// Outputs must disagree with the command for this long to count as a mismatch.
const MISMATCH_AFTER: Duration = Duration::from_millis(1_000);
/// Resend the pad configuration at most this often until the pad echoes it.
const CFG_RESEND: Duration = Duration::from_millis(1_000);

/// Operator-adjustable pad settings (persisted in the app config).
#[derive(Clone, Copy, Debug, PartialEq, Eq)]
pub struct PadSettings {
    pub wdg_ds: u8,
    pub ign_max_ds: u8,
    pub travel_fill: u8,
    pub travel_veh: u8,
    pub travel_prg: u8,
}

impl PadSettings {
    pub fn from_config(cfg: &crate::config::Config) -> Self {
        PadSettings {
            wdg_ds: cfg.pad_wdg_ds,
            ign_max_ds: cfg.pad_ign_max_ds,
            travel_fill: cfg.pad_travel_fill,
            travel_veh: cfg.pad_travel_veh,
            travel_prg: cfg.pad_travel_prg,
        }
    }
}

/// One log line produced by the pad link, emitted by the engine.
#[derive(Clone, Debug)]
pub struct PadNote {
    pub level: Level,
    pub from_pad: bool,
    pub text: String,
    pub code: &'static str,
}

/// What one [`PadLink::poll`] produced.
#[derive(Default)]
pub struct PadPoll {
    /// Supervisor frames for the engine (as if received from the vehicle).
    pub to_engine: Vec<Frame>,
    /// Messages to transmit to the pad (`PAD_CMD`, `PAD_CFG`).
    pub to_pad: Vec<Message>,
}

/// UI projection of the pad link.
#[derive(Clone, Debug, Default)]
pub struct PadView {
    pub fresh: bool,
    pub readback: Option<PadState>,
    pub readback_age_ms: Option<u64>,
    pub command: Option<PadCommand>,
    pub armed: bool,
    pub purge_enabled: bool,
    pub purge_open: bool,
    pub config: PadConfig,
    pub cfg_synced: bool,
    pub mismatch: Option<String>,
    pub latched_safe: bool,
}

/// `VehicleHal` for the in-process GSE state machine.
struct SupervisorHal {
    epoch: Instant,
    fill: bool,
    srv: bool,
    ign: bool,
    radio_in: VecDeque<u8>,
    radio_out: Vec<u8>,
}

impl VehicleHal for SupervisorHal {
    fn now_ms(&self) -> u32 {
        self.epoch.elapsed().as_millis() as u32
    }

    fn set_fill(&mut self, on: bool) {
        self.fill = on;
    }

    fn set_srv(&mut self, on: bool) {
        self.srv = on;
    }

    fn set_ign(&mut self, on: bool) {
        self.ign = on;
    }

    fn read_radio(&mut self, buf: &mut [u8]) -> usize {
        let n = buf.len().min(self.radio_in.len());
        for (slot, byte) in buf.iter_mut().zip(self.radio_in.drain(..n)) {
            *slot = byte;
        }
        n
    }

    fn write_radio(&mut self, data: &[u8]) {
        self.radio_out.extend_from_slice(data);
    }

    fn link_state(&self) -> LinkState {
        LinkState {
            usb_baud: UsbBaud::Baud115200,
            uart_baud: DxLr42Baud::Baud115200,
            radio: RadioModule::EspNow,
            flash_saved: false,
        }
    }

    fn probe_radio_module(&mut self, radio: RadioModule) -> bool {
        radio.is_esp_now()
    }

    fn apply_link_config(&mut self, _cfg: LinkConfig) -> Result<bool, Reason> {
        Err(Reason::NotPermitted)
    }

    fn set_leds(&mut self, _leds: Leds) {}

    fn kick_watchdog(&mut self) {}

    fn device_id(&self) -> u32 {
        0
    }
}

struct HeldAck {
    frame: Frame,
    expect: PadCommand,
    since: Instant,
    /// Only readback received after this sequence number may confirm it.
    after_seq: u64,
}

pub struct PadLink {
    supervisor: Vehicle<SupervisorHal>,
    sup_decoder: FrameDecoder,
    settings: PadSettings,
    started: Instant,
    readback: Option<PadState>,
    readback_at: Option<Instant>,
    readback_seq: u64,
    /// Readback showed the igniter fire (on or spent) since the request began.
    ign_seen: bool,
    held_ack: Option<HeldAck>,
    armed: bool,
    purge_enabled: bool,
    purge_open: bool,
    cfg_seq: u8,
    last_cfg_tx: Option<Instant>,
    last_cmd_tx: Option<Instant>,
    last_cmd: Option<PadCommand>,
    cmd_changed_at: Instant,
    mismatch: Option<String>,
    mismatch_logged: bool,
    /// Latest pad HELLO not yet shown to the engine (it arrived while stale).
    pending_hello: Option<Frame>,
    was_fresh: bool,
    was_latched: bool,
    notes: Vec<PadNote>,
}

impl PadLink {
    pub fn new(settings: PadSettings, purge_enabled: bool) -> Self {
        let now = Instant::now();
        let hal = SupervisorHal {
            epoch: now,
            fill: false,
            srv: false,
            ign: false,
            radio_in: VecDeque::new(),
            radio_out: Vec::new(),
        };
        PadLink {
            supervisor: Vehicle::new(hal),
            sup_decoder: FrameDecoder::new(),
            settings,
            started: now,
            readback: None,
            readback_at: None,
            readback_seq: 0,
            ign_seen: false,
            held_ack: None,
            armed: false,
            purge_enabled,
            purge_open: false,
            cfg_seq: 0,
            last_cfg_tx: None,
            last_cmd_tx: None,
            last_cmd: None,
            cmd_changed_at: now,
            mismatch: None,
            mismatch_logged: false,
            pending_hello: None,
            was_fresh: false,
            was_latched: false,
            notes: Vec::new(),
        }
    }

    // ------------------------------------------------------------------
    // Operator controls (pad-only)
    // ------------------------------------------------------------------

    pub fn armed(&self) -> bool {
        self.armed
    }

    pub fn set_armed(&mut self, armed: bool) {
        self.armed = armed;
    }

    pub fn set_purge_enabled(&mut self, enabled: bool) {
        self.purge_enabled = enabled;
        if !enabled {
            self.purge_open = false;
        }
    }

    pub fn purge_open(&self) -> bool {
        self.purge_open
    }

    pub fn set_purge_open(&mut self, open: bool) {
        self.purge_open = open && self.purge_enabled;
    }

    pub fn set_settings(&mut self, settings: PadSettings) {
        self.settings = settings;
        self.last_cfg_tx = None;
    }

    /// Pad telemetry is fresh and the pad is hearing the command stream.
    pub fn fresh(&self, now: Instant) -> bool {
        match (self.readback, self.readback_at) {
            (Some(rb), Some(at)) => {
                now.duration_since(at) <= Duration::from_millis(pad::STATE_STALE_MS as u64)
                    && rb.link_ok()
            }
            _ => false,
        }
    }

    /// The supervisor holds a latched safe state (E-STOP, link loss, INHIBITED).
    pub fn latched_safe(&self) -> bool {
        latched_safe(&self.supervisor.snapshot())
    }

    pub fn take_notes(&mut self) -> Vec<PadNote> {
        std::mem::take(&mut self.notes)
    }

    /// GSE-simple `safe_close`: the SAFE command (VEHICLE open) for shutdown.
    pub fn safe_command(&self) -> PadCommand {
        PadCommand::safe(self.settings.wdg_ds, self.settings.ign_max_ds)
    }

    // ------------------------------------------------------------------
    // Engine <-> supervisor
    // ------------------------------------------------------------------

    /// A message the engine addressed to the vehicle (COMMAND, HEARTBEAT,
    /// LINK_CONFIG). Delivered to the supervisor only while the pad is fresh,
    /// so the supervisor's own link-loss timer tracks the real pad link.
    pub fn deliver_from_engine(&mut self, message: &Message, now: Instant) {
        if !self.fresh(now) {
            return;
        }
        let mut buf = [0u8; Frame::MAX_ENCODED];
        if let Some(n) = encode_message(message, Endpoint::Host, Endpoint::Vehicle, 0, &mut buf) {
            self.supervisor
                .hal_mut()
                .radio_in
                .extend(buf[..n].iter().copied());
        }
    }

    /// A frame received from the pad. Returns frames for the engine.
    pub fn on_pad_frame(&mut self, frame: Frame, now: Instant) -> Option<Frame> {
        match frame.message()? {
            Message::PadState(state) => {
                self.on_readback(state, now);
                None
            }
            Message::PadEvent(event) => {
                self.on_pad_event(event.kind(), event.code, event.arg);
                None
            }
            // Identification only (firmware, device id); liveness comes from
            // the readback, so it is held until the pad is fresh.
            Message::Hello(_) => {
                if self.fresh(now) {
                    Some(frame)
                } else {
                    self.pending_hello = Some(frame);
                    None
                }
            }
            Message::Pong(mut pong) => {
                if !self.fresh(now) {
                    return None;
                }
                // Real round trip through the pad, but the engine's state
                // machine view comes from the supervisor.
                pong.state = self.supervisor.snapshot();
                Frame::from_message(&Message::Pong(pong), frame.src, frame.dst, frame.seq)
            }
            _ => None,
        }
    }

    fn on_readback(&mut self, state: PadState, now: Instant) {
        if let Some(prev) = self.readback {
            if state.uptime_s + 2 < prev.uptime_s {
                self.note(
                    Level::Warn,
                    true,
                    "Pad rebooted - outputs went SAFE and resume from the command stream"
                        .to_string(),
                    "pad_reboot",
                );
            }
            if state.comms_loss() && !prev.comms_loss() && prev.hello_ok() {
                self.note(
                    Level::Error,
                    true,
                    "PAD COMMS LOSS - pad watchdog expired; FILL/PURGE closed, VEHICLE open, IGN off"
                        .to_string(),
                    "pad_comms_loss",
                );
            }
        }
        if state.ign_on() || state.ign_spent() {
            self.ign_seen = true;
        }
        self.readback = Some(state);
        self.readback_at = Some(now);
        self.readback_seq = self.readback_seq.wrapping_add(1);
    }

    fn on_pad_event(&mut self, kind: Option<PadEventCode>, code: u8, arg: u8) {
        let (level, text, tag) = match kind {
            Some(PadEventCode::Boot) => (
                Level::Warn,
                "Pad booted - outputs SAFE (FILL/PURGE closed, VEHICLE open)".to_string(),
                "pad_boot",
            ),
            Some(PadEventCode::HelloOk) => (
                Level::Ok,
                "Pad receiving the command stream".to_string(),
                "pad_hello_ok",
            ),
            // Logged from the readback edge (it carries more context).
            Some(PadEventCode::CommsLoss) => return,
            Some(PadEventCode::Arm) => (
                Level::Warn,
                "Pad ARMED - igniter interlock satisfied".to_string(),
                "pad_arm",
            ),
            Some(PadEventCode::Disarm) => (Level::Info, "Pad disarmed".to_string(), "pad_disarm"),
            Some(PadEventCode::IgnOn) => (
                Level::Warn,
                "IGNITER ENERGISED by the pad".to_string(),
                "pad_ign_on",
            ),
            Some(PadEventCode::IgnOff) => (
                Level::Info,
                if arg == 1 {
                    "Igniter relay cut by the on-time limit - release IGN before it can fire again"
                        .to_string()
                } else {
                    "Igniter relay off".to_string()
                },
                "pad_ign_off",
            ),
            Some(PadEventCode::CfgChanged) => (
                Level::Ok,
                format!("Pad applied configuration #{arg}"),
                "pad_cfg",
            ),
            Some(PadEventCode::BadCmd) => (
                Level::Warn,
                format!("Pad rejected a malformed frame (type 0x{arg:02X})"),
                "pad_bad_cmd",
            ),
            None => (
                Level::Info,
                format!("Pad event {code} (arg {arg})"),
                "pad_event",
            ),
        };
        self.note(level, true, text, tag);
    }

    /// Run the supervisor and produce frames for the engine and the pad.
    pub fn poll(&mut self, now: Instant) -> PadPoll {
        let mut out = PadPoll::default();
        let fresh = self.fresh(now);
        if fresh != self.was_fresh {
            self.was_fresh = fresh;
            if fresh {
                self.note(
                    Level::Ok,
                    true,
                    "Pad telemetry fresh - link OK".to_string(),
                    "pad_fresh",
                );
            } else if self.readback.is_some() {
                self.note(
                    Level::Warn,
                    true,
                    "Pad telemetry stale - controls locked until the pad answers".to_string(),
                    "pad_stale",
                );
            }
        }

        if fresh {
            if let Some(hello) = self.pending_hello.take() {
                out.to_engine.push(hello);
            }
        }

        self.supervisor.poll();
        let bytes = std::mem::take(&mut self.supervisor.hal_mut().radio_out);
        for &byte in &bytes {
            let Some(frame) = self.sup_decoder.push(byte) else {
                continue;
            };
            match frame.message() {
                // The pad's own HELLO identifies the board.
                Some(Message::Hello(_)) => {}
                Some(Message::Event(e)) if e.code == EventCode::Boot => {}
                Some(Message::Ack(ack)) => self.hold_ack(frame, ack, now),
                Some(_) if fresh => out.to_engine.push(frame),
                _ => {}
            }
        }

        let latched = self.latched_safe();
        if latched && !self.was_latched {
            if self.armed {
                self.armed = false;
                self.note(
                    Level::Warn,
                    false,
                    "Igniter disarmed - safe state latched".to_string(),
                    "pad_disarm",
                );
            }
            self.purge_open = false;
        }
        self.was_latched = latched;

        let command = self.command(now);
        if !command.ign {
            self.ign_seen = false;
        }
        let changed = self
            .last_cmd
            .map_or(true, |c| strip_tx(c) != strip_tx(command));
        if changed {
            self.cmd_changed_at = now;
        }
        let due = self.last_cmd_tx.map_or(true, |t| {
            now.duration_since(t) >= Duration::from_millis(pad::CMD_PERIOD_MS as u64)
        });
        if due || changed {
            self.last_cmd_tx = Some(now);
            out.to_pad.push(Message::PadCmd(command));
        }
        self.last_cmd = Some(command);

        if let Some(cfg) = self.config_to_send(now) {
            out.to_pad.push(Message::PadConfig(cfg));
        }

        self.release_ack(now, fresh, &mut out);
        self.check_mismatch(now, fresh);
        out
    }

    /// The desired pad command from the supervisor outputs and operator state.
    fn command(&self, now: Instant) -> PadCommand {
        let snap = self.supervisor.snapshot();
        let hal = self.supervisor.hal();
        let base = PadCommand {
            tx_ms: (now.duration_since(self.started).as_millis() & 0xFFFF) as u16,
            ..PadCommand::safe(self.settings.wdg_ds, self.settings.ign_max_ds)
        };
        if latched_safe(&snap) {
            return base;
        }
        // Never arm blind: the arm code needs fresh pad telemetry.
        let arm = self.armed && self.fresh(now);
        PadCommand {
            arm_code: if arm { ARM_CODE } else { 0 },
            fill: hal.fill,
            veh: hal.srv,
            prg: self.purge_enabled && self.purge_open,
            ign: hal.ign && arm,
            ..base
        }
    }

    /// The pad command implied by a supervisor snapshot (ACK confirmation).
    fn command_for(&self, snap: &StateSnapshot) -> PadCommand {
        let base = PadCommand::safe(self.settings.wdg_ds, self.settings.ign_max_ds);
        if latched_safe(snap) {
            return base;
        }
        PadCommand {
            arm_code: if self.armed { ARM_CODE } else { 0 },
            fill: snap.fill_open(),
            veh: snap.srv_on(),
            prg: self.purge_enabled && self.purge_open,
            ign: snap.ign_pwr() && self.armed,
            ..base
        }
    }

    fn desired_config(&self) -> PadConfig {
        PadConfig {
            travel_fill: self.settings.travel_fill,
            travel_veh: self.settings.travel_veh,
            travel_prg: self.settings.travel_prg,
            wdg_ds: self.settings.wdg_ds,
            ign_max_ds: self.settings.ign_max_ds,
            allow_purge: self.purge_enabled,
            cfg_seq: self.cfg_seq,
        }
        .clamped()
    }

    fn cfg_synced(&self) -> bool {
        self.readback
            .is_some_and(|rb| rb.config_in_force().same_values(&self.desired_config()))
    }

    fn config_to_send(&mut self, now: Instant) -> Option<PadConfig> {
        if self.readback.is_none() || self.cfg_synced() {
            return None;
        }
        if self
            .last_cfg_tx
            .is_some_and(|t| now.duration_since(t) < CFG_RESEND)
        {
            return None;
        }
        self.last_cfg_tx = Some(now);
        self.cfg_seq = self.cfg_seq.wrapping_add(1);
        Some(self.desired_config())
    }

    fn hold_ack(&mut self, frame: Frame, ack: Ack, now: Instant) {
        let expect = self.command_for(&ack.state);
        self.held_ack = Some(HeldAck {
            frame,
            expect,
            since: now,
            after_seq: self.readback_seq,
        });
    }

    fn release_ack(&mut self, now: Instant, fresh: bool, out: &mut PadPoll) {
        let Some(held) = self.held_ack.as_ref() else {
            return;
        };
        let waited = now.duration_since(held.since);
        let confirmed = fresh
            && self.readback_seq > held.after_seq
            && self
                .readback
                .is_some_and(|rb| readback_matches(&rb, &held.expect, self.ign_seen));
        if confirmed {
            let held = self.held_ack.take().expect("held ack");
            out.to_engine.push(held.frame);
            return;
        }
        if waited < ACK_CONFIRM_TIMEOUT {
            return;
        }
        let held = self.held_ack.take().expect("held ack");
        if !fresh {
            // Pad unreachable: drop it; the engine reports the missing ACK.
            return;
        }
        let actual = self
            .readback
            .map(|rb| describe_readback(&rb))
            .unwrap_or_else(|| "no readback".to_string());
        self.note(
            Level::Error,
            true,
            format!(
                "PAD READBACK MISMATCH - commanded {}, pad reports {actual}",
                describe_command(&held.expect)
            ),
            "pad_mismatch",
        );
        out.to_engine.push(held.frame);
    }

    fn check_mismatch(&mut self, now: Instant, fresh: bool) {
        let (Some(rb), Some(cmd)) = (self.readback, self.last_cmd) else {
            self.mismatch = None;
            return;
        };
        let differs = fresh
            && now.duration_since(self.cmd_changed_at) >= MISMATCH_AFTER
            && !readback_matches(&rb, &cmd, self.ign_seen);
        if differs {
            if self.mismatch.is_none() {
                let text = format!(
                    "commanded {}, pad reports {}",
                    describe_command(&cmd),
                    describe_readback(&rb)
                );
                if !self.mismatch_logged {
                    self.mismatch_logged = true;
                    self.note(
                        Level::Error,
                        true,
                        format!("PAD READBACK MISMATCH - {text}"),
                        "pad_mismatch",
                    );
                }
                self.mismatch = Some(text);
            }
        } else {
            if self.mismatch.is_some() {
                self.note(
                    Level::Ok,
                    true,
                    "Pad readback matches the command again".to_string(),
                    "pad_mismatch_clear",
                );
            }
            self.mismatch = None;
            self.mismatch_logged = false;
        }
    }

    pub fn view(&self, now: Instant) -> PadView {
        PadView {
            fresh: self.fresh(now),
            readback: self.readback,
            readback_age_ms: self
                .readback_at
                .map(|t| now.duration_since(t).as_millis() as u64),
            command: self.last_cmd,
            armed: self.armed,
            purge_enabled: self.purge_enabled,
            purge_open: self.purge_open,
            config: self.desired_config(),
            cfg_synced: self.cfg_synced(),
            mismatch: self.mismatch.clone(),
            latched_safe: self.latched_safe(),
        }
    }

    fn note(&mut self, level: Level, from_pad: bool, text: String, code: &'static str) {
        self.notes.push(PadNote {
            level,
            from_pad,
            text,
            code,
        });
    }
}

fn latched_safe(snap: &StateSnapshot) -> bool {
    snap.estop() || snap.inhibited() || snap.has(flags::LINK_LOSS_SAFE)
}

fn strip_tx(mut c: PadCommand) -> PadCommand {
    c.tx_ms = 0;
    c
}

/// The pad drives what was commanded. The igniter is a one-shot pulse, so a
/// held request counts as confirmed once the readback has shown it fire.
fn readback_matches(rb: &PadState, cmd: &PadCommand, ign_seen: bool) -> bool {
    rb.out_fill == cmd.fill
        && rb.out_veh == cmd.veh
        && rb.out_prg == cmd.prg
        && if cmd.ign {
            ign_seen || rb.ign_on() || rb.ign_spent()
        } else {
            !rb.out_ign
        }
}

fn open_closed(open: bool) -> &'static str {
    if open {
        "OPEN"
    } else {
        "CLOSED"
    }
}

pub fn describe_command(c: &PadCommand) -> String {
    format!(
        "FILL {} · VEH {} · PURGE {} · IGN {}{}",
        open_closed(c.fill),
        open_closed(c.veh),
        open_closed(c.prg),
        if c.ign { "FIRE" } else { "OFF" },
        if c.armed() { " (armed)" } else { "" }
    )
}

pub fn describe_readback(rb: &PadState) -> String {
    format!(
        "FILL {} · VEH {} · PURGE {} · IGN {}{}",
        open_closed(rb.out_fill),
        open_closed(rb.out_veh),
        open_closed(rb.out_prg),
        if rb.out_ign {
            "ON"
        } else if rb.ign_spent() {
            "SPENT"
        } else {
            "OFF"
        },
        if rb.comms_loss() { " (comms loss)" } else { "" }
    )
}

#[cfg(test)]
mod tests {
    use super::*;
    use gse_fw_core::mock::MockPad;
    use gse_fw_core::pad::{Pad, PadServo};
    use gse_proto::{Command, Control, Heartbeat, ACTION_ON};

    /// Supervisor + a real pad core joined by a zero-latency pipe.
    struct Rig {
        link: PadLink,
        pad: Pad<MockPad>,
        engine_rx: Vec<Message>,
    }

    impl Rig {
        fn new() -> Self {
            let settings = PadSettings {
                wdg_ds: 10,
                ign_max_ds: 5,
                travel_fill: 45,
                travel_veh: 45,
                travel_prg: 45,
            };
            Rig {
                link: PadLink::new(settings, false),
                pad: Pad::new(MockPad::default()),
                engine_rx: Vec::new(),
            }
        }

        /// Advance 10 ms of pad time per step; the supervisor uses the host clock.
        fn run(&mut self, steps: u32) {
            for _ in 0..steps {
                let now = Instant::now();
                let out = self.link.poll(now);
                for m in out.to_pad {
                    self.pad
                        .hal_mut()
                        .feed_frame(&m, Endpoint::Host, Endpoint::Vehicle, 0);
                }
                self.engine_rx
                    .extend(out.to_engine.iter().filter_map(|f| f.message()));
                self.pad.hal_mut().advance(10);
                self.pad.poll();
                for f in self.pad.hal_mut().drain_frames() {
                    if let Some(fwd) = self.link.on_pad_frame(f, Instant::now()) {
                        self.engine_rx.extend(fwd.message());
                    }
                }
                std::thread::sleep(Duration::from_millis(2));
            }
        }

        fn engine_send(&mut self, msg: Message) {
            self.link.deliver_from_engine(&msg, Instant::now());
        }

        fn acks(&self) -> Vec<Ack> {
            self.engine_rx
                .iter()
                .filter_map(|m| match m {
                    Message::Ack(a) => Some(*a),
                    _ => None,
                })
                .collect()
        }
    }

    fn command(cmd_id: u16, control: Control) -> Message {
        Message::Command(Command {
            cmd_id,
            control,
            action: ACTION_ON,
            arg_s: 60,
            flags: 0,
        })
    }

    #[test]
    fn stream_brings_pad_out_of_comms_loss_and_closes_vehicle_valves() {
        let mut rig = Rig::new();
        assert!(!rig.link.fresh(Instant::now()));
        // > 250 ms of host time: the supervisor's first STATE is due then.
        rig.run(150);
        assert!(rig.link.fresh(Instant::now()));
        // Supervisor idles at FILL OFF / SRV OFF -> VEH closed on the pad.
        assert_eq!(rig.pad.hal().servo(PadServo::Veh), 1_000);
        assert_eq!(rig.pad.hal().servo(PadServo::Fill), 1_000);
        // Supervisor STATE reaches the engine once the pad is fresh.
        assert!(rig.engine_rx.iter().any(|m| matches!(m, Message::State(_))));
    }

    #[test]
    fn fill_ack_waits_for_pad_readback() {
        let mut rig = Rig::new();
        rig.run(30);
        rig.engine_send(Message::Heartbeat(Heartbeat {
            uptime_ms: 0,
            mode: 0,
        }));
        rig.engine_send(command(1, Control::Fill));
        // The supervisor settles 350 ms (host clock) before it acknowledges.
        rig.run(250);
        let acks = rig.acks();
        assert_eq!(acks.len(), 1, "exactly one ACK, after readback");
        assert!(acks[0].result.is_ok() && acks[0].state.fill_open());
        assert_eq!(rig.pad.hal().servo(PadServo::Fill), 2_000);
        let view = rig.link.view(Instant::now());
        assert!(view.readback.unwrap().out_fill && view.mismatch.is_none());
    }

    #[test]
    fn igniter_requires_operator_arm() {
        let mut rig = Rig::new();
        rig.run(30);
        // Drive the supervisor into IGN PWR via its normal sequence would need
        // a full fill; check the mapping directly instead.
        let mut snap = StateSnapshot {
            flags: flags::IGN_PWR | flags::SRV_ON,
            ..StateSnapshot::default()
        };
        assert!(!rig.link.command_for(&snap).ign, "not armed");
        rig.link.set_armed(true);
        let c = rig.link.command_for(&snap);
        assert!(c.ign && c.veh && c.armed());
        snap.flags |= flags::ESTOP;
        let c = rig.link.command_for(&snap);
        assert_eq!(
            c,
            PadCommand::safe(10, 5),
            "E-STOP holds the GSE-simple SAFE set"
        );
    }

    #[test]
    fn purge_needs_enable_and_reaches_pad_config() {
        let mut rig = Rig::new();
        rig.run(20);
        rig.link.set_purge_open(true);
        assert!(!rig.link.purge_open(), "purge disabled by default (ECR H8)");
        rig.link.set_purge_enabled(true);
        rig.link.set_purge_open(true);
        rig.run(150);
        assert!(rig.link.view(Instant::now()).cfg_synced);
        assert_eq!(rig.pad.hal().servo(PadServo::Purge), 2_000);
        rig.link.set_purge_enabled(false);
        rig.run(150);
        assert_eq!(rig.pad.hal().servo(PadServo::Purge), 1_000);
    }

    #[test]
    fn stale_pad_blocks_supervisor_traffic() {
        let mut rig = Rig::new();
        rig.run(30);
        rig.engine_rx.clear();
        // Stop running the pad; only the supervisor polls.
        for _ in 0..90 {
            let _ = rig.link.poll(Instant::now());
            std::thread::sleep(Duration::from_millis(20));
        }
        assert!(!rig.link.fresh(Instant::now()));
        // Over the next 400 ms nothing reaches the engine, but the 20 Hz
        // command stream keeps going.
        let mut to_engine = 0;
        let mut commands = 0;
        for _ in 0..20 {
            let out = rig.link.poll(Instant::now());
            to_engine += out.to_engine.len();
            commands += out
                .to_pad
                .iter()
                .filter(|m| matches!(m, Message::PadCmd(_)))
                .count();
            std::thread::sleep(Duration::from_millis(20));
        }
        assert_eq!(to_engine, 0);
        assert!(commands >= 5, "only {commands} commands in 400 ms");
    }
}
