//! Vehicle (flight) NUCLEO-H753ZI logic.
//!
//! The state machine below owns every safety interlock that matters in flight.
//! The ground app enforces the same rules for the operator interface, but the
//! vehicle never trusts the app: each command is re-validated here against the
//! real output state, and the vehicle also closes the fill valve on its own if
//! the command link disappears.
//!
//! Safety rules (mirrored by `gse-app`):
//!
//! 1. FILL and SRV can never be on at the same time.
//! 2. FILL opens for `arg_s` seconds; turning FILL OFF before expiry pauses the
//!    fill timer (preserving `fill_target_ms`) and unlocks SRV / SRV+IGN, while
//!    subsequent FILL ON resumes from the paused timer. At expiry the app closes
//!    FILL automatically (with a `deadline + FILL_FALLBACK_EXTRA_MS` hardware
//!    fallback on the vehicle).
//! 3. SRV ON, IGN PWR, and SRV+IGN ON require FILL to have run and turned OFF
//!    (`fill_seq_complete` or `fill_paused`, without `fill_seq_aborted`). An
//!    uncommanded fault abort (E-STOP / link timeout mid-fill) cannot unlock
//!    downstream channels.
//! 4. IGN PWR requires SRV ON.
//! 5. Any command link timeout while outputs are active forces all outputs safe
//!    and inhibits ignition until the operator resets the sequence.
//!
//! Acks are emitted only after the action has physically settled (servo travel
//! time), never before.

use gse_proto::control::{cmd_flags, ACTION_ON};
use gse_proto::fsm::{ControlEvent, ControlState, FillState, IgnState, Interlocks, SrvState};
use gse_proto::Transition;
use gse_proto::{
    encode_message, link_cfg_flags, AckResult, Command, Control, Endpoint, Event, EventCode, Frame,
    Heartbeat, Hello, LinkConfig, LinkConfigAck, Message, Pong, Reason, StateSnapshot,
};

use crate::hal::{deadline_passed, elapsed_ms, Leds, VehicleHal};
use crate::radio::{DxLr42Receiver, DxLr42Stats};

/// Snapshot broadcast rate.
pub const STATE_PERIOD_MS: u32 = 250;
/// HELLO rate (so a newly-connected app learns the firmware version quickly).
pub const HELLO_PERIOD_MS: u32 = 2_000;
/// Any valid command/heartbeat/ping resets this timer.
pub const LINK_TIMEOUT_MS: u32 = 5_000;
/// Vehicle-side last-resort fill close, measured from the configured deadline.
pub const FILL_FALLBACK_EXTRA_MS: u32 = 2_000;
/// A fill closed this close to the deadline still counts as complete.
pub const FILL_COMPLETE_TOLERANCE_MS: u32 = 500;
/// Time allowed for the servo to reach its new position before the ack is sent.
pub const SETTLE_SERVO_MS: u16 = 350;
/// Relay/igniter settle time.
pub const SETTLE_RELAY_MS: u16 = 40;
/// How many bytes to drain from the UART per poll.
const RX_CHUNK: usize = 128;

#[derive(Clone, Copy, Debug, PartialEq, Eq)]
pub struct VehicleState {
    pub fill: bool,
    pub srv: bool,
    pub ign: bool,
    pub fill_timer_active: bool,
    pub fill_start_ms: u32,
    /// Active segment duration in milliseconds (full `fill_duration_s * 1000` on
    /// a fresh fill, or the remaining ms saved when a fill was paused early).
    pub fill_target_ms: u32,
    pub fill_duration_s: u16,
    pub fill_paused: bool,
    pub fill_seq_complete: bool,
    pub fill_seq_aborted: bool,
    pub fill_auto_off: bool,
    pub estop: bool,
    pub inhibited: bool,
    pub link_loss_safe: bool,
    pub last_rejected: bool,
    pub boot_safe: bool,
    pub faults: u16,
    pub last_cmd_id: u16,
}

impl Default for VehicleState {
    fn default() -> Self {
        VehicleState {
            fill: false,
            srv: false,
            ign: false,
            fill_timer_active: false,
            fill_start_ms: 0,
            fill_target_ms: 0,
            fill_duration_s: 0,
            fill_paused: false,
            fill_seq_complete: false,
            fill_seq_aborted: false,
            fill_auto_off: false,
            estop: false,
            inhibited: false,
            link_loss_safe: false,
            last_rejected: false,
            boot_safe: true,
            faults: 0,
            last_cmd_id: 0,
        }
    }
}

#[derive(Clone, Copy, Debug)]
struct PendingAck {
    cmd_id: u16,
    control: Control,
    action: u8,
    result: AckResult,
    reason: Reason,
    applied_ms: u32,
    due_ms: u32,
}

#[derive(Clone, Copy, Debug)]
struct Outcome {
    result: AckResult,
    reason: Reason,
    settle_ms: u16,
    event: Option<(EventCode, u8)>,
}

impl Outcome {
    fn ok(settle_ms: u16, event: Option<(EventCode, u8)>) -> Self {
        Outcome {
            result: AckResult::Ok,
            reason: Reason::None,
            settle_ms,
            event,
        }
    }

    fn reject(result: AckResult, reason: Reason) -> Self {
        Outcome {
            result,
            reason,
            settle_ms: 0,
            event: None,
        }
    }
}

pub struct Vehicle<H: VehicleHal> {
    hal: H,
    state: VehicleState,
    decoder: DxLr42Receiver,
    tx_seq: u8,
    boot_ms: u32,
    booted: bool,
    last_host_activity: u32,
    saw_host: bool,
    link_lost: bool,
    pending: Option<PendingAck>,
    last_ack: Option<gse_proto::Ack>,
    last_state_tx: u32,
    last_hello_tx: u32,
    last_activity: u32,
    crc_errors_seen: u32,
    at_recoveries_seen: u16,
    pending_link_cfg: Option<LinkConfig>,
}

impl<H: VehicleHal> Vehicle<H> {
    pub fn new(hal: H) -> Self {
        let now = hal.now_ms();
        Vehicle {
            hal,
            state: VehicleState::default(),
            decoder: DxLr42Receiver::new(),
            tx_seq: 0,
            boot_ms: now,
            booted: false,
            last_host_activity: now.wrapping_sub(LINK_TIMEOUT_MS).wrapping_sub(1),
            saw_host: false,
            link_lost: false,
            pending: None,
            last_ack: None,
            last_state_tx: now,
            last_hello_tx: now,
            last_activity: now,
            crc_errors_seen: 0,
            at_recoveries_seen: 0,
            pending_link_cfg: None,
        }
    }

    pub fn hal(&self) -> &H {
        &self.hal
    }

    pub fn hal_mut(&mut self) -> &mut H {
        &mut self.hal
    }

    pub fn state(&self) -> &VehicleState {
        &self.state
    }

    /// The three outputs as the shared deterministic machine sees them.
    fn control_state(&self) -> ControlState {
        ControlState {
            fill: if self.state.fill {
                FillState::On
            } else {
                FillState::Off
            },
            srv: if self.state.srv {
                SrvState::On
            } else {
                SrvState::Off
            },
            ign: if self.state.ign {
                IgnState::Pwr
            } else {
                IgnState::Safe
            },
        }
    }

    fn interlocks(&self) -> Interlocks {
        Interlocks {
            fill_seq_complete: self.state.fill_seq_complete,
            fill_paused: self.state.fill_paused,
            fill_seq_aborted: self.state.fill_seq_aborted,
            inhibited: self.state.inhibited,
            estop: self.state.estop,
        }
    }

    /// Run the shared transition table against the real output state.
    fn transition(&self, event: ControlEvent) -> Transition {
        self.control_state().transition(event, self.interlocks())
    }

    pub fn snapshot(&self) -> StateSnapshot {
        self.snapshot_at(self.hal.now_ms())
    }

    pub fn decoder_stats(&self) -> gse_proto::DecodeStats {
        self.decoder.stats()
    }

    pub fn radio_driver_stats(&self) -> DxLr42Stats {
        self.decoder.radio_stats()
    }

    fn snapshot_at(&self, now: u32) -> StateSnapshot {
        let mut flags = 0u16;
        if self.state.fill {
            flags |= gse_proto::flags::FILL_OPEN;
        }
        if self.state.srv {
            flags |= gse_proto::flags::SRV_ON;
        }
        if self.state.ign {
            flags |= gse_proto::flags::IGN_PWR;
        }
        if self.state.fill_timer_active {
            flags |= gse_proto::flags::FILL_TIMER_ACTIVE;
        }
        if self.state.fill_paused {
            flags |= gse_proto::flags::FILL_PAUSED;
        }
        if self.state.fill_seq_complete {
            flags |= gse_proto::flags::FILL_SEQ_COMPLETE;
        }
        if self.state.fill_seq_aborted {
            flags |= gse_proto::flags::FILL_SEQ_ABORTED;
        }
        if self.state.fill_auto_off {
            flags |= gse_proto::flags::FILL_AUTO_OFF;
        }
        if self.state.link_loss_safe {
            flags |= gse_proto::flags::LINK_LOSS_SAFE;
        }
        if self.state.last_rejected {
            flags |= gse_proto::flags::LAST_REJECTED;
        }
        if self.state.boot_safe {
            flags |= gse_proto::flags::BOOT_SAFE;
        }
        if self.state.estop {
            flags |= gse_proto::flags::ESTOP;
        }
        if self.state.inhibited {
            flags |= gse_proto::flags::INHIBITED;
        }
        let fill_remaining_ms = if self.state.fill && self.state.fill_timer_active {
            let deadline = self
                .state
                .fill_start_ms
                .wrapping_add(self.state.fill_target_ms);
            if deadline_passed(now, deadline) {
                0
            } else {
                deadline.wrapping_sub(now)
            }
        } else if self.state.fill_paused {
            self.state.fill_target_ms
        } else {
            0
        };
        StateSnapshot {
            flags,
            faults: self.state.faults,
            fill_remaining_ms,
            // Keep the configured duration after the valve closes so the
            // completion/pause log and the console show total fill progress.
            fill_duration_s: self.state.fill_duration_s,
            last_cmd_id: self.state.last_cmd_id,
            uptime_ms: elapsed_ms(now, self.boot_ms),
        }
    }

    /// Run one iteration of the main loop. Call this continuously; it never
    /// blocks longer than the HAL's UART read/write.
    pub fn poll(&mut self) {
        let now = self.hal.now_ms();
        if !self.booted {
            self.booted = true;
            self.hal.set_radio_transparent();
            self.emit_event(EventCode::Boot, 0, now);
            self.send_hello(now);
            self.last_state_tx = now;
        }

        // Complete any staged RF link switchover strictly after `LinkConfigAck`
        // has drained out of the old radio's TX buffer / shift register.
        if let Some(cfg) = self.pending_link_cfg {
            if !self.hal.radio_tx_busy() {
                self.pending_link_cfg = None;
                if self.hal.apply_link_config(cfg).is_ok() {
                    self.decoder = DxLr42Receiver::new();
                    self.emit_event(EventCode::LinkReconfigured, cfg.radio.code(), now);
                    self.send_hello(now);
                    self.send_state(now);
                } else {
                    self.state.faults |= gse_proto::faults::CONFIG_ERROR;
                }
            }
        }

        let mut buf = [0u8; RX_CHUNK];
        for _ in 0..4 {
            let n = self.hal.read_radio(&mut buf);
            if n == 0 {
                break;
            }
            for &byte in &buf[..n] {
                if let Some(frame) = self.decoder.push(byte, now) {
                    self.last_activity = now;
                    self.handle_frame(frame, now);
                }
            }
        }

        if let Some(escape) = self.decoder.poll_recovery_tx(now) {
            self.hal.set_radio_transparent();
            self.hal.write_radio(escape);
        }

        self.check_link(now);
        self.check_fill_timer(now);
        self.flush_pending_ack(now);

        if elapsed_ms(now, self.last_state_tx) >= STATE_PERIOD_MS {
            self.send_state(now);
        }
        if elapsed_ms(now, self.last_hello_tx) >= HELLO_PERIOD_MS {
            self.send_hello(now);
        }

        let crc_errors = self.decoder.stats().crc_errors;
        if crc_errors != self.crc_errors_seen {
            self.state.faults |= gse_proto::faults::CRC_ERROR;
            self.crc_errors_seen = crc_errors;
        }
        let at_recoveries = self.decoder.radio_stats().at_recoveries;
        if at_recoveries != self.at_recoveries_seen {
            self.state.faults |= gse_proto::faults::RADIO_ERROR;
            self.at_recoveries_seen = at_recoveries;
        }

        let link_up = self.saw_host
            && !self.link_lost
            && elapsed_ms(now, self.last_host_activity) < LINK_TIMEOUT_MS;
        self.hal.set_leds(Leds {
            link: link_up,
            activity: elapsed_ms(now, self.last_activity) < 250,
            fault: self.state.faults != 0 || self.state.ign,
        });
        self.hal.kick_watchdog();
    }

    fn handle_frame(&mut self, frame: Frame, now: u32) {
        if frame.src != Endpoint::Host {
            return;
        }
        if frame.dst != Endpoint::Vehicle && frame.dst != Endpoint::Broadcast {
            return;
        }
        self.last_host_activity = now;
        self.saw_host = true;
        if self.link_lost {
            self.link_lost = false;
            self.state.link_loss_safe = false;
        }
        match frame.message() {
            Some(Message::Command(cmd)) => self.handle_command(cmd, now),
            Some(Message::LinkConfig(cfg)) => self.handle_link_config(cfg, now),
            Some(Message::Ping(ping)) => {
                let state = self.snapshot_at(now);
                self.send_msg(
                    Message::Pong(Pong {
                        nonce: ping.nonce,
                        uptime_ms: state.uptime_ms,
                        state,
                    }),
                    Endpoint::Host,
                );
            }
            Some(Message::Heartbeat(Heartbeat { .. })) => {}
            _ => {}
        }
    }

    fn handle_link_config(&mut self, cfg: LinkConfig, now: u32) {
        let cur = self.hal.link_state();
        if self.state.ign {
            self.emit_event(EventCode::SafetyReject, Reason::IgnPowered.as_u8(), now);
            let _ = self.send_msg(
                Message::LinkConfigAck(LinkConfigAck {
                    req_id: cfg.req_id,
                    result: AckResult::RejectedSafety,
                    reason: Reason::IgnPowered,
                    usb_baud: cur.usb_baud,
                    uart_baud: cur.uart_baud,
                    radio: cur.radio,
                    flags: if cur.flash_saved {
                        link_cfg_flags::PERSIST
                    } else {
                        0
                    },
                }),
                Endpoint::Host,
            );
            return;
        }
        if self.state.fill || self.state.srv {
            let reason = if self.state.fill {
                Reason::FillActive
            } else {
                Reason::NotPermitted
            };
            self.emit_event(EventCode::SafetyReject, reason.as_u8(), now);
            let _ = self.send_msg(
                Message::LinkConfigAck(LinkConfigAck {
                    req_id: cfg.req_id,
                    result: AckResult::RejectedSafety,
                    reason,
                    usb_baud: cur.usb_baud,
                    uart_baud: cur.uart_baud,
                    radio: cur.radio,
                    flags: if cur.flash_saved {
                        link_cfg_flags::PERSIST
                    } else {
                        0
                    },
                }),
                Endpoint::Host,
            );
            return;
        }
        if !self.hal.probe_radio_module(cfg.radio) {
            self.state.faults |= gse_proto::faults::RADIO_ERROR;
            let _ = self.send_msg(
                Message::LinkConfigAck(LinkConfigAck {
                    req_id: cfg.req_id,
                    result: AckResult::HardwareFault,
                    reason: Reason::HardwareFault,
                    usb_baud: cur.usb_baud,
                    uart_baud: cur.uart_baud,
                    radio: cur.radio,
                    flags: if cur.flash_saved {
                        link_cfg_flags::PERSIST
                    } else {
                        0
                    },
                }),
                Endpoint::Host,
            );
            return;
        }
        let ack = LinkConfigAck {
            req_id: cfg.req_id,
            result: AckResult::Ok,
            reason: Reason::None,
            usb_baud: cur.usb_baud,
            uart_baud: cfg.uart_baud,
            radio: cfg.radio,
            flags: if cfg.persist() {
                link_cfg_flags::PERSIST
            } else {
                0
            },
        };
        if self.send_msg(Message::LinkConfigAck(ack), Endpoint::Host) {
            self.pending_link_cfg = Some(cfg);
        }
    }

    fn handle_command(&mut self, cmd: Command, now: u32) {
        // A retry of the command that is still settling.
        if let Some(pending) = self.pending {
            if pending.cmd_id == cmd.cmd_id {
                return;
            }
        }
        // A retry after the ack was already sent: repeat the cached ack.
        if let Some(last) = self.last_ack {
            if last.cmd_id == cmd.cmd_id {
                self.send_msg(Message::Ack(last), Endpoint::Host);
                return;
            }
        }
        if self.pending.is_some() {
            self.flush_pending_ack(now);
        }

        let outcome = self.apply(&cmd, now);
        self.state.last_cmd_id = cmd.cmd_id;
        if outcome.result.is_ok() {
            self.state.boot_safe = false;
            self.state.last_rejected = false;
        } else {
            self.state.last_rejected = true;
            self.emit_event(EventCode::SafetyReject, outcome.reason.as_u8(), now);
        }
        self.pending = Some(PendingAck {
            cmd_id: cmd.cmd_id,
            control: cmd.control,
            action: cmd.action,
            result: outcome.result,
            reason: outcome.reason,
            applied_ms: now,
            due_ms: now.wrapping_add(outcome.settle_ms as u32),
        });
        if let Some((code, detail)) = outcome.event {
            self.emit_event(code, detail, now);
        }
    }

    fn apply(&mut self, cmd: &Command, now: u32) -> Outcome {
        if !gse_proto::is_valid_action(cmd.control, cmd.action) {
            return Outcome::reject(AckResult::UnknownAction, Reason::NotPermitted);
        }
        match cmd.control {
            Control::System => self.apply_system(cmd, now),
            Control::Fill => self.apply_fill(cmd, now),
            Control::Srv => self.apply_srv(cmd, now),
            Control::Ign => self.apply_ign(cmd, now),
            Control::SrvIgn => self.apply_srv_ign(cmd, now),
        }
    }

    fn apply_system(&mut self, cmd: &Command, now: u32) -> Outcome {
        match cmd.action {
            gse_proto::SYSTEM_ACTION_SAFE_ALL => {
                let estop = cmd.flags & cmd_flags::ESTOP != 0;
                let was_fill = self.state.fill;
                let was_srv = self.state.srv;
                let was_ign = self.state.ign;
                if was_fill {
                    if let Some(ev) = self.close_fill(now, estop, false) {
                        self.emit_event(ev.0, ev.1, now);
                    }
                }
                if was_srv {
                    self.state.srv = false;
                    self.hal.set_srv(false);
                }
                if was_ign {
                    self.state.ign = false;
                    self.hal.set_ign(false);
                }
                if estop {
                    self.state.fill_paused = false;
                    self.state.fill_target_ms = 0;
                    self.state.estop = true;
                    self.emit_event(EventCode::Estop, 0, now);
                } else if was_fill || was_srv || was_ign {
                    self.emit_event(EventCode::IgnSafe, 0, now);
                }
                self.state.last_rejected = false;
                Outcome::ok(0, None)
            }
            gse_proto::SYSTEM_ACTION_RESET_FILL => {
                if self.state.fill {
                    return Outcome::reject(AckResult::RejectedState, Reason::FillActive);
                }
                if self.state.ign {
                    return Outcome::reject(AckResult::RejectedSafety, Reason::IgnPowered);
                }
                self.state.fill_paused = false;
                self.state.fill_target_ms = 0;
                self.state.fill_duration_s = 0;
                self.state.fill_seq_complete = false;
                self.state.fill_seq_aborted = false;
                self.state.fill_auto_off = false;
                self.state.estop = false;
                self.state.inhibited = false;
                self.state.link_loss_safe = false;
                self.state.last_rejected = false;
                self.emit_event(EventCode::FillSeqReset, 0, now);
                Outcome::ok(0, None)
            }
            gse_proto::SYSTEM_ACTION_CLEAR_FAULTS => {
                self.state.faults = 0;
                self.state.inhibited = false;
                self.state.last_rejected = false;
                self.state.link_loss_safe = false;
                self.emit_event(EventCode::FaultsCleared, 0, now);
                Outcome::ok(0, None)
            }
            _ => Outcome::reject(AckResult::UnknownAction, Reason::NotPermitted),
        }
    }

    /// FILL / SRV / IGN all share the transition table from `gse-proto`; this
    /// function only applies the side effects after the machine has allowed
    /// the move.
    fn apply_control(&mut self, control: Control, cmd: &Command, now: u32) -> Outcome {
        let event = match ControlEvent::from_command(control, cmd.action) {
            Some(event) => event,
            None => return Outcome::reject(AckResult::UnknownAction, Reason::NotPermitted),
        };
        match self.transition(event) {
            Transition::Blocked(reason) => Outcome::reject(AckResult::RejectedSafety, reason),
            Transition::NoOp(_) => Outcome::ok(0, None),
            Transition::Allowed(_) => match control {
                Control::Fill if cmd.action == ACTION_ON => {
                    let resuming = self.state.fill_paused && self.state.fill_target_ms > 0;
                    if !resuming {
                        self.state.fill_duration_s = cmd.arg_s;
                        self.state.fill_target_ms = (cmd.arg_s as u32).saturating_mul(1000);
                    }
                    let event_secs = self.state.fill_target_ms.div_ceil(1000).min(255) as u8;
                    self.state.fill = true;
                    self.state.fill_timer_active = true;
                    self.state.fill_start_ms = now;
                    self.state.fill_paused = false;
                    self.state.fill_seq_complete = false;
                    self.state.fill_seq_aborted = false;
                    self.state.fill_auto_off = false;
                    self.state.inhibited = false;
                    self.hal.set_fill(true);
                    Outcome::ok(SETTLE_SERVO_MS, Some((EventCode::FillOn, event_secs)))
                }
                Control::Fill => {
                    let is_auto = cmd.flags & cmd_flags::AUTO != 0;
                    let ev = self.close_fill(now, false, is_auto);
                    Outcome::ok(SETTLE_SERVO_MS, ev)
                }
                Control::Srv if cmd.action == ACTION_ON => {
                    self.state.srv = true;
                    self.hal.set_srv(true);
                    Outcome::ok(SETTLE_SERVO_MS, Some((EventCode::SrvOn, 0)))
                }
                Control::Srv => {
                    self.state.srv = false;
                    self.hal.set_srv(false);
                    Outcome::ok(SETTLE_SERVO_MS, Some((EventCode::SrvOff, 0)))
                }
                Control::Ign if cmd.action == ACTION_ON => {
                    self.state.ign = true;
                    self.hal.set_ign(true);
                    Outcome::ok(SETTLE_RELAY_MS, Some((EventCode::IgnPwr, 0)))
                }
                Control::Ign => {
                    self.state.ign = false;
                    self.hal.set_ign(false);
                    Outcome::ok(SETTLE_RELAY_MS, Some((EventCode::IgnSafe, 0)))
                }
                Control::SrvIgn if cmd.action == ACTION_ON => {
                    self.state.srv = true;
                    self.hal.set_srv(true);
                    self.emit_event(EventCode::SrvOn, 0, now);
                    self.state.ign = true;
                    self.hal.set_ign(true);
                    Outcome::ok(SETTLE_SERVO_MS, Some((EventCode::IgnPwr, 0)))
                }
                Control::SrvIgn => {
                    // Always safe the pyrotechnic igniter first before removing servo power.
                    if self.state.ign {
                        self.state.ign = false;
                        self.hal.set_ign(false);
                        self.emit_event(EventCode::IgnSafe, 0, now);
                    }
                    self.state.srv = false;
                    self.hal.set_srv(false);
                    Outcome::ok(SETTLE_SERVO_MS, Some((EventCode::SrvOff, 0)))
                }
                // Not reachable: `apply_control` is only called for actuator
                // controls. Reject rather than panic on the vehicle.
                Control::System => Outcome::reject(AckResult::UnknownControl, Reason::NotPermitted),
            },
        }
    }

    fn apply_fill(&mut self, cmd: &Command, now: u32) -> Outcome {
        if cmd.action == ACTION_ON
            && !(gse_proto::MIN_FILL_SECONDS..=gse_proto::MAX_FILL_SECONDS).contains(&cmd.arg_s)
        {
            return Outcome::reject(AckResult::BadArgument, Reason::InvalidFillTime);
        }
        self.apply_control(Control::Fill, cmd, now)
    }

    fn apply_srv(&mut self, cmd: &Command, now: u32) -> Outcome {
        self.apply_control(Control::Srv, cmd, now)
    }

    fn apply_ign(&mut self, cmd: &Command, now: u32) -> Outcome {
        self.apply_control(Control::Ign, cmd, now)
    }

    fn apply_srv_ign(&mut self, cmd: &Command, now: u32) -> Outcome {
        self.apply_control(Control::SrvIgn, cmd, now)
    }

    /// Close the fill valve and record whether the sequence completed, paused,
    /// or aborted due to an emergency stop / link loss.
    fn close_fill(
        &mut self,
        now: u32,
        force_abort: bool,
        is_auto: bool,
    ) -> Option<(EventCode, u8)> {
        if !self.state.fill {
            return None;
        }
        let was_timed = self.state.fill_timer_active;
        let elapsed = elapsed_ms(now, self.state.fill_start_ms);
        let target_ms = self.state.fill_target_ms;
        let completed = was_timed
            && (elapsed >= target_ms
                || (is_auto && elapsed.saturating_add(FILL_COMPLETE_TOLERANCE_MS) >= target_ms));
        self.state.fill = false;
        self.state.fill_timer_active = false;
        self.hal.set_fill(false);
        if completed {
            self.state.fill_paused = false;
            self.state.fill_target_ms = 0;
            self.state.fill_seq_complete = true;
            self.state.fill_seq_aborted = false;
            Some((
                EventCode::FillOff,
                self.state.fill_duration_s.min(255) as u8,
            ))
        } else if force_abort {
            self.state.fill_paused = false;
            self.state.fill_target_ms = 0;
            self.state.fill_seq_complete = false;
            self.state.fill_seq_aborted = true;
            Some((EventCode::FillAborted, (elapsed / 1000).min(255) as u8))
        } else {
            let remaining_ms = target_ms.saturating_sub(elapsed);
            self.state.fill_paused = true;
            self.state.fill_target_ms = remaining_ms;
            self.state.fill_seq_complete = false;
            self.state.fill_seq_aborted = false;
            Some((
                EventCode::FillOff,
                remaining_ms.div_ceil(1000).min(255) as u8,
            ))
        }
    }

    fn check_fill_timer(&mut self, now: u32) {
        if !self.state.fill || !self.state.fill_timer_active {
            return;
        }
        let elapsed = elapsed_ms(now, self.state.fill_start_ms);
        if elapsed
            >= self
                .state
                .fill_target_ms
                .saturating_add(FILL_FALLBACK_EXTRA_MS)
        {
            let duration = self.state.fill_duration_s;
            self.state.fill_auto_off = true;
            if let Some(_ev) = self.close_fill(now, false, true) {
                self.state.fill_paused = false;
                self.state.fill_target_ms = 0;
                self.state.fill_seq_complete = true;
                self.state.fill_seq_aborted = false;
                self.emit_event(EventCode::FillAutoOff, duration.min(255) as u8, now);
            }
        }
    }

    fn check_link(&mut self, now: u32) {
        if !self.saw_host || self.link_lost {
            return;
        }
        if elapsed_ms(now, self.last_host_activity) <= LINK_TIMEOUT_MS {
            return;
        }
        self.link_lost = true;
        self.state.link_loss_safe = true;
        self.state.faults |= gse_proto::faults::LINK_TIMEOUT;
        let active = self.state.fill || self.state.srv || self.state.ign;
        if active {
            if self.state.fill {
                let _ = self.close_fill(now, true, false);
            }
            self.state.srv = false;
            self.hal.set_srv(false);
            self.state.ign = false;
            self.hal.set_ign(false);
            self.state.inhibited = true;
            self.emit_event(EventCode::LinkTimeoutSafe, 0, now);
        }
    }

    fn flush_pending_ack(&mut self, now: u32) {
        let pending = match self.pending {
            Some(p) => p,
            None => return,
        };
        if !deadline_passed(now, pending.due_ms) {
            return;
        }
        let state = self.snapshot_at(now);
        let ack = gse_proto::Ack {
            cmd_id: pending.cmd_id,
            control: pending.control,
            action: pending.action,
            result: pending.result,
            reason: pending.reason,
            exec_ms: elapsed_ms(now, pending.applied_ms).min(u16::MAX as u32) as u16,
            state,
        };
        if self.send_msg(Message::Ack(ack), Endpoint::Host) {
            self.last_ack = Some(ack);
            self.pending = None;
        }
    }

    fn send_state(&mut self, now: u32) {
        let state = self.snapshot_at(now);
        if self.send_msg(Message::State(state), Endpoint::Host) {
            self.last_state_tx = now;
        }
    }

    fn send_hello(&mut self, now: u32) {
        let hello = Hello {
            protocol: gse_proto::PROTOCOL_VERSION,
            fw: {
                let (a, b, c) = gse_proto::FIRMWARE_VERSION;
                [a, b, c]
            },
            role: Endpoint::Vehicle,
            uptime_ms: elapsed_ms(now, self.boot_ms),
            device_id: self.hal.device_id(),
            capabilities: gse_proto::capabilities::RADIO
                | gse_proto::capabilities::SERVOS
                | gse_proto::capabilities::IGNITER
                | gse_proto::capabilities::WATCHDOG
                | gse_proto::capabilities::FILL_TIMER
                | gse_proto::capabilities::LINK_CONFIG
                | gse_proto::capabilities::RADIO_E28_2G4M27S,
        };
        if self.send_msg(Message::Hello(hello), Endpoint::Broadcast) {
            self.last_hello_tx = now;
        } else {
            // Retry next poll.
            self.last_hello_tx = now.wrapping_sub(HELLO_PERIOD_MS - 10);
        }
    }

    fn emit_event(&mut self, code: EventCode, detail: u8, now: u32) {
        let state = self.snapshot_at(now);
        let event = Event {
            code,
            detail,
            t_ms: elapsed_ms(now, self.boot_ms),
            state,
        };
        self.send_msg(Message::Event(event), Endpoint::Host);
    }

    fn send_msg(&mut self, message: Message, dst: Endpoint) -> bool {
        let now = self.hal.now_ms();
        let mut buf = [0u8; Frame::MAX_ENCODED];
        let seq = self.tx_seq;
        match encode_message(&message, Endpoint::Vehicle, dst, seq, &mut buf) {
            Some(n) => {
                if self.decoder.tx_inhibited(now) || !self.hal.radio_ready() {
                    return false;
                }
                self.hal.write_radio(&buf[..n]);
                self.tx_seq = seq.wrapping_add(1);
                true
            }
            None => false,
        }
    }
}

#[cfg(test)]
mod tests {
    use super::*;
    use crate::mock::MockVehicle;
    use gse_proto::cmd_flags::AUTO;
    use gse_proto::{flags, Ack, Ping};

    fn new_vehicle() -> Vehicle<MockVehicle> {
        Vehicle::new(MockVehicle::default())
    }

    fn cmd(control: Control, action: u8, arg_s: u16, id: u16) -> Command {
        Command {
            cmd_id: id,
            control,
            action,
            arg_s,
            flags: 0,
        }
    }

    fn send(vehicle: &mut Vehicle<MockVehicle>, command: Command) {
        vehicle.hal_mut().feed_frame(
            &Message::Command(command),
            Endpoint::Host,
            Endpoint::Vehicle,
            1,
        );
    }

    fn poll_until_ack(vehicle: &mut Vehicle<MockVehicle>) -> Vec<Ack> {
        let mut acks = Vec::new();
        for _ in 0..20 {
            vehicle.poll();
            for frame in vehicle.hal_mut().drain_frames() {
                if let Some(Message::Ack(ack)) = frame.message() {
                    acks.push(ack);
                }
            }
            if !acks.is_empty() {
                return acks;
            }
            vehicle.hal_mut().advance(SETTLE_SERVO_MS as u32 + 10);
        }
        acks
    }

    #[test]
    fn boots_safe_and_announces() {
        let mut vehicle = new_vehicle();
        assert!(vehicle.state().boot_safe);
        vehicle.poll();
        let frames = vehicle.hal_mut().drain_frames();
        assert!(frames
            .iter()
            .any(|f| matches!(f.message(), Some(Message::Event(e)) if e.code == EventCode::Boot)));
        assert!(frames
            .iter()
            .any(|f| matches!(f.message(), Some(Message::Hello(_)))));
        assert!(!vehicle.hal().fill && !vehicle.hal().srv && !vehicle.hal().ign);
        assert!(vehicle.hal().watchdog_kicks > 0);
    }

    #[test]
    fn fill_on_then_ack_after_settle() {
        let mut vehicle = new_vehicle();
        vehicle.poll();
        vehicle.hal_mut().drain_frames();
        send(&mut vehicle, cmd(Control::Fill, 1, 60, 1));
        vehicle.poll();
        assert!(vehicle.hal().fill, "valve should be open immediately");
        let acks = poll_until_ack(&mut vehicle);
        assert_eq!(acks.len(), 1);
        assert_eq!(acks[0].cmd_id, 1);
        assert!(acks[0].result.is_ok());
        assert!(acks[0].state.fill_open());
        assert!(acks[0].exec_ms >= SETTLE_SERVO_MS);
        assert!(vehicle.state().fill_timer_active);
    }

    #[test]
    fn fill_and_srv_are_mutually_exclusive() {
        let mut vehicle = new_vehicle();
        vehicle.poll();

        // SRV is locked prior to any FILL run.
        send(&mut vehicle, cmd(Control::Srv, 1, 0, 1));
        let acks = poll_until_ack(&mut vehicle);
        assert_eq!(acks[0].result, AckResult::RejectedSafety);
        assert_eq!(acks[0].reason, Reason::FillSeqIncomplete);

        // Open FILL for 60 s: SRV is blocked while FILL is open.
        send(&mut vehicle, cmd(Control::Fill, 1, 60, 2));
        let _ = poll_until_ack(&mut vehicle);
        send(&mut vehicle, cmd(Control::Srv, 1, 0, 3));
        let acks = poll_until_ack(&mut vehicle);
        assert_eq!(acks[0].result, AckResult::RejectedSafety);
        assert_eq!(acks[0].reason, Reason::FillActive);
        assert!(!vehicle.hal().srv);

        // Pause FILL early: SRV unlocks, and while SRV is ON, FILL ON is blocked.
        send(&mut vehicle, cmd(Control::Fill, 0, 0, 4));
        let _ = poll_until_ack(&mut vehicle);
        send(&mut vehicle, cmd(Control::Srv, 1, 0, 5));
        let acks = poll_until_ack(&mut vehicle);
        assert!(acks[0].result.is_ok());
        assert!(vehicle.hal().srv);

        send(&mut vehicle, cmd(Control::Fill, 1, 60, 6));
        let acks = poll_until_ack(&mut vehicle);
        assert_eq!(acks[0].result, AckResult::RejectedSafety);
        assert_eq!(acks[0].reason, Reason::FillSrvExclusive);
        assert!(!vehicle.hal().fill);
    }

    #[test]
    fn ign_requires_fill_run_and_srv() {
        let mut vehicle = new_vehicle();
        vehicle.poll();

        // Nothing done: IGN rejected for SRV; SRV rejected for FILL sequence.
        send(&mut vehicle, cmd(Control::Ign, 1, 0, 1));
        let acks = poll_until_ack(&mut vehicle);
        assert_eq!(acks[0].reason, Reason::SrvRequired);

        send(&mut vehicle, cmd(Control::Srv, 1, 0, 2));
        let acks = poll_until_ack(&mut vehicle);
        assert_eq!(acks[0].reason, Reason::FillSeqIncomplete);
        assert!(!vehicle.hal().srv && !vehicle.hal().ign);
    }

    #[test]
    fn full_sequence_reaches_ignition() {
        let mut vehicle = new_vehicle();
        vehicle.poll();

        // FILL ON for 2 seconds.
        send(&mut vehicle, cmd(Control::Fill, 1, 2, 1));
        let _ = poll_until_ack(&mut vehicle);
        assert!(vehicle.hal().fill);

        // Host closes it after the full duration.
        vehicle.hal_mut().advance(2100);
        vehicle.poll();
        send(&mut vehicle, cmd(Control::Fill, 0, 0, 2));
        let acks = poll_until_ack(&mut vehicle);
        assert!(acks[0].result.is_ok());
        assert!(acks[0].state.fill_seq_complete());
        assert!(!acks[0].state.fill_paused());
        assert!(!vehicle.hal().fill);

        // SRV ON, IGN PWR.
        vehicle.hal_mut().advance(1000);
        send(&mut vehicle, cmd(Control::Srv, 1, 0, 3));
        let _ = poll_until_ack(&mut vehicle);
        assert!(vehicle.hal().srv);
        send(&mut vehicle, cmd(Control::Ign, 1, 0, 4));
        let acks = poll_until_ack(&mut vehicle);
        assert!(acks[0].result.is_ok());
        assert_eq!(acks[0].reason, Reason::None);
        assert!(vehicle.hal().ign);
    }

    #[test]
    fn early_fill_off_pauses_timer_unlocks_outputs_and_resumes_cleanly() {
        let mut vehicle = new_vehicle();
        vehicle.poll();

        // Start a 10 s fill and advance 3.5 s after ACK (total ~3.86 s elapsed).
        send(&mut vehicle, cmd(Control::Fill, 1, 10, 1));
        let _ = poll_until_ack(&mut vehicle);
        vehicle.hal_mut().advance(3_500);

        // Manual FILL -> OFF prior to expiry pauses the timer and unlocks SRV.
        send(&mut vehicle, cmd(Control::Fill, 0, 0, 2));
        let acks = poll_until_ack(&mut vehicle);
        assert!(acks[0].result.is_ok());
        assert!(acks[0].state.fill_paused());
        assert!(!acks[0].state.fill_seq_complete());
        assert!(!acks[0].state.fill_seq_aborted());
        let saved_remaining_ms = acks[0].state.fill_remaining_ms;
        assert!(
            (6_000..=6_300).contains(&saved_remaining_ms),
            "unexpected remaining ms: {saved_remaining_ms}"
        );

        // Advancing time while paused does NOT decrement the remaining fill duration.
        vehicle.hal_mut().advance(4_000);
        assert_eq!(vehicle.snapshot().fill_remaining_ms, saved_remaining_ms);

        // SRV unlocks and turns ON cleanly while FILL is paused, then OFF again.
        send(&mut vehicle, cmd(Control::Srv, 1, 0, 3));
        let acks = poll_until_ack(&mut vehicle);
        assert!(acks[0].result.is_ok());
        assert!(vehicle.hal().srv);
        send(&mut vehicle, cmd(Control::Srv, 0, 0, 4));
        let _ = poll_until_ack(&mut vehicle);
        assert!(!vehicle.hal().srv);

        // Turning FILL -> ON resumes from `saved_remaining_ms` rather than restarting 10 s.
        send(&mut vehicle, cmd(Control::Fill, 1, 10, 5));
        let acks = poll_until_ack(&mut vehicle);
        assert!(acks[0].result.is_ok());
        assert!(acks[0].state.fill_open());
        assert!(!acks[0].state.fill_paused());
        assert!(acks[0].state.fill_remaining_ms < saved_remaining_ms);

        // Advancing just past `saved_remaining_ms` completes the full 10 s fill!
        vehicle.hal_mut().advance(saved_remaining_ms + 100);
        send(&mut vehicle, cmd(Control::Fill, 0, 0, 6));
        let acks = poll_until_ack(&mut vehicle);
        assert!(acks[0].result.is_ok());
        assert!(acks[0].state.fill_seq_complete());
        assert!(!acks[0].state.fill_paused());
        assert_eq!(acks[0].state.fill_remaining_ms, 0);
    }

    #[test]
    fn vehicle_auto_closes_fill_if_host_vanishes() {
        let mut vehicle = new_vehicle();
        vehicle.poll();
        send(&mut vehicle, cmd(Control::Fill, 1, 1, 1));
        let _ = poll_until_ack(&mut vehicle);
        assert!(vehicle.hal().fill);

        // Advance past duration + fallback without any host traffic.
        vehicle
            .hal_mut()
            .advance(1_000 + FILL_FALLBACK_EXTRA_MS + 100);
        vehicle.poll();
        assert!(!vehicle.hal().fill, "vehicle must close the valve itself");
        let frames = vehicle.hal_mut().drain_frames();
        let event = frames
            .iter()
            .find_map(|f| match f.message() {
                Some(Message::Event(e)) if e.code == EventCode::FillAutoOff => Some(e),
                _ => None,
            })
            .expect("FillAutoOff event");
        assert!(event.state.fill_seq_complete());
        assert!(!event.state.fill_open());
        assert!(vehicle.state().fill_auto_off);
    }

    #[test]
    fn link_timeout_forces_all_safe_and_inhibits() {
        let mut vehicle = new_vehicle();
        vehicle.poll();
        // Start and pause a fill so SRV unlocks, then enable SRV.
        send(&mut vehicle, cmd(Control::Fill, 1, 10, 1));
        let _ = poll_until_ack(&mut vehicle);
        send(&mut vehicle, cmd(Control::Fill, 0, 0, 2));
        let _ = poll_until_ack(&mut vehicle);
        send(&mut vehicle, cmd(Control::Srv, 1, 0, 3));
        let _ = poll_until_ack(&mut vehicle);
        assert!(vehicle.hal().srv);
        vehicle.hal_mut().drain_frames();

        vehicle.hal_mut().advance(LINK_TIMEOUT_MS + 1);
        vehicle.poll();
        assert!(!vehicle.hal().srv);
        assert!(vehicle.state().link_loss_safe);
        assert!(vehicle.state().inhibited);
        let frames = vehicle.hal_mut().drain_frames();
        assert!(frames.iter().any(|f| matches!(
            f.message(),
            Some(Message::Event(e)) if e.code == EventCode::LinkTimeoutSafe
        )));
    }

    #[test]
    fn link_loss_mid_fill_marks_fill_aborted_and_locks_srv_and_ign() {
        let mut vehicle = new_vehicle();
        vehicle.poll();
        send(&mut vehicle, cmd(Control::Fill, 1, 60, 1));
        let _ = poll_until_ack(&mut vehicle);
        assert!(vehicle.hal().fill);
        vehicle.hal_mut().drain_frames();

        vehicle.hal_mut().advance(LINK_TIMEOUT_MS + 1);
        vehicle.poll();
        assert!(!vehicle.hal().fill);
        assert!(vehicle.state().fill_seq_aborted);
        assert!(!vehicle.state().fill_paused);

        // Even after clearing faults, the aborted fill cannot unlock SRV or SRV+IGN.
        send(
            &mut vehicle,
            cmd(Control::System, gse_proto::SYSTEM_ACTION_CLEAR_FAULTS, 0, 2),
        );
        let _ = poll_until_ack(&mut vehicle);
        send(&mut vehicle, cmd(Control::Srv, 1, 0, 3));
        let acks = poll_until_ack(&mut vehicle);
        assert_eq!(acks[0].result, AckResult::RejectedSafety);
        assert_eq!(acks[0].reason, Reason::FillSeqIncomplete);
    }

    #[test]
    fn srv_cannot_be_removed_while_ignited() {
        let mut vehicle = new_vehicle();
        vehicle.poll();
        // Complete a short fill.
        send(&mut vehicle, cmd(Control::Fill, 1, 1, 1));
        let _ = poll_until_ack(&mut vehicle);
        vehicle.hal_mut().advance(1100);
        send(&mut vehicle, cmd(Control::Fill, 0, 0, 2));
        let _ = poll_until_ack(&mut vehicle);
        send(&mut vehicle, cmd(Control::Srv, 1, 0, 3));
        let _ = poll_until_ack(&mut vehicle);
        send(&mut vehicle, cmd(Control::Ign, 1, 0, 4));
        let _ = poll_until_ack(&mut vehicle);
        assert!(vehicle.hal().ign);

        send(&mut vehicle, cmd(Control::Srv, 0, 0, 5));
        let acks = poll_until_ack(&mut vehicle);
        assert_eq!(acks[0].result, AckResult::RejectedSafety);
        assert_eq!(acks[0].reason, Reason::IgnPowered);
        assert!(vehicle.hal().srv);

        // But SAFE is always allowed.
        send(&mut vehicle, cmd(Control::Ign, 0, 0, 6));
        let acks = poll_until_ack(&mut vehicle);
        assert!(acks[0].result.is_ok());
        assert!(!vehicle.hal().ign);
    }

    #[test]
    fn estop_safes_everything() {
        let mut vehicle = new_vehicle();
        vehicle.poll();
        send(&mut vehicle, cmd(Control::Fill, 1, 60, 1));
        let _ = poll_until_ack(&mut vehicle);
        let mut estop = cmd(Control::System, gse_proto::SYSTEM_ACTION_SAFE_ALL, 0, 2);
        estop.flags = cmd_flags::ESTOP;
        send(&mut vehicle, estop);
        let acks = poll_until_ack(&mut vehicle);
        assert!(acks[0].result.is_ok());
        assert!(!vehicle.hal().fill && !vehicle.hal().srv && !vehicle.hal().ign);
        assert!(vehicle.state().estop);
        assert!(vehicle.state().fill_seq_aborted);
    }

    #[test]
    fn duplicate_command_reacks_without_side_effects() {
        let mut vehicle = new_vehicle();
        vehicle.poll();
        send(&mut vehicle, cmd(Control::Fill, 1, 10, 5));
        let first = poll_until_ack(&mut vehicle);
        assert!(first[0].result.is_ok());
        assert!(vehicle.hal().fill);

        // Retry the exact same command after the ack was sent.
        send(&mut vehicle, cmd(Control::Fill, 1, 10, 5));
        vehicle.poll();
        let frames = vehicle.hal_mut().drain_frames();
        let acks: Vec<_> = frames
            .iter()
            .filter_map(|f| match f.message() {
                Some(Message::Ack(a)) => Some(a),
                _ => None,
            })
            .collect();
        assert_eq!(acks.len(), 1);
        assert_eq!(acks[0].cmd_id, 5);
        assert!(acks[0].result.is_ok());
    }

    #[test]
    fn out_of_range_fill_duration_rejected() {
        let mut vehicle = new_vehicle();
        vehicle.poll();
        send(&mut vehicle, cmd(Control::Fill, 1, 0, 1));
        let acks = poll_until_ack(&mut vehicle);
        assert_eq!(acks[0].result, AckResult::BadArgument);
        assert_eq!(acks[0].reason, Reason::InvalidFillTime);
        assert!(!vehicle.hal().fill);
    }

    #[test]
    fn ping_gets_pong_and_keeps_link_alive() {
        let mut vehicle = new_vehicle();
        vehicle.poll();
        vehicle.hal_mut().drain_frames();
        vehicle.hal_mut().feed_frame(
            &Message::Ping(Ping {
                nonce: 7,
                host_ms: 1,
            }),
            Endpoint::Host,
            Endpoint::Vehicle,
            9,
        );
        vehicle.poll();
        let frames = vehicle.hal_mut().drain_frames();
        let pong = frames.iter().find_map(|f| match f.message() {
            Some(Message::Pong(p)) => Some(p),
            _ => None,
        });
        assert_eq!(pong.expect("pong").nonce, 7);
        assert!(vehicle.state().boot_safe);
    }

    #[test]
    fn state_snapshot_reports_flags_and_remaining() {
        let mut vehicle = new_vehicle();
        vehicle.poll();
        send(&mut vehicle, cmd(Control::Fill, 1, 10, 1));
        let _ = poll_until_ack(&mut vehicle);
        vehicle.hal_mut().advance(4000);
        let state = vehicle.snapshot();
        assert!(state.fill_open());
        assert!(state.fill_timer_active());
        // 10 s fill, ~4.36 s elapsed since FILL ON.
        assert!((5_500..=5_700).contains(&state.fill_remaining_ms));
        assert_eq!(state.fill_duration_s, 10);
        assert!(state.has(flags::FILL_OPEN));
    }

    #[test]
    fn reset_fill_sequence_clears_latch() {
        let mut vehicle = new_vehicle();
        vehicle.poll();
        send(&mut vehicle, cmd(Control::Fill, 1, 1, 1));
        let _ = poll_until_ack(&mut vehicle);
        vehicle.hal_mut().advance(1100);
        send(&mut vehicle, cmd(Control::Fill, 0, 0, 2));
        let _ = poll_until_ack(&mut vehicle);
        assert!(vehicle.state().fill_seq_complete);

        send(
            &mut vehicle,
            cmd(Control::System, gse_proto::SYSTEM_ACTION_RESET_FILL, 0, 3),
        );
        let acks = poll_until_ack(&mut vehicle);
        assert!(acks[0].result.is_ok());
        assert!(!vehicle.state().fill_seq_complete);
        assert!(!vehicle.state().fill_seq_aborted);
    }

    #[test]
    fn ack_is_held_until_settle_time() {
        let mut vehicle = new_vehicle();
        vehicle.poll();
        vehicle.hal_mut().drain_frames();
        send(&mut vehicle, cmd(Control::Fill, 1, 30, 1));
        vehicle.poll();
        // Immediately: valve open but no ack yet.
        let frames = vehicle.hal_mut().drain_frames();
        assert!(!frames
            .iter()
            .any(|f| matches!(f.message(), Some(Message::Ack(_)))));
        // After the servo settle time the ack appears.
        vehicle.hal_mut().advance(SETTLE_SERVO_MS as u32);
        vehicle.poll();
        let frames = vehicle.hal_mut().drain_frames();
        assert!(frames
            .iter()
            .any(|f| matches!(f.message(), Some(Message::Ack(_)))));
    }

    #[test]
    fn corrupt_frames_do_not_change_outputs() {
        let mut vehicle = new_vehicle();
        vehicle.poll();
        vehicle
            .hal_mut()
            .feed_bytes(&[0x00, 0x12, 0x34, 0x56, 0x78, 0x00, 0xFF]);
        vehicle.poll();
        assert!(!vehicle.hal().fill && !vehicle.hal().srv && !vehicle.hal().ign);
        assert!(
            vehicle.state().faults & gse_proto::faults::CRC_ERROR != 0
                || vehicle.decoder_stats().malformed > 0
        );
    }

    #[test]
    fn retry_while_pending_produces_one_ack() {
        let mut vehicle = new_vehicle();
        vehicle.poll();
        send(&mut vehicle, cmd(Control::Fill, 1, 30, 9));
        vehicle.poll();
        // Retry while the servo is still settling: no panic, no extra ack.
        send(&mut vehicle, cmd(Control::Fill, 1, 30, 9));
        vehicle.poll();
        let acks = poll_until_ack(&mut vehicle);
        assert_eq!(acks.len(), 1);
        assert_eq!(acks[0].cmd_id, 9);
        assert!(acks[0].result.is_ok());
    }

    #[test]
    fn estop_latch_blocks_ignition_until_reset() {
        let mut vehicle = new_vehicle();
        vehicle.poll();
        send(&mut vehicle, cmd(Control::Fill, 1, 1, 1));
        let _ = poll_until_ack(&mut vehicle);
        vehicle.hal_mut().advance(1100);
        send(&mut vehicle, cmd(Control::Fill, 0, 0, 2));
        let _ = poll_until_ack(&mut vehicle);
        send(&mut vehicle, cmd(Control::Srv, 1, 0, 3));
        let _ = poll_until_ack(&mut vehicle);

        let mut estop = cmd(Control::System, gse_proto::SYSTEM_ACTION_SAFE_ALL, 0, 4);
        estop.flags = cmd_flags::ESTOP;
        send(&mut vehicle, estop);
        let acks = poll_until_ack(&mut vehicle);
        assert!(acks[0].result.is_ok());
        assert!(vehicle.state().estop);
        assert!(!vehicle.hal().fill && !vehicle.hal().srv && !vehicle.hal().ign);

        // Both SRV and IGN remain inhibited by the latched E-STOP (the shared
        // machine returns NotPermitted).
        send(&mut vehicle, cmd(Control::Srv, 1, 0, 5));
        let acks = poll_until_ack(&mut vehicle);
        assert_eq!(acks[0].result, AckResult::RejectedSafety);
        assert_eq!(acks[0].reason, Reason::NotPermitted);
        assert!(!vehicle.hal().srv && !vehicle.hal().ign);

        // RESET CONTROLS (SAFE ALL + RESET FILL) clears the latch and the
        // fill-sequence latch, returning every output to its initial state.
        send(
            &mut vehicle,
            cmd(Control::System, gse_proto::SYSTEM_ACTION_SAFE_ALL, 0, 7),
        );
        let _ = poll_until_ack(&mut vehicle);
        send(
            &mut vehicle,
            cmd(Control::System, gse_proto::SYSTEM_ACTION_RESET_FILL, 0, 8),
        );
        let acks = poll_until_ack(&mut vehicle);
        assert!(acks[0].result.is_ok());
        assert!(!vehicle.state().estop);
        assert!(!vehicle.state().inhibited);
        assert!(!vehicle.state().fill_seq_complete);
        assert!(!vehicle.state().fill_seq_aborted);
        assert!(!vehicle.hal().fill && !vehicle.hal().srv && !vehicle.hal().ign);
    }

    #[test]
    fn auto_flag_is_accepted_without_changing_semantics() {
        let mut vehicle = new_vehicle();
        vehicle.poll();
        let mut c = cmd(Control::Fill, 1, 2, 1);
        c.flags = AUTO;
        send(&mut vehicle, c);
        let acks = poll_until_ack(&mut vehicle);
        assert!(acks[0].result.is_ok());
        assert!(vehicle.hal().fill);
    }

    #[test]
    fn live_srv_ign_sequence_enforces_fill_interlock_and_toggles_outputs() {
        let mut vehicle = new_vehicle();
        vehicle.poll();

        // Reject SRV+IGN ON before fill sequence completes.
        send(&mut vehicle, cmd(Control::SrvIgn, 1, 0, 1));
        let acks = poll_until_ack(&mut vehicle);
        assert_eq!(acks[0].result, AckResult::RejectedSafety);
        assert_eq!(acks[0].reason, Reason::FillSeqIncomplete);
        assert!(!vehicle.hal().srv && !vehicle.hal().ign);

        // Open fill for 2 s and reject SRV+IGN while valve is still open.
        send(&mut vehicle, cmd(Control::Fill, 1, 2, 2));
        let _ = poll_until_ack(&mut vehicle);
        send(&mut vehicle, cmd(Control::SrvIgn, 1, 0, 3));
        let acks = poll_until_ack(&mut vehicle);
        assert_eq!(acks[0].result, AckResult::RejectedSafety);
        assert_eq!(acks[0].reason, Reason::FillActive);
        assert!(!vehicle.hal().srv && !vehicle.hal().ign);

        // Complete fill sequence.
        vehicle.hal_mut().advance(2_100);
        send(&mut vehicle, cmd(Control::Fill, 0, 0, 4));
        let acks = poll_until_ack(&mut vehicle);
        assert!(acks[0].result.is_ok());
        assert!(vehicle.state().fill_seq_complete);

        // SRV+IGN ON energises both SRV and IGN and ACKs ON/PWR.
        send(&mut vehicle, cmd(Control::SrvIgn, 1, 0, 5));
        let acks = poll_until_ack(&mut vehicle);
        assert!(acks[0].result.is_ok());
        assert!(vehicle.hal().srv && vehicle.hal().ign);
        assert!(acks[0].state.srv_on() && acks[0].state.ign_pwr());

        // SRV+IGN OFF safes both IGN and SRV back to OFF/SAFE.
        send(&mut vehicle, cmd(Control::SrvIgn, 0, 0, 6));
        let acks = poll_until_ack(&mut vehicle);
        assert!(acks[0].result.is_ok());
        assert!(!vehicle.hal().srv && !vehicle.hal().ign);
        assert!(acks[0].state.outputs_safe());
    }

    #[test]
    fn dx_lr42_at_mode_trap_sets_radio_fault_and_self_recovers() {
        let mut vehicle = new_vehicle();
        vehicle.poll();
        vehicle.hal_mut().radio_out.clear();

        vehicle.hal_mut().feed_bytes(b"Entry AT\r\n");
        vehicle.poll();
        assert_eq!(vehicle.radio_driver_stats().at_recoveries, 1);
        assert_ne!(vehicle.state().faults & gse_proto::faults::RADIO_ERROR, 0);

        vehicle.hal_mut().advance(crate::radio::AT_PRE_GUARD_MS);
        vehicle.poll();
        assert_eq!(&vehicle.hal().radio_out[..], b"+++");
        vehicle.hal_mut().radio_out.clear();

        vehicle.hal_mut().advance(crate::radio::AT_POST_SETTLE_MS);
        send(&mut vehicle, cmd(Control::Fill, 1, 2, 1));
        let acks = poll_until_ack(&mut vehicle);
        assert!(acks[0].result.is_ok());
        assert!(vehicle.hal().fill);
    }

    #[test]
    fn link_config_requires_safe_outputs_probes_spi_and_switches_after_ack() {
        let mut vehicle = new_vehicle();
        vehicle.poll();
        vehicle.hal_mut().drain_frames();

        // 1. Reject live radio reconfiguration while FILL valve is open.
        send(&mut vehicle, cmd(Control::Fill, 1, 10, 1));
        let _ = poll_until_ack(&mut vehicle);
        let req = LinkConfig {
            req_id: 77,
            usb_baud: gse_proto::UsbBaud::Baud230400,
            uart_baud: gse_proto::DxLr42Baud::Baud115200,
            radio: gse_proto::RadioModule::E28_2G4M27S,
            flags: link_cfg_flags::PERSIST,
        };
        vehicle.hal_mut().feed_frame(
            &Message::LinkConfig(req),
            Endpoint::Host,
            Endpoint::Vehicle,
            2,
        );
        vehicle.poll();
        let frames = vehicle.hal_mut().drain_frames();
        assert!(frames.iter().any(|f| matches!(
            f.message(),
            Some(Message::LinkConfigAck(a))
                if a.req_id == 77
                    && a.result == AckResult::RejectedSafety
                    && a.reason == Reason::FillActive
        )));
        assert_eq!(vehicle.hal().link.radio, gse_proto::RadioModule::DxLr42);

        // Close FILL so all outputs are SAFE.
        send(&mut vehicle, cmd(Control::Fill, 0, 0, 2));
        let _ = poll_until_ack(&mut vehicle);

        // 2. Missing E28 hardware fails pre-flight probe without losing the active link.
        vehicle.hal_mut().e28_present = false;
        vehicle.hal_mut().feed_frame(
            &Message::LinkConfig(req),
            Endpoint::Host,
            Endpoint::Vehicle,
            3,
        );
        vehicle.poll();
        let frames = vehicle.hal_mut().drain_frames();
        assert!(frames.iter().any(|f| matches!(
            f.message(),
            Some(Message::LinkConfigAck(a))
                if a.req_id == 77 && a.result == AckResult::HardwareFault
        )));
        assert_eq!(vehicle.hal().link.radio, gse_proto::RadioModule::DxLr42);

        // 3. With E28 hardware present, ACK leaves on the old radio BEFORE switchover.
        vehicle.hal_mut().e28_present = true;
        vehicle.hal_mut().feed_frame(
            &Message::LinkConfig(req),
            Endpoint::Host,
            Endpoint::Vehicle,
            4,
        );
        vehicle.poll();
        assert_eq!(
            vehicle.hal().link.radio,
            gse_proto::RadioModule::DxLr42,
            "must transmit LinkConfigAck on old radio before switching"
        );
        let frames = vehicle.hal_mut().drain_frames();
        assert!(frames.iter().any(|f| matches!(
            f.message(),
            Some(Message::LinkConfigAck(a)) if a.req_id == 77 && a.result == AckResult::Ok
        )));

        // Next poll after radio TX buffer drains commits switch + flash persistence.
        vehicle.poll();
        assert_eq!(
            vehicle.hal().link.radio,
            gse_proto::RadioModule::E28_2G4M27S
        );
        assert_eq!(
            vehicle.hal().link.uart_baud,
            gse_proto::DxLr42Baud::Baud115200
        );
        assert!(vehicle.hal().link.flash_saved);
    }
}
