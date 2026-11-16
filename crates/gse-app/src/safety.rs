//! Operator-side safety interlocks.
//!
//! Every gate is derived from the shared deterministic control machine in
//! [`gse_proto::fsm`], so the button that greys out and the firmware that
//! rejects a command agree by construction. The app never lets an unsafe
//! transition leave the browser; the vehicle independently re-checks the same
//! table before driving an output.
//!
//! Operating modes:
//! * [`OperationMode::Test`] — vehicle control testing: independent `FILL`,
//!   `SRV`, and `IGN` channels (`SRV` unlocked once `FILL` completes or is
//!   paused `OFF`; `FILL` <-> `SRV` exclusive; `IGN` gated on `SRV ON` +
//!   `FILL` ready, with an abortable countdown that holds at T-0 until the
//!   operator explicitly triggers ignition).
//! * [`OperationMode::Live`] — live launch sequence: `FILL` with pause/resume,
//!   plus combined `SRV+IGN` (`OFF/SAFE` -> abortable countdown -> T-0 HOLD ->
//!   explicit operator trigger -> `ON/PWR` -> `OFF/SAFE`), unlocked once
//!   `FILL` completes or is paused `OFF`.
//!
//! Additional app-only gates:
//!
//! * nothing is enabled before the ground and vehicle links are up,
//! * nothing except E-STOP is enabled while a command is awaiting its ACK,
//! * while a launch/test countdown is running, other channels lock while
//!   `abort_countdown`, the active channel's safe action (`ign_safe` or
//!   `srv_ign_off`), `RESET CONTROLS`, and `E-STOP` remain active; at T-0 HOLD,
//!   only the matching explicit ON trigger is additionally enabled,
//! * control actions stay locked until the startup handshake has confirmed
//!   every output is OFF/SAFE ([`Inputs::controls_ready`]).
//!
//! ESP-NOW pad (GSE-simple control set, [`Inputs::pad`]):
//!
//! * `IGN PWR` / `SRV+IGN ON` (and so their countdowns) additionally require
//!   the operator's ARM, which is what attaches the arm code to the stream,
//! * ARM is refused while a safe state is latched (E-STOP / link loss /
//!   INHIBITED); DISARM is always available,
//! * PURGE needs the per-session purge enable (ECR-01 hazard H8) and is
//!   otherwise a free channel, as in GSE-simple (no FILL interlock).

use gse_proto::fsm::{ControlEvent, ControlState, Interlocks, Transition};
use gse_proto::{Reason, StateSnapshot};

use crate::config::OperationMode;
use crate::state::{ActionState, Actions, CountdownTarget};

pub struct Inputs {
    pub mode: OperationMode,
    pub countdown_seconds: u16,
    pub ground_online: bool,
    pub vehicle_online: bool,
    pub pending: bool,
    /// Target channel while an ignition countdown is ticking or held at T-0.
    pub countdown_target: Option<CountdownTarget>,
    /// Countdown reached T-0; only the matching explicit ignition action is allowed.
    pub countdown_holding: bool,
    /// The startup/reset handshake confirmed FILL OFF, SRV OFF, IGN SAFE.
    pub controls_ready: bool,
    pub state: StateSnapshot,
    /// ESP-NOW pad operator state; `None` for the NUCLEO/LoRa boards.
    pub pad: Option<PadGate>,
}

/// Operator-side state of the ESP-NOW pad link.
#[derive(Clone, Copy, Debug, Default, PartialEq, Eq)]
pub struct PadGate {
    pub armed: bool,
    pub purge_enabled: bool,
    pub purge_open: bool,
    /// E-STOP, link loss or INHIBITED latched: the pad is held SAFE.
    pub latched: bool,
}

const PAD_ONLY: &str = "ESP-NOW pad only";

fn blocked(reason: impl Into<String>) -> ActionState {
    ActionState {
        enabled: false,
        reason: reason.into(),
    }
}

fn allowed() -> ActionState {
    ActionState {
        enabled: true,
        reason: String::new(),
    }
}

/// Operator text for a redundant command.
fn no_op_text(event: ControlEvent) -> &'static str {
    match event {
        ControlEvent::FillOn => "FILL is already ON",
        ControlEvent::FillOff => "FILL is already OFF",
        ControlEvent::SrvOn => "SRV is already ON",
        ControlEvent::SrvOff => "SRV is already OFF",
        ControlEvent::IgnPwr => "IGN is already PWR",
        ControlEvent::IgnSafe => "IGN is already SAFE",
        ControlEvent::SrvIgnOn => "SRV+IGN is already ON/PWR",
        ControlEvent::SrvIgnOff => "SRV+IGN is already OFF/SAFE",
        ControlEvent::SafeAll | ControlEvent::Reset => "All controls are already OFF/SAFE",
    }
}

/// Operator text for a refused transition.
fn blocked_text(event: ControlEvent, interlocks: Interlocks, reason: Reason) -> String {
    match (event, reason) {
        (
            ControlEvent::SrvOn | ControlEvent::IgnPwr | ControlEvent::SrvIgnOn,
            Reason::NotPermitted,
        ) if interlocks.inhibited => {
            "Vehicle inhibited - press RESET CONTROLS and run a new fill".to_string()
        }
        (
            ControlEvent::SrvOn | ControlEvent::IgnPwr | ControlEvent::SrvIgnOn,
            Reason::NotPermitted,
        ) if interlocks.estop => "E-STOP is latched - press RESET CONTROLS first".to_string(),
        (ControlEvent::SrvOn, Reason::FillSeqIncomplete) => {
            "SRV requires FILL to run and turn OFF first".to_string()
        }
        (ControlEvent::IgnPwr, Reason::FillSeqIncomplete) => {
            "IGN requires FILL to run and turn OFF first".to_string()
        }
        (ControlEvent::SrvIgnOn, Reason::FillSeqIncomplete) => {
            "SRV+IGN requires FILL to run and turn OFF first".to_string()
        }
        (ControlEvent::SrvIgnOn, Reason::FillActive) => {
            "SRV+IGN cannot be enabled while FILL is open".to_string()
        }
        _ => reason.label().to_string(),
    }
}

/// Convert one FSM transition into a button gate.
fn gate(
    base: Option<&ActionState>,
    state: ControlState,
    interlocks: Interlocks,
    event: ControlEvent,
) -> ActionState {
    if let Some(base) = base {
        return base.clone();
    }
    match state.transition(event, interlocks) {
        Transition::Allowed(_) => allowed(),
        Transition::NoOp(_) => blocked(no_op_text(event)),
        Transition::Blocked(reason) => blocked(blocked_text(event, interlocks, reason)),
    }
}

pub fn evaluate(inputs: &Inputs) -> Actions {
    let state = ControlState::from_snapshot(&inputs.state);
    let interlocks = Interlocks::from_snapshot(&inputs.state);

    let offline_reason = if !inputs.ground_online {
        "Ground board not connected"
    } else if !inputs.vehicle_online {
        "Vehicle not responding - waiting for PONG"
    } else {
        ""
    };
    let offline = !inputs.ground_online || !inputs.vehicle_online;
    let base = if offline {
        Some(blocked(offline_reason))
    } else if inputs.pending {
        Some(blocked("Command in progress"))
    } else if !inputs.controls_ready {
        Some(blocked(
            "Initialising - confirming all controls are OFF/SAFE",
        ))
    } else {
        None
    };

    let countdown_gate = inputs.countdown_target.map(|target| {
        blocked(format!(
            "{} {} s countdown in progress",
            target.label(),
            inputs.countdown_seconds
        ))
    });
    let channel_base = base.as_ref().or(countdown_gate.as_ref());
    let held_target_base = if inputs.countdown_holding {
        base.as_ref()
    } else {
        channel_base
    };

    let fill_on = gate(channel_base, state, interlocks, ControlEvent::FillOn);
    let fill_off = gate(channel_base, state, interlocks, ControlEvent::FillOff);

    let (srv_on, srv_off, ign_pwr, ign_safe, srv_ign_on, srv_ign_off) = match inputs.mode {
        OperationMode::Test => (
            gate(channel_base, state, interlocks, ControlEvent::SrvOn),
            gate(channel_base, state, interlocks, ControlEvent::SrvOff),
            gate(
                if inputs.countdown_target == Some(CountdownTarget::Ign) {
                    held_target_base
                } else {
                    channel_base
                },
                state,
                interlocks,
                ControlEvent::IgnPwr,
            ),
            if inputs.countdown_target == Some(CountdownTarget::Ign) && base.is_none() {
                allowed()
            } else {
                gate(channel_base, state, interlocks, ControlEvent::IgnSafe)
            },
            blocked("Only available in LIVE mode"),
            blocked("Only available in LIVE mode"),
        ),
        OperationMode::Live => (
            blocked("Disabled in LIVE mode (use SRV+IGN)"),
            blocked("Disabled in LIVE mode (use SRV+IGN)"),
            blocked("Disabled in LIVE mode (use SRV+IGN)"),
            blocked("Disabled in LIVE mode (use SRV+IGN)"),
            gate(
                if inputs.countdown_target == Some(CountdownTarget::SrvIgn) {
                    held_target_base
                } else {
                    channel_base
                },
                state,
                interlocks,
                ControlEvent::SrvIgnOn,
            ),
            if inputs.countdown_target == Some(CountdownTarget::SrvIgn) && base.is_none() {
                allowed()
            } else {
                gate(channel_base, state, interlocks, ControlEvent::SrvIgnOff)
            },
        ),
    };

    let (ign_pwr, srv_ign_on) = match inputs.pad {
        Some(pad) if !pad.armed => (
            require_arm(ign_pwr, "IGN PWR"),
            require_arm(srv_ign_on, "SRV+IGN"),
        ),
        _ => (ign_pwr, srv_ign_on),
    };

    let abort_countdown = match (&base, inputs.countdown_target) {
        (Some(b), _) => b.clone(),
        (None, Some(_)) => allowed(),
        (None, None) => blocked("No active countdown"),
    };

    // RESET CONTROLS always resolves to OFF/SAFE (and cancels any active
    // countdown), so only link state, a pending command or init gate it.
    let reset_controls = match &base {
        Some(base) => base.clone(),
        None => allowed(),
    };

    let reset_fill = match channel_base {
        Some(b) => b.clone(),
        None => {
            if inputs.state.fill_open() {
                blocked("Close FILL first")
            } else if inputs.state.ign_pwr() {
                blocked("SAFE the igniter first")
            } else if !(inputs.state.fill_seq_complete()
                || inputs.state.fill_paused()
                || inputs.state.fill_seq_aborted()
                || inputs.state.inhibited()
                || inputs.state.estop())
            {
                blocked("Nothing to reset")
            } else {
                allowed()
            }
        }
    };

    let clear_faults = match channel_base {
        Some(b) => b.clone(),
        None => {
            if inputs.state.faults == 0 && !inputs.state.inhibited() && !inputs.state.estop() {
                blocked("No latched faults")
            } else {
                allowed()
            }
        }
    };

    // The emergency stop is never blocked by a pending command, countdown, or
    // output state; it is only useless when the vehicle is unreachable.
    let estop = if !inputs.vehicle_online {
        blocked(offline_reason)
    } else {
        allowed()
    };

    let pad = pad_actions(inputs, base.as_ref(), channel_base);

    Actions {
        fill_on,
        fill_off,
        srv_on,
        srv_off,
        ign_pwr,
        ign_safe,
        srv_ign_on,
        srv_ign_off,
        abort_countdown,
        reset_controls,
        reset_fill,
        clear_faults,
        estop,
        arm: pad.0,
        disarm: pad.1,
        purge_open: pad.2,
        purge_close: pad.3,
        purge_enable: pad.4,
        purge_disable: pad.5,
    }
}

/// An otherwise-allowed ignition action still needs the operator's ARM.
fn require_arm(gate: ActionState, what: &str) -> ActionState {
    if gate.enabled {
        blocked(format!("{what} requires ARM - arm the igniter first"))
    } else {
        gate
    }
}

/// `(arm, disarm, purge_open, purge_close, purge_enable, purge_disable)`.
fn pad_actions(
    inputs: &Inputs,
    base: Option<&ActionState>,
    channel_base: Option<&ActionState>,
) -> (
    ActionState,
    ActionState,
    ActionState,
    ActionState,
    ActionState,
    ActionState,
) {
    let Some(pad) = inputs.pad else {
        return (
            blocked(PAD_ONLY),
            blocked(PAD_ONLY),
            blocked(PAD_ONLY),
            blocked(PAD_ONLY),
            blocked(PAD_ONLY),
            blocked(PAD_ONLY),
        );
    };
    let latched_text = "Safe state latched - press RESET CONTROLS first";
    let arm = if let Some(b) = base {
        b.clone()
    } else if pad.armed {
        blocked("Already armed")
    } else if pad.latched || inputs.state.estop() || inputs.state.inhibited() {
        blocked(latched_text)
    } else {
        allowed()
    };
    let disarm = if pad.armed {
        allowed()
    } else {
        blocked("Not armed")
    };
    let purge_open = if let Some(b) = channel_base {
        b.clone()
    } else if !pad.purge_enabled {
        blocked("Purge valve not enabled for this session")
    } else if pad.purge_open {
        blocked("PURGE is already OPEN")
    } else if pad.latched || inputs.state.estop() {
        blocked(latched_text)
    } else {
        allowed()
    };
    // Closing purge is always a move toward safe.
    let purge_close = if pad.purge_open {
        allowed()
    } else {
        blocked("PURGE is already CLOSED")
    };
    let purge_enable = if pad.purge_enabled {
        blocked("Purge valve already enabled")
    } else {
        allowed()
    };
    let purge_disable = if pad.purge_enabled {
        allowed()
    } else {
        blocked("Purge valve already disabled")
    };
    (
        arm,
        disarm,
        purge_open,
        purge_close,
        purge_enable,
        purge_disable,
    )
}

#[cfg(test)]
mod tests {
    use super::*;
    use gse_proto::flags;

    fn state(flags: u16) -> StateSnapshot {
        StateSnapshot {
            flags,
            ..StateSnapshot::default()
        }
    }

    fn online(state: StateSnapshot) -> Inputs {
        Inputs {
            mode: OperationMode::Test,
            countdown_seconds: gse_proto::DEFAULT_COUNTDOWN_SECONDS,
            ground_online: true,
            vehicle_online: true,
            pending: false,
            countdown_target: None,
            countdown_holding: false,
            controls_ready: true,
            state,
            pad: None,
        }
    }

    fn online_live(state: StateSnapshot) -> Inputs {
        Inputs {
            mode: OperationMode::Live,
            ..online(state)
        }
    }

    #[test]
    fn everything_blocked_when_offline() {
        let actions = evaluate(&Inputs {
            mode: OperationMode::Test,
            countdown_seconds: gse_proto::DEFAULT_COUNTDOWN_SECONDS,
            ground_online: false,
            vehicle_online: false,
            pending: false,
            countdown_target: None,
            countdown_holding: false,
            controls_ready: false,
            state: state(flags::FILL_OPEN),
            pad: None,
        });
        assert!(!actions.fill_on.enabled);
        assert!(!actions.fill_off.enabled);
        assert!(!actions.srv_ign_on.enabled);
        assert!(!actions.abort_countdown.enabled);
        assert!(!actions.reset_controls.enabled);
        assert!(!actions.estop.enabled);
        assert!(actions.fill_on.reason.contains("Ground board"));
    }

    #[test]
    fn controls_wait_for_the_startup_safe_handshake() {
        let actions = evaluate(&Inputs {
            mode: OperationMode::Live,
            countdown_seconds: gse_proto::DEFAULT_COUNTDOWN_SECONDS,
            ground_online: true,
            vehicle_online: true,
            pending: false,
            countdown_target: None,
            countdown_holding: false,
            controls_ready: false,
            state: StateSnapshot::default(),
            pad: None,
        });
        assert!(!actions.fill_on.enabled);
        assert!(!actions.srv_ign_on.enabled);
        assert!(actions.fill_on.reason.contains("Initialising"));
        assert!(!actions.reset_controls.enabled);
    }

    #[test]
    fn fill_and_srv_interlock() {
        // SRV is locked before FILL has run.
        let actions = evaluate(&online(StateSnapshot::default()));
        assert!(actions.fill_on.enabled);
        assert!(!actions.srv_on.enabled);
        assert!(actions.srv_on.reason.contains("FILL"));

        // Paused FILL unlocks SRV AND allows RESET FILL SEQ.
        let actions = evaluate(&online(state(flags::FILL_PAUSED)));
        assert!(actions.fill_on.enabled);
        assert!(actions.srv_on.enabled);
        assert!(actions.reset_fill.enabled);

        let actions = evaluate(&online(state(flags::SRV_ON | flags::FILL_PAUSED)));
        assert!(!actions.fill_on.enabled);
        assert!(actions.fill_on.reason.contains("interlocked"));
        assert!(!actions.fill_off.enabled);

        let actions = evaluate(&online(state(flags::FILL_OPEN)));
        assert!(!actions.srv_on.enabled);
        assert!(actions.srv_on.reason.contains("FILL"));
        assert!(actions.fill_off.enabled);
        assert!(!actions.fill_on.enabled);
    }

    #[test]
    fn ignition_accepts_completed_or_paused_fill_and_supports_countdown_abort() {
        // SRV on without a completed/paused fill is blocked.
        let actions = evaluate(&online(state(flags::SRV_ON)));
        assert!(!actions.ign_pwr.enabled);
        assert!(actions.ign_pwr.reason.contains("FILL"));

        // Fill complete or paused with SRV off requires SRV first.
        let actions = evaluate(&online(state(flags::FILL_SEQ_COMPLETE)));
        assert!(!actions.ign_pwr.enabled);
        assert!(actions.ign_pwr.reason.contains("SRV"));

        // Both paused fill + SRV ON and completed fill + SRV ON enable IGN PWR.
        assert!(
            evaluate(&online(state(flags::SRV_ON | flags::FILL_PAUSED)))
                .ign_pwr
                .enabled
        );
        assert!(
            evaluate(&online(state(flags::SRV_ON | flags::FILL_SEQ_COMPLETE)))
                .ign_pwr
                .enabled
        );

        // During TEST IGN countdown, abort_countdown + ign_safe + reset_controls + estop are enabled!
        let mut counting = online(state(flags::SRV_ON | flags::FILL_PAUSED));
        counting.countdown_target = Some(CountdownTarget::Ign);
        let actions = evaluate(&counting);
        assert!(!actions.ign_pwr.enabled);
        assert!(actions.ign_safe.enabled);
        assert!(actions.abort_countdown.enabled);
        assert!(!actions.srv_off.enabled);
        assert!(!actions.fill_on.enabled);
        assert!(actions.reset_controls.enabled);
        assert!(actions.estop.enabled);

        // At T-0 HOLD, the target IGN action alone is enabled for manual release.
        counting.countdown_holding = true;
        let actions = evaluate(&counting);
        assert!(actions.ign_pwr.enabled);
        assert!(actions.ign_safe.enabled);
        assert!(actions.abort_countdown.enabled);
        assert!(!actions.srv_off.enabled);
        assert!(!actions.fill_on.enabled);

        // Aborted fill never enables SRV or IGN.
        let actions = evaluate(&online(state(flags::SRV_ON | flags::FILL_SEQ_ABORTED)));
        assert!(!actions.ign_pwr.enabled);
    }

    #[test]
    fn live_mode_gates_srv_ign_on_completed_or_paused_fill_and_supports_countdown_abort() {
        // At rest before fill: FILL ON enabled, SRV+IGN disabled, individual SRV/IGN disabled.
        let actions = evaluate(&online_live(StateSnapshot::default()));
        assert!(actions.fill_on.enabled);
        assert!(!actions.srv_ign_on.enabled);
        assert!(!actions.srv_ign_off.enabled);
        assert!(!actions.srv_on.enabled);
        assert!(!actions.ign_pwr.enabled);
        assert!(actions.srv_ign_on.reason.contains("FILL"));

        // While filling: FILL OFF enabled, SRV+IGN disabled.
        let actions = evaluate(&online_live(state(flags::FILL_OPEN)));
        assert!(actions.fill_off.enabled);
        assert!(!actions.srv_ign_on.enabled);

        // Aborted fill: SRV+IGN remains disabled.
        let actions = evaluate(&online_live(state(flags::FILL_SEQ_ABORTED)));
        assert!(!actions.srv_ign_on.enabled);

        // Paused fill OR completed fill: SRV+IGN ON enabled!
        assert!(
            evaluate(&online_live(state(flags::FILL_PAUSED)))
                .srv_ign_on
                .enabled
        );
        let actions = evaluate(&online_live(state(flags::FILL_SEQ_COMPLETE)));
        assert!(actions.srv_ign_on.enabled);
        assert!(!actions.srv_ign_off.enabled);

        // During LIVE countdown: abort_countdown + srv_ign_off + RESET CONTROLS + E-STOP stay enabled!
        let mut counting = online_live(state(flags::FILL_SEQ_COMPLETE));
        counting.countdown_target = Some(CountdownTarget::SrvIgn);
        let actions = evaluate(&counting);
        assert!(!actions.srv_ign_on.enabled);
        assert!(actions.srv_ign_off.enabled);
        assert!(actions.abort_countdown.enabled);
        assert!(!actions.fill_on.enabled);
        assert!(actions.reset_controls.enabled);
        assert!(actions.estop.enabled);

        // At T-0 HOLD, the matching LIVE trigger is available, but nothing else.
        counting.countdown_holding = true;
        let actions = evaluate(&counting);
        assert!(actions.srv_ign_on.enabled);
        assert!(actions.srv_ign_off.enabled);
        assert!(actions.abort_countdown.enabled);
        assert!(!actions.fill_on.enabled);

        // Once SRV+IGN is ON/PWR: SRV+IGN OFF enabled, FILL ON blocked.
        let actions = evaluate(&online_live(state(
            flags::FILL_SEQ_COMPLETE | flags::SRV_ON | flags::IGN_PWR,
        )));
        assert!(!actions.srv_ign_on.enabled);
        assert!(actions.srv_ign_off.enabled);
        assert!(!actions.fill_on.enabled);
    }

    #[test]
    fn ignition_blocked_when_inhibited_or_estopped() {
        let actions = evaluate(&online(state(
            flags::SRV_ON | flags::FILL_SEQ_COMPLETE | flags::INHIBITED,
        )));
        assert!(!actions.ign_pwr.enabled);
        assert!(actions.ign_pwr.reason.contains("inhibited"));
        assert!(actions.reset_controls.enabled);
        assert!(actions.reset_fill.enabled);

        let actions = evaluate(&online(state(
            flags::SRV_ON | flags::FILL_SEQ_COMPLETE | flags::ESTOP,
        )));
        assert!(!actions.ign_pwr.enabled);
        assert!(actions.ign_pwr.reason.contains("E-STOP"));
        assert!(actions.reset_controls.enabled);
    }

    #[test]
    fn srv_cannot_be_removed_while_ignited() {
        let actions = evaluate(&online(state(flags::SRV_ON | flags::IGN_PWR)));
        assert!(!actions.srv_off.enabled);
        assert!(actions.ign_safe.enabled);
        assert!(actions.estop.enabled);
        assert!(!actions.ign_pwr.enabled);
    }

    #[test]
    fn reset_controls_is_always_available_online() {
        // Even when everything is already safe, the operator can re-assert it.
        let actions = evaluate(&online(StateSnapshot::default()));
        assert!(actions.reset_controls.enabled);
        assert!(actions.reset_controls.reason.is_empty());

        // And while an output is active.
        let actions = evaluate(&online(state(flags::FILL_OPEN)));
        assert!(actions.reset_controls.enabled);
    }

    #[test]
    fn reset_controls_blocked_during_pending_and_offline() {
        let mut inputs = online(StateSnapshot::default());
        inputs.pending = true;
        assert!(!evaluate(&inputs).reset_controls.enabled);

        inputs.pending = false;
        inputs.vehicle_online = false;
        assert!(!evaluate(&inputs).reset_controls.enabled);
    }

    #[test]
    fn estop_available_during_pending() {
        let actions = evaluate(&Inputs {
            mode: OperationMode::Live,
            countdown_seconds: gse_proto::DEFAULT_COUNTDOWN_SECONDS,
            ground_online: true,
            vehicle_online: true,
            pending: true,
            countdown_target: None,
            countdown_holding: false,
            controls_ready: true,
            state: StateSnapshot::default(),
            pad: None,
        });
        assert!(actions.estop.enabled);
        assert!(!actions.fill_on.enabled);
        assert!(!actions.reset_controls.enabled);
    }

    #[test]
    fn clear_faults_requires_faults() {
        let actions = evaluate(&online(StateSnapshot::default()));
        assert!(!actions.clear_faults.enabled);
        let actions = evaluate(&online(StateSnapshot {
            faults: gse_proto::faults::LINK_TIMEOUT,
            ..StateSnapshot::default()
        }));
        assert!(actions.clear_faults.enabled);
    }

    #[test]
    fn every_action_has_a_reason_when_disabled() {
        // A greyed-out control must always explain itself.
        let actions = evaluate(&Inputs {
            mode: OperationMode::Test,
            countdown_seconds: gse_proto::DEFAULT_COUNTDOWN_SECONDS,
            ground_online: true,
            vehicle_online: true,
            pending: false,
            countdown_target: None,
            countdown_holding: false,
            controls_ready: true,
            state: state(flags::FILL_OPEN | flags::IGN_PWR),
            pad: None,
        });
        for (name, action) in [
            ("fill_on", &actions.fill_on),
            ("fill_off", &actions.fill_off),
            ("srv_on", &actions.srv_on),
            ("srv_off", &actions.srv_off),
            ("ign_pwr", &actions.ign_pwr),
            ("ign_safe", &actions.ign_safe),
            ("srv_ign_on", &actions.srv_ign_on),
            ("srv_ign_off", &actions.srv_ign_off),
            ("abort_countdown", &actions.abort_countdown),
            ("reset_fill", &actions.reset_fill),
        ] {
            if !action.enabled {
                assert!(
                    !action.reason.is_empty(),
                    "{name} disabled without a reason"
                );
            }
        }
    }
}

#[cfg(test)]
mod pad_tests {
    use super::*;
    use gse_proto::flags;

    fn pad_inputs(state: StateSnapshot, pad: PadGate) -> Inputs {
        Inputs {
            mode: OperationMode::Test,
            countdown_seconds: gse_proto::DEFAULT_COUNTDOWN_SECONDS,
            ground_online: true,
            vehicle_online: true,
            pending: false,
            countdown_target: None,
            countdown_holding: false,
            controls_ready: true,
            state,
            pad: Some(pad),
        }
    }

    fn ready_for_ign() -> StateSnapshot {
        StateSnapshot {
            flags: flags::SRV_ON | flags::FILL_SEQ_COMPLETE,
            ..StateSnapshot::default()
        }
    }

    #[test]
    fn ignition_needs_arm_on_the_pad() {
        let a = evaluate(&pad_inputs(ready_for_ign(), PadGate::default()));
        assert!(!a.ign_pwr.enabled);
        assert!(a.ign_pwr.reason.contains("ARM"));
        assert!(a.arm.enabled && !a.disarm.enabled);
        let a = evaluate(&pad_inputs(
            ready_for_ign(),
            PadGate {
                armed: true,
                ..PadGate::default()
            },
        ));
        assert!(a.ign_pwr.enabled && a.disarm.enabled && !a.arm.enabled);
    }

    #[test]
    fn arm_refused_while_latched() {
        let a = evaluate(&pad_inputs(
            StateSnapshot {
                flags: flags::ESTOP,
                ..StateSnapshot::default()
            },
            PadGate {
                latched: true,
                ..PadGate::default()
            },
        ));
        assert!(!a.arm.enabled);
        assert!(a.arm.reason.contains("RESET"));
    }

    #[test]
    fn purge_needs_session_enable() {
        let a = evaluate(&pad_inputs(StateSnapshot::default(), PadGate::default()));
        assert!(!a.purge_open.enabled && a.purge_enable.enabled && !a.purge_disable.enabled);
        let a = evaluate(&pad_inputs(
            StateSnapshot::default(),
            PadGate {
                purge_enabled: true,
                ..PadGate::default()
            },
        ));
        assert!(a.purge_open.enabled && !a.purge_close.enabled && a.purge_disable.enabled);
        let a = evaluate(&pad_inputs(
            StateSnapshot {
                flags: flags::FILL_OPEN | flags::FILL_TIMER_ACTIVE,
                ..StateSnapshot::default()
            },
            PadGate {
                purge_enabled: true,
                purge_open: true,
                ..PadGate::default()
            },
        ));
        assert!(a.purge_close.enabled, "closing purge is always allowed");
    }

    #[test]
    fn lora_boards_have_no_pad_actions() {
        let mut inputs = pad_inputs(StateSnapshot::default(), PadGate::default());
        inputs.pad = None;
        let a = evaluate(&inputs);
        for gate in [&a.arm, &a.disarm, &a.purge_open, &a.purge_enable] {
            assert!(!gate.enabled && gate.reason == PAD_ONLY);
        }
    }
}
