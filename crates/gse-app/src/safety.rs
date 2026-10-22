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
//!   `SRV`, and `IGN` channels (`FILL` <-> `SRV` exclusive, `IGN` gated on a
//!   completed fill + `SRV ON`).
//! * [`OperationMode::Live`] — live launch sequence: `FILL` unchanged, plus a
//!   combined `SRV+IGN` channel (`OFF/SAFE` -> 5 s countdown -> `ON/PWR` ->
//!   `OFF/SAFE`), greyed out until the fill sequence has completed.
//!
//! Additional app-only gates:
//!
//! * nothing is enabled before the ground and vehicle links are up,
//! * nothing except E-STOP is enabled while a command is awaiting its ACK,
//! * while the 5 s `SRV+IGN` launch countdown is running, only E-STOP and
//!   RESET CONTROLS remain active (both immediately cancel the countdown),
//! * control actions stay locked until the startup handshake has confirmed
//!   every output is OFF/SAFE ([`Inputs::controls_ready`]).

use gse_proto::fsm::{ControlEvent, ControlState, Interlocks, Transition};
use gse_proto::{Reason, StateSnapshot};

use crate::config::OperationMode;
use crate::state::{ActionState, Actions};

pub struct Inputs {
    pub mode: OperationMode,
    pub ground_online: bool,
    pub vehicle_online: bool,
    pub pending: bool,
    /// True during the 5-second LIVE launch countdown before `SRV+IGN ON`.
    pub countdown_active: bool,
    /// The startup/reset handshake confirmed FILL OFF, SRV OFF, IGN SAFE.
    pub controls_ready: bool,
    pub state: StateSnapshot,
}

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
        (ControlEvent::IgnPwr | ControlEvent::SrvIgnOn, Reason::NotPermitted)
            if interlocks.inhibited =>
        {
            "Vehicle inhibited - press RESET CONTROLS and run a new fill".to_string()
        }
        (ControlEvent::IgnPwr | ControlEvent::SrvIgnOn, Reason::NotPermitted)
            if interlocks.estop =>
        {
            "E-STOP is latched - press RESET CONTROLS first".to_string()
        }
        (ControlEvent::SrvIgnOn, Reason::FillSeqIncomplete) => {
            "SRV+IGN requires a completed FILL sequence".to_string()
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

    let countdown_gate = if inputs.countdown_active {
        Some(blocked("SRV+IGN 5 s countdown in progress"))
    } else {
        None
    };
    let channel_base = base.as_ref().or(countdown_gate.as_ref());

    let fill_on = gate(channel_base, state, interlocks, ControlEvent::FillOn);
    let fill_off = gate(channel_base, state, interlocks, ControlEvent::FillOff);

    let (srv_on, srv_off, ign_pwr, ign_safe, srv_ign_on, srv_ign_off) = match inputs.mode {
        OperationMode::Test => (
            gate(channel_base, state, interlocks, ControlEvent::SrvOn),
            gate(channel_base, state, interlocks, ControlEvent::SrvOff),
            gate(channel_base, state, interlocks, ControlEvent::IgnPwr),
            gate(channel_base, state, interlocks, ControlEvent::IgnSafe),
            blocked("Only available in LIVE mode"),
            blocked("Only available in LIVE mode"),
        ),
        OperationMode::Live => (
            blocked("Disabled in LIVE mode (use SRV+IGN)"),
            blocked("Disabled in LIVE mode (use SRV+IGN)"),
            blocked("Disabled in LIVE mode (use SRV+IGN)"),
            blocked("Disabled in LIVE mode (use SRV+IGN)"),
            gate(channel_base, state, interlocks, ControlEvent::SrvIgnOn),
            gate(channel_base, state, interlocks, ControlEvent::SrvIgnOff),
        ),
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

    Actions {
        fill_on,
        fill_off,
        srv_on,
        srv_off,
        ign_pwr,
        ign_safe,
        srv_ign_on,
        srv_ign_off,
        reset_controls,
        reset_fill,
        clear_faults,
        estop,
    }
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
            ground_online: true,
            vehicle_online: true,
            pending: false,
            countdown_active: false,
            controls_ready: true,
            state,
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
            ground_online: false,
            vehicle_online: false,
            pending: false,
            countdown_active: false,
            controls_ready: false,
            state: state(flags::FILL_OPEN),
        });
        assert!(!actions.fill_on.enabled);
        assert!(!actions.fill_off.enabled);
        assert!(!actions.srv_ign_on.enabled);
        assert!(!actions.reset_controls.enabled);
        assert!(!actions.estop.enabled);
        assert!(actions.fill_on.reason.contains("Ground board"));
    }

    #[test]
    fn controls_wait_for_the_startup_safe_handshake() {
        let actions = evaluate(&Inputs {
            mode: OperationMode::Live,
            ground_online: true,
            vehicle_online: true,
            pending: false,
            countdown_active: false,
            controls_ready: false,
            state: StateSnapshot::default(),
        });
        assert!(!actions.fill_on.enabled);
        assert!(!actions.srv_ign_on.enabled);
        assert!(actions.fill_on.reason.contains("Initialising"));
        assert!(!actions.reset_controls.enabled);
    }

    #[test]
    fn fill_and_srv_interlock() {
        let actions = evaluate(&online(state(flags::SRV_ON)));
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
    fn ignition_requires_full_sequence() {
        // SRV on but no completed fill.
        let actions = evaluate(&online(state(flags::SRV_ON)));
        assert!(!actions.ign_pwr.enabled);
        assert!(actions.ign_pwr.reason.contains("FILL sequence"));

        // Fill complete but SRV off.
        let actions = evaluate(&online(state(flags::FILL_SEQ_COMPLETE)));
        assert!(!actions.ign_pwr.enabled);
        assert!(actions.ign_pwr.reason.contains("SRV"));

        // Both conditions met.
        let actions = evaluate(&online(state(flags::SRV_ON | flags::FILL_SEQ_COMPLETE)));
        assert!(actions.ign_pwr.enabled);

        // Aborted fill never enables ignition.
        let actions = evaluate(&online(state(flags::SRV_ON | flags::FILL_SEQ_ABORTED)));
        assert!(!actions.ign_pwr.enabled);
    }

    #[test]
    fn live_mode_gates_srv_ign_on_completed_fill_and_blocks_individual_channels() {
        // At rest before fill: FILL ON enabled, SRV+IGN disabled, individual SRV/IGN disabled.
        let actions = evaluate(&online_live(StateSnapshot::default()));
        assert!(actions.fill_on.enabled);
        assert!(!actions.srv_ign_on.enabled);
        assert!(!actions.srv_ign_off.enabled);
        assert!(!actions.srv_on.enabled);
        assert!(!actions.ign_pwr.enabled);
        assert!(actions.srv_ign_on.reason.contains("completed FILL"));

        // While filling: FILL OFF enabled, SRV+IGN disabled.
        let actions = evaluate(&online_live(state(flags::FILL_OPEN)));
        assert!(actions.fill_off.enabled);
        assert!(!actions.srv_ign_on.enabled);

        // Aborted fill: SRV+IGN remains disabled.
        let actions = evaluate(&online_live(state(flags::FILL_SEQ_ABORTED)));
        assert!(!actions.srv_ign_on.enabled);

        // Completed fill: SRV+IGN ON enabled!
        let actions = evaluate(&online_live(state(flags::FILL_SEQ_COMPLETE)));
        assert!(actions.srv_ign_on.enabled);
        assert!(!actions.srv_ign_off.enabled);

        // During 5 s countdown: channel actions locked, RESET CONTROLS + E-STOP stay enabled.
        let mut counting = online_live(state(flags::FILL_SEQ_COMPLETE));
        counting.countdown_active = true;
        let actions = evaluate(&counting);
        assert!(!actions.srv_ign_on.enabled);
        assert!(!actions.fill_on.enabled);
        assert!(actions.reset_controls.enabled);
        assert!(actions.estop.enabled);

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
            ground_online: true,
            vehicle_online: true,
            pending: true,
            countdown_active: false,
            controls_ready: true,
            state: StateSnapshot::default(),
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
            ground_online: true,
            vehicle_online: true,
            pending: false,
            countdown_active: false,
            controls_ready: true,
            state: state(flags::FILL_OPEN | flags::IGN_PWR),
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
