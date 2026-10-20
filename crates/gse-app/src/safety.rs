//! Operator-side safety interlocks.
//!
//! Every gate is derived from the shared deterministic control machine in
//! [`gse_proto::fsm`], so the button that greys out and the firmware that
//! rejects a command agree by construction. The app never lets an unsafe
//! transition leave the browser; the vehicle independently re-checks the same
//! table before driving an output.
//!
//! Additional app-only gates:
//!
//! * nothing is enabled before the ground and vehicle links are up,
//! * nothing except E-STOP is enabled while a command is awaiting its ACK,
//! * control actions stay locked until the startup handshake has confirmed
//!   every output is OFF/SAFE ([`Inputs::controls_ready`]).

use gse_proto::fsm::{ControlEvent, ControlState, Interlocks, Transition};
use gse_proto::{Reason, StateSnapshot};

use crate::state::{ActionState, Actions};

pub struct Inputs {
    pub ground_online: bool,
    pub vehicle_online: bool,
    pub pending: bool,
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
        ControlEvent::SafeAll | ControlEvent::Reset => "All controls are already OFF/SAFE",
    }
}

/// Operator text for a refused transition.
fn blocked_text(event: ControlEvent, interlocks: Interlocks, reason: Reason) -> String {
    match (event, reason) {
        (ControlEvent::IgnPwr, Reason::NotPermitted) if interlocks.inhibited => {
            "Vehicle inhibited - press RESET CONTROLS and run a new fill".to_string()
        }
        (ControlEvent::IgnPwr, Reason::NotPermitted) if interlocks.estop => {
            "E-STOP is latched - press RESET CONTROLS first".to_string()
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

    // Every one of these comes straight from the shared transition table.
    let fill_on = gate(base.as_ref(), state, interlocks, ControlEvent::FillOn);
    let fill_off = gate(base.as_ref(), state, interlocks, ControlEvent::FillOff);
    let srv_on = gate(base.as_ref(), state, interlocks, ControlEvent::SrvOn);
    let srv_off = gate(base.as_ref(), state, interlocks, ControlEvent::SrvOff);
    let ign_pwr = gate(base.as_ref(), state, interlocks, ControlEvent::IgnPwr);
    let ign_safe = gate(base.as_ref(), state, interlocks, ControlEvent::IgnSafe);

    // RESET CONTROLS always resolves to OFF/SAFE, so it is never refused by an
    // interlock: only link state, a pending command or the init handshake gate
    // it. This is the operator's way back from any latched condition.
    let reset_controls = match &base {
        Some(base) => base.clone(),
        None => allowed(),
    };

    let reset_fill = match &base {
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

    let clear_faults = match &base {
        Some(b) => b.clone(),
        None => {
            if inputs.state.faults == 0 && !inputs.state.inhibited() && !inputs.state.estop() {
                blocked("No latched faults")
            } else {
                allowed()
            }
        }
    };

    // The emergency stop is never blocked by a pending command or by the
    // current output state; it is only useless when the vehicle is unreachable.
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
            ground_online: true,
            vehicle_online: true,
            pending: false,
            controls_ready: true,
            state,
        }
    }

    #[test]
    fn everything_blocked_when_offline() {
        let actions = evaluate(&Inputs {
            ground_online: false,
            vehicle_online: false,
            pending: false,
            controls_ready: false,
            state: state(flags::FILL_OPEN),
        });
        assert!(!actions.fill_on.enabled);
        assert!(!actions.fill_off.enabled);
        assert!(!actions.reset_controls.enabled);
        assert!(!actions.estop.enabled);
        assert!(actions.fill_on.reason.contains("Ground board"));
    }

    #[test]
    fn controls_wait_for_the_startup_safe_handshake() {
        let actions = evaluate(&Inputs {
            ground_online: true,
            vehicle_online: true,
            pending: false,
            controls_ready: false,
            state: StateSnapshot::default(),
        });
        assert!(!actions.fill_on.enabled);
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
            ground_online: true,
            vehicle_online: true,
            pending: true,
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
            ground_online: true,
            vehicle_online: true,
            pending: false,
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
