//! Operator-side safety interlocks.
//!
//! These gates are intentionally duplicated in the vehicle firmware: the app
//! greys out impossible actions (and refuses them from the API/keyboard), and
//! the vehicle independently re-checks every command. Neither side trusts the
//! other, and there is no override.

use gse_proto::StateSnapshot;

use crate::state::{ActionState, Actions};

pub struct Inputs {
    pub ground_online: bool,
    pub vehicle_online: bool,
    pub pending: bool,
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

pub fn evaluate(inputs: &Inputs) -> Actions {
    let s = &inputs.state;
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
    } else {
        None
    };

    let fill_on = match &base {
        Some(b) => b.clone(),
        None => {
            if s.fill_open() {
                blocked("FILL is already open")
            } else if s.srv_on() {
                blocked("SRV is ON - FILL and SRV are interlocked")
            } else if s.ign_pwr() {
                blocked("IGN is PWR - SAFE the igniter first")
            } else {
                allowed()
            }
        }
    };

    let fill_off = match &base {
        Some(b) => b.clone(),
        None => {
            if !s.fill_open() && !s.fill_timer_active() {
                blocked("FILL is already closed")
            } else {
                allowed()
            }
        }
    };

    let srv_on = match &base {
        Some(b) => b.clone(),
        None => {
            if s.srv_on() {
                blocked("SRV is already ON")
            } else if s.fill_open() {
                blocked("FILL is open - close FILL before enabling SRV")
            } else if s.ign_pwr() {
                blocked("IGN is PWR")
            } else {
                allowed()
            }
        }
    };

    let srv_off = match &base {
        Some(b) => b.clone(),
        None => {
            if !s.srv_on() {
                blocked("SRV is already OFF")
            } else if s.ign_pwr() {
                blocked("IGN must be SAFE before SRV is removed")
            } else {
                allowed()
            }
        }
    };

    let ign_pwr = match &base {
        Some(b) => b.clone(),
        None => {
            if s.ign_pwr() {
                blocked("IGN is already PWR")
            } else if s.fill_open() {
                blocked("FILL is open - close FILL first")
            } else if !s.srv_on() {
                blocked("SRV must be ON")
            } else if !s.fill_seq_complete() {
                blocked("FILL sequence incomplete - run a full fill first")
            } else if s.inhibited() {
                blocked("Vehicle inhibited - reset the fill sequence")
            } else {
                allowed()
            }
        }
    };

    let ign_safe = match &base {
        Some(b) => b.clone(),
        None => {
            if !s.ign_pwr() {
                blocked("IGN is already SAFE")
            } else {
                allowed()
            }
        }
    };

    let reset_fill = match &base {
        Some(b) => b.clone(),
        None => {
            if s.fill_open() {
                blocked("Close FILL first")
            } else if s.ign_pwr() {
                blocked("SAFE the igniter first")
            } else if !(s.fill_seq_complete()
                || s.fill_seq_aborted()
                || s.inhibited()
                || s.estop())
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
            if s.faults == 0 && !s.inhibited() && !s.estop() {
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
            state,
        }
    }

    #[test]
    fn everything_blocked_when_offline() {
        let actions = evaluate(&Inputs {
            ground_online: false,
            vehicle_online: false,
            pending: false,
            state: state(flags::FILL_OPEN),
        });
        assert!(!actions.fill_on.enabled);
        assert!(!actions.fill_off.enabled);
        assert!(!actions.estop.enabled);
        assert!(actions.fill_on.reason.contains("Ground board"));
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
        let actions = evaluate(&online(state(
            flags::SRV_ON | flags::FILL_SEQ_COMPLETE,
        )));
        assert!(actions.ign_pwr.enabled);

        // Aborted fill never enables ignition.
        let actions = evaluate(&online(state(
            flags::SRV_ON | flags::FILL_SEQ_ABORTED,
        )));
        assert!(!actions.ign_pwr.enabled);
    }

    #[test]
    fn ignition_blocked_when_inhibited() {
        let actions = evaluate(&online(state(
            flags::SRV_ON | flags::FILL_SEQ_COMPLETE | flags::INHIBITED,
        )));
        assert!(!actions.ign_pwr.enabled);
        assert!(actions.ign_pwr.reason.contains("inhibited"));
        assert!(actions.reset_fill.enabled);
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
    fn estop_available_during_pending() {
        let actions = evaluate(&Inputs {
            ground_online: true,
            vehicle_online: true,
            pending: true,
            state: StateSnapshot::default(),
        });
        assert!(actions.estop.enabled);
        assert!(!actions.fill_on.enabled);
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
}
