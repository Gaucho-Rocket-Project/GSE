//! Deterministic control-state machine shared by the ground app and the
//! firmware.
//!
//! Every operator action is expressed as a [`ControlEvent`]. The machine maps
//! the last *confirmed* [`ControlState`] plus the event to exactly one of three
//! outcomes:
//!
//! * [`Transition::Allowed`] — the command is permitted and produces the
//!   returned state,
//! * [`Transition::NoOp`] — the control is already in the requested state,
//! * [`Transition::Blocked`] — an interlock refuses the transition; the state
//!   does not change.
//!
//! The same table is used in three places, so a stale UI, a rebound shortcut,
//! a hand-written HTTP request and a corrupted-but-valid radio frame can never
//! disagree about what is safe:
//!
//! * `gse-app` greys out every action whose transition is not allowed,
//! * `gse-fw-core` re-checks each command before touching an output,
//! * the simulator/board firmware runs that same vehicle code.
//!
//! Invariants (checked exhaustively by the tests):
//!
//! 1. `FILL ON` and `SRV ON` are mutually exclusive.
//! 2. `SRV ON`, `IGN PWR`, and `SRV+IGN ON` all require `FILL` to have run and
//!    been turned `OFF` (`FILL_SEQ_COMPLETE` or operator-paused `FILL_PAUSED`,
//!    with no uncommanded `FILL_SEQ_ABORTED`), plus no inhibit latch and no
//!    `ESTOP`. `IGN PWR` additionally requires `SRV ON`.
//! 3. Starting from `OFF`/`SAFE`, no legal sequence can produce an unsafe
//!    combination; the UI keeps all controls at that state until the board
//!    confirms it.
//! 4. `SAFE ALL`/`RESET` are always legal and land in [`ControlState::SAFE`].

use crate::control::{Control, Reason, ACTION_OFF, ACTION_ON};
use crate::state::{flags, StateSnapshot};

/// Fill-valve state. `Off` is the de-energised (closed) position.
#[repr(u8)]
#[derive(Clone, Copy, Debug, Default, PartialEq, Eq, Hash)]
pub enum FillState {
    #[default]
    Off = 0,
    On = 1,
}

impl FillState {
    pub const fn label(self) -> &'static str {
        match self {
            FillState::Off => "OFF",
            FillState::On => "ON",
        }
    }

    pub const fn is_on(self) -> bool {
        matches!(self, FillState::On)
    }
}

/// Vehicle-servo power state.
#[repr(u8)]
#[derive(Clone, Copy, Debug, Default, PartialEq, Eq, Hash)]
pub enum SrvState {
    #[default]
    Off = 0,
    On = 1,
}

impl SrvState {
    pub const fn label(self) -> &'static str {
        match self {
            SrvState::Off => "OFF",
            SrvState::On => "ON",
        }
    }

    pub const fn is_on(self) -> bool {
        matches!(self, SrvState::On)
    }
}

/// Igniter output state.
#[repr(u8)]
#[derive(Clone, Copy, Debug, Default, PartialEq, Eq, Hash)]
pub enum IgnState {
    #[default]
    Safe = 0,
    Pwr = 1,
}

impl IgnState {
    pub const fn label(self) -> &'static str {
        match self {
            IgnState::Safe => "SAFE",
            IgnState::Pwr => "PWR",
        }
    }

    pub const fn is_pwr(self) -> bool {
        matches!(self, IgnState::Pwr)
    }
}

/// The three control axes, exactly as the vehicle reports them.
#[derive(Clone, Copy, Debug, Default, PartialEq, Eq, Hash)]
pub struct ControlState {
    pub fill: FillState,
    pub srv: SrvState,
    pub ign: IgnState,
}

impl ControlState {
    /// Initialisation / shutdown / reset state: every control OFF or SAFE.
    pub const SAFE: ControlState = ControlState {
        fill: FillState::Off,
        srv: SrvState::Off,
        ign: IgnState::Safe,
    };

    pub const fn is_safe(self) -> bool {
        matches!(self.fill, FillState::Off)
            && matches!(self.srv, SrvState::Off)
            && matches!(self.ign, IgnState::Safe)
    }

    pub const fn fill_on(self) -> bool {
        self.fill.is_on()
    }

    pub const fn srv_on(self) -> bool {
        self.srv.is_on()
    }

    pub const fn ign_pwr(self) -> bool {
        self.ign.is_pwr()
    }

    /// Decode the three output bits of a `STATE`/`ACK` snapshot.
    pub const fn from_flags(bits: u16) -> Self {
        ControlState {
            fill: if bits & flags::FILL_OPEN != 0 {
                FillState::On
            } else {
                FillState::Off
            },
            srv: if bits & flags::SRV_ON != 0 {
                SrvState::On
            } else {
                SrvState::Off
            },
            ign: if bits & flags::IGN_PWR != 0 {
                IgnState::Pwr
            } else {
                IgnState::Safe
            },
        }
    }

    pub const fn from_snapshot(snapshot: &StateSnapshot) -> Self {
        Self::from_flags(snapshot.flags)
    }

    /// The single transition table. See the module docs for the rules.
    pub const fn transition(self, event: ControlEvent, il: Interlocks) -> Transition {
        use ControlEvent as E;
        match event {
            E::FillOn => {
                if self.fill.is_on() {
                    Transition::NoOp(Reason::AlreadyInState)
                } else if self.ign.is_pwr() {
                    // An armed igniter must be made SAFE before anything else.
                    Transition::Blocked(Reason::IgnPowered)
                } else if self.srv.is_on() {
                    // Safety rule 1: FILL and SRV can never both be on.
                    Transition::Blocked(Reason::FillSrvExclusive)
                } else {
                    Transition::Allowed(ControlState {
                        fill: FillState::On,
                        ..self
                    })
                }
            }
            E::FillOff => {
                if self.fill.is_on() {
                    Transition::Allowed(ControlState {
                        fill: FillState::Off,
                        ..self
                    })
                } else {
                    Transition::NoOp(Reason::NotActive)
                }
            }
            E::SrvOn => {
                if self.srv.is_on() {
                    Transition::NoOp(Reason::AlreadyInState)
                } else if self.fill.is_on() {
                    Transition::Blocked(Reason::FillActive)
                } else if self.ign.is_pwr() {
                    Transition::Blocked(Reason::IgnPowered)
                } else if !il.fill_ready() {
                    // SRV remains locked until FILL has run and turned OFF
                    // (either automatically at timer expiry or manually paused).
                    Transition::Blocked(Reason::FillSeqIncomplete)
                } else if il.inhibited || il.estop {
                    Transition::Blocked(Reason::NotPermitted)
                } else {
                    Transition::Allowed(ControlState {
                        srv: SrvState::On,
                        ..self
                    })
                }
            }
            E::SrvOff => {
                if !self.srv.is_on() {
                    Transition::NoOp(Reason::NotActive)
                } else if self.ign.is_pwr() {
                    Transition::Blocked(Reason::IgnPowered)
                } else {
                    Transition::Allowed(ControlState {
                        srv: SrvState::Off,
                        ..self
                    })
                }
            }
            E::IgnPwr => {
                if self.ign.is_pwr() {
                    Transition::NoOp(Reason::AlreadyInState)
                } else if self.fill.is_on() {
                    Transition::Blocked(Reason::FillActive)
                } else if !self.srv.is_on() {
                    // Safety rule: SRV must be ON before IGN PWR.
                    Transition::Blocked(Reason::SrvRequired)
                } else if !il.fill_ready() {
                    // Safety rule: FILL must have completed or been paused OFF.
                    Transition::Blocked(Reason::FillSeqIncomplete)
                } else if il.inhibited || il.estop {
                    Transition::Blocked(Reason::NotPermitted)
                } else {
                    Transition::Allowed(ControlState {
                        ign: IgnState::Pwr,
                        ..self
                    })
                }
            }
            E::IgnSafe => {
                if self.ign.is_pwr() {
                    Transition::Allowed(ControlState {
                        ign: IgnState::Safe,
                        ..self
                    })
                } else {
                    Transition::NoOp(Reason::NotActive)
                }
            }
            E::SrvIgnOn => {
                if self.srv.is_on() && self.ign.is_pwr() {
                    Transition::NoOp(Reason::AlreadyInState)
                } else if self.fill.is_on() {
                    // Safety rule 1: FILL and SRV can never both be on.
                    Transition::Blocked(Reason::FillActive)
                } else if !il.fill_ready() {
                    // Live launch rule: SRV+IGN requires FILL complete or paused OFF.
                    Transition::Blocked(Reason::FillSeqIncomplete)
                } else if il.inhibited || il.estop {
                    Transition::Blocked(Reason::NotPermitted)
                } else {
                    Transition::Allowed(ControlState {
                        srv: SrvState::On,
                        ign: IgnState::Pwr,
                        ..self
                    })
                }
            }
            E::SrvIgnOff => {
                if !self.srv.is_on() && !self.ign.is_pwr() {
                    Transition::NoOp(Reason::NotActive)
                } else {
                    Transition::Allowed(ControlState {
                        srv: SrvState::Off,
                        ign: IgnState::Safe,
                        ..self
                    })
                }
            }
            // Both of these force the machine to its initial state and are
            // always safe, from any state, including during a fault.
            E::SafeAll | E::Reset => Transition::Allowed(ControlState::SAFE),
        }
    }
}

/// Non-output latches that participate in the transition table.
#[derive(Clone, Copy, Debug, Default, PartialEq, Eq)]
pub struct Interlocks {
    pub fill_seq_complete: bool,
    pub fill_paused: bool,
    pub fill_seq_aborted: bool,
    pub inhibited: bool,
    pub estop: bool,
}

impl Interlocks {
    pub const fn fill_ready(self) -> bool {
        (self.fill_seq_complete || self.fill_paused) && !self.fill_seq_aborted
    }

    pub const fn from_snapshot(snapshot: &StateSnapshot) -> Self {
        Interlocks {
            fill_seq_complete: snapshot.fill_seq_complete(),
            fill_paused: snapshot.fill_paused(),
            fill_seq_aborted: snapshot.fill_seq_aborted(),
            inhibited: snapshot.has(flags::INHIBITED),
            estop: snapshot.has(flags::ESTOP),
        }
    }
}

/// A request to move the machine.
#[repr(u8)]
#[derive(Clone, Copy, Debug, PartialEq, Eq, Hash)]
pub enum ControlEvent {
    FillOn = 0,
    FillOff = 1,
    SrvOn = 2,
    SrvOff = 3,
    IgnPwr = 4,
    IgnSafe = 5,
    /// Combined live-launch transition: `OFF/SAFE -> ON/PWR` (`SRV ON + IGN PWR`).
    SrvIgnOn = 6,
    /// Combined live-launch safe transition: `ON/PWR -> OFF/SAFE` (`IGN SAFE + SRV OFF`).
    SrvIgnOff = 7,
    /// All outputs to OFF/SAFE (`SYSTEM/SAFE_ALL`).
    SafeAll = 8,
    /// Operator reset: all outputs OFF/SAFE and latches cleared.
    Reset = 9,
}

impl ControlEvent {
    pub const ALL: [ControlEvent; 10] = [
        ControlEvent::FillOn,
        ControlEvent::FillOff,
        ControlEvent::SrvOn,
        ControlEvent::SrvOff,
        ControlEvent::IgnPwr,
        ControlEvent::IgnSafe,
        ControlEvent::SrvIgnOn,
        ControlEvent::SrvIgnOff,
        ControlEvent::SafeAll,
        ControlEvent::Reset,
    ];

    pub const fn as_u8(self) -> u8 {
        self as u8
    }

    pub const fn label(self) -> &'static str {
        match self {
            ControlEvent::FillOn => "FILL ON",
            ControlEvent::FillOff => "FILL OFF",
            ControlEvent::SrvOn => "SRV ON",
            ControlEvent::SrvOff => "SRV OFF",
            ControlEvent::IgnPwr => "IGN PWR",
            ControlEvent::IgnSafe => "IGN SAFE",
            ControlEvent::SrvIgnOn => "SRV+IGN ON",
            ControlEvent::SrvIgnOff => "SRV+IGN OFF",
            ControlEvent::SafeAll => "ALL SAFE",
            ControlEvent::Reset => "RESET CONTROLS",
        }
    }

    /// Stable snake_case id used by the HTTP API, shortcuts and the log.
    pub const fn id(self) -> &'static str {
        match self {
            ControlEvent::FillOn => "fill_on",
            ControlEvent::FillOff => "fill_off",
            ControlEvent::SrvOn => "srv_on",
            ControlEvent::SrvOff => "srv_off",
            ControlEvent::IgnPwr => "ign_pwr",
            ControlEvent::IgnSafe => "ign_safe",
            ControlEvent::SrvIgnOn => "srv_ign_on",
            ControlEvent::SrvIgnOff => "srv_ign_off",
            ControlEvent::SafeAll => "safe_all",
            ControlEvent::Reset => "reset_controls",
        }
    }

    /// Map a wire command to its event. System actions return `None` except
    /// `SAFE ALL`, which is the `SafeAll` event.
    pub const fn from_command(control: Control, action: u8) -> Option<Self> {
        match control {
            Control::Fill => match action {
                ACTION_ON => Some(ControlEvent::FillOn),
                ACTION_OFF => Some(ControlEvent::FillOff),
                _ => None,
            },
            Control::Srv => match action {
                ACTION_ON => Some(ControlEvent::SrvOn),
                ACTION_OFF => Some(ControlEvent::SrvOff),
                _ => None,
            },
            Control::Ign => match action {
                ACTION_ON => Some(ControlEvent::IgnPwr),
                ACTION_OFF => Some(ControlEvent::IgnSafe),
                _ => None,
            },
            Control::SrvIgn => match action {
                ACTION_ON => Some(ControlEvent::SrvIgnOn),
                ACTION_OFF => Some(ControlEvent::SrvIgnOff),
                _ => None,
            },
            Control::System => None,
        }
    }
}

/// Outcome of [`ControlState::transition`].
#[derive(Clone, Copy, Debug, PartialEq, Eq)]
pub enum Transition {
    /// Legal; the machine moves to the contained state.
    Allowed(ControlState),
    /// Legal but redundant; the state is unchanged and the vehicle acks OK.
    NoOp(Reason),
    /// Refused by an interlock; the state is unchanged and no output moves.
    Blocked(Reason),
}

impl Transition {
    /// State after applying this transition to `current`.
    pub const fn next(self, current: ControlState) -> ControlState {
        match self {
            Transition::Allowed(next) => next,
            _ => current,
        }
    }

    pub const fn is_allowed(self) -> bool {
        matches!(self, Transition::Allowed(_))
    }

    pub const fn is_noop(self) -> bool {
        matches!(self, Transition::NoOp(_))
    }

    pub const fn is_blocked(self) -> bool {
        matches!(self, Transition::Blocked(_))
    }

    /// True when the command is accepted (moves the state or is a no-op).
    pub const fn is_accepted(self) -> bool {
        !self.is_blocked()
    }

    pub const fn reason(self) -> Reason {
        match self {
            Transition::Allowed(_) => Reason::None,
            Transition::NoOp(reason) | Transition::Blocked(reason) => reason,
        }
    }

    pub const fn label(self) -> &'static str {
        match self {
            Transition::Allowed(_) => "allowed",
            Transition::NoOp(_) => "no-op",
            Transition::Blocked(_) => "blocked",
        }
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    fn il(fill_seq_complete: bool) -> Interlocks {
        Interlocks {
            fill_seq_complete,
            ..Interlocks::default()
        }
    }

    fn allowed(current: ControlState, event: ControlEvent, interlocks: Interlocks) -> ControlState {
        match current.transition(event, interlocks) {
            Transition::Allowed(next) => next,
            other => panic!("{event:?} should be allowed from {current:?}: {other:?}"),
        }
    }

    #[test]
    fn initial_state_is_off_and_safe() {
        let init = ControlState::default();
        assert_eq!(init, ControlState::SAFE);
        assert!(init.is_safe());
        assert!(!init.fill_on() && !init.srv_on() && !init.ign_pwr());
        assert_eq!(init.fill.label(), "OFF");
        assert_eq!(init.srv.label(), "OFF");
        assert_eq!(init.ign.label(), "SAFE");
        assert_eq!(ControlState::from_flags(0), ControlState::SAFE);
        assert_eq!(
            ControlState::from_snapshot(&StateSnapshot::default()),
            ControlState::SAFE
        );
        assert!(ControlState::from_snapshot(&StateSnapshot::default()).is_safe());
        assert!(StateSnapshot::default().outputs_safe());
    }

    #[test]
    fn fill_and_srv_never_coexist() {
        let paused = Interlocks {
            fill_paused: true,
            ..Interlocks::default()
        };

        // SRV is locked before FILL has run.
        assert_eq!(
            ControlState::SAFE.transition(ControlEvent::SrvOn, il(false)),
            Transition::Blocked(Reason::FillSeqIncomplete)
        );

        // Once FILL has completed or been paused OFF, SRV unlocks; SRV ON blocks FILL ON.
        let srv = allowed(ControlState::SAFE, ControlEvent::SrvOn, paused);
        assert_eq!(srv.srv, SrvState::On);
        assert_eq!(
            srv.transition(ControlEvent::FillOn, il(true)),
            Transition::Blocked(Reason::FillSrvExclusive)
        );

        // FILL ON blocks SRV ON even if fill flags were set.
        let fill = allowed(ControlState::SAFE, ControlEvent::FillOn, il(false));
        assert_eq!(fill.fill, FillState::On);
        assert_eq!(
            fill.transition(ControlEvent::SrvOn, il(true)),
            Transition::Blocked(Reason::FillActive)
        );

        // Turning FILL OFF into paused state returns outputs SAFE and unlocks SRV.
        let back = allowed(fill, ControlEvent::FillOff, il(false));
        assert_eq!(back, ControlState::SAFE);
        assert_eq!(
            back.transition(ControlEvent::SrvOn, paused),
            Transition::Allowed(ControlState {
                srv: SrvState::On,
                ..ControlState::SAFE
            })
        );
    }

    #[test]
    fn ignition_requires_srv_and_fill_complete_or_paused() {
        let paused = Interlocks {
            fill_paused: true,
            ..Interlocks::default()
        };
        let srv_only = ControlState {
            srv: SrvState::On,
            ..ControlState::SAFE
        };
        assert_eq!(
            srv_only.transition(ControlEvent::IgnPwr, il(false)),
            Transition::Blocked(Reason::FillSeqIncomplete)
        );

        let complete = ControlState::SAFE.transition(ControlEvent::IgnPwr, il(true));
        assert_eq!(complete, Transition::Blocked(Reason::SrvRequired));

        // Allowed when FILL is paused OFF just as when FILL completed.
        let ready_paused = allowed(srv_only, ControlEvent::IgnPwr, paused);
        assert_eq!(ready_paused.ign, IgnState::Pwr);

        let ready = allowed(srv_only, ControlEvent::IgnPwr, il(true));
        assert_eq!(ready.ign, IgnState::Pwr);
        assert_eq!(
            ready.transition(ControlEvent::SrvOff, il(true)),
            Transition::Blocked(Reason::IgnPowered)
        );
        assert_eq!(
            ready.transition(ControlEvent::FillOn, il(true)),
            Transition::Blocked(Reason::IgnPowered)
        );

        // Aborted fill and latched faults keep both SRV and IGN safe.
        let aborted = Interlocks {
            fill_seq_aborted: true,
            ..il(true)
        };
        assert_eq!(
            ControlState::SAFE.transition(ControlEvent::SrvOn, aborted),
            Transition::Blocked(Reason::FillSeqIncomplete)
        );
        assert_eq!(
            srv_only.transition(ControlEvent::IgnPwr, aborted),
            Transition::Blocked(Reason::FillSeqIncomplete)
        );
        let inhibited = Interlocks {
            inhibited: true,
            ..il(true)
        };
        assert_eq!(
            ControlState::SAFE.transition(ControlEvent::SrvOn, inhibited),
            Transition::Blocked(Reason::NotPermitted)
        );
        assert_eq!(
            srv_only.transition(ControlEvent::IgnPwr, inhibited),
            Transition::Blocked(Reason::NotPermitted)
        );
        let estop = Interlocks {
            estop: true,
            ..il(true)
        };
        assert_eq!(
            ControlState::SAFE.transition(ControlEvent::SrvOn, estop),
            Transition::Blocked(Reason::NotPermitted)
        );
        assert_eq!(
            srv_only.transition(ControlEvent::IgnPwr, estop),
            Transition::Blocked(Reason::NotPermitted)
        );
    }

    #[test]
    fn safe_all_and_reset_are_always_allowed() {
        for bits in 0..8u16 {
            let state = ControlState::from_flags(bits);
            for event in [ControlEvent::SafeAll, ControlEvent::Reset] {
                assert_eq!(
                    state.transition(event, Interlocks::default()),
                    Transition::Allowed(ControlState::SAFE),
                    "{event:?} must always reach SAFE from {state:?}"
                );
            }
        }
    }

    #[test]
    fn redundant_commands_are_no_ops() {
        let safe = ControlState::SAFE;
        assert_eq!(
            safe.transition(ControlEvent::FillOff, il(false)),
            Transition::NoOp(Reason::NotActive)
        );
        assert_eq!(
            safe.transition(ControlEvent::SrvOff, il(false)),
            Transition::NoOp(Reason::NotActive)
        );
        assert_eq!(
            safe.transition(ControlEvent::IgnSafe, il(false)),
            Transition::NoOp(Reason::NotActive)
        );
        let on = ControlState {
            srv: SrvState::On,
            ..ControlState::SAFE
        };
        assert_eq!(
            on.transition(ControlEvent::SrvOn, il(false)),
            Transition::NoOp(Reason::AlreadyInState)
        );
    }

    #[test]
    fn every_reachable_state_keeps_the_invariants() {
        let paused = Interlocks {
            fill_paused: true,
            ..Interlocks::default()
        };
        // Exhaustive walk: no sequence of events may produce FILL+SRV, enable
        // SRV before FILL has run, or arm IGN without SRV ON + FILL ready.
        let mut frontier = [ControlState::SAFE; 8];
        let mut len = 1;
        let mut seen = [ControlState::SAFE; 8];
        seen[0] = ControlState::SAFE;
        let mut seen_len = 1;
        while len > 0 {
            let state = frontier[len - 1];
            len -= 1;
            for event in ControlEvent::ALL {
                for interlocks in [il(false), paused, il(true)] {
                    if let Transition::Allowed(next) = state.transition(event, interlocks) {
                        assert!(
                            !(next.fill_on() && next.srv_on()),
                            "FILL+SRV reached via {event:?} from {state:?}"
                        );
                        if !state.srv_on() && next.srv_on() {
                            assert!(interlocks.fill_ready());
                        }
                        if next.ign_pwr() {
                            assert!(next.srv_on() && interlocks.fill_ready());
                        }
                        if !seen[..seen_len].contains(&next) && seen_len < seen.len() {
                            seen[seen_len] = next;
                            seen_len += 1;
                            frontier[len] = next;
                            len += 1;
                        }
                    }
                }
            }
        }
        // FILL+SRV and IGN-without-SRV are unreachable, so only SAFE,
        // FILL_ON, SRV_ON and SRV_ON+IGN_PWR can ever be reached.
        assert_eq!(seen_len, 4);
        for state in &seen[..seen_len] {
            if state.ign_pwr() {
                assert!(state.srv_on(), "IGN PWR without SRV must be impossible");
            }
        }
    }

    #[test]
    fn events_map_to_wire_commands() {
        assert_eq!(
            ControlEvent::from_command(Control::Fill, ACTION_ON),
            Some(ControlEvent::FillOn)
        );
        assert_eq!(
            ControlEvent::from_command(Control::Srv, ACTION_OFF),
            Some(ControlEvent::SrvOff)
        );
        assert_eq!(
            ControlEvent::from_command(Control::Ign, ACTION_ON),
            Some(ControlEvent::IgnPwr)
        );
        assert_eq!(
            ControlEvent::from_command(Control::Ign, ACTION_OFF),
            Some(ControlEvent::IgnSafe)
        );
        assert_eq!(
            ControlEvent::from_command(Control::SrvIgn, ACTION_ON),
            Some(ControlEvent::SrvIgnOn)
        );
        assert_eq!(
            ControlEvent::from_command(Control::SrvIgn, ACTION_OFF),
            Some(ControlEvent::SrvIgnOff)
        );
        assert_eq!(ControlEvent::from_command(Control::System, 0), None);
        assert_eq!(ControlEvent::from_command(Control::Fill, 7), None);
        assert_eq!(ControlEvent::FillOn.id(), "fill_on");
        assert_eq!(ControlEvent::SrvIgnOn.id(), "srv_ign_on");
        assert_eq!(ControlEvent::SrvIgnOff.id(), "srv_ign_off");
        assert_eq!(ControlEvent::Reset.label(), "RESET CONTROLS");
    }

    #[test]
    fn live_srv_ign_requires_completed_fill_and_toggles_cleanly() {
        // Blocked before fill completes.
        assert_eq!(
            ControlState::SAFE.transition(ControlEvent::SrvIgnOn, il(false)),
            Transition::Blocked(Reason::FillSeqIncomplete)
        );
        // Blocked while FILL valve is open even if flag were set.
        let filling = ControlState {
            fill: FillState::On,
            ..ControlState::SAFE
        };
        assert_eq!(
            filling.transition(ControlEvent::SrvIgnOn, il(true)),
            Transition::Blocked(Reason::FillActive)
        );
        // Blocked when aborted, inhibited or estopped.
        assert_eq!(
            ControlState::SAFE.transition(
                ControlEvent::SrvIgnOn,
                Interlocks {
                    fill_seq_aborted: true,
                    ..il(true)
                }
            ),
            Transition::Blocked(Reason::FillSeqIncomplete)
        );
        assert_eq!(
            ControlState::SAFE.transition(
                ControlEvent::SrvIgnOn,
                Interlocks {
                    inhibited: true,
                    ..il(true)
                }
            ),
            Transition::Blocked(Reason::NotPermitted)
        );
        assert_eq!(
            ControlState::SAFE.transition(
                ControlEvent::SrvIgnOn,
                Interlocks {
                    estop: true,
                    ..il(true)
                }
            ),
            Transition::Blocked(Reason::NotPermitted)
        );
        // Allowed after completed fill: OFF/SAFE -> ON/PWR atomically, and FILL blocked.
        let armed = allowed(ControlState::SAFE, ControlEvent::SrvIgnOn, il(true));
        assert_eq!(
            armed,
            ControlState {
                fill: FillState::Off,
                srv: SrvState::On,
                ign: IgnState::Pwr,
            }
        );
        assert_eq!(
            armed.transition(ControlEvent::FillOn, il(true)),
            Transition::Blocked(Reason::IgnPowered)
        );
        // Operator turns SRV+IGN -> OFF/SAFE atomically.
        let safed = allowed(armed, ControlEvent::SrvIgnOff, il(true));
        assert_eq!(safed, ControlState::SAFE);
    }

    #[test]
    fn from_flags_decodes_output_bits_only() {
        let bits = flags::FILL_OPEN | flags::SRV_ON | flags::IGN_PWR | flags::FILL_SEQ_COMPLETE;
        let state = ControlState::from_flags(bits);
        assert!(state.fill_on() && state.srv_on() && state.ign_pwr());
        assert!(!state.is_safe());
    }
}
