"""
supervisor.py  --  main's operator-control layer on top of the GSE-simple pad.

The GSE `main` branch (Rust, crates/gse-app + gse-proto) runs a control state
machine for FILL / SRV / IGN with LIVE and TEST modes, a fill timer that pauses
and resumes, FILL/SRV interlocks, an ignition countdown that holds at T-0 until
the operator fires explicitly, E-STOP and link-loss latches, and an INIT
handshake.  The ESP-NOW port of that app (branch esp32-and-stm32,
crates/gse-app/src/pad_link.rs) runs all of it *app-side*, as a supervisor
whose outputs are streamed to the unchanged GSE-simple pad:

    operator -> Supervisor (this file) -> 20 Hz Cmd stream -> TX node -> pad
             <- gates / logs / timers  <- pad STATE read-back (10 Hz)

This module is the Python version of that supervisor, so the two stacks behave
the same.  It has no widgets and no I/O: the controller feeds it pad read-backs
and the clock, the GUI asks it which buttons are enabled (and why not), and the
controller streams `command()` to the pad.  Nothing here changes the pad, its
firmware or the wire protocol; the pad's own failsafes (1 s watchdog, armed
one-shot igniter, SAFE set on link loss) still apply underneath.

Mapping onto the pad (pad_link.rs PL:549-569):
    FSM FILL -> pad fill,  FSM SRV -> pad VEHICLE valves (fuel + ox),
    FSM IGN  -> pad igniter request, only with the operator's ARM and fresh
                telemetry,  PURGE is app-side (session-enabled),
    E-STOP / INHIBITED / link-loss latched -> the pad's SAFE set
                (FILL/PURGE closed, VEHICLE OPEN = vented, IGN off).
So in normal operation the vehicle valves are CLOSED (FSM "SRV OFF"); they open
for SRV ON (TEST), SRV+IGN ON (LIVE), a latched safe state, or on exit.

Rule sources (cited as file:line on branch esp32-and-stm32):
    fsm.rs      transition table           safety.rs   button gates + texts
    vehicle.rs  fill timer and latches     engine.rs   countdown, E-STOP, reset
    pad_link.rs stream mapping, ARM, purge, read-back confirmation

Deliberate differences from the Rust app (both make the safe action stronger):
    * E-STOP applies straight to the streamed command and needs only the laptop
      link; Rust drops it while pad telemetry is stale (spec flag 4.1).
    * A command that the pad's read-back has not confirmed after 1.5 s is
      released with a "PAD READBACK MISMATCH" / "no confirmation" error; in Rust
      the engine's 800 ms retries keep resetting that timer (spec flag 1.4).
    * There is no ACK/settle delay: commands apply immediately and stay
      "in progress" until the pad's read-back shows them.
Rust quirks kept on purpose so both stacks agree (raise them upstream, do not
fix them here alone): FILL ON is not blocked by E-STOP/INHIBITED and clears
INHIBITED; RESET FILL SEQ alone clears E-STOP; CLEAR FAULTS clears INHIBITED
and link-loss but not E-STOP; the T-0 HOLD never times out.
"""
from __future__ import annotations

import json
import os
import time
from dataclasses import asdict, dataclass, field
from typing import Callable

# ----------------------------------------------------------------- constants
LIVE, TEST = "live", "test"

DEFAULT_FILL_SECONDS = 60          # gse-proto lib.rs:60-64
MIN_FILL_SECONDS, MAX_FILL_SECONDS = 1, 600
DEFAULT_COUNTDOWN_SECONDS = 5      # lib.rs:69-71
MIN_COUNTDOWN_SECONDS, MAX_COUNTDOWN_SECONDS = 1, 600

STATE_STALE_S = 1.5                # pad.rs PP:82 -- pad telemetry fresh window
LINK_TIMEOUT_S = 5.0               # vehicle.rs:44 -- supervisor link-loss latch
FILL_FALLBACK_EXTRA_S = 2.0        # vehicle.rs:46 -- auto-close while offline
ACK_CONFIRM_TIMEOUT_S = 1.5        # pad_link.rs:53
MISMATCH_AFTER_S = 1.0             # pad_link.rs:55

FAULT_LINK_TIMEOUT = 1 << 2        # state.rs faults

# Operator actions (ids as in safety.rs / engine.rs)
FILL_ON, FILL_OFF = "fill_on", "fill_off"
SRV_ON, SRV_OFF = "srv_on", "srv_off"
IGN_PWR, IGN_SAFE = "ign_pwr", "ign_safe"
SRV_IGN_ON, SRV_IGN_OFF = "srv_ign_on", "srv_ign_off"
ABORT_COUNTDOWN = "abort_countdown"
RESET_CONTROLS, RESET_FILL, CLEAR_FAULTS, ESTOP = (
    "reset_controls", "reset_fill", "clear_faults", "estop")
ARM, DISARM = "arm", "disarm"
PURGE_OPEN, PURGE_CLOSE = "purge_open", "purge_close"
PURGE_ENABLE, PURGE_DISABLE = "purge_enable", "purge_disable"

FSM_EVENTS = (FILL_ON, FILL_OFF, SRV_ON, SRV_OFF, IGN_PWR, IGN_SAFE,
              SRV_IGN_ON, SRV_IGN_OFF)
PAD_ACTIONS = (ARM, DISARM, PURGE_OPEN, PURGE_CLOSE, PURGE_ENABLE, PURGE_DISABLE)
ALL_ACTIONS = FSM_EVENTS + (ABORT_COUNTDOWN, RESET_CONTROLS, RESET_FILL,
                            CLEAR_FAULTS, ESTOP) + PAD_ACTIONS

LABEL = {                                       # engine.rs:148-170
    FILL_ON: "FILL ON", FILL_OFF: "FILL OFF", SRV_ON: "SRV ON", SRV_OFF: "SRV OFF",
    IGN_PWR: "IGN PWR", IGN_SAFE: "IGN SAFE", SRV_IGN_ON: "SRV+IGN ON",
    SRV_IGN_OFF: "SRV+IGN OFF", ABORT_COUNTDOWN: "ABORT COUNTDOWN",
    RESET_CONTROLS: "RESET CONTROLS", RESET_FILL: "RESET FILL SEQ",
    CLEAR_FAULTS: "CLEAR FAULTS", ESTOP: "EMERGENCY STOP", ARM: "ARM",
    DISARM: "DISARM", PURGE_OPEN: "PURGE OPEN", PURGE_CLOSE: "PURGE CLOSE",
    PURGE_ENABLE: "PURGE ENABLE", PURGE_DISABLE: "PURGE DISABLE",
}

# Countdown targets (state.rs:216-236)
CD_SRV_IGN, CD_IGN = "srv_ign", "ign"
CD_LABEL = {CD_SRV_IGN: "SRV+IGN", CD_IGN: "IGN"}
CD_EVENT = {CD_SRV_IGN: SRV_IGN_ON, CD_IGN: IGN_PWR}
CD_MODE = {CD_SRV_IGN: LIVE, CD_IGN: TEST}

# Reason labels (control.rs:293-363), raw
REASON = {
    "FillSrvExclusive": "FILL and SRV are interlocked: both cannot be on at the same time",
    "FillActive": "SRV cannot be enabled while FILL is open",
    "SrvRequired": "IGN PWR requires SRV ON",
    "FillSeqIncomplete": "IGN PWR requires a completed FILL sequence",
    "IgnPowered": "IGN is in PWR; SAFE the igniter first",
    "AlreadyInState": "Already in the requested state",
    "NotActive": "Control is not active",
    "NotPermitted": "Action not permitted in the current state",
}

NO_OP_TEXT = {                                  # safety.rs:90-102
    FILL_ON: "FILL is already ON", FILL_OFF: "FILL is already OFF",
    SRV_ON: "SRV is already ON", SRV_OFF: "SRV is already OFF",
    IGN_PWR: "IGN is already PWR", IGN_SAFE: "IGN is already SAFE",
    SRV_IGN_ON: "SRV+IGN is already ON/PWR", SRV_IGN_OFF: "SRV+IGN is already OFF/SAFE",
}

TXT_OFFLINE_GROUND = "Ground board not connected"
TXT_OFFLINE_VEHICLE = "Vehicle not responding - waiting for pad telemetry"
TXT_PENDING = "Command in progress"
TXT_INIT = "Initialising - confirming all controls are OFF/SAFE"
TXT_LIVE_ONLY = "Only available in LIVE mode"
TXT_TEST_ONLY = "Disabled in LIVE mode (use SRV+IGN)"
TXT_LATCHED = "Safe state latched - press RESET CONTROLS first"


# ------------------------------------------------------------- pure helpers
@dataclass(frozen=True)
class Interlocks:
    fill_seq_complete: bool = False
    fill_paused: bool = False
    fill_seq_aborted: bool = False
    inhibited: bool = False
    estop: bool = False

    @property
    def fill_ready(self) -> bool:               # fsm.rs:302-304
        return (self.fill_seq_complete or self.fill_paused) and not self.fill_seq_aborted


def transition(state: tuple, event: str, il: Interlocks) -> tuple:
    """fsm.rs:163-288, line for line.

    state = (fill, srv, ign) booleans.  Returns ("allowed", next_state, None),
    ("noop", state, reason_key) or ("blocked", state, reason_key).
    """
    fill, srv, ign = state
    latch = il.inhibited or il.estop
    if event == FILL_ON:
        if fill:
            return ("noop", state, "AlreadyInState")
        if ign:
            return ("blocked", state, "IgnPowered")
        if srv:
            return ("blocked", state, "FillSrvExclusive")
        return ("allowed", (True, srv, ign), None)
    if event == FILL_OFF:
        return (("allowed", (False, srv, ign), None) if fill
                else ("noop", state, "NotActive"))
    if event == SRV_ON:
        if srv:
            return ("noop", state, "AlreadyInState")
        if fill:
            return ("blocked", state, "FillActive")
        if ign:
            return ("blocked", state, "IgnPowered")
        if not il.fill_ready:
            return ("blocked", state, "FillSeqIncomplete")
        if latch:
            return ("blocked", state, "NotPermitted")
        return ("allowed", (fill, True, ign), None)
    if event == SRV_OFF:
        if not srv:
            return ("noop", state, "NotActive")
        if ign:
            return ("blocked", state, "IgnPowered")
        return ("allowed", (fill, False, ign), None)
    if event == IGN_PWR:
        if ign:
            return ("noop", state, "AlreadyInState")
        if fill:
            return ("blocked", state, "FillActive")
        if not srv:
            return ("blocked", state, "SrvRequired")
        if not il.fill_ready:
            return ("blocked", state, "FillSeqIncomplete")
        if latch:
            return ("blocked", state, "NotPermitted")
        return ("allowed", (fill, srv, True), None)
    if event == IGN_SAFE:
        return (("allowed", (fill, srv, False), None) if ign
                else ("noop", state, "NotActive"))
    if event == SRV_IGN_ON:
        if srv and ign:
            return ("noop", state, "AlreadyInState")
        if fill:
            return ("blocked", state, "FillActive")
        if not il.fill_ready:
            return ("blocked", state, "FillSeqIncomplete")
        if latch:
            return ("blocked", state, "NotPermitted")
        return ("allowed", (fill, True, True), None)
    if event == SRV_IGN_OFF:
        if not srv and not ign:
            return ("noop", state, "NotActive")
        return ("allowed", (fill, False, False), None)
    raise ValueError(f"not an FSM event: {event}")


def blocked_text(event: str, il: Interlocks, reason: str) -> str:
    """safety.rs:105-131 -- first match wins."""
    if event in (SRV_ON, IGN_PWR, SRV_IGN_ON) and reason == "NotPermitted":
        if il.inhibited:
            return "Vehicle inhibited - press RESET CONTROLS and run a new fill"
        if il.estop:
            return "E-STOP is latched - press RESET CONTROLS first"
    if reason == "FillSeqIncomplete":
        if event == SRV_ON:
            return "SRV requires FILL to run and turn OFF first"
        if event == IGN_PWR:
            return "IGN requires FILL to run and turn OFF first"
        if event == SRV_IGN_ON:
            return "SRV+IGN requires FILL to run and turn OFF first"
    if event == SRV_IGN_ON and reason == "FillActive":
        return "SRV+IGN cannot be enabled while FILL is open"
    return REASON.get(reason, reason)


@dataclass
class Gate:
    enabled: bool
    reason: str = ""


def _ok() -> Gate:
    return Gate(True, "")


def _no(reason: str) -> Gate:
    return Gate(False, reason)


@dataclass(frozen=True)
class PadCommand:
    """What the 20 Hz stream carries (fields of protocol.Cmd)."""
    fill: bool
    veh: bool
    prg: bool
    ign: bool
    arm: bool

    def describe(self) -> str:                   # pad_link.rs:767-780
        o = lambda b: "OPEN" if b else "CLOSED"  # noqa: E731
        return (f"FILL {o(self.fill)} · VEH {o(self.veh)} · PURGE {o(self.prg)} · "
                f"IGN {'FIRE' if self.ign else 'OFF'}{' (armed)' if self.arm else ''}")


SAFE_SET = PadCommand(fill=False, veh=True, prg=False, ign=False, arm=False)


def describe_readback(rb) -> str:               # pad_link.rs:782-793
    if rb is None:
        return "no readback"
    o = lambda b: "OPEN" if b else "CLOSED"     # noqa: E731
    ign = "ON" if rb.out_ign else "OFF"
    comms_loss = bool(getattr(rb, "flags", 0) & 0x10)      # protocol.ST_COMMS_LOSS
    return (f"FILL {o(rb.out_fill)} · VEH {o(rb.out_veh)} · PURGE {o(rb.out_prg)} · "
            f"IGN {ign}{' (comms loss)' if comms_loss else ''}")


def readback_matches(rb, cmd: PadCommand, ign_seen: bool) -> bool:
    """pad_link.rs:748-757.  The pad has no IGN_SPENT flag; a one-shot that has
    been seen firing counts as confirmed (ign_seen)."""
    if rb is None:
        return False
    if bool(rb.out_fill) != cmd.fill or bool(rb.out_veh) != cmd.veh \
            or bool(rb.out_prg) != cmd.prg:
        return False
    return (ign_seen or bool(rb.out_ign)) if cmd.ign else not rb.out_ign


# ------------------------------------------------------------------ settings
@dataclass
class Settings:
    """Operator settings persisted between sessions (config.rs).  Purge enable
    is deliberately NOT here: it is per session (ECR-01 H8)."""
    op_mode: str = LIVE
    fill_seconds: int = DEFAULT_FILL_SECONDS
    countdown_seconds: int = DEFAULT_COUNTDOWN_SECONDS
    confirm_ign: bool = True

    def sanitize(self) -> "Settings":
        self.op_mode = self.op_mode if self.op_mode in (LIVE, TEST) else LIVE
        self.fill_seconds = max(MIN_FILL_SECONDS, min(MAX_FILL_SECONDS, int(self.fill_seconds)))
        self.countdown_seconds = max(MIN_COUNTDOWN_SECONDS,
                                     min(MAX_COUNTDOWN_SECONDS, int(self.countdown_seconds)))
        self.confirm_ign = bool(self.confirm_ign)
        return self

    @classmethod
    def load(cls, path: str | None) -> "Settings":
        s = cls()
        if path and os.path.isfile(path):
            try:
                with open(path, encoding="utf-8") as fh:
                    data = json.load(fh)
                for k in asdict(s):
                    if k in data:
                        setattr(s, k, data[k])
            except (OSError, ValueError, TypeError):
                s = cls()
        return s.sanitize()

    def save(self, path: str | None) -> None:
        if not path:
            return
        try:
            with open(path, "w", encoding="utf-8") as fh:
                json.dump(asdict(self), fh, indent=2)
        except OSError:
            pass


# ---------------------------------------------------------------- supervisor
@dataclass
class _Pending:
    label: str
    action: str
    expect: PadCommand
    since: float
    after_seq: int


Logger = Callable[[str, str], None]          # (level, text); level: info/ok/warn/error


class Supervisor:
    def __init__(self, settings: Settings | None = None,
                 clock: Callable[[], float] = time.monotonic,
                 log: Logger | None = None):
        self.settings = (settings or Settings()).sanitize()
        self.clock = clock
        self._log = log or (lambda level, text: None)
        self.purge_enabled = False           # session-only, survives reconnects
        self.connected = False
        self.reset_session()

    # ------------------------------------------------------------ lifecycle
    def reset_session(self) -> None:
        """A new link: fresh state machine, no fill progress, no latches
        (engine.rs:1368-1441 creates a new PadLink per connection)."""
        self.fill = self.srv = self.ign = False
        self.fill_timer_active = False
        self.fill_start = 0.0
        self.fill_target_ms = 0
        self.fill_duration_s = 0
        self.fill_paused = self.fill_seq_complete = self.fill_seq_aborted = False
        self.fill_auto_off = False
        self.estop = self.inhibited = self.link_loss_safe = False
        self.faults = 0
        self.armed = False
        self.purge_open = False
        self.controls_ready = False
        self.cd_target: str | None = None
        self.cd_deadline = 0.0
        self.cd_holding = False
        self.pending: _Pending | None = None
        self.shutting_down = False
        # read-back bookkeeping
        self.readback = None
        self.readback_seq = 0
        self.fresh = False
        self.ign_seen = False
        self._saw_fresh = False
        self._last_fresh_at = 0.0
        self._link_lost = False
        self._was_online = False
        self._was_latched = False
        self._last_cmd: PadCommand | None = None
        self._cmd_changed_at = self.clock()
        self.mismatch: str = ""

    def log(self, level: str, text: str) -> None:
        self._log(level, text)

    # ---------------------------------------------------------- read-only view
    @property
    def mode(self) -> str:
        return self.settings.op_mode

    @property
    def vehicle_online(self) -> bool:
        return self.connected and self.fresh

    def state(self) -> tuple:
        return (self.fill, self.srv, self.ign)

    def interlocks(self) -> Interlocks:
        return Interlocks(self.fill_seq_complete, self.fill_paused,
                          self.fill_seq_aborted, self.inhibited, self.estop)

    def outputs_safe(self) -> bool:
        return not (self.fill or self.srv or self.ign)

    def latched_safe(self) -> bool:            # pad_link.rs:737-739
        return self.estop or self.inhibited or self.link_loss_safe

    def fill_elapsed_ms(self) -> int:
        return int((self.clock() - self.fill_start) * 1000) if self.fill_timer_active else 0

    def fill_remaining_ms(self) -> int:
        if self.fill and self.fill_timer_active:
            return max(0, self.fill_target_ms - self.fill_elapsed_ms())
        if self.fill_paused:
            return self.fill_target_ms
        return 0

    def fill_total_s(self) -> int:
        return self.fill_duration_s if (self.fill or self.fill_paused) else self.settings.fill_seconds

    def countdown_remaining_s(self) -> float:
        if self.cd_target is None or self.cd_holding:
            return 0.0
        return max(0.0, self.cd_deadline - self.clock())

    # -------------------------------------------------------------- stream
    def command(self) -> PadCommand:
        """The pad command for the 20 Hz stream (pad_link.rs:549-569)."""
        if self.latched_safe():
            return SAFE_SET
        arm = self.armed and self.fresh                      # never arm blind
        return PadCommand(fill=self.fill, veh=self.srv,
                          prg=self.purge_enabled and self.purge_open,
                          ign=self.ign and arm, arm=arm)

    def _command_for_ack(self) -> PadCommand:
        """command() without the freshness term (pad_link.rs:572-585)."""
        if self.latched_safe():
            return SAFE_SET
        return PadCommand(fill=self.fill, veh=self.srv,
                          prg=self.purge_enabled and self.purge_open,
                          ign=self.ign and self.armed, arm=self.armed)

    # ---------------------------------------------------------------- gates
    def _base(self) -> str | None:
        if not self.connected:
            return TXT_OFFLINE_GROUND
        if not self.fresh:
            return TXT_OFFLINE_VEHICLE
        if self.pending is not None:
            return TXT_PENDING
        if not self.controls_ready:
            return TXT_INIT
        return None

    def gates(self) -> dict[str, Gate]:
        """safety.rs evaluate() + pad_actions(): every action -> enabled/reason."""
        st, il = self.state(), self.interlocks()
        base = self._base()
        cd_gate = (f"{CD_LABEL[self.cd_target]} {self.settings.countdown_seconds} s "
                   "countdown in progress") if self.cd_target else None
        channel_base = base or cd_gate
        held_base = base if self.cd_holding else channel_base

        def gate(b: str | None, ev: str) -> Gate:
            if b:
                return _no(b)
            outcome, _, reason = transition(st, ev, il)
            if outcome == "allowed":
                return _ok()
            if outcome == "noop":
                return _no(NO_OP_TEXT[ev])
            return _no(blocked_text(ev, il, reason))

        g: dict[str, Gate] = {
            FILL_ON: gate(channel_base, FILL_ON),
            FILL_OFF: gate(channel_base, FILL_OFF),
        }
        if self.mode == TEST:
            g[SRV_ON] = gate(channel_base, SRV_ON)
            g[SRV_OFF] = gate(channel_base, SRV_OFF)
            g[IGN_PWR] = gate(held_base if self.cd_target == CD_IGN else channel_base, IGN_PWR)
            g[IGN_SAFE] = (_ok() if self.cd_target == CD_IGN and base is None
                           else gate(channel_base, IGN_SAFE))
            g[SRV_IGN_ON] = _no(TXT_LIVE_ONLY)
            g[SRV_IGN_OFF] = _no(TXT_LIVE_ONLY)
        else:
            for a in (SRV_ON, SRV_OFF, IGN_PWR, IGN_SAFE):
                g[a] = _no(TXT_TEST_ONLY)
            g[SRV_IGN_ON] = gate(held_base if self.cd_target == CD_SRV_IGN else channel_base,
                                 SRV_IGN_ON)
            g[SRV_IGN_OFF] = (_ok() if self.cd_target == CD_SRV_IGN and base is None
                              else gate(channel_base, SRV_IGN_OFF))
        # pad ARM requirement (safety.rs:236-242)
        if not self.armed:
            for a, what in ((IGN_PWR, "IGN PWR"), (SRV_IGN_ON, "SRV+IGN")):
                if g[a].enabled:
                    g[a] = _no(f"{what} requires ARM - arm the igniter first")

        g[ABORT_COUNTDOWN] = (_no(base) if base else
                              _ok() if self.cd_target else _no("No active countdown"))
        g[RESET_CONTROLS] = _no(base) if base else _ok()
        if channel_base:
            g[RESET_FILL] = _no(channel_base)
            g[CLEAR_FAULTS] = _no(channel_base)
        else:
            if self.fill:
                g[RESET_FILL] = _no("Close FILL first")
            elif self.ign:
                g[RESET_FILL] = _no("SAFE the igniter first")
            elif not (il.fill_seq_complete or il.fill_paused or il.fill_seq_aborted
                      or il.inhibited or il.estop):
                g[RESET_FILL] = _no("Nothing to reset")
            else:
                g[RESET_FILL] = _ok()
            g[CLEAR_FAULTS] = (_no("No latched faults")
                               if self.faults == 0 and not il.inhibited and not il.estop
                               else _ok())
        # E-STOP: needs only the laptop link (deliberate difference, see module doc)
        g[ESTOP] = _ok() if self.connected else _no(TXT_OFFLINE_GROUND)

        # pad actions (safety.rs:354-394)
        latched = self.latched_safe()
        if base:
            g[ARM] = _no(base)
        elif self.armed:
            g[ARM] = _no("Already armed")
        elif latched:
            g[ARM] = _no(TXT_LATCHED)
        else:
            g[ARM] = _ok()
        g[DISARM] = _ok() if self.armed else _no("Not armed")
        if channel_base:
            g[PURGE_OPEN] = _no(channel_base)
        elif not self.purge_enabled:
            g[PURGE_OPEN] = _no("Purge valve not enabled for this session")
        elif self.purge_open:
            g[PURGE_OPEN] = _no("PURGE is already OPEN")
        elif self.link_loss_safe or self.estop:
            g[PURGE_OPEN] = _no(TXT_LATCHED)
        else:
            g[PURGE_OPEN] = _ok()
        g[PURGE_CLOSE] = _ok() if self.purge_open else _no("PURGE is already CLOSED")
        g[PURGE_ENABLE] = _no("Purge valve already enabled") if self.purge_enabled else _ok()
        g[PURGE_DISABLE] = _ok() if self.purge_enabled else _no("Purge valve already disabled")
        return g

    # -------------------------------------------------------------- actions
    def act(self, action: str) -> bool:
        """One operator action, in engine.rs user_action order (EN:953-1150).
        Returns True if something was done."""
        if action not in ALL_ACTIONS:
            raise ValueError(f"unknown action {action!r}")
        label = LABEL[action]
        if self.shutting_down:
            self.log("warn", f"{label} ignored - shutdown in progress")
            return False
        if action in PAD_ACTIONS:
            return self._pad_action(action)
        gate = self.gates()[action]

        if action == ESTOP:
            return self._estop(gate)
        if action == RESET_CONTROLS:
            if self.cd_target:
                self._cancel_countdown(f"{CD_LABEL[self.cd_target]} countdown cancelled "
                                       "by RESET CONTROLS")
            self._pad_make_safe("RESET CONTROLS")

        if not gate.enabled:
            self.log("warn", f"{label} blocked: {gate.reason}")
            return False

        # T-0 manual fire (EN:1062-1093)
        if self.cd_holding and self.cd_target and action == CD_EVENT[self.cd_target]:
            t = CD_LABEL[self.cd_target]
            self._clear_countdown()
            self.log("warn", f"{t} explicitly triggered by operator after T-0 HOLD")
            return self._send(action)
        # countdown abort via the active channel's safe action (EN:1095-1102)
        if self.cd_target and (action == ABORT_COUNTDOWN
                               or (action == IGN_SAFE and self.cd_target == CD_IGN)
                               or (action == SRV_IGN_OFF and self.cd_target == CD_SRV_IGN)):
            self._abort_countdown_by_operator()
            return True
        # countdown start (EN:1104-1147)
        if self.cd_target is None and ((action == IGN_PWR and self.mode == TEST)
                                       or (action == SRV_IGN_ON and self.mode == LIVE)):
            target = CD_IGN if action == IGN_PWR else CD_SRV_IGN
            self.cd_target = target
            self.cd_deadline = self.clock() + self.settings.countdown_seconds
            self.cd_holding = False
            t = CD_LABEL[target]
            self.log("warn", f"{t} armed - {self.settings.countdown_seconds} s countdown "
                             f"started (click ABORT or {t} to abort)")
            return True
        return self._send(action)

    def _send(self, action: str, label: str | None = None, auto: bool = False) -> bool:
        """Apply one command to the hosted state machine and wait for the pad
        to confirm it in its read-back."""
        label = label or LABEL[action]
        now = self.clock()
        if action == FILL_ON:
            if self.fill_paused and self.fill_target_ms > 0:
                self.log("info", f"FILL ON requested (resume, "
                                 f"{self.fill_target_ms / 1000:.1f} s remaining)")
            else:
                self.log("info", f"FILL ON requested ({self.settings.fill_seconds} s fill)")
        elif not auto:
            self.log("info", f"{label} requested")

        if action in FSM_EVENTS:
            outcome, nxt, reason = transition(self.state(), action, self.interlocks())
            if outcome == "blocked":                 # gates should prevent this
                self.log("warn", f"{label} rejected: {blocked_text(action, self.interlocks(), reason)}")
                return False
            if outcome == "allowed":
                self._apply(action, nxt, now, auto)
        elif action == RESET_CONTROLS:
            self._safe_all(now, estop=False)
            # engine.rs:2644-2655: RESET CONTROLS is followed by RESET FILL SEQ
            # whenever a fill/latch flag is left, so the net effect is a clean start
            if (self.fill_seq_complete or self.fill_paused or self.fill_seq_aborted
                    or self.inhibited or self.estop):
                self.log("info", "Resetting latched fill sequence")
                self._reset_fill()
        elif action == RESET_FILL:
            self._reset_fill()
        elif action == CLEAR_FAULTS:
            self.faults = 0
            self.inhibited = False
            self.link_loss_safe = False
            self.log("ok", "Latched faults cleared")
        self.pending = _Pending(label, action, self._command_for_ack(), now, self.readback_seq)
        return True

    def _apply(self, action: str, nxt: tuple, now: float, auto: bool) -> None:
        if action == FILL_ON:                    # vehicle.rs:642-659
            resuming = self.fill_paused and self.fill_target_ms > 0
            if not resuming:
                self.fill_duration_s = self.settings.fill_seconds
                self.fill_target_ms = self.settings.fill_seconds * 1000
            self.fill = True
            self.fill_timer_active = True
            self.fill_start = now
            self.fill_paused = self.fill_seq_complete = self.fill_seq_aborted = False
            self.fill_auto_off = False
            self.inhibited = False
            return
        if action == FILL_OFF:
            self._close_fill(now, force_abort=False, is_auto=auto)
            return
        self.fill, self.srv, self.ign = nxt
        if action == IGN_PWR or (action == SRV_IGN_ON and nxt[2]):
            self.log("warn", "IGN PWR asserted")

    def _close_fill(self, now: float, force_abort: bool, is_auto: bool) -> None:
        """vehicle.rs:734-778 -- the single place a fill's outcome is decided."""
        if not self.fill:
            return
        was_timed = self.fill_timer_active
        elapsed = int((now - self.fill_start) * 1000)
        target = self.fill_target_ms
        completed = was_timed and (elapsed >= target or (is_auto and elapsed + 500 >= target))
        self.fill = False
        self.fill_timer_active = False
        if completed:
            self.fill_paused = False
            self.fill_target_ms = 0
            self.fill_seq_complete, self.fill_seq_aborted = True, False
            self.log("ok", f"FILL sequence COMPLETE ({self.fill_duration_s} s) - "
                           "downstream channels unlocked")
        elif force_abort:
            self.fill_paused = False
            self.fill_target_ms = 0
            self.fill_seq_complete, self.fill_seq_aborted = False, True
            self.log("warn", "FILL sequence ABORTED before the configured duration - "
                             "ignition remains inhibited")
        else:
            self.fill_paused = True
            self.fill_target_ms = max(0, target - elapsed)
            self.fill_seq_complete = self.fill_seq_aborted = False
            self.log("ok", f"FILL paused ({self.fill_target_ms / 1000:.1f} s remaining of "
                           f"{self.fill_duration_s} s) - downstream channels unlocked; "
                           "turn FILL -> ON to resume")

    def _safe_all(self, now: float, estop: bool) -> None:
        """SYSTEM/SAFE_ALL (vehicle.rs:567-597)."""
        was_on = self.srv or self.ign
        self._close_fill(now, force_abort=estop, is_auto=False)
        self.srv = self.ign = False
        if estop:
            self.fill_paused = False
            self.fill_target_ms = 0
            self.estop = True
            self.log("warn", "E-STOP asserted on vehicle - all outputs SAFE")
        elif was_on:
            self.log("ok", "IGN SAFE")

    def _reset_fill(self) -> None:
        """SYSTEM/RESET_FILL (vehicle.rs:598-616)."""
        if self.fill or self.ign:
            self.log("warn", "RESET FILL SEQ rejected: close FILL and SAFE the igniter first")
            return
        self.fill_paused = False
        self.fill_target_ms = 0
        self.fill_duration_s = 0
        self.fill_seq_complete = self.fill_seq_aborted = self.fill_auto_off = False
        self.estop = self.inhibited = self.link_loss_safe = False
        self.log("info", "Fill sequence latch reset")

    def _estop(self, gate: Gate) -> bool:
        """engine.rs:990-1031, applied straight to the stream."""
        if self.cd_target:
            self._cancel_countdown(f"{CD_LABEL[self.cd_target]} countdown aborted by E-STOP")
        if not gate.enabled:
            self.log("error", f"E-STOP NOT TRANSMITTED: {gate.reason or 'vehicle unreachable'}")
            return False
        if self.pending is not None:
            self.log("warn", f"{self.pending.label} superseded by E-STOP")
            self.pending = None
        self._pad_make_safe("E-STOP")
        self.log("warn", "EMERGENCY STOP - all outputs SAFE")
        now = self.clock()
        self._safe_all(now, estop=True)
        self.pending = _Pending(LABEL[ESTOP], ESTOP, self._command_for_ack(), now,
                                self.readback_seq)
        return True

    def _pad_action(self, action: str) -> bool:
        gate = self.gates()[action]
        label = LABEL[action]
        if not gate.enabled:
            self.log("warn", f"{label} blocked: {gate.reason}")
            return False
        if action == ARM:
            self.armed = True
            self.log("warn", "ARMED - the arm code now rides on every pad command")
        elif action == DISARM:
            self.armed = False
            self.log("ok", "Disarmed - igniter inhibited")
        elif action == PURGE_OPEN:
            self.purge_open = self.purge_enabled
            self.log("warn", "PURGE OPEN requested")
        elif action == PURGE_CLOSE:
            self.purge_open = False
            self.log("info", "PURGE CLOSE requested")
        elif action == PURGE_ENABLE:
            self.purge_enabled = True
            self.log("warn", "Purge valve ENABLED for this session - never open it with "
                             "the vehicle connected")
        elif action == PURGE_DISABLE:
            self.purge_enabled = False
            self.purge_open = False
            self.log("ok", "Purge valve disabled - forced closed on the pad")
        return True

    def _pad_make_safe(self, why: str) -> None:
        """engine.rs:3287-3310: disarm and close purge."""
        disarmed, closed = self.armed, self.purge_open
        self.armed = False
        self.purge_open = False
        if disarmed and closed:
            self.log("warn", f"{why}: igniter disarmed and PURGE closed")
        elif disarmed:
            self.log("warn", f"{why}: igniter disarmed")
        elif closed:
            self.log("warn", f"{why}: PURGE closed")

    # ------------------------------------------------------------ countdown
    def _clear_countdown(self) -> str | None:
        t = self.cd_target
        self.cd_target = None
        self.cd_holding = False
        self.cd_deadline = 0.0
        return t

    def _cancel_countdown(self, reason: str | None) -> str | None:
        t = self._clear_countdown()
        if t and reason:
            self.log("warn", reason)
        return t

    def _abort_countdown_by_operator(self) -> None:
        """engine.rs:922-951 -- FILL progress preserved."""
        t = self._cancel_countdown(None)
        if t is None:
            return
        self._pad_make_safe("countdown abort")
        if self.srv:
            self.log("warn", f"{CD_LABEL[t]} countdown aborted by operator - returning "
                             "SRV -> OFF; FILL progress preserved")
            self._send(SRV_OFF, label="SRV OFF (countdown abort)")
        else:
            self.log("warn", f"{CD_LABEL[t]} countdown aborted by operator - all outputs "
                             "OFF/SAFE; FILL progress preserved")

    # ---------------------------------------------------------------- mode
    def set_mode(self, mode: str) -> bool:
        """engine.rs:631-650: only while every output is OFF/SAFE and no
        countdown exists.  Latches and a paused fill do not block it."""
        if mode not in (LIVE, TEST):
            raise ValueError(mode)
        if mode == self.settings.op_mode:
            return True
        if not self.outputs_safe() or self.cd_target is not None:
            self.log("warn", "Cannot change operating mode while controls or countdown "
                             "are active")
            return False
        self.settings.op_mode = mode
        self.log("ok", f"operating mode set to {mode.upper()}")
        return True

    # ---------------------------------------------------------------- tick
    def tick(self, readback, readback_seq: int, readback_age_s: float,
             connected: bool) -> None:
        """Advance timers and latches.  Call every GUI tick (>= 20 Hz)."""
        now = self.clock()
        self.connected = connected
        if readback_seq != self.readback_seq:
            self.readback_seq = readback_seq
            self.readback = readback
            if readback is not None and readback.out_ign:
                self.ign_seen = True
        self.fresh = bool(connected and readback is not None
                          and readback_age_s <= STATE_STALE_S and readback.link_ok)
        online = self.vehicle_online

        # link-loss latch (vehicle.rs:803-825); self-clears when the pad is heard
        if self.fresh:
            self._saw_fresh = True
            self._last_fresh_at = now
            if self._link_lost:
                self._link_lost = False
                self.link_loss_safe = False
        elif self._saw_fresh and not self._link_lost and now - self._last_fresh_at > LINK_TIMEOUT_S:
            self._link_lost = True
            self.link_loss_safe = True
            self.faults |= FAULT_LINK_TIMEOUT
            if self.fill or self.srv or self.ign:
                if self.fill:
                    self._close_fill(now, force_abort=True, is_auto=False)
                self.srv = self.ign = False
                self.inhibited = True
                self.log("error", "VEHICLE LINK TIMEOUT - all outputs forced SAFE, "
                                  "ignition inhibited")

        # vehicle link edges (engine.rs:1708-1750)
        if online and not self._was_online:
            self.log("ok", "Pad telemetry fresh - link OK")
        elif self._was_online and not online:
            if self.cd_target:
                self._cancel_countdown(f"{CD_LABEL[self.cd_target]} countdown aborted: "
                                       "vehicle link lost")
            if self.pending is not None:
                self.log("warn", f"Vehicle link lost while waiting for confirmation of "
                                 f"{self.pending.label}")
                self.pending = None
            self._pad_make_safe("vehicle link loss")
            self.log("warn", "Pad telemetry stale - controls locked until the pad answers")
        self._was_online = online

        # INIT handshake (engine.rs:2922-2953)
        if not self.controls_ready and online and self.outputs_safe() and self.pending is None:
            self.controls_ready = True
            self.log("ok", "Control state machine initialised: FILL OFF · SRV OFF · IGN SAFE "
                           "(all controls OFF/SAFE)")

        # latched-safe rising edge: disarm + close purge (pad_link.rs:504-517)
        latched = self.latched_safe()
        if latched and not self._was_latched:
            if self.armed:
                self.armed = False
                self.log("warn", "Igniter disarmed - safe state latched")
            self.purge_open = False
        self._was_latched = latched

        # fill auto-close (engine.rs:1859-1890; vehicle fallback vehicle.rs:780-801)
        if self.fill and self.fill_timer_active and not self.shutting_down:
            elapsed = int((now - self.fill_start) * 1000)
            if online and self.pending is None and elapsed >= self.fill_target_ms:
                self.log("info", "FILL OFF (auto) requested")
                self.log("warn", "FILL timer expired - automatic FILL OFF sent")
                self._send(FILL_OFF, label="FILL OFF (auto)", auto=True)
            elif elapsed >= self.fill_target_ms + FILL_FALLBACK_EXTRA_S * 1000:
                self._close_fill(now, force_abort=False, is_auto=True)
                self.fill_auto_off = True
                self.log("warn", f"FILL auto-closed after {self.fill_duration_s} s "
                                 "(timer fallback while the pad was not answering)")

        # countdown re-validation and T-0 HOLD (engine.rs:1892-1956)
        if self.cd_target:
            t = self.cd_target
            ok = (self.mode == CD_MODE[t] and online and self.controls_ready
                  and self.pending is None and self.armed
                  and transition(self.state(), CD_EVENT[t], self.interlocks())[0] == "allowed")
            if not ok:
                self._cancel_countdown(f"{CD_LABEL[t]} countdown aborted because a safety "
                                       "interlock changed")
            elif not self.cd_holding and now >= self.cd_deadline:
                self.cd_holding = True
                self.log("warn", f"{CD_LABEL[t]} countdown reached T-0.0 s - HOLD; press L "
                                 "to explicitly trigger ignition or Shift+L to cancel")

        # stream bookkeeping (pad_link.rs:519-541, 669-707)
        cmd = self.command()
        if cmd != self._last_cmd:
            self._last_cmd = cmd
            self._cmd_changed_at = now
        if not cmd.ign:
            self.ign_seen = False

        # pending confirmation (pad_link.rs:620-667)
        p = self.pending
        if p is not None:
            if (self.fresh and self.readback_seq > p.after_seq
                    and readback_matches(self.readback, p.expect, self.ign_seen)):
                self.pending = None
                self.log("ok", f"{p.label} completed in {int((now - p.since) * 1000)} ms")
                if p.action == RESET_CONTROLS:
                    self.log("ok", "All outputs confirmed OFF/SAFE")
            elif now - p.since >= ACK_CONFIRM_TIMEOUT_S:
                self.pending = None
                if not self.fresh:
                    self.log("error", f"No confirmation from the pad for {p.label} - "
                                      "command state unknown")
                else:
                    self.log("error", f"PAD READBACK MISMATCH - commanded "
                                      f"{p.expect.describe()}, pad reports "
                                      f"{describe_readback(self.readback)}")

        # continuous read-back monitor (display only)
        if (self.fresh and now - self._cmd_changed_at >= MISMATCH_AFTER_S
                and not readback_matches(self.readback, cmd, self.ign_seen)):
            if not self.mismatch:
                self.mismatch = (f"commanded {cmd.describe()}, pad reports "
                                 f"{describe_readback(self.readback)}")
                self.log("error", f"PAD READBACK MISMATCH - {self.mismatch}")
        elif self.mismatch:
            self.mismatch = ""
            self.log("ok", "Pad readback matches the command again")

    # ------------------------------------------------------------ shutdown
    def shutdown(self) -> None:
        """engine.rs:2983-3112: cancel the countdown, ALL SAFE (no fill-latch
        follow-up), then the caller sends the pad its SAFE set and closes."""
        if self.cd_target:
            self._cancel_countdown(f"{CD_LABEL[self.cd_target]} countdown cancelled for "
                                   "shutdown")
        self.log("info", "Shutdown requested - commanding ALL SAFE before exit")
        self._safe_all(self.clock(), estop=False)
        self.shutting_down = True

    # --------------------------------------------------------- UI summaries
    def machine_text(self) -> str:
        """The one-line state summary (app.js machine line)."""
        if not self.controls_ready:
            return "INIT · confirming OFF/SAFE"
        fill = "ON" if self.fill else ("PAUSED" if self.fill_paused else "OFF")
        txt = (f"FILL {fill} · VEH {'OPEN' if self.srv else 'CLOSED'} · "
               f"IGN {'PWR' if self.ign else 'SAFE'}")
        if self.armed:
            txt += " · ARMED"
        if self.purge_open:
            txt += " · PURGE OPEN"
        if self.latched_safe():
            txt += " · PAD HELD IN SAFE SET (VEH vented)"
        return txt

    def fill_status(self) -> str:
        """Fill bar text, in app.js priority order."""
        if self.fill:
            rem = self.fill_remaining_ms() / 1000
            return f"closing soon ({rem:.1f} s)" if rem < 5 else f"filling ({rem:.0f} s left)"
        if self.fill_seq_aborted:
            return "sequence aborted"
        if self.fill_paused:
            return f"paused ({self.fill_target_ms / 1000:.1f} s left) · channels unlocked"
        if self.fill_seq_complete:
            return "sequence complete"
        return "idle"

    def fill_fraction(self) -> float:
        if self.fill_seq_complete and not self.fill:
            return 1.0
        total = self.fill_total_s() * 1000
        if (self.fill or self.fill_paused) and total > 0:
            return max(0.0, min(1.0, 1.0 - self.fill_remaining_ms() / total))
        return 0.0
