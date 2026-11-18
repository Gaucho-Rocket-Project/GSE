"""
test_supervisor.py -- the GSE `main` control rules, ported to supervisor.py.

Pure logic: a fake clock and a fake pad that reports whatever it is commanded
(or not, to test the failure paths).  Every rule cites its Rust source on
branch esp32-and-stm32 so the two stacks can be compared case by case.

    python software/tests/test_supervisor.py
"""
from __future__ import annotations

import itertools
import os
import sys
from dataclasses import dataclass

HERE = os.path.dirname(os.path.abspath(__file__))
sys.path.insert(0, os.path.join(HERE, ".."))

from gse_control import supervisor as S          # noqa: E402


# ------------------------------------------------------------------ fixtures
class Clock:
    def __init__(self):
        self.t = 1000.0

    def __call__(self):
        return self.t


@dataclass
class RB:
    """Just the PAD_STATE fields the supervisor reads."""
    out_fill: int = 0
    out_veh: int = 1
    out_prg: int = 0
    out_ign: int = 0
    link_ok: bool = True
    armed: bool = False
    flags: int = 0x01


class Rig:
    """Supervisor + clock + a pad that follows the stream (unless told not to)."""

    def __init__(self, mode=S.LIVE, fill_s=10, cd_s=5):
        self.clock = Clock()
        self.logs: list[tuple[str, str]] = []
        self.sup = S.Supervisor(S.Settings(op_mode=mode, fill_seconds=fill_s,
                                           countdown_seconds=cd_s),
                                clock=self.clock,
                                log=lambda lv, tx: self.logs.append((lv, tx)))
        self.seq = 0
        self.pad_follows = True
        self.pad_alive = True
        self.rb = RB()
        self.rb_at = self.clock.t
        self.sup.connected = True

    def step(self, dt=0.05, n=1):
        for _ in range(n):
            self.clock.t += dt
            if self.pad_alive:
                if self.pad_follows:
                    c = self.sup.command()
                    self.rb = RB(out_fill=int(c.fill), out_veh=int(c.veh),
                                 out_prg=int(c.prg), out_ign=int(c.ign), armed=c.arm)
                self.seq += 1
                self.rb_at = self.clock.t
            self.sup.tick(self.rb, self.seq, self.clock.t - self.rb_at, connected=True)

    def run(self, seconds, dt=0.05):
        self.step(dt, int(round(seconds / dt)))

    def ready(self):
        self.step(n=3)
        assert self.sup.controls_ready
        return self

    def act(self, a):
        r = self.sup.act(a)
        self.step()
        return r

    def gate(self, a):
        return self.sup.gates()[a]

    def logged(self, text):
        return any(text in tx for _, tx in self.logs)


def complete_fill(r: Rig):
    assert r.act(S.FILL_ON)
    r.run(r.sup.settings.fill_seconds + 0.5)
    assert r.sup.fill_seq_complete and not r.sup.fill
    r.step(n=3)


# ---------------------------------------------------------- fsm.rs (table)
def test_transition_invariants_and_reachable_states():
    """fsm.rs:656-703 -- exhaustive walk from SAFE."""
    for il in (S.Interlocks(), S.Interlocks(fill_paused=True),
               S.Interlocks(fill_seq_complete=True)):
        seen = {(False, False, False)}
        frontier = [(False, False, False)]
        while frontier:
            st = frontier.pop()
            for ev in S.FSM_EVENTS:
                outcome, nxt, _ = S.transition(st, ev, il)
                if outcome == "allowed":
                    fill, srv, ign = nxt
                    assert not (fill and srv), (st, ev, nxt)
                    if srv and not st[1]:
                        assert il.fill_ready
                    if ign:
                        assert srv and il.fill_ready
                    if nxt not in seen:
                        seen.add(nxt)
                        frontier.append(nxt)
        assert seen <= {(False, False, False), (True, False, False),
                        (False, True, False), (False, True, True)}


def test_transition_specific_outcomes():
    safe, filling, srv, armed = ((False,) * 3, (True, False, False),
                                 (False, True, False), (False, True, True))
    none, paused = S.Interlocks(), S.Interlocks(fill_paused=True)
    assert S.transition(safe, S.SRV_ON, none)[2] == "FillSeqIncomplete"
    assert S.transition(safe, S.SRV_ON, paused)[0] == "allowed"
    assert S.transition(srv, S.FILL_ON, paused)[2] == "FillSrvExclusive"
    assert S.transition(filling, S.SRV_ON, paused)[2] == "FillActive"
    assert S.transition(safe, S.IGN_PWR, paused)[2] == "SrvRequired"
    assert S.transition(armed, S.SRV_OFF, paused)[2] == "IgnPowered"
    assert S.transition(armed, S.FILL_ON, paused)[2] == "IgnPowered"
    aborted = S.Interlocks(fill_seq_complete=True, fill_seq_aborted=True)
    assert S.transition(srv, S.IGN_PWR, aborted)[2] == "FillSeqIncomplete"
    for latch in (S.Interlocks(fill_seq_complete=True, inhibited=True),
                  S.Interlocks(fill_seq_complete=True, estop=True)):
        assert S.transition(safe, S.SRV_IGN_ON, latch)[2] == "NotPermitted"
        assert S.transition(safe, S.FILL_ON, latch)[0] == "allowed"   # Rust quirk kept
    assert S.transition(safe, S.FILL_OFF, none)[0] == "noop"
    assert S.transition(armed, S.SRV_IGN_OFF, none)[1] == (False, False, False)


# ------------------------------------------------------------- safety.rs gates
def test_everything_locked_until_init_and_offline_texts():
    r = Rig()
    r.sup.connected = False
    assert r.gate(S.FILL_ON).reason == S.TXT_OFFLINE_GROUND
    assert not r.gate(S.ESTOP).enabled
    r.sup.connected = True
    assert r.gate(S.FILL_ON).reason == S.TXT_OFFLINE_VEHICLE
    r.step()
    assert r.sup.controls_ready, "INIT completes on the first fresh read-back"
    assert r.logged("Control state machine initialised")
    assert r.gate(S.FILL_ON).enabled


def test_live_and_test_mode_buttons():
    r = Rig(mode=S.LIVE).ready()
    assert r.gate(S.SRV_ON).reason == "Disabled in LIVE mode (use SRV+IGN)"
    assert r.gate(S.SRV_IGN_ON).reason == "SRV+IGN requires FILL to run and turn OFF first"
    r = Rig(mode=S.TEST).ready()
    assert r.gate(S.SRV_IGN_ON).reason == "Only available in LIVE mode"
    assert r.gate(S.SRV_ON).reason == "SRV requires FILL to run and turn OFF first"
    assert r.gate(S.IGN_PWR).reason == "IGN PWR requires SRV ON"


def test_ignition_needs_arm_and_arm_needs_fresh_telemetry():
    r = Rig(mode=S.LIVE, fill_s=1).ready()
    complete_fill(r)
    assert r.gate(S.SRV_IGN_ON).reason == "SRV+IGN requires ARM - arm the igniter first"
    assert r.act(S.ARM)
    assert r.gate(S.SRV_IGN_ON).enabled
    assert r.sup.command().arm
    r.pad_alive = False                      # telemetry goes stale
    r.run(1.6)
    assert not r.sup.command().arm, "never arm blind"


# ------------------------------------------------- vehicle.rs fill sequence
def test_fill_pause_resume_and_auto_complete():
    r = Rig(mode=S.TEST, fill_s=10).ready()
    assert r.act(S.FILL_ON) and r.sup.command().fill
    assert not r.sup.command().veh, "vehicle valves stay CLOSED while filling"
    r.run(4)
    assert r.act(S.FILL_OFF)
    assert r.sup.fill_paused and 5500 <= r.sup.fill_remaining_ms() <= 6100
    assert r.logged("FILL paused")
    assert r.gate(S.SRV_ON).enabled, "a paused fill unlocks SRV"
    r.run(3)
    assert 5500 <= r.sup.fill_remaining_ms() <= 6100, "paused time is frozen"
    assert r.act(S.FILL_ON)                  # resume
    r.run(6.3)
    assert not r.sup.fill and r.sup.fill_seq_complete and not r.sup.fill_paused
    assert r.logged("FILL timer expired - automatic FILL OFF sent")
    assert r.logged("FILL sequence COMPLETE")


def test_fill_srv_exclusive_both_ways():
    r = Rig(mode=S.TEST, fill_s=1).ready()
    r.act(S.FILL_ON)
    assert r.gate(S.SRV_ON).reason == "SRV cannot be enabled while FILL is open"
    r.run(1.5)
    assert r.act(S.SRV_ON) and r.sup.command().veh
    r.step(n=3)
    assert r.gate(S.FILL_ON).reason == ("FILL and SRV are interlocked: both cannot be "
                                        "on at the same time")


# ------------------------------------------------------ engine.rs countdown
def test_live_countdown_holds_at_t0_and_needs_a_manual_trigger():
    r = Rig(mode=S.LIVE, fill_s=1, cd_s=3).ready()
    complete_fill(r)
    r.act(S.ARM)
    assert r.act(S.SRV_IGN_ON)
    assert r.sup.cd_target == S.CD_SRV_IGN
    assert not r.sup.command().veh and not r.sup.command().ign, "nothing moves at start"
    assert r.gate(S.FILL_ON).reason == "SRV+IGN 3 s countdown in progress"
    r.run(3.2)
    assert r.sup.cd_holding and r.logged("HOLD")
    r.run(10)
    assert not r.sup.command().ign, "the HOLD never fires by itself"
    assert r.act(S.SRV_IGN_ON)               # the manual trigger (L)
    c = r.sup.command()
    assert c.veh and c.ign and c.arm
    assert r.logged("explicitly triggered by operator after T-0 HOLD")
    r.step(n=5)
    assert r.act(S.SRV_IGN_OFF)
    c = r.sup.command()
    assert not c.veh and not c.ign


def test_test_mode_abort_returns_srv_off_and_keeps_fill_progress():
    r = Rig(mode=S.TEST, fill_s=10, cd_s=5).ready()
    r.act(S.FILL_ON)
    r.run(2)
    r.act(S.FILL_OFF)                        # paused
    r.step(n=3)
    r.act(S.SRV_ON)
    r.step(n=3)
    r.act(S.ARM)
    assert r.act(S.IGN_PWR) and r.sup.cd_target == S.CD_IGN
    r.run(1)
    assert r.act(S.ABORT_COUNTDOWN)
    assert r.sup.cd_target is None and not r.sup.srv and not r.sup.armed
    assert r.sup.fill_paused, "abort preserves FILL progress"
    assert r.logged("returning SRV -> OFF; FILL progress preserved")


def test_countdown_cancelled_when_an_interlock_changes():
    r = Rig(mode=S.LIVE, fill_s=1, cd_s=5).ready()
    complete_fill(r)
    r.act(S.ARM)
    r.act(S.SRV_IGN_ON)
    r.act(S.DISARM)
    r.step()
    assert r.sup.cd_target is None
    assert r.logged("countdown aborted because a safety interlock changed")


# ----------------------------------------------------- E-STOP and resets
def test_estop_vents_latches_and_reset_controls_clears():
    r = Rig(mode=S.LIVE, fill_s=10).ready()
    r.act(S.ARM)
    r.act(S.FILL_ON)
    r.run(2)
    assert r.act(S.ESTOP)
    assert r.sup.command() == S.SAFE_SET, "E-STOP streams the SAFE set (VEHICLE open)"
    assert r.sup.estop and r.sup.fill_seq_aborted and not r.sup.armed
    r.step(n=3)
    assert r.gate(S.ARM).reason == S.TXT_LATCHED
    assert r.gate(S.SRV_IGN_ON).reason == "SRV+IGN requires FILL to run and turn OFF first"
    assert r.act(S.RESET_CONTROLS)
    r.step(n=3)
    assert not (r.sup.estop or r.sup.fill_seq_aborted or r.sup.fill_paused)
    c = r.sup.command()
    assert not c.veh and not c.fill, "after RESET CONTROLS the vehicle valves close again"
    assert r.logged("All outputs confirmed OFF/SAFE")


def test_estop_works_while_pad_telemetry_is_stale():
    """Deliberate difference from Rust: E-STOP needs only the laptop link."""
    r = Rig(mode=S.TEST, fill_s=10).ready()
    r.act(S.FILL_ON)
    r.pad_alive = False
    r.run(2)
    assert r.gate(S.ESTOP).enabled
    assert r.act(S.ESTOP)
    assert r.sup.command() == S.SAFE_SET


def test_reset_fill_and_clear_faults_rules():
    r = Rig(mode=S.TEST, fill_s=1).ready()
    assert r.gate(S.RESET_FILL).reason == "Nothing to reset"
    assert r.gate(S.CLEAR_FAULTS).reason == "No latched faults"
    r.act(S.FILL_ON)
    assert r.gate(S.RESET_FILL).reason == "Close FILL first"
    r.run(1.5)
    assert r.act(S.RESET_FILL)
    r.step(n=3)
    assert not r.sup.fill_seq_complete


# ------------------------------------------------------------- link loss
def test_link_loss_latch_inhibits_and_self_clears_link_loss_only():
    r = Rig(mode=S.TEST, fill_s=1).ready()
    complete_fill(r)
    r.act(S.SRV_ON)
    r.step(n=3)
    r.pad_alive = False
    # telemetry is stale 1.5 s after the last frame; the latch fires 5 s after
    # the last *fresh* moment (vehicle.rs:803-825 via pad_link.rs:326-329)
    r.run(6.2)
    assert r.sup.srv, "nothing latches before 1.5 + 5 s"
    r.run(0.6)
    assert r.sup.link_loss_safe and r.sup.inhibited and not r.sup.srv
    assert r.sup.command() == S.SAFE_SET
    r.pad_alive = True
    r.step(n=3)
    assert not r.sup.link_loss_safe and r.sup.inhibited
    assert r.gate(S.SRV_ON).reason == ("Vehicle inhibited - press RESET CONTROLS and run "
                                       "a new fill")


def test_link_loss_mid_fill_aborts_the_fill():
    r = Rig(mode=S.TEST, fill_s=30).ready()
    r.act(S.FILL_ON)
    r.run(1)
    r.pad_alive = False
    r.run(7)
    assert r.sup.fill_seq_aborted and not r.sup.fill


# ------------------------------------------------------ mode, confirm, misc
def test_mode_change_only_when_all_safe():
    r = Rig(mode=S.LIVE, fill_s=10).ready()
    r.act(S.FILL_ON)
    assert not r.sup.set_mode(S.TEST)
    assert r.logged("Cannot change operating mode")
    r.act(S.FILL_OFF)                        # paused counts as safe
    assert r.sup.set_mode(S.TEST) and r.sup.mode == S.TEST


def test_unconfirmed_command_reports_a_readback_mismatch():
    r = Rig(mode=S.LIVE, fill_s=10).ready()
    r.pad_follows = False                    # pad ignores the stream
    r.act(S.FILL_ON)
    assert r.gate(S.FILL_OFF).reason == S.TXT_PENDING
    r.run(1.6)
    assert r.sup.pending is None
    assert r.logged("PAD READBACK MISMATCH")


def test_purge_needs_session_enable_and_closes_on_latch():
    r = Rig(mode=S.LIVE).ready()
    assert r.gate(S.PURGE_OPEN).reason == "Purge valve not enabled for this session"
    r.act(S.PURGE_ENABLE)
    assert r.act(S.PURGE_OPEN) and r.sup.command().prg
    r.act(S.ESTOP)
    r.step()
    assert not r.sup.purge_open and not r.sup.command().prg


def test_settings_roundtrip_and_clamping(tmp=None):
    import tempfile
    d = tempfile.mkdtemp()
    p = os.path.join(d, "s.json")
    S.Settings(op_mode=S.TEST, fill_seconds=5000, countdown_seconds=0).sanitize().save(p)
    s = S.Settings.load(p)
    assert (s.op_mode, s.fill_seconds, s.countdown_seconds) == (S.TEST, 600, 1)
    with open(p, "w") as fh:
        fh.write("not json")
    assert S.Settings.load(p) == S.Settings()


if __name__ == "__main__":
    tests = [v for k, v in sorted(globals().items()) if k.startswith("test_")]
    for fn in tests:
        fn()
        print(f"  ok  {fn.__name__}")
    print(f"ALL PASS ({len(tests)} tests)")
