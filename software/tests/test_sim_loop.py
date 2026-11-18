"""
test_sim_loop.py -- end-to-end logic test with no hardware and no GUI.

Drives controller.Controller (and through it the GSE control set in
supervisor.py) against sim.PadModel through link.SimLink, and checks what the
*pad* reports, on the range-relevant paths:

  1. a live link keeps the pad out of comms-loss, and INIT completes;
  2. killing the command stream makes the pad safe within its watchdog;
  3. LIVE: fill -> VEH+IGN countdown -> HOLD -> manual trigger fires once;
  4. E-STOP vents the vehicle and latches until RESET CONTROLS;
  5. purge needs the per-session enable;
  6. the banner names the failure.
"""
from __future__ import annotations

import os
import sys
import time

HERE = os.path.dirname(os.path.abspath(__file__))
sys.path.insert(0, os.path.join(HERE, ".."))

from gse_control import protocol as P          # noqa: E402
from gse_control import supervisor as S        # noqa: E402
from gse_control.controller import Controller  # noqa: E402


def pump(ctl, seconds):
    t0 = time.monotonic()
    while time.monotonic() - t0 < seconds:
        ctl.tick()
        time.sleep(0.01)


def until(ctl, pred, timeout, what):
    t0 = time.monotonic()
    while time.monotonic() - t0 < timeout:
        ctl.tick()
        if pred():
            return
        time.sleep(0.01)
    raise AssertionError(f"timed out waiting for: {what}")


def connected(mode=S.LIVE, fill_s=1, cd_s=1):
    ctl = Controller(port="SIM")
    ctl.sup.settings.op_mode = mode
    ctl.sup.settings.fill_seconds = fill_s
    ctl.sup.settings.countdown_seconds = cd_s
    ctl.connect()
    until(ctl, lambda: ctl.sup.controls_ready, 2.0, "INIT handshake")
    return ctl


def run_fill(ctl):
    assert ctl.act(S.FILL_ON)
    until(ctl, lambda: ctl.pad.out_fill == 1, 1.0, "pad FILL open")
    until(ctl, lambda: ctl.sup.fill_seq_complete, ctl.sup.settings.fill_seconds + 1.5,
          "fill complete")
    until(ctl, lambda: ctl.pad.out_fill == 0 and ctl.sup.pending is None, 1.0,
          "pad FILL closed")


def test_link_stays_up_and_init_completes():
    ctl = connected()
    st = ctl.pad
    assert st is not None, "no telemetry from the simulated pad"
    assert st.link_ok and not (st.flags & P.ST_COMMS_LOSS)
    assert st.out_veh == 0, "in normal operation the vehicle valves are CLOSED"
    assert ctl.sup.controls_ready
    ctl.disconnect()


def test_watchdog_safes_on_silence():
    ctl = connected(fill_s=30)
    ctl.act(S.FILL_ON)
    until(ctl, lambda: ctl.pad.out_fill == 1, 1.0, "pad FILL open")
    # simulate a dead GUI: stop ticking, keep polling the link for telemetry
    t0 = time.monotonic()
    while time.monotonic() - t0 < 1.6:
        ctl.link.poll()
        time.sleep(0.02)
    st = ctl.pad
    assert st.flags & P.ST_COMMS_LOSS, "pad did not declare comms loss"
    assert st.out_fill == 0 and st.out_veh == 1 and st.out_prg == 0
    assert not st.ign_on
    ctl.disconnect()


def test_live_sequence_fires_only_after_the_manual_trigger():
    ctl = connected(mode=S.LIVE, fill_s=1, cd_s=1)
    assert not ctl.gates()[S.SRV_IGN_ON].enabled, "locked until the fill has run"
    run_fill(ctl)
    assert ctl.act(S.ARM)
    until(ctl, lambda: ctl.pad.armed, 1.0, "pad sees the arm code")
    assert ctl.act(S.SRV_IGN_ON)                         # starts the countdown
    until(ctl, lambda: ctl.sup.cd_holding, 2.0, "T-0 HOLD")
    pump(ctl, 0.5)
    assert not ctl.pad.ign_on and ctl.pad.out_veh == 0, "HOLD must not fire"
    assert ctl.act(S.SRV_IGN_ON)                         # the manual trigger
    until(ctl, lambda: ctl.pad.ign_on, 1.0, "igniter energised")
    assert ctl.pad.out_veh == 1, "VEH+IGN opens the vehicle valves"
    until(ctl, lambda: not ctl.pad.ign_on, 1.5, "on-time limit cuts the igniter")
    until(ctl, lambda: ctl.sup.pending is None, 1.0, "trigger confirmed")
    assert ctl.act(S.SRV_IGN_OFF)
    until(ctl, lambda: ctl.pad.out_veh == 0, 1.0, "vehicle valves closed")
    ctl.disconnect()


def test_estop_vents_and_latches_until_reset():
    ctl = connected(mode=S.TEST, fill_s=30)
    ctl.act(S.FILL_ON)
    until(ctl, lambda: ctl.pad.out_fill == 1, 1.0, "pad FILL open")
    assert ctl.act(S.ESTOP)
    until(ctl, lambda: ctl.pad.out_fill == 0 and ctl.pad.out_veh == 1, 1.0,
          "SAFE set: FILL closed, VEHICLE vented")
    assert ctl.verdict()[1].startswith("SAFE SET LATCHED")
    assert not ctl.gates()[S.ARM].enabled
    until(ctl, lambda: ctl.sup.pending is None, 1.0, "E-STOP confirmed")
    assert ctl.act(S.RESET_CONTROLS)
    until(ctl, lambda: ctl.pad.out_veh == 0, 1.0, "vehicle valves closed after reset")
    assert not ctl.sup.estop
    ctl.disconnect()


def test_purge_needs_the_session_enable():
    ctl = connected()
    assert not ctl.act(S.PURGE_OPEN)
    assert ctl.act(S.PURGE_ENABLE)
    until(ctl, ctl.cfg_synced, 2.5, "pad reports allow_purge")
    assert ctl.act(S.PURGE_OPEN)
    until(ctl, lambda: ctl.pad.out_prg == 1, 1.0, "pad PURGE open")
    assert ctl.act(S.PURGE_DISABLE)
    until(ctl, lambda: ctl.pad.out_prg == 0, 1.0, "pad PURGE closed")
    ctl.disconnect()


def test_verdict_names_the_failure():
    """The banner must distinguish 'pad radio broken' from 'link dead'."""
    from gse_control.link import Link

    class FakeLink(Link):
        name = "fake"
        def open(self): pass
        def close(self): pass
        def _write(self, data): pass

    ctl = Controller(port="SIM")
    ctl.link = FakeLink(None)
    ctl.link.state = None
    ctl.link.node_log = []
    assert ctl.verdict() == ("red", "NO PAD TELEMETRY")

    ctl.link.node_log = ["PAD LINK OK  (waiting for the TX node)"]
    colour, text = ctl.verdict()
    assert colour == "red" and "NO DATA" in text, text

    ctl.link.node_log.append("PAD LINK FAULT: CHANNEL did not stick")
    colour, text = ctl.verdict()
    assert "LINK FAULT" in text and "did not stick" in text, text
    ctl.link.node_log = ["TX esp32s3 fw 1.0 RADIO FAULT"]
    colour, text = ctl.verdict()
    assert colour == "red" and "TX LINK FAULT" in text, text


if __name__ == "__main__":
    tests = [v for k, v in sorted(globals().items()) if k.startswith("test_")]
    for fn in tests:
        fn()
        print(f"  ok  {fn.__name__}")
    print(f"ALL PASS ({len(tests)} tests)")
