"""
test_sim_loop.py -- end-to-end logic test with no hardware and no GUI.

Drives controller.Controller against sim.PadModel through link.SimLink and
checks the three behaviours that matter on the range:

  1. a live link keeps the pad out of comms-loss;
  2. killing the command stream makes the pad safe within the watchdog;
  3. the igniter can only energise while armed, and is cut by the on-time limit.
"""
from __future__ import annotations

import os
import sys
import time

HERE = os.path.dirname(os.path.abspath(__file__))
sys.path.insert(0, os.path.join(HERE, ".."))

from gse_control import protocol as P          # noqa: E402
from gse_control.controller import Controller  # noqa: E402


def pump(ctl, seconds):
    t0 = time.monotonic()
    while time.monotonic() - t0 < seconds:
        ctl.tick()
        time.sleep(0.01)


def test_link_stays_up():
    ctl = Controller(port="SIM")
    ctl.connect()
    pump(ctl, 0.4)
    st = ctl.pad
    assert st is not None, "no telemetry from the simulated pad"
    assert st.link_ok and not (st.flags & P.ST_COMMS_LOSS)
    assert st.age_ms < st.wdg_ds * 100
    ctl.disconnect()


def test_fill_and_srv_controls_follow_both_states():
    ctl = Controller(port="SIM")
    ctl.connect()
    pump(ctl, 0.3)
    ctl.set_fill(True)
    ctl.set_veh(True)
    pump(ctl, 0.2)
    assert ctl.pad.out_fill == 1 and ctl.pad.out_veh == 1
    ctl.set_fill(False)
    ctl.set_veh(False)
    pump(ctl, 0.2)
    assert ctl.pad.out_fill == 0 and ctl.pad.out_veh == 0
    ctl.disconnect()


def test_watchdog_safes_on_silence():
    ctl = Controller(port="SIM")
    ctl.connect()
    pump(ctl, 0.3)
    ctl.set_fill(True)                       # open the fill valve
    pump(ctl, 0.3)
    assert ctl.pad.out_fill == 1
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


def test_igniter_requires_arm_and_is_time_limited():
    ctl = Controller(port="SIM")
    ctl.connect()
    pump(ctl, 0.3)

    ctl.press_ign()                          # ARM is off -> must not fire
    pump(ctl, 0.2)
    assert not ctl.pad.ign_on

    ctl.set_arm(True)
    pump(ctl, 0.2)
    assert ctl.pad.armed
    ctl.press_ign()
    pump(ctl, 0.1)
    assert ctl.pad.ign_on, "armed igniter did not energise"

    # keep holding: the pad's hard limit must cut it anyway
    pump(ctl, ctl.cfg.ign_max_ds / 10 + 0.4)
    assert not ctl.pad.ign_on, "igniter on-time limiter failed"

    ctl.release_ign()
    ctl.set_arm(False)
    pump(ctl, 0.2)
    assert not ctl.pad.armed
    ctl.disconnect()


def test_purge_blocked_until_enabled():
    ctl = Controller(port="SIM")
    ctl.connect()
    pump(ctl, 0.3)
    ctl.set_prg(True)
    pump(ctl, 0.2)
    assert ctl.pad.out_prg == 0, "purge opened without being enabled"
    ctl.ui.purge_enabled = True
    ctl.set_prg(True)
    pump(ctl, 0.2)
    assert ctl.pad.out_prg == 1
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

    # the wording is "PAD LINK OK / PAD LINK FAULT" now that the pad can be on
    # either transport (the LoRa module or Bluetooth); the old text said RADIO
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
