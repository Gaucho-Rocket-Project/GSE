#!/usr/bin/env python3
"""End-to-end smoke test for the ESP32 + ESP-NOW pad (GSE-simple control set).

Runs against a gse-app console connected to `gse-sim --radio esp-now`:

  * pad telemetry fresh, controls initialised, idle readback (VEH closed),
    pad configuration pushed and echoed in sync, pad HELLO identified,
  * FILL ON is only acknowledged once the pad readback shows the valve open;
    FILL OFF pauses the app-side fill timer (app-only extra),
  * LIVE VEH+IGN stays locked without ARM; with ARM the countdown holds at
    T-0, the explicit trigger opens VEH and fires the igniter once, and the
    pad cuts it after the on-time limit (SPENT); VEH+IGN OFF releases it,
  * PURGE is refused until enabled for the session, then opens on the pad,
  * E-STOP puts the pad in the GSE-simple SAFE set (FILL/PURGE closed,
    VEHICLE open, IGN off), disarms and blocks ARM until RESET CONTROLS,
  * shutdown leaves the console cleanly.

Usage: smoke_pad.py BASE_URL TOKEN
"""

import json
import sys
import time
import urllib.error
import urllib.request

BASE = sys.argv[1].rstrip("/")
TOKEN = sys.argv[2]
TIMEOUT = 15.0

FAILURES = []


def request(path, body=None, timeout=5):
    data = None if body is None else json.dumps(body).encode()
    req = urllib.request.Request(BASE + path, data=data, method="GET" if data is None else "POST")
    req.add_header("X-GSE-Token", TOKEN)
    if data is not None:
        req.add_header("Content-Type", "application/json")
    with urllib.request.urlopen(req, timeout=timeout) as resp:
        return json.loads(resp.read().decode())


def state():
    return request("/api/state")


def command(action):
    request("/api/command", {"action": action})


def wait_for(predicate, label, timeout=TIMEOUT):
    deadline = time.time() + timeout
    last = None
    while time.time() < deadline:
        try:
            last = state()
            if predicate(last):
                return last
        except (urllib.error.URLError, json.JSONDecodeError, TimeoutError, KeyError, TypeError):
            pass
        time.sleep(0.06)
    raise AssertionError(f"timeout waiting for {label}; last={json.dumps(last)[:600]}")


def check(condition, message):
    if not condition:
        FAILURES.append(message)
        print(f"  FAIL {message}")
        return
    print(f"  ok  {message}")


def rb(s):
    return s["pad"]["readback"]


def main():
    request("/api/config", {"op_mode": "live", "countdown_seconds": 1})
    s = wait_for(
        lambda st: st["pad"]["active"]
        and st["pad"]["fresh"]
        and st["vehicle"]["status"] == "online"
        and st["controls"]["ready"]
        and st["pad"]["cfg_synced"]
        and st["vehicle"]["fw"] is not None,
        "pad fresh, controls ready, config in sync, HELLO seen",
    )
    check(s["pad"]["verdict"]["text"] == "LINK OK - NOT ARMED", "verdict LINK OK - NOT ARMED")
    check(not rb(s)["fill"] and not rb(s)["veh"] and not rb(s)["prg"] and not rb(s)["ign"],
          "idle readback: FILL, VEH, PURGE closed and IGN off")

    command("fill_on")
    s = wait_for(lambda st: st["controls"]["fill"] == "ON" and st["pending"] is None, "FILL ON acknowledged")
    check(rb(s)["fill"], "FILL ON acknowledged only with the pad reporting the valve open")
    check(not s["actions"]["srv_ign_on"]["enabled"], "VEH+IGN locked while FILL is open")

    command("fill_off")
    s = wait_for(lambda st: st["fill"]["paused"] and st["pending"] is None, "FILL paused")
    check(not rb(s)["fill"], "FILL OFF closes the valve and pauses the app-side fill timer")
    gate = s["actions"]["srv_ign_on"]
    check(not gate["enabled"] and "ARM" in gate["reason"],
          f"VEH+IGN requires ARM (gate: {gate['reason'] or 'enabled'})")

    command("arm")
    s = wait_for(lambda st: rb(st)["armed"], "pad armed")
    check(s["pad"]["verdict"]["text"] == "LINK OK - ARMED", "ARM -> pad reports the arm interlock satisfied")

    command("srv_ign_on")
    s = wait_for(lambda st: st["countdown"]["holding"], "countdown HOLD")
    check(not rb(s)["veh"] and not rb(s)["ign"], "countdown holds at T-0 with VEH closed and IGN off")
    command("srv_ign_on")
    s = wait_for(lambda st: rb(st)["ign_spent"] and rb(st)["veh"], "igniter fired and spent")
    check(not rb(s)["ign"], "trigger opens VEH, fires the igniter once and the pad cuts it (SPENT)")
    s = wait_for(lambda st: st["pending"] is None and st["controls"]["srv"] == "ON", "VEH+IGN acknowledged")
    check(s["pad"]["mismatch"] is None, "no readback mismatch for a spent one-shot pulse")

    command("srv_ign_off")
    s = wait_for(lambda st: st["pending"] is None and st["controls"]["srv"] == "OFF"
                 and not rb(st)["veh"] and not rb(st)["ign_spent"], "VEH+IGN OFF")
    check(True, "VEH+IGN OFF closes VEH and releases the igniter")

    s = state()
    check(not s["actions"]["purge_open"]["enabled"], "PURGE refused until enabled for the session")
    command("purge_enable")
    command("purge_open")
    s = wait_for(lambda st: rb(st)["prg"], "purge open on the pad")
    check(rb(s)["allow_purge"], "PURGE enable reaches the pad config and the valve opens")

    command("estop")
    s = wait_for(lambda st: st["pad"]["latched_safe"] and rb(st)["veh"] and not rb(st)["prg"]
                 and st["pending"] is None, "E-STOP safe set")
    check(not rb(s)["fill"] and not rb(s)["ign"],
          "E-STOP -> pad SAFE set (FILL/PURGE closed, VEHICLE open, IGN off)")
    check(not s["pad"]["armed"] and not s["actions"]["arm"]["enabled"], "E-STOP disarms and blocks ARM")

    command("reset_controls")
    s = wait_for(lambda st: not st["pad"]["latched_safe"] and not rb(st)["veh"]
                 and st["pending"] is None and st["controls"]["ready"], "reset")
    check(s["actions"]["arm"]["enabled"], "RESET CONTROLS clears the latch and closes VEH")

    try:
        request("/api/shutdown", {})
    except (urllib.error.URLError, ConnectionError):
        pass
    deadline = time.time() + 8
    stopped = False
    while time.time() < deadline:
        try:
            request("/api/health")
            time.sleep(0.2)
        except (urllib.error.URLError, ConnectionError, TimeoutError):
            stopped = True
            break
    check(stopped, "console stopped after the shutdown request")


if __name__ == "__main__":
    try:
        main()
    except AssertionError as e:
        FAILURES.append(str(e))
        print(f"  FAIL {e}")
    if FAILURES:
        print(f"PAD SMOKE FAIL ({len(FAILURES)} failures)")
        sys.exit(1)
    print("PAD SMOKE PASS")
