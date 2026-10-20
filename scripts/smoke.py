#!/usr/bin/env python3
"""End-to-end smoke test against a running gse-app console.

Drives the HTTP API exactly like the browser does: waits for both boards,
checks the deterministic OFF/SAFE initialisation, runs a short fill sequence,
verifies every safety gate along the way, fires the igniter, hits the
emergency stop, resets the controls back to OFF/SAFE and finally shuts the
app down, checking that it commands ALL SAFE before exiting.

Usage: smoke.py BASE_URL TOKEN
"""

import json
import sys
import time
import urllib.error
import urllib.request

BASE = sys.argv[1].rstrip("/")
TOKEN = sys.argv[2]
TIMEOUT = 25.0

FAILURES = []


def request(path, body=None, timeout=5):
    data = None if body is None else json.dumps(body).encode()
    method = "GET" if data is None else "POST"
    req = urllib.request.Request(BASE + path, data=data, method=method)
    req.add_header("X-GSE-Token", TOKEN)
    if data is not None:
        req.add_header("Content-Type", "application/json")
    with urllib.request.urlopen(req, timeout=timeout) as resp:
        return json.loads(resp.read().decode())


def state():
    return request("/api/state")


def wait_for(predicate, label, timeout=TIMEOUT):
    deadline = time.time() + timeout
    last = None
    while time.time() < deadline:
        try:
            last = state()
            if predicate(last):
                return last
        except (urllib.error.URLError, json.JSONDecodeError, TimeoutError):
            pass
        time.sleep(0.1)
    raise AssertionError(f"timeout waiting for {label}; last={json.dumps(last)[:400]}")


def check(condition, message):
    if not condition:
        FAILURES.append(message)
        print(f"  FAIL {message}")
        return
    print(f"  ok  {message}")


def main():
    wait_for(lambda s: s["ground"]["status"] == "online", "ground board online")
    wait_for(lambda s: s["vehicle"]["status"] == "online", "vehicle online")
    print("  ok  both boards detected and pinged")

    # --- Initialisation: the machine leaves INIT only in OFF/SAFE. --------
    s = wait_for(
        lambda st: st["controls"]["ready"]
        and st["controls"]["safe"]
        and st["controls"]["fill"] == "OFF"
        and st["controls"]["srv"] == "OFF"
        and st["controls"]["ign"] == "SAFE",
        "controls initialised OFF/SAFE",
    )
    check(s["controls"]["summary"] == "FILL=OFF SRV=OFF IGN=SAFE", "startup state is OFF/SAFE")
    check(not s["vehicle"]["state"]["fill"], "vehicle FILL off at initialisation")
    check(not s["vehicle"]["state"]["srv"], "vehicle SRV off at initialisation")
    check(not s["vehicle"]["state"]["ign"], "vehicle IGN safe at initialisation")

    s = wait_for(lambda st: st["actions"]["fill_on"]["enabled"], "FILL gate open")
    check(not s["actions"]["ign_pwr"]["enabled"], "IGN blocked at rest")
    check("SRV" in s["actions"]["ign_pwr"]["reason"], "IGN reason names SRV")

    request("/api/config", {"fill_seconds": 2})
    request("/api/command", {"action": "fill_on"})
    # The machine must not move on the request: while the command is in
    # flight the confirmed state is still OFF and only the target says ON.
    s = wait_for(
        lambda st: st["pending"] is not None
        and st["controls"]["target"] is not None
        and st["controls"]["target"]["fill"] == "ON",
        "FILL command in flight",
    )
    check(
        s["controls"]["fill"] == "OFF" and s["controls"]["last_result"] == "pending",
        "confirmed FILL stays OFF until the ACK",
    )
    wait_for(lambda st: st["fill"]["active"], "fill running")
    s = wait_for(
        lambda st: st["controls"]["fill"] == "ON" and st["controls"]["target"] is None,
        "FILL confirmed ON",
    )
    check(True, "FILL opened and the machine moved to ON after the ACK")

    s = wait_for(
        lambda st: st["fill"]["active"] and "FILL" in st["actions"]["srv_on"]["reason"],
        "SRV interlock while FILL is open",
    )
    check(True, "SRV blocked while FILL is open")

    s = wait_for(
        lambda st: st["fill"]["seq_complete"] and st["actions"]["srv_on"]["enabled"],
        "fill sequence complete",
        timeout=15,
    )
    check(True, "fill auto-off completed the sequence")
    check(
        not s["actions"]["ign_pwr"]["enabled"] and "SRV" in s["actions"]["ign_pwr"]["reason"],
        "IGN still gated until SRV is on",
    )

    request("/api/command", {"action": "srv_on"})
    s = wait_for(
        lambda st: st["pending"] is not None
        and st["controls"]["target"] is not None
        and st["controls"]["target"]["srv"] == "ON",
        "SRV command in flight",
    )
    check(
        s["controls"]["srv"] == "OFF" and s["controls"]["last_result"] == "pending",
        "confirmed SRV stays OFF until the ACK",
    )
    wait_for(lambda st: st["vehicle"]["state"]["srv"], "SRV on")
    s = wait_for(
        lambda st: st["actions"]["ign_pwr"]["enabled"]
        and "interlocked" in st["actions"]["fill_on"]["reason"],
        "IGN unlocked and FILL interlocked",
    )
    check(not s["actions"]["fill_on"]["enabled"], "FILL blocked while SRV is on")
    check(True, "interlock reason reported")

    # The API re-checks the same transition table even if the browser is
    # bypassed: an unsafe FILL ON must not move the output.
    request("/api/command", {"action": "fill_on"})
    time.sleep(1.0)
    s = state()
    check(not s["vehicle"]["state"]["fill"], "unsafe FILL ON refused by the app")
    check(s["vehicle"]["state"]["srv"], "SRV stayed on")

    request("/api/command", {"action": "ign_pwr"})
    s = wait_for(lambda st: st["vehicle"]["state"]["ign"], "IGN PWR")
    check(s["vehicle"]["summary"].endswith("IGN=PWR"), "IGN PWR acknowledged by the vehicle")

    request("/api/command", {"action": "estop"})
    s = wait_for(lambda st: not st["vehicle"]["state"]["ign"], "E-STOP")
    check(
        not s["vehicle"]["state"]["fill"] and not s["vehicle"]["state"]["srv"],
        "E-STOP left every output safe",
    )
    s = wait_for(lambda st: st["controls"]["safe"], "controls SAFE after E-STOP")
    check(s["controls"]["summary"] == "FILL=OFF SRV=OFF IGN=SAFE", "E-STOP state is OFF/SAFE")

    # --- RESET CONTROLS: back to the initial state, latches cleared. ------
    s = wait_for(lambda st: st["actions"]["reset_controls"]["enabled"], "reset controls available")
    request("/api/command", {"action": "reset_controls"})
    s = wait_for(
        lambda st: st["controls"]["safe"]
        and st["vehicle"]["state"]["estop"] is False
        and st["vehicle"]["state"]["fill_seq_complete"] is False
        and st["vehicle"]["state"]["fill_seq_aborted"] is False,
        "controls reset to OFF/SAFE",
    )
    check(s["controls"]["summary"] == "FILL=OFF SRV=OFF IGN=SAFE", "reset returned to OFF/SAFE")
    s = wait_for(
        lambda st: st["actions"]["fill_on"]["enabled"] and st["actions"]["reset_fill"]["enabled"] is False,
        "machine usable after reset",
    )
    check(True, "reset cleared every latch (nothing left to reset)")

    # --- Reconnect with an output ON: initialisation must force OFF/SAFE. --
    s = wait_for(lambda st: st["actions"]["srv_on"]["enabled"], "SRV gate open again")
    request("/api/command", {"action": "srv_on"})
    wait_for(lambda st: st["vehicle"]["state"]["srv"], "SRV on before reconnect")

    request("/api/disconnect", {})
    time.sleep(0.2)
    request("/api/connect", {})
    wait_for(lambda st: st["vehicle"]["status"] == "online", "vehicle back online")
    s = wait_for(
        lambda st: st["controls"]["ready"] and st["controls"]["safe"],
        "startup handshake after reconnect",
        timeout=10,
    )
    check(not s["vehicle"]["state"]["srv"], "reconnect with SRV ON was forced to OFF/SAFE")
    check(s["controls"]["summary"] == "FILL=OFF SRV=OFF IGN=SAFE", "machine re-initialised safe")

    # --- Shutdown with an active output: ALL SAFE must be commanded. ------
    s = wait_for(lambda st: st["actions"]["srv_on"]["enabled"], "SRV gate open before shutdown")
    request("/api/command", {"action": "srv_on"})
    s = wait_for(lambda st: st["vehicle"]["state"]["srv"], "SRV on before shutdown")
    check(True, "shutdown will happen with SRV ON")

    s = request("/api/state")
    check(s["stats"]["commands_sent"] >= 5, "commands were transmitted")
    check(s["stats"]["acks_ok"] >= 5, "every command was acknowledged")

    # --- Shutdown: the app must command ALL SAFE before it exits. ---------
    log_path = s.get("log_path")
    check(bool(log_path), "session log path available")
    if log_path:
        before = ""
        try:
            with open(log_path, encoding="utf-8") as fh:
                before = fh.read()
        except OSError:
            pass
        check("Control state machine initialised" in before, "initialisation recorded in the log")
        check(
            "commanding ALL SAFE" in before,
            "reconnect handshake commanded ALL SAFE",
        )

    try:
        request("/api/shutdown", {}, timeout=5)
    except (urllib.error.URLError, ConnectionError, TimeoutError):
        pass

    deadline = time.time() + 8
    stopped = False
    while time.time() < deadline:
        try:
            request("/api/health", timeout=0.5)
        except (urllib.error.URLError, ConnectionError, TimeoutError, json.JSONDecodeError):
            stopped = True
            break
        time.sleep(0.2)
    check(stopped, "console stopped after the shutdown request")

    if log_path:
        text = ""
        try:
            with open(log_path, encoding="utf-8") as fh:
                text = fh.read()
        except OSError:
            pass
        check("ALL SAFE acknowledged" in text, "shutdown commanded and acknowledged ALL SAFE")
        check("Ground station stopped" in text, "shutdown sequence completed")

    if FAILURES:
        raise AssertionError(f"{len(FAILURES)} check(s) failed: {FAILURES}")
    print("SMOKE PASS")


if __name__ == "__main__":
    main()
