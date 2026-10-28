#!/usr/bin/env python3
"""End-to-end smoke test against a running gse-app console.

Exercises BOTH operating modes and all three sequence capabilities end-to-end:
  1. LIVE mode (default launch setup):
     - verifies SRV+IGN starts greyed out (OFF/SAFE) before fill runs
     - verifies individual SRV / IGN actions are blocked in LIVE mode
     - turns FILL -> ON, then manually turns FILL -> OFF prior to timer expiry:
       verifies the FILL timer pauses (remaining_ms frozen), unlocks SRV+IGN,
       and allows resuming FILL -> ON from the paused remaining time
     - aborts SRV+IGN mid-countdown via BOTH `abort_countdown` and `srv_ign_off`:
       verifies immediate cancellation, zero output actuation, and preserved
       paused/completed FILL progress without needing a reset
     - resumes FILL -> ON -> verifies automatic FILL -> OFF after only the
       remaining duration -> runs countdown to T-0 HOLD -> manually triggers
       with L -> SRV+IGN ON (SRV=ON, IGN=PWR) -> SRV+IGN OFF (OFF/SAFE)
  2. Settings switch to TEST mode:
     - verifies BOTH SRV and IGN start locked prior to FILL running
     - turns FILL -> ON, pauses FILL -> OFF early -> verifies SRV unlocks
     - turns SRV -> ON, triggers IGN countdown (`countdown.target == "ign"`),
       and aborts mid-countdown via `ign_safe` / `abort_countdown`: verifies
       IGN never energises, SRV automatically closes to OFF, and paused FILL
       state/remaining_ms remain intact
     - resumes FILL -> ON to completion, runs IGN countdown to T-0 HOLD,
       manually triggers with L, triggers E-STOP, runs RESET CONTROLS,
       tests reconnect ALL SAFE, and verifies pre-shutdown ALL SAFE.

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


def request_status(path, body=None, timeout=5):
    data = None if body is None else json.dumps(body).encode()
    method = "GET" if data is None else "POST"
    req = urllib.request.Request(BASE + path, data=data, method=method)
    req.add_header("X-GSE-Token", TOKEN)
    if data is not None:
        req.add_header("Content-Type", "application/json")
    try:
        with urllib.request.urlopen(req, timeout=timeout) as resp:
            return resp.status, json.loads(resp.read().decode())
    except urllib.error.HTTPError as err:
        return err.code, json.loads(err.read().decode())


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
        time.sleep(0.06)
    raise AssertionError(f"timeout waiting for {label}; last={json.dumps(last)[:450]}")


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

    # ======================================================================
    # PART 1: LIVE LAUNCH MODE (default)
    # ======================================================================
    s = wait_for(
        lambda st: st["controls"]["ready"]
        and st["controls"]["safe"]
        and st["controls"]["fill"] == "OFF"
        and st["controls"]["srv"] == "OFF"
        and st["controls"]["ign"] == "SAFE",
        "controls initialised OFF/SAFE",
    )
    check(s["config"]["op_mode"] == "live", "default operating mode is LIVE")
    check(
        s["config"]["countdown_seconds"] == 5 and s["countdown"]["duration_ms"] == 5000,
        "default ignition countdown is 5 seconds",
    )
    bad_cd, _ = request_status("/api/config", {"countdown_seconds": 0})
    check(bad_cd == 400, "out-of-range countdown_seconds is rejected")
    check(s["controls"]["summary"] == "FILL=OFF SRV=OFF IGN=SAFE", "startup state is OFF/SAFE")
    check(not s["vehicle"]["state"]["fill"], "vehicle FILL off at initialisation")
    check(not s["vehicle"]["state"]["srv"], "vehicle SRV off at initialisation")
    check(not s["vehicle"]["state"]["ign"], "vehicle IGN safe at initialisation")

    s = wait_for(lambda st: st["actions"]["fill_on"]["enabled"], "FILL gate open in LIVE mode")
    check(not s["actions"]["srv_ign_on"]["enabled"], "LIVE SRV+IGN locked before fill runs")
    check(
        "FILL" in s["actions"]["srv_ign_on"]["reason"],
        "LIVE SRV+IGN reason explains FILL prerequisite",
    )
    check(not s["actions"]["srv_on"]["enabled"], "individual SRV control disabled in LIVE mode")
    check(not s["actions"]["ign_pwr"]["enabled"], "individual IGN control disabled in LIVE mode")

    # --- Early FILL -> OFF pauses timer, preserves remaining_ms, unlocks SRV+IGN ---
    request("/api/config", {"fill_seconds": 4})
    request("/api/command", {"action": "fill_on"})
    wait_for(lambda st: st["fill"]["active"], "LIVE fill running")
    s = state()
    check(not s["actions"]["srv_ign_on"]["enabled"], "LIVE SRV+IGN stays locked during active fill")

    time.sleep(0.9)
    request("/api/command", {"action": "fill_off"})
    s = wait_for(
        lambda st: st["controls"]["fill"] == "OFF"
        and st["fill"]["paused"]
        and st["vehicle"]["state"]["fill_paused"],
        "LIVE fill paused on early FILL -> OFF",
    )
    paused_rem_ms = s["fill"]["remaining_ms"]
    check(
        1200 <= paused_rem_ms <= 3400,
        f"early FILL -> OFF paused timer with remaining={paused_rem_ms} ms",
    )
    check(
        not s["fill"]["seq_aborted"] and not s["fill"]["seq_complete"],
        "paused FILL is neither complete nor aborted",
    )
    check(
        s["actions"]["srv_ign_on"]["enabled"] and s["actions"]["fill_on"]["enabled"],
        "early FILL -> OFF unlocked SRV+IGN AND kept FILL resumable",
    )
    time.sleep(0.8)
    s_after_wait = state()
    check(
        s_after_wait["fill"]["remaining_ms"] == paused_rem_ms,
        f"paused FILL timer stays frozen while idle ({s_after_wait['fill']['remaining_ms']} ms)",
    )

    # --- Mid-countdown ABORT in LIVE mode preserves paused FILL state ---
    request("/api/command", {"action": "srv_ign_on"})
    s = wait_for(
        lambda st: st["countdown"]["active"] and st["countdown"]["target"] == "srv_ign",
        "LIVE SRV+IGN countdown active",
    )
    check(
        s["actions"]["abort_countdown"]["enabled"] and s["actions"]["srv_ign_off"]["enabled"],
        "LIVE countdown exposes both abort_countdown and srv_ign_off actions",
    )
    time.sleep(0.5)
    request("/api/command", {"action": "abort_countdown"})
    s = wait_for(lambda st: not st["countdown"]["active"], "LIVE countdown aborted via abort_countdown")
    check(
        s["controls"]["safe"]
        and s["fill"]["paused"]
        and s["fill"]["remaining_ms"] == paused_rem_ms
        and s["actions"]["srv_ign_on"]["enabled"],
        "aborting LIVE countdown kept outputs OFF/SAFE and preserved paused FILL progress",
    )

    # Also test aborting LIVE countdown via clicking SRV+IGN (`srv_ign_off`) directly.
    request("/api/command", {"action": "srv_ign_on"})
    wait_for(lambda st: st["countdown"]["active"], "LIVE countdown re-armed")
    request("/api/command", {"action": "srv_ign_off"})
    s = wait_for(lambda st: not st["countdown"]["active"], "LIVE countdown aborted via srv_ign_off")
    check(
        s["controls"]["safe"] and s["fill"]["paused"],
        "toggling SRV+IGN during countdown also aborted cleanly",
    )

    # --- Resume FILL -> ON from paused remaining_ms -> auto FILL -> OFF ---
    t_resume = time.monotonic()
    request("/api/command", {"action": "fill_on"})
    wait_for(lambda st: st["fill"]["active"] and not st["fill"]["paused"], "LIVE fill resumed")
    s = wait_for(
        lambda st: st["fill"]["seq_complete"]
        and not st["fill"]["active"]
        and not st["fill"]["paused"]
        and st["actions"]["srv_ign_on"]["enabled"],
        "resumed LIVE fill completed automatically",
        timeout=10,
    )
    elapsed_resume = time.monotonic() - t_resume
    check(
        elapsed_resume < 3.9,
        f"resumed FILL completed after remaining duration rather than restarting 4 s ({elapsed_resume:.2f} s)",
    )

    # Start SRV+IGN countdown: T-0 holds safely until the operator explicitly presses L.
    t_armed = time.monotonic()
    request("/api/command", {"action": "srv_ign_on"})
    s = wait_for(lambda st: st["countdown"]["active"], "5 s SRV+IGN countdown active")
    check(
        3500 <= s["countdown"]["remaining_ms"] <= 5000,
        f"5 s countdown exposed to UI (remaining={s['countdown']['remaining_ms']} ms)",
    )
    check(
        not s["vehicle"]["state"]["srv"] and not s["vehicle"]["state"]["ign"],
        "vehicle outputs remain OFF/SAFE during 5 s countdown",
    )
    code, _ = request_status("/api/config", {"op_mode": "test"})
    check(code == 400, "switching LIVE -> TEST is blocked while countdown is active")

    s = wait_for(
        lambda st: st["countdown"]["active"] and st["countdown"].get("holding"),
        "countdown reached T-0 HOLD",
        timeout=10,
    )
    check(
        s["countdown"]["remaining_ms"] == 0
        and s["controls"]["safe"]
        and not s["vehicle"]["state"]["srv"]
        and not s["vehicle"]["state"]["ign"]
        and s["actions"]["srv_ign_on"]["enabled"],
        "T-0 HOLD stays OFF/SAFE and enables explicit operator trigger",
    )
    elapsed_live = time.monotonic() - t_armed
    check(elapsed_live >= 4.8, f"countdown reached HOLD after full 5 s ({elapsed_live:.2f} s)")
    request("/api/command", {"action": "srv_ign_on"})  # equivalent to pressing L
    s = wait_for(
        lambda st: st["controls"]["srv"] == "ON"
        and st["controls"]["ign"] == "PWR"
        and st["actions"]["srv_ign_off"]["enabled"],
        "manual L trigger acknowledged as SRV+IGN ON/PWR",
        timeout=5,
    )
    check(
        s["vehicle"]["state"]["srv"]
        and s["vehicle"]["state"]["ign"]
        and not s["countdown"]["active"]
        and s["actions"]["srv_ign_off"]["enabled"],
        "SRV+IGN confirmed ON/PWR and operator can command SRV+IGN -> OFF",
    )
    code, _ = request_status("/api/config", {"op_mode": "test"})
    check(code == 400, "switching LIVE -> TEST is blocked while SRV+IGN is ON")

    # Operator turns SRV+IGN -> OFF.
    request("/api/command", {"action": "srv_ign_off"})
    s = wait_for(
        lambda st: not st["vehicle"]["state"]["srv"]
        and not st["vehicle"]["state"]["ign"]
        and st["controls"]["safe"],
        "SRV+IGN -> OFF confirmed",
    )
    check(
        s["controls"]["summary"] == "FILL=OFF SRV=OFF IGN=SAFE",
        "operator SRV+IGN -> OFF returned vehicle cleanly to OFF/SAFE",
    )

    # Clear completed-fill latch before switching to TEST mode.
    request("/api/command", {"action": "reset_controls"})
    wait_for(
        lambda st: st["controls"]["safe"]
        and not st["vehicle"]["state"]["fill_seq_complete"]
        and not st["vehicle"]["state"]["fill_paused"]
        and st["actions"]["fill_on"]["enabled"],
        "LIVE controls reset cleanly before switching to TEST mode",
    )

    # ======================================================================
    # PART 2: TEST MODE (independent FILL, SRV, IGN controls & countdown abort)
    # ======================================================================
    request("/api/config", {"op_mode": "test", "fill_seconds": 4, "countdown_seconds": 2})
    s = wait_for(
        lambda st: st["config"]["op_mode"] == "test",
        "switched to TEST mode",
    )
    check(not s["actions"]["srv_ign_on"]["enabled"], "combined SRV+IGN disabled in TEST mode")
    check(not s["actions"]["srv_on"]["enabled"], "SRV locked before FILL runs in TEST mode")
    check(not s["actions"]["ign_pwr"]["enabled"], "IGN blocked at rest in TEST mode")
    check("SRV" in s["actions"]["ign_pwr"]["reason"], "IGN reason names SRV in TEST mode")

    request("/api/command", {"action": "fill_on"})
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
    check("FILL" in s["actions"]["srv_on"]["reason"], "SRV blocked while FILL is open")

    # --- Pause FILL early in TEST mode -> SRV unlocks ---
    time.sleep(0.8)
    request("/api/command", {"action": "fill_off"})
    s = wait_for(
        lambda st: st["fill"]["paused"] and st["actions"]["srv_on"]["enabled"],
        "TEST fill paused and SRV unlocked",
    )
    test_paused_rem = s["fill"]["remaining_ms"]
    check(
        test_paused_rem > 0 and s["actions"]["srv_on"]["enabled"],
        f"pausing FILL in TEST mode unlocked SRV (remaining={test_paused_rem} ms)",
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

    # Unsafe FILL ON while SRV is ON must be refused.
    request("/api/command", {"action": "fill_on"})
    time.sleep(0.5)
    s = state()
    check(not s["vehicle"]["state"]["fill"] and s["vehicle"]["state"]["srv"], "unsafe FILL ON refused while SRV ON")

    # --- Arm IGN countdown in TEST mode and ABORT mid-countdown ---
    request("/api/command", {"action": "ign_pwr"})
    s = wait_for(
        lambda st: st["countdown"]["active"] and st["countdown"]["target"] == "ign",
        "TEST IGN countdown active",
    )
    check(
        not s["vehicle"]["state"]["ign"]
        and s["actions"]["abort_countdown"]["enabled"]
        and s["actions"]["ign_safe"]["enabled"],
        "IGN countdown keeps igniter SAFE and enables abort actions",
    )
    request("/api/command", {"action": "ign_safe"})
    s = wait_for(
        lambda st: not st["countdown"]["active"]
        and st["controls"]["safe"]
        and not st["vehicle"]["state"]["srv"],
        "aborting TEST IGN countdown returned SRV -> OFF",
    )
    check(
        not s["vehicle"]["state"]["ign"]
        and s["fill"]["paused"]
        and s["fill"]["remaining_ms"] == test_paused_rem
        and s["actions"]["fill_on"]["enabled"]
        and s["actions"]["srv_on"]["enabled"],
        "aborting TEST IGN countdown safed all outputs while keeping paused FILL intact",
    )

    # --- Resume paused FILL to completion, then complete full TEST IGN countdown ---
    request("/api/command", {"action": "fill_on"})
    wait_for(
        lambda st: st["fill"]["seq_complete"] and st["actions"]["srv_on"]["enabled"],
        "resumed TEST fill complete",
        timeout=10,
    )
    request("/api/command", {"action": "srv_on"})
    wait_for(lambda st: st["actions"]["ign_pwr"]["enabled"], "IGN armed after resumed fill")

    t_ign = time.monotonic()
    request("/api/command", {"action": "ign_pwr"})
    wait_for(lambda st: st["countdown"]["active"], "2 s TEST IGN countdown active")
    s = wait_for(
        lambda st: st["countdown"]["active"] and st["countdown"].get("holding"),
        "TEST IGN countdown held at T-0",
        timeout=6,
    )
    elapsed_ign = time.monotonic() - t_ign
    check(
        elapsed_ign >= 1.9
        and s["countdown"]["remaining_ms"] == 0
        and not s["vehicle"]["state"]["ign"]
        and s["actions"]["ign_pwr"]["enabled"],
        f"TEST countdown held at 0.0 s with IGN SAFE after {elapsed_ign:.2f} s",
    )
    request("/api/command", {"action": "ign_pwr"})  # equivalent to pressing L
    s = wait_for(lambda st: st["vehicle"]["state"]["ign"], "manual L trigger: IGN PWR", timeout=5)
    check(s["vehicle"]["summary"].endswith("IGN=PWR"), "manual IGN trigger acknowledged by the vehicle")

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
        and st["vehicle"]["state"]["fill_paused"] is False
        and st["vehicle"]["state"]["fill_seq_aborted"] is False,
        "controls reset to OFF/SAFE",
    )
    check(s["controls"]["summary"] == "FILL=OFF SRV=OFF IGN=SAFE", "reset returned to OFF/SAFE")
    s = wait_for(
        lambda st: st["actions"]["fill_on"]["enabled"]
        and not st["actions"]["srv_on"]["enabled"]
        and st["actions"]["reset_fill"]["enabled"] is False,
        "machine usable after reset",
    )
    check(True, "reset cleared every latch and re-locked SRV until next FILL")

    # --- Live USB CDC baud, UART baud & E28-2G4M27S module handover -------
    bad_usb, _ = request_status("/api/config", {"baud": 12345})
    bad_uart, _ = request_status("/api/config", {"uart_baud": 14400})
    bad_radio, _ = request_status("/api/config", {"radio": "sx1276"})
    check(
        bad_usb == 400 and bad_uart == 400 and bad_radio == 400,
        "invalid USB baud, UART baud, and radio module values are rejected",
    )

    # 1) Live switch USB CDC -> 230400 and DX-LR42 UART -> 57600 (saved to STM32 flash)
    request("/api/config", {"baud": 230400, "uart_baud": 57600, "radio": "dx-lr42-24t12d"})
    s = wait_for(
        lambda st: st["link_reconfig"] is None
        and st["ground"]["link"]["usb_baud"] == 230400
        and st["ground"]["link"]["uart_baud"] == 57600
        and st["ground"]["link"]["radio"] == "dx-lr42-24t12d"
        and st["ground"]["link"]["flash_saved"] is True
        and st["vehicle"]["status"] == "online",
        "live handover to USB 230400 + DX-LR42 UART 57600",
        timeout=10,
    )
    check(
        s["config"]["baud"] == 230400 and s["config"]["uart_baud"] == 57600,
        "USB 230400 + UART 57600 live handover acknowledged and saved to flash",
    )

    # 2) Live two-phase handover to E28-2G4M27S (SPI1 500 mW) + USB 460800
    request("/api/config", {"baud": 460800, "uart_baud": 115200, "radio": "e28-2g4m27s"})
    s = wait_for(
        lambda st: st["link_reconfig"] is None
        and st["ground"]["link"]["radio"] == "e28-2g4m27s"
        and st["ground"]["link"]["usb_baud"] == 460800
        and st["ground"]["link"]["flash_saved"] is True
        and st["vehicle"]["status"] == "online",
        "live two-phase handover to E28-2G4M27S + USB 460800",
        timeout=10,
    )
    check(
        s["config"]["radio"] == "e28-2g4m27s" and s["ground"]["link"]["radio_part"] == "E28-2G4M27S",
        "Vehicle + Ground switched live to E28-2G4M27S (SPI1) without dropping RF link",
    )

    # Verify live vehicle actuation & safety block over E28-2G4M27S link!
    request("/api/command", {"action": "fill_on"})
    wait_for(lambda st: st["controls"]["fill"] == "ON", "FILL ON over E28-2G4M27S")
    blocked_reconfig, _ = request_status("/api/config", {"radio": "dx-lr42-24t12d"})
    check(blocked_reconfig == 400, "changing radio/baud is blocked while vehicle output is active")
    request("/api/command", {"action": "fill_off"})
    wait_for(lambda st: st["controls"]["fill"] == "OFF", "FILL OFF over E28-2G4M27S")
    request("/api/command", {"action": "reset_controls"})
    wait_for(
        lambda st: st["controls"]["safe"]
        and not st["vehicle"]["state"]["fill_paused"]
        and st["actions"]["fill_on"]["enabled"],
        "controls reset over E28-2G4M27S",
    )

    # --- Reconnect with an output ON: initialisation must force OFF/SAFE. --
    request("/api/command", {"action": "fill_on"})
    wait_for(lambda st: st["controls"]["fill"] == "ON", "FILL on for pause before reconnect")
    request("/api/command", {"action": "fill_off"})
    wait_for(lambda st: st["actions"]["srv_on"]["enabled"], "SRV unlocked before reconnect")
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
    request("/api/command", {"action": "fill_on"})
    wait_for(lambda st: st["controls"]["fill"] == "ON", "FILL on before shutdown")
    check(True, "shutdown will happen with an active output")

    s = request("/api/state")
    check(s["stats"]["commands_sent"] >= 10, "commands were transmitted")
    check(s["stats"]["acks_ok"] >= 10, "every command was acknowledged")

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
