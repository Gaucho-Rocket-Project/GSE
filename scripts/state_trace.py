#!/usr/bin/env python3
"""Drive the running gse-app console through the full sequence and print a
human-readable trace of every state transition seen on the SSE stream.

Walks both operating modes end-to-end:
  1. LIVE launch mode:
     - verifies SRV+IGN starts greyed out (OFF/SAFE) prior to fill running
     - turns FILL -> ON, then manually turns FILL -> OFF prior to expiry:
       verifies FILL pauses (remaining_ms frozen) AND unlocks SRV+IGN
     - starts SRV+IGN countdown, aborts mid-countdown, verifies outputs stay
       OFF/SAFE and paused FILL progress remains intact
     - resumes FILL -> ON -> automatic FILL -> OFF -> SRV+IGN unlocked
     - runs full countdown -> SRV+IGN ON/PWR -> operator SRV+IGN OFF/SAFE
  2. Settings switch to TEST mode:
     - verifies SRV and IGN start locked prior to FILL running
     - turns FILL -> ON -> pauses FILL -> OFF -> SRV unlocks
     - turns SRV -> ON -> starts IGN countdown -> aborts mid-countdown ->
       verifies SRV returns OFF and paused FILL progress remains intact
     - resumes FILL -> ON -> automatic FILL -> OFF -> SRV ON -> IGN countdown ->
       IGN PWR -> RESET CONTROLS -> ALL SAFE

Usage: python3 scripts/state_trace.py [path/to/gse-app]
"""

import json
import os
import re
import socket
import subprocess
import sys
import tempfile
import threading
import time
import urllib.request


def free_port() -> int:
    with socket.socket() as s:
        s.bind(("127.0.0.1", 0))
        return s.getsockname()[1]


def main() -> int:
    binary = sys.argv[1] if len(sys.argv) > 1 else "./target/debug/gse-app"
    sim_binary = os.path.join(os.path.dirname(binary), "gse-sim")
    sim_port = free_port()
    port = free_port()
    tmp = tempfile.TemporaryDirectory()
    env = dict(os.environ, XDG_STATE_HOME=os.path.join(tmp.name, "state"))

    sim_proc = subprocess.Popen(
        [sim_binary, "--listen", f"127.0.0.1:{sim_port}", "--latency", "10"],
        env=env,
        stdout=subprocess.PIPE,
        stderr=subprocess.STDOUT,
        text=True,
    )
    proc = subprocess.Popen(
        [
            binary,
            "--port",
            f"tcp:127.0.0.1:{sim_port}",
            "--no-browser",
            "--http-port",
            str(port),
            "--config",
            os.path.join(tmp.name, "config.json"),
        ],
        env=env,
        stdout=subprocess.PIPE,
        stderr=subprocess.STDOUT,
        text=True,
    )

    try:
        base = f"http://127.0.0.1:{port}"
        token = None
        for _ in range(80):
            try:
                page = urllib.request.urlopen(base + "/", timeout=1).read().decode()
                m = re.search(r'gse-token" content="([0-9a-f]+)"', page)
                if m:
                    token = m.group(1)
                    break
            except OSError:
                time.sleep(0.1)
        if not token:
            raise RuntimeError("did not obtain UI token")

        def api(path: str, body=None):
            data = None if body is None else json.dumps(body).encode()
            req = urllib.request.Request(
                base + path, data=data, method="GET" if data is None else "POST"
            )
            req.add_header("X-GSE-Token", token)
            if data is not None:
                req.add_header("Content-Type", "application/json")
            with urllib.request.urlopen(req, timeout=5) as resp:
                return json.loads(resp.read().decode())

        start = time.monotonic()
        seen_states = []
        stop = threading.Event()

        def watch_events():
            url = f"{base}/api/events?token={token}"
            with urllib.request.urlopen(url, timeout=30) as stream:
                event = None
                for raw in stream:
                    if stop.is_set():
                        return
                    line = raw.decode(errors="replace").rstrip("\r\n")
                    if line.startswith("event: "):
                        event = line[7:]
                    elif line.startswith("data: ") and event == "state":
                        snap = json.loads(line[6:])
                        ctl = snap["controls"]
                        vs = snap["vehicle"]["state"]
                        cd = snap["countdown"]
                        act = snap["actions"]
                        key = (
                            snap["config"]["op_mode"],
                            ctl["ready"],
                            ctl["summary"],
                            cd["active"],
                            vs["fill_paused"],
                            vs["fill_seq_complete"],
                            act["srv_ign_on"]["enabled"],
                            act["ign_pwr"]["enabled"],
                        )
                        if not seen_states or seen_states[-1][1] != key:
                            t = time.monotonic() - start
                            seen_states.append((t, key))
                            print(
                                f"{t:6.2f}s  [{key[0].upper():4s}]  ready={str(key[1]):5s}  {key[2]:24s}"
                                f"  cd={str(key[3]):5s}  paused={str(key[4]):5s}  seq_ok={str(key[5]):5s}"
                                f"  srv_ign={str(key[6]):5s}  ign={str(key[7]):5s}"
                            )

        watcher = threading.Thread(target=watch_events, daemon=True)
        watcher.start()

        def wait(pred, what, timeout=20.0):
            deadline = time.monotonic() + timeout
            while time.monotonic() < deadline:
                snap = api("/api/state")
                if pred(snap):
                    return snap
                time.sleep(0.06)
            raise AssertionError(f"timed out waiting for {what}")

        def step(title: str):
            print(f"\n--- {title} ---")

        # ------------------------------------------------------------------
        # PHASE A: LIVE LAUNCH MODE
        # ------------------------------------------------------------------
        step("A1. Wait for startup SAFE handshake in default LIVE mode")
        s = wait(lambda x: x["controls"]["ready"] and x["controls"]["safe"], "ready")
        assert s["config"]["op_mode"] == "live"
        assert not s["actions"]["srv_ign_on"]["enabled"]
        assert not s["actions"]["srv_on"]["enabled"]
        assert not s["actions"]["ign_pwr"]["enabled"]

        step("A2. LIVE: Early manual FILL -> OFF pauses timer AND unlocks SRV+IGN")
        api("/api/config", {"fill_seconds": 4, "countdown_seconds": 2})
        api("/api/command", {"action": "fill_on"})
        wait(lambda x: x["controls"]["fill"] == "ON", "LIVE fill ON")
        time.sleep(0.9)
        api("/api/command", {"action": "fill_off"})
        s = wait(lambda x: x["controls"]["fill"] == "OFF" and x["fill"]["paused"], "LIVE fill paused")
        rem_live = s["fill"]["remaining_ms"]
        assert rem_live > 0
        assert s["actions"]["srv_ign_on"]["enabled"], "paused fill must unlock SRV+IGN"

        step("A3. LIVE: Start SRV+IGN countdown, abort mid-countdown, keep FILL progress intact")
        api("/api/command", {"action": "srv_ign_on"})
        wait(lambda x: x["countdown"]["active"] and x["countdown"]["target"] == "srv_ign", "LIVE countdown")
        time.sleep(0.5)
        api("/api/command", {"action": "abort_countdown"})
        s = wait(lambda x: not x["countdown"]["active"], "LIVE countdown aborted")
        assert s["controls"]["safe"]
        assert s["fill"]["paused"] and s["fill"]["remaining_ms"] == rem_live

        step("A4. LIVE: Resume FILL -> ON -> auto FILL -> OFF -> full countdown -> SRV+IGN ON -> OFF")
        api("/api/command", {"action": "fill_on"})
        wait(lambda x: x["controls"]["fill"] == "ON", "LIVE resumed fill ON")
        wait(
            lambda x: x["controls"]["fill"] == "OFF"
            and x["fill"]["seq_complete"]
            and not x["fill"]["paused"]
            and x["actions"]["srv_ign_on"]["enabled"],
            "LIVE resumed fill complete + SRV+IGN unlocked",
        )
        t0 = time.monotonic()
        api("/api/command", {"action": "srv_ign_on"})
        wait(lambda x: x["countdown"]["active"], "LIVE countdown started")
        s = wait(
            lambda x: x["controls"]["srv"] == "ON"
            and x["controls"]["ign"] == "PWR"
            and x["actions"]["srv_ign_off"]["enabled"],
            "SRV+IGN ON/PWR acknowledged after countdown",
            timeout=8.0,
        )
        assert (time.monotonic() - t0) >= 1.9
        assert s["vehicle"]["state"]["srv"] and s["vehicle"]["state"]["ign"]
        api("/api/command", {"action": "srv_ign_off"})
        wait(lambda x: x["controls"]["safe"], "SRV+IGN OFF/SAFE")
        api("/api/command", {"action": "reset_controls"})
        wait(
            lambda x: x["controls"]["safe"] and not x["vehicle"]["state"]["fill_seq_complete"],
            "clean slate for TEST mode",
        )

        # ------------------------------------------------------------------
        # PHASE B: TEST MODE (independent FILL, SRV, IGN controls + countdown)
        # ------------------------------------------------------------------
        step("B1. Switch to TEST mode in Settings (SRV & IGN both start locked)")
        api("/api/config", {"op_mode": "test", "fill_seconds": 4, "countdown_seconds": 2})
        s = wait(lambda x: x["config"]["op_mode"] == "test", "TEST mode active")
        assert not s["actions"]["srv_on"]["enabled"]
        assert not s["actions"]["ign_pwr"]["enabled"]
        assert not s["actions"]["srv_ign_on"]["enabled"]

        step("B2. TEST: Early manual FILL -> OFF pauses timer AND unlocks SRV")
        api("/api/command", {"action": "fill_on"})
        wait(lambda x: x["controls"]["fill"] == "ON", "TEST fill ON")
        time.sleep(0.9)
        api("/api/command", {"action": "fill_off"})
        s = wait(lambda x: x["controls"]["fill"] == "OFF" and x["fill"]["paused"], "TEST fill paused")
        rem_test = s["fill"]["remaining_ms"]
        assert rem_test > 0 and s["actions"]["srv_on"]["enabled"]

        step("B3. TEST: SRV ON -> start IGN countdown -> ABORT -> SRV returns OFF & FILL preserved")
        api("/api/command", {"action": "srv_on"})
        wait(lambda x: x["controls"]["srv"] == "ON" and x["actions"]["ign_pwr"]["enabled"], "SRV ON")
        api("/api/command", {"action": "ign_pwr"})
        wait(lambda x: x["countdown"]["active"] and x["countdown"]["target"] == "ign", "TEST IGN countdown")
        time.sleep(0.5)
        api("/api/command", {"action": "ign_safe"})
        s = wait(
            lambda x: not x["countdown"]["active"] and x["controls"]["safe"],
            "TEST countdown aborted & SRV returned OFF",
        )
        assert not s["vehicle"]["state"]["ign"]
        assert s["fill"]["paused"] and s["fill"]["remaining_ms"] == rem_test

        step("B4. TEST: Resume FILL -> auto OFF -> SRV ON -> IGN countdown -> IGN PWR -> RESET")
        api("/api/command", {"action": "fill_on"})
        wait(lambda x: x["controls"]["fill"] == "OFF" and x["fill"]["seq_complete"], "resumed TEST fill complete")
        api("/api/command", {"action": "srv_on"})
        wait(lambda x: x["controls"]["srv"] == "ON" and x["actions"]["ign_pwr"]["enabled"], "IGN armed")
        api("/api/command", {"action": "ign_pwr"})
        wait(lambda x: x["countdown"]["active"], "TEST IGN countdown")
        wait(lambda x: x["controls"]["ign"] == "PWR", "IGN PWR")

        api("/api/command", {"action": "reset_controls"})
        s = wait(
            lambda x: x["controls"]["safe"]
            and not x["fill"]["seq_complete"]
            and not x["fill"]["paused"]
            and not x["fill"]["seq_aborted"],
            "reset complete",
        )
        assert s["actions"]["fill_on"]["enabled"]
        assert not s["actions"]["srv_on"]["enabled"]
        assert not s["actions"]["ign_pwr"]["enabled"]

        stop.set()
        print("\nALL ASSERTIONS PASSED (LIVE + TEST MODES)")
        return 0
    finally:
        proc.terminate()
        sim_proc.terminate()
        for child in (proc, sim_proc):
            try:
                child.wait(timeout=5)
            except subprocess.TimeoutExpired:
                child.kill()
        tmp.cleanup()


if __name__ == "__main__":
    raise SystemExit(main())
