#!/usr/bin/env python3
"""Drive the running gse-app console through the full sequence and print a
human-readable trace of every state transition seen on the SSE stream.

Walks both operating modes end-to-end:
  1. LIVE launch mode:
     - verifies SRV+IGN starts greyed out (OFF/SAFE) prior to fill completion
     - verifies early manual FILL OFF leaves SRV+IGN locked
     - runs full timed FILL sequence -> automatic FILL OFF -> SRV+IGN unlocked
     - starts SRV+IGN 5-second countdown, tests mid-countdown E-STOP abort, then
       re-arms and completes the full 5-second countdown -> SRV+IGN ON/PWR ->
       operator commands SRV+IGN OFF/SAFE
  2. Settings switch to TEST mode:
     - FILL ON -> manual FILL OFF -> IGN still locked
     - RESET FILL -> timed FILL ON -> automatic FILL OFF -> FILL_SEQ_COMPLETE
     - SRV ON -> IGN PWR unlocked -> IGN PWR -> RESET CONTROLS -> ALL SAFE

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
                            vs["fill_seq_complete"],
                            vs["fill_seq_aborted"],
                            act["srv_ign_on"]["enabled"],
                            act["ign_pwr"]["enabled"],
                        )
                        if not seen_states or seen_states[-1][1] != key:
                            t = time.monotonic() - start
                            seen_states.append((t, key))
                            print(
                                f"{t:6.2f}s  [{key[0].upper():4s}]  ready={str(key[1]):5s}  {key[2]:24s}"
                                f"  cd={str(key[3]):5s}  seq_ok={str(key[4]):5s}  abort={str(key[5]):5s}"
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
                time.sleep(0.08)
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

        step("A2. LIVE: Early manual FILL OFF aborts fill and keeps SRV+IGN greyed out")
        api("/api/config", {"fill_seconds": 3})
        api("/api/command", {"action": "fill_on"})
        wait(lambda x: x["controls"]["fill"] == "ON", "LIVE fill ON")
        time.sleep(0.7)
        api("/api/command", {"action": "fill_off"})
        s = wait(lambda x: x["controls"]["fill"] == "OFF" and x["fill"]["seq_aborted"], "LIVE fill aborted")
        assert not s["actions"]["srv_ign_on"]["enabled"], "aborted fill must never unlock SRV+IGN"

        step("A3. LIVE: Reset fill, run full timed FILL -> auto FILL OFF -> SRV+IGN unlocks")
        api("/api/command", {"action": "reset_fill"})
        wait(lambda x: not x["fill"]["seq_aborted"], "LIVE latch cleared")
        api("/api/command", {"action": "fill_on"})
        wait(lambda x: x["controls"]["fill"] == "ON", "LIVE timed fill ON")
        s = wait(
            lambda x: x["controls"]["fill"] == "OFF"
            and x["fill"]["seq_complete"]
            and not x["fill"]["seq_aborted"]
            and x["actions"]["srv_ign_on"]["enabled"],
            "LIVE timed fill complete + SRV+IGN unlocked",
        )

        step("A4. LIVE: Start 5 s SRV+IGN countdown, then abort mid-countdown via E-STOP")
        api("/api/command", {"action": "srv_ign_on"})
        wait(lambda x: x["countdown"]["active"], "LIVE countdown active")
        time.sleep(1.0)
        api("/api/command", {"action": "estop"})
        s = wait(
            lambda x: not x["countdown"]["active"] and x["vehicle"]["state"]["estop"],
            "mid-countdown E-STOP cancelled countdown",
        )
        assert not s["vehicle"]["state"]["srv"] and not s["vehicle"]["state"]["ign"]

        step("A5. LIVE: Reset controls, re-run timed FILL, run full 5 s countdown -> SRV+IGN ON -> SRV+IGN OFF")
        api("/api/command", {"action": "reset_controls"})
        wait(
            lambda x: x["controls"]["safe"]
            and not x["vehicle"]["state"]["estop"]
            and x["actions"]["fill_on"]["enabled"],
            "reset after mid-countdown E-STOP",
        )
        api("/api/config", {"fill_seconds": 2})
        api("/api/command", {"action": "fill_on"})
        wait(lambda x: x["actions"]["srv_ign_on"]["enabled"], "SRV+IGN unlocked again")
        t0 = time.monotonic()
        api("/api/command", {"action": "srv_ign_on"})
        wait(lambda x: x["countdown"]["active"], "5 s countdown started")
        s = wait(
            lambda x: x["controls"]["srv"] == "ON"
            and x["controls"]["ign"] == "PWR"
            and x["actions"]["srv_ign_off"]["enabled"],
            "SRV+IGN ON/PWR acknowledged after 5 s countdown",
            timeout=10.0,
        )
        assert (time.monotonic() - t0) >= 4.8
        assert s["vehicle"]["state"]["srv"] and s["vehicle"]["state"]["ign"]
        api("/api/command", {"action": "srv_ign_off"})
        wait(lambda x: x["controls"]["safe"], "SRV+IGN OFF/SAFE")
        api("/api/command", {"action": "reset_controls"})
        wait(
            lambda x: x["controls"]["safe"] and not x["vehicle"]["state"]["fill_seq_complete"],
            "clean slate for TEST mode",
        )

        # ------------------------------------------------------------------
        # PHASE B: TEST MODE (independent FILL, SRV, IGN controls)
        # ------------------------------------------------------------------
        step("B1. Switch to TEST mode in Settings")
        api("/api/config", {"op_mode": "test", "fill_seconds": 3})
        s = wait(lambda x: x["config"]["op_mode"] == "test", "TEST mode active")
        assert s["actions"]["srv_on"]["enabled"]
        assert not s["actions"]["srv_ign_on"]["enabled"]

        step("B2. TEST: Early manual FILL OFF aborts fill and keeps IGN locked")
        api("/api/command", {"action": "fill_on"})
        wait(lambda x: x["controls"]["fill"] == "ON", "TEST fill ON")
        time.sleep(0.8)
        api("/api/command", {"action": "fill_off"})
        s = wait(lambda x: x["controls"]["fill"] == "OFF" and x["fill"]["seq_aborted"], "TEST fill aborted")
        api("/api/command", {"action": "srv_on"})
        s = wait(lambda x: x["controls"]["srv"] == "ON", "SRV ON after abort")
        assert not s["actions"]["ign_pwr"]["enabled"], "aborted fill must never arm IGN"
        api("/api/command", {"action": "srv_off"})
        wait(lambda x: x["controls"]["srv"] == "OFF", "SRV OFF")

        step("B3. TEST: Full timed FILL -> auto FILL OFF -> SRV ON -> IGN PWR -> RESET CONTROLS")
        api("/api/command", {"action": "reset_fill"})
        wait(lambda x: not x["fill"]["seq_aborted"], "latch cleared")
        api("/api/command", {"action": "fill_on"})
        wait(lambda x: x["controls"]["fill"] == "ON", "timed fill ON")
        s = wait(
            lambda x: x["controls"]["fill"] == "OFF" and x["fill"]["seq_complete"],
            "timed fill complete",
        )
        assert not s["fill"]["seq_aborted"], "completed fill must not set fill_aborted"
        api("/api/command", {"action": "srv_on"})
        s = wait(lambda x: x["controls"]["srv"] == "ON" and x["actions"]["ign_pwr"]["enabled"], "IGN armed")
        api("/api/command", {"action": "ign_pwr"})
        wait(lambda x: x["controls"]["ign"] == "PWR", "IGN PWR")

        api("/api/command", {"action": "reset_controls"})
        s = wait(
            lambda x: x["controls"]["safe"]
            and not x["fill"]["seq_complete"]
            and not x["fill"]["seq_aborted"],
            "reset complete",
        )
        assert s["actions"]["fill_on"]["enabled"]
        assert s["actions"]["srv_on"]["enabled"]
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
