#!/usr/bin/env python3
"""Print the live control-state machine of a running GSE console.

Runs a short, safe scenario against a simulator (never against a charged
vehicle) and logs every distinct machine state, so the deterministic behaviour
is visible: INIT -> OFF/SAFE, command -> target, ACK -> confirmed, reset ->
OFF/SAFE, optional shutdown -> ALL SAFE.

Usage:
    scripts/state_trace.py BASE_URL TOKEN [--shutdown]

Requires gse-sim + gse-app to be running (scripts/smoke.sh starts its own pair).
"""

import json
import sys
import time
import urllib.error
import urllib.request

BASE = ""
TOKEN = ""
SHUTDOWN = "--shutdown" in sys.argv[1:]


def request(path, body=None, timeout=5):
    data = None if body is None else json.dumps(body).encode()
    req = urllib.request.Request(
        BASE + path, data=data, method="GET" if data is None else "POST"
    )
    req.add_header("X-GSE-Token", TOKEN)
    if data is not None:
        req.add_header("Content-Type", "application/json")
    with urllib.request.urlopen(req, timeout=timeout) as resp:
        return json.loads(resp.read().decode())


def fmt_state(controls):
    return (
        f"FILL {controls['fill']:3} · SRV {controls['srv']:3} · "
        f"IGN {controls['ign']:4}"
    )


def fmt_target(controls):
    target = controls.get("target")
    if not target:
        return ""
    return (
        f" -> FILL {target['fill']:3} · SRV {target['srv']:3} · IGN {target['ign']:4}"
    )


def signature(state):
    ctl = state["controls"]
    return (
        ctl["ready"],
        ctl["fill"],
        ctl["srv"],
        ctl["ign"],
        None
        if not ctl.get("target")
        else (ctl["target"]["fill"], ctl["target"]["srv"], ctl["target"]["ign"]),
        None if not state.get("pending") else state["pending"]["action"],
    )


class Trace:
    def __init__(self, start):
        self.start = start
        self.last = None

    def note(self, state):
        """Print one line per distinct machine state."""
        sig = signature(state)
        if sig == self.last:
            return
        previous = self.last
        self.last = sig
        ctl = state["controls"]
        elapsed = time.monotonic() - self.start
        pending = state.get("pending")
        tag = "state"
        if previous is not None:
            was_pending = previous[5]
            if pending and not was_pending:
                tag = "pending"
            elif not pending and was_pending:
                tag = "ack"
        pending_text = f"  [pending {pending['action']}]" if pending else ""
        print(
            f"  +{elapsed:6.3f}s  {tag:7}  {fmt_state(ctl)}"
            f"{fmt_target(ctl)}{pending_text}"
        )

    def sample(self, label, seconds, predicate):
        deadline = time.monotonic() + seconds
        while time.monotonic() < deadline:
            state = request("/api/state")
            self.note(state)
            if predicate(state):
                return state
            time.sleep(0.01)
        state = request("/api/state")
        raise AssertionError(
            f"timeout waiting for {label}; last={json.dumps(state['controls'])}"
        )


def main():
    global BASE, TOKEN
    positional = [a for a in sys.argv[1:] if not a.startswith("--")]
    if len(positional) != 2:
        print(__doc__)
        return 2
    BASE, TOKEN = positional[0].rstrip("/"), positional[1]
    trace = Trace(time.monotonic())

    print("GSE control-state trace")
    state = None
    deadline = time.monotonic() + 20
    while time.monotonic() < deadline:
        state = request("/api/state")
        trace.note(state)
        if state["vehicle"]["status"] == "online" and state["controls"]["ready"]:
            break
        time.sleep(0.1)
    else:
        raise AssertionError("boards never came online")
    print(f"  initial  {state['controls']['summary']}  (ready)")

    # Start from a known state even if the board was left mid-sequence.
    if not state["controls"]["safe"]:
        request("/api/command", {"action": "reset_controls"})
        state = trace.sample(
            "initial reset",
            6,
            lambda s: s["controls"]["safe"] and not s["pending"],
        )

    request("/api/config", {"fill_seconds": 3})

    print("  action   FILL ON (3 s fill)")
    request("/api/command", {"action": "fill_on"})
    state = trace.sample(
        "FILL ON acknowledgement",
        5,
        lambda s: s["controls"]["fill"] == "ON" and not s["pending"],
    )
    assert state["vehicle"]["state"]["fill"], "FILL is not open after the ACK"

    state = trace.sample(
        "fill auto-off",
        10,
        lambda s: s["controls"]["fill"] == "OFF"
        and s["fill"]["seq_complete"]
        and not s["pending"],
    )
    enabled = [k for k, v in state["actions"].items() if v["enabled"]]
    print(f"  gates    enabled: {', '.join(enabled)}")
    assert "ign_pwr" not in enabled, "IGN must stay gated until SRV is ON"

    print("  action   SRV ON")
    request("/api/command", {"action": "srv_on"})
    state = trace.sample(
        "SRV ON acknowledgement",
        5,
        lambda s: s["controls"]["srv"] == "ON"
        and not s["pending"]
        and s["actions"]["ign_pwr"]["enabled"],
    )
    print("  gates    IGN PWR is now enabled (SRV ON + completed fill)")

    print("  action   RESET CONTROLS")
    request("/api/command", {"action": "reset_controls"})
    state = trace.sample(
        "reset controls",
        8,
        lambda s: s["controls"]["safe"]
        and not s["pending"]
        and not s["vehicle"]["state"]["fill_seq_complete"]
        and not s["vehicle"]["state"]["estop"],
    )
    print(f"  final    {state['controls']['summary']}  (latches cleared)")

    if SHUTDOWN:
        log_path = state.get("log_path")
        print("  shutdown -> ALL SAFE")
        try:
            request("/api/shutdown", {}, timeout=5)
        except (urllib.error.URLError, ConnectionError, TimeoutError):
            pass
        deadline = time.monotonic() + 8
        while time.monotonic() < deadline:
            try:
                request("/api/health", timeout=0.5)
            except (urllib.error.URLError, ConnectionError, TimeoutError):
                break
            time.sleep(0.2)
        if log_path:
            with open(log_path, encoding="utf-8") as fh:
                text = fh.read()
            if "ALL SAFE acknowledged" not in text:
                raise AssertionError("shutdown did not confirm ALL SAFE in the log")
            print("  verified ALL SAFE was acknowledged before exit")

    print("TRACE OK")
    return 0


if __name__ == "__main__":
    sys.exit(main())
