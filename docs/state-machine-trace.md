# Control-state trace

`scripts/state_trace.py` spawns isolated `gse-sim` and `gse-app` processes on
ephemeral TCP ports, subscribes to the live SSE stream (`/api/events`), and
drives both operating modes (**`LIVE`** and **`TEST`**) end-to-end over the real
HTTP API (`/api/config` + `/api/command` → link → vehicle `fsm` → `ACK` → SSE
`state`).

## Running it

```bash
cargo build --workspace
python3 scripts/state_trace.py ./target/debug/gse-app
```

The script walks through eight phases automatically:

### Phase A — `LIVE` launch mode (`op_mode: "live"`, default)

1. **`A1`** — Waits for the startup `INIT` handshake to confirm
   `FILL=OFF SRV=OFF IGN=SAFE`, and checks that `SRV+IGN`, `SRV`, and `IGN` are
   all disabled at rest.
2. **`A2`** — Opens `FILL` and closes it manually early; verifies
   `fill_seq_aborted = true` and confirms `SRV+IGN` remains greyed out.
3. **`A3`** — Clears the fill latch (`reset_fill`), runs a complete timed `FILL`
   cycle (`fill_seconds = 3`) through automatic `FILL OFF`, and verifies
   `fill_seq_complete = true` unlocks `SRV+IGN` (`srv_ign = True`).
4. **`A4`** — Arms `SRV+IGN` (`cd = True`) to start the on-screen launch
   countdown, then fires `E-STOP` mid-countdown and verifies the countdown is
   cancelled immediately without `SRV` or `IGN` ever turning on.
5. **`A5`** — Resets controls, completes another timed fill, runs the full `5 s`
   countdown to `T-0 HOLD`, verifies `remaining_ms = 0` and every output remains
   safe, then explicitly triggers with the `L` action and returns `SRV+IGN` to
   `OFF/SAFE`.

### Phase B — `TEST` checkout mode (`op_mode: "test"`)

6. **`B1`** — Switches `op_mode` to `"test"` via `POST /api/config` and checks
   that independent `SRV` is enabled while combined `SRV+IGN` is disabled.
7. **`B2`** — Verifies an aborted manual fill + `SRV ON` still keeps `IGN PWR`
   locked (`ign = False`).
8. **`B3`** — Exercises the test-mode `IGN` countdown abort path and confirms
   `SRV` returns `OFF` while FILL progress is preserved.
9. **`B4`** — Completes FILL, runs the `IGN` countdown to T-0 HOLD with IGN
   still SAFE, explicitly triggers using the `L` action, and resets controls.

## Recorded run

The trace below is generated dynamically by `scripts/state_trace.py`. Re-run the
command above to capture the current behavior; older captures showing ignition
at T-0 predate the explicit operator-trigger hold and should not be used as the
current safety behavior.

## What the run demonstrates

| Requirement | Assertion in the live trace |
|---|---|
| Countdown cancellation is safe | Aborting mid-countdown leaves IGN SAFE, returns SRV OFF when needed, and preserves paused FILL progress. |
| No automatic ignition at T-0 | Both LIVE and TEST expose `countdown.active = true`, `countdown.holding = true`, `remaining_ms = 0`, while outputs remain OFF/SAFE. |
| Explicit operator trigger works | The matching `srv_ign_on` (LIVE) or `ign_pwr` (TEST) action is enabled only at HOLD; the test trace sends it as the operator's L key action and verifies the acknowledged transition. |
| Existing state-machine interlocks remain enforced | Completed/paused FILL is required; E-STOP, safe toggles and RESET CONTROLS remain available during countdown/hold. |
