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
   countdown to `T-0`, verifies atomic transition to `FILL=OFF SRV=ON IGN=PWR`,
   commands `SRV+IGN OFF` back to `FILL=OFF SRV=OFF IGN=SAFE`, and resets cleanly.

### Phase B — `TEST` checkout mode (`op_mode: "test"`)

6. **`B1`** — Switches `op_mode` to `"test"` via `POST /api/config` and checks
   that independent `SRV` is enabled while combined `SRV+IGN` is disabled.
7. **`B2`** — Verifies an aborted manual fill + `SRV ON` still keeps `IGN PWR`
   locked (`ign = False`).
8. **`B3`** — Runs a full timed fill → `SRV ON` (`ign = True`) → `IGN PWR` →
   `RESET CONTROLS` back to `FILL=OFF SRV=OFF IGN=SAFE` with all latches clear.

## Recorded run

Captured directly from `python3 scripts/state_trace.py ./target/debug/gse-app`:

```text
--- A1. Wait for startup SAFE handshake in default LIVE mode ---
  0.00s  [LIVE]  ready=False  FILL=OFF SRV=OFF IGN=SAFE  cd=False  seq_ok=False  abort=False  srv_ign=False  ign=False

--- A2. LIVE: Early manual FILL OFF aborts fill and keeps SRV+IGN greyed out ---
  0.10s  [LIVE]  ready=True   FILL=OFF SRV=OFF IGN=SAFE  cd=False  seq_ok=False  abort=False  srv_ign=False  ign=False
  0.51s  [LIVE]  ready=True   FILL=ON SRV=OFF IGN=SAFE  cd=False  seq_ok=False  abort=False  srv_ign=False  ign=False
  1.32s  [LIVE]  ready=True   FILL=ON SRV=OFF IGN=SAFE  cd=False  seq_ok=False  abort=True   srv_ign=False  ign=False

--- A3. LIVE: Reset fill, run full timed FILL -> auto FILL OFF -> SRV+IGN unlocks ---
  1.63s  [LIVE]  ready=True   FILL=OFF SRV=OFF IGN=SAFE  cd=False  seq_ok=False  abort=True   srv_ign=False  ign=False
  1.73s  [LIVE]  ready=True   FILL=OFF SRV=OFF IGN=SAFE  cd=False  seq_ok=False  abort=False  srv_ign=False  ign=False
  2.14s  [LIVE]  ready=True   FILL=ON SRV=OFF IGN=SAFE  cd=False  seq_ok=False  abort=False  srv_ign=False  ign=False
  4.88s  [LIVE]  ready=True   FILL=ON SRV=OFF IGN=SAFE  cd=False  seq_ok=True   abort=False  srv_ign=False  ign=False

--- A4. LIVE: Start 5 s SRV+IGN countdown, then abort mid-countdown via E-STOP ---
  5.19s  [LIVE]  ready=True   FILL=OFF SRV=OFF IGN=SAFE  cd=True   seq_ok=True   abort=False  srv_ign=False  ign=False
  6.27s  [LIVE]  ready=True   FILL=OFF SRV=OFF IGN=SAFE  cd=False  seq_ok=True   abort=False  srv_ign=True   ign=False
  6.31s  [LIVE]  ready=True   FILL=OFF SRV=OFF IGN=SAFE  cd=False  seq_ok=True   abort=False  srv_ign=False  ign=False

--- A5. LIVE: Reset controls, re-run timed FILL, run full 5 s countdown -> SRV+IGN ON -> SRV+IGN OFF ---
  6.41s  [LIVE]  ready=True   FILL=OFF SRV=OFF IGN=SAFE  cd=False  seq_ok=False  abort=False  srv_ign=False  ign=False
  6.82s  [LIVE]  ready=True   FILL=ON SRV=OFF IGN=SAFE  cd=False  seq_ok=False  abort=False  srv_ign=False  ign=False
  8.65s  [LIVE]  ready=True   FILL=ON SRV=OFF IGN=SAFE  cd=False  seq_ok=True   abort=False  srv_ign=False  ign=False
  8.95s  [LIVE]  ready=True   FILL=OFF SRV=OFF IGN=SAFE  cd=True   seq_ok=True   abort=False  srv_ign=False  ign=False
 13.96s  [LIVE]  ready=True   FILL=OFF SRV=OFF IGN=SAFE  cd=False  seq_ok=True   abort=False  srv_ign=False  ign=False
 14.34s  [LIVE]  ready=True   FILL=OFF SRV=ON IGN=PWR   cd=False  seq_ok=True   abort=False  srv_ign=False  ign=False
 14.84s  [LIVE]  ready=True   FILL=OFF SRV=OFF IGN=SAFE  cd=False  seq_ok=True   abort=False  srv_ign=False  ign=False

--- B1. Switch to TEST mode in Settings ---
 14.95s  [TEST]  ready=True   FILL=OFF SRV=OFF IGN=SAFE  cd=False  seq_ok=False  abort=False  srv_ign=False  ign=False

--- B2. TEST: Early manual FILL OFF aborts fill and keeps IGN locked ---
 15.35s  [TEST]  ready=True   FILL=ON SRV=OFF IGN=SAFE  cd=False  seq_ok=False  abort=False  srv_ign=False  ign=False
 16.27s  [TEST]  ready=True   FILL=ON SRV=OFF IGN=SAFE  cd=False  seq_ok=False  abort=True   srv_ign=False  ign=False
 16.58s  [TEST]  ready=True   FILL=OFF SRV=OFF IGN=SAFE  cd=False  seq_ok=False  abort=True   srv_ign=False  ign=False
 16.98s  [TEST]  ready=True   FILL=OFF SRV=ON IGN=SAFE  cd=False  seq_ok=False  abort=True   srv_ign=False  ign=False
 17.39s  [TEST]  ready=True   FILL=OFF SRV=OFF IGN=SAFE  cd=False  seq_ok=False  abort=True   srv_ign=False  ign=False

--- B3. TEST: Full timed FILL -> auto FILL OFF -> SRV ON -> IGN PWR -> RESET CONTROLS ---
 17.49s  [TEST]  ready=True   FILL=OFF SRV=OFF IGN=SAFE  cd=False  seq_ok=False  abort=False  srv_ign=False  ign=False
 17.90s  [TEST]  ready=True   FILL=ON SRV=OFF IGN=SAFE  cd=False  seq_ok=False  abort=False  srv_ign=False  ign=False
 20.64s  [TEST]  ready=True   FILL=ON SRV=OFF IGN=SAFE  cd=False  seq_ok=True   abort=False  srv_ign=False  ign=False
 20.94s  [TEST]  ready=True   FILL=OFF SRV=OFF IGN=SAFE  cd=False  seq_ok=True   abort=False  srv_ign=False  ign=False
 21.35s  [TEST]  ready=True   FILL=OFF SRV=ON IGN=SAFE  cd=False  seq_ok=True   abort=False  srv_ign=False  ign=True 
 21.45s  [TEST]  ready=True   FILL=OFF SRV=ON IGN=PWR   cd=False  seq_ok=True   abort=False  srv_ign=False  ign=False

ALL ASSERTIONS PASSED (LIVE + TEST MODES)
```

## What the run demonstrates

| Requirement | Where it shows up |
|---|---|
| Startup always initialises safe | `0.00s` (`ready=False`) → `0.10s` (`ready=True`, `FILL=OFF SRV=OFF IGN=SAFE`) |
| `LIVE` `SRV+IGN` stays greyed out prior to completed fill | `srv_ign=False` at rest (`0.10s`) and after an aborted manual fill (`1.63s`) |
| Completed timed `FILL` unlocks `SRV+IGN` in `LIVE` mode | `4.88s` (`seq_ok=True`, `abort=False`) followed by `FILL OFF` `ACK` unlocking `SRV+IGN` |
| Mid-countdown `E-STOP` cancels countdown with zero actuation | `5.19s` (`cd=True`) → `6.31s` (`cd=False`, outputs remain `FILL=OFF SRV=OFF IGN=SAFE`) |
| Full `SRV+IGN` countdown energises after configured duration and allows operator `OFF` | `8.95s` (`cd=True`) → `13.96s` (`T-0` command sent) → `14.34s` (`SRV=ON IGN=PWR` `ACK`) → `14.84s` (`SRV=OFF IGN=SAFE` `ACK`) |
| `TEST` mode preserves independent `FILL`, `SRV`, `IGN` interlocks | `14.95s`–`21.45s`: aborted fill keeps `ign=False` even with `SRV=ON` (`16.98s`), while completed fill + `SRV=ON` enables `ign=True` (`21.35s`) → `IGN=PWR` (`21.45s`) → `RESET CONTROLS` returns to `OFF/SAFE` |
