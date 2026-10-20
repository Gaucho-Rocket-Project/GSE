# Control-state trace

`scripts/state_trace.py` drives a running console through one complete, safe
scenario and prints every distinct machine state it observes, so the
deterministic behaviour can be checked against the specification rather than
taken on trust. It only ever talks to the HTTP API with the same token and the
same commands the operator uses, so it exercises exactly the production path:
`/api/command` → link → vehicle `fsm` → ACK → `/api/state`.

## Running it

```bash
./target/debug/gse-sim --listen 127.0.0.1:8787 --latency 10 &
./target/debug/gse-app --port tcp:127.0.0.1:8787 --http-port 8765 \
    --no-browser --config /tmp/trace.json &

TOKEN=$(curl -s http://127.0.0.1:8765/ \
    | sed -n 's/.*name="gse-token" content="\([a-f0-9]*\)".*/\1/p')
scripts/state_trace.py http://127.0.0.1:8765 "$TOKEN" [--shutdown]
```

The script sets the fill time to 3 s so the whole scenario takes about five
seconds, then:

1. waits until both boards report `online` and the controls are `ready`,
2. commands **FILL ON** and waits for the ACK,
3. waits for the timed auto-close and for the fill sequence to be marked
   complete,
4. checks that `ign_pwr` is still refused,
5. commands **SRV ON** and checks that `ign_pwr` becomes enabled,
6. commands **RESET CONTROLS** and checks that every control is back at
   OFF/SAFE and that the fill-sequence and E-STOP latches are cleared,
7. with `--shutdown`, requests shutdown and greps the session log for
   `ALL SAFE acknowledged`.

Every line is printed only when the state changes. `pending` is the moment a
command is sent and the target appears (the confirmed state is untouched),
`state` is a change in the confirmed state, `ack` is the ACK landing and the
pending target disappearing.

## Recorded run

Console `http://127.0.0.1:8798`, simulator with 10 ms radio latency,
`fill_seconds = 3`, `--shutdown`:

```text
GSE control-state trace
  + 0.004s  state    FILL OFF · SRV OFF · IGN SAFE
  initial  FILL=OFF SRV=OFF IGN=SAFE  (ready)
  action   FILL ON (3 s fill)
  + 0.019s  pending  FILL OFF · SRV OFF · IGN SAFE -> FILL ON  · SRV OFF · IGN SAFE  [pending fill_on]
  + 0.041s  state    FILL ON  · SRV OFF · IGN SAFE -> FILL ON  · SRV OFF · IGN SAFE  [pending fill_on]
  + 0.412s  ack      FILL ON  · SRV OFF · IGN SAFE
  + 3.417s  pending  FILL ON  · SRV OFF · IGN SAFE -> FILL OFF · SRV OFF · IGN SAFE  [pending fill_off]
  + 3.451s  state    FILL OFF · SRV OFF · IGN SAFE -> FILL OFF · SRV OFF · IGN SAFE  [pending fill_off]
  + 3.813s  ack      FILL OFF · SRV OFF · IGN SAFE
  gates    enabled: fill_on, srv_on, reset_controls, reset_fill, estop
  action   SRV ON
  + 3.814s  pending  FILL OFF · SRV OFF · IGN SAFE -> FILL OFF · SRV ON  · IGN SAFE  [pending srv_on]
  + 3.847s  state    FILL OFF · SRV ON  · IGN SAFE -> FILL OFF · SRV ON  · IGN SAFE  [pending srv_on]
  + 4.221s  ack      FILL OFF · SRV ON  · IGN SAFE
  gates    IGN PWR is now enabled (SRV ON + completed fill)
  action   RESET CONTROLS
  + 4.233s  pending  FILL OFF · SRV ON  · IGN SAFE -> FILL OFF · SRV OFF · IGN SAFE  [pending reset_controls]
  + 4.267s  state    FILL OFF · SRV OFF · IGN SAFE  [pending reset_fill]
  + 4.290s  ack      FILL OFF · SRV OFF · IGN SAFE
  final    FILL=OFF SRV=OFF IGN=SAFE  (latches cleared)
  shutdown -> ALL SAFE
  verified ALL SAFE was acknowledged before exit
TRACE OK
```

## What the run demonstrates

| Requirement | Where it shows up |
|---|---|
| everything starts OFF/SAFE | first line, `ready` after both boards answered the handshake |
| a control only changes after the ACK | `pending` line keeps `FILL OFF` confirmed with `-> FILL ON` as target; the confirmed `FILL ON` appears on the `ack` line ~370 ms later |
| FILL auto-closes at the configured time | second `pending`/`ack` pair at ~3.4 s, `fill_off` issued by the app, not the operator |
| IGN stays greyed until the fill sequence is complete *and* SRV is ON | `gates enabled:` after the fill omits `ign_pwr`; it appears only after the `srv_on` ACK |
| no unsafe transition is reachable | FILL/SRV were never both ON; the vehicle would reject the command with `RejectedSafety` even if the console were bypassed |
| reset returns everything to OFF/SAFE | `reset_controls` target drops SRV, and the final state is `FILL=OFF SRV=OFF IGN=SAFE` with `fill_seq_complete` and `estop` cleared |
| shutdown leaves every control OFF/SAFE | `shutdown -> ALL SAFE`, and the log line `ALL SAFE acknowledged - every control is OFF/SAFE` |

The same guarantees are covered automatically by `scripts/smoke.sh` (API level,
including rejected commands and the reconnect-forced-safe case) and by the unit
tests over `gse_proto::fsm` (every event from every reachable state).
