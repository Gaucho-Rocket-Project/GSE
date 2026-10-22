# GSE — Ground Support Equipment

Operator console, wire protocol and flight-hardware firmware for a two-board
rocket ground support station:

- **Ground board** — NUCLEO-H753ZI connected to the operator laptop over USB-C /
  ST-LINK Virtual COM Port, bridged to an SX1281 LoRa module over UART.
- **Vehicle board** — NUCLEO-H753ZI on the rocket with a paired SX1281 module,
  driving the fill valve servo (`FILL`), vehicle servos (`SRV`) and igniter
  relay (`IGN`).

The host application (`gse-app`) is a single dependency-free Rust binary. It
serves an embedded terminal-style browser console on port `8765`, speaks a
compact COBS + CRC-16 framed protocol over USB serial (or TCP against the
built-in simulator), and enforces deterministic launch interlocks across two
selectable operating modes (**`LIVE`** and **`TEST`**). Every safety rule is
independently re-checked in the `no_std` vehicle firmware — the vehicle never
trusts the ground station.

```
┌──────────────────────┐  USB CDC   ┌───────────────┐  UART   ┌──────────┐
│  gse-app (laptop)    │ 115200 bd  │ ground board  │ 9600 bd │ SX1281   │
│  HTTP console :8765  │◄──────────►│ NUCLEO-H753ZI │◄───────►│ (LoRa)   │
└──────────────────────┘            └───────────────┘         └────┬─────┘
                                                                   │ air
                                                              ┌────┴─────┐
                                                              │ SX1281   │
                                                              │ vehicle  │
                                                              │ H753ZI   │
                                                              └────┬─────┘
                                                FILL servo ◄───────┤
                                                SRV servo  ◄───────┤
                                                IGN relay  ◄───────┘
```

## Features

- **Terminal-style web operator console** served locally (`127.0.0.1:8765` by
  default) with per-session token auth, live Server-Sent Events (`/api/events`),
  and zero external/CDN assets.
- **Selectable `LIVE` and `TEST` operating modes** in **Settings** (or by
  clicking the **`SETUP`** status pill in the header):
  - **`LIVE` mode (`op_mode: "live"`, default)** — streamlined 2-button launch
    interface (**`FILL[ON/OFF]`** + combined **`SRV+IGN[ON/OFF]`**) with a
    configurable on-screen launch countdown (default `5 s`).
  - **`TEST` mode (`op_mode: "test"`)** — 3-button checkout interface
    (**`FILL[ON/OFF]`**, **`SRV[ON/OFF]`**, **`IGN[PWR/SAFE]`**) for testing
    vehicle channels independently with full safety interlocks.
- **Configurable sequence timers** in **Settings** (`1..=600 s`):
  - **Fill duration** (`fill_seconds`, default `60 s`) — auto-closes `FILL` at
    expiry and marks the fill sequence complete.
  - **`SRV+IGN` launch countdown** (`countdown_seconds`, default `5 s`) —
    displayed live on screen before `SRV+IGN` energises in `LIVE` mode.
- **Rebindable keyboard shortcuts** — defaults:
  - `F` — `FILL` toggle (`ON` / `OFF`)
  - `L` — `SRV+IGN` toggle (`LIVE` mode)
  - `S` — `SRV` toggle (`TEST` mode)
  - `I` — `IGN` toggle (`TEST` mode)
  - `E` — `ALL SAFE · E-STOP`
  - `Shift+R` — `Reset controls` (forces `OFF/SAFE` and clears latches)
  - `R` — `Reset fill seq`
  - `Shift+C` — `Clear faults`
- **Live telemetry & system monitor rail** — ground/vehicle link status, RTT,
  firmware version, device UID, safety-gate status, decoded vehicle flags,
  latched faults, and frame counters.
- **Session logging** — real-time filterable log stream in the UI plus
  append-only `session-<timestamp>.log` on disk and one-click CSV download.
- **Cross-platform native serial** — Win32 Comm API on Windows and POSIX
  `termios`/`poll` on Linux and macOS, with automatic ground-board port
  discovery (`port: "auto"`).
- **Hardware-accurate simulator (`gse-sim`)** — executes the exact `no_std`
  ground-bridge and vehicle state machines over TCP with configurable RF
  latency, packet drops and bit errors.

## Operating modes (`LIVE` vs `TEST`)

Operators can switch between **`LIVE`** and **`TEST`** in **Settings → Operating
Mode** whenever all vehicle outputs are `OFF/SAFE` and no launch countdown or
activating command is in progress.

| Aspect | `LIVE` Mode (`op_mode: "live"`, default) | `TEST` Mode (`op_mode: "test"`) |
|---|---|---|
| Console controls | `FILL[ON/OFF]` + `SRV+IGN[ON/OFF]` (2 buttons) | `FILL[ON/OFF]`, `SRV[ON/OFF]`, `IGN[PWR/SAFE]` (3 buttons) |
| Fill sequence | `FILL ON` → auto `FILL OFF` after `fill_seconds` (default `60 s`) → `FILL_SEQ_COMPLETE` | Identical (`FILL ON` → auto `FILL OFF` after `fill_seconds` → `FILL_SEQ_COMPLETE`) |
| Pre-fill lock | `SRV+IGN` defaults to `OFF/SAFE` and is greyed out until `FILL` completes without abort | `SRV` can be toggled when `FILL` is `OFF`; `IGN PWR` stays greyed out until `FILL` completes **and** `SRV` is `ON` |
| Firing sequence | Pressing `SRV+IGN` runs an on-screen countdown (`countdown_seconds`, default `5 s`), holding outputs `OFF/SAFE` until `T-0.0 s`, then sends `Control::SrvIgn` (`ON`) → `ON/PWR` | Operator enables `SRV ON`, then `IGN PWR` (with optional confirmation dialog) |
| Safe shutoff | Pressing `SRV+IGN` while `ON/PWR` commands `Control::SrvIgn` (`OFF`) → safes `IGN` first, then `SRV` → `OFF/SAFE` | Operator commands `IGN SAFE`, then `SRV OFF` (`SRV OFF` is blocked while `IGN` is `PWR`) |
| Countdown abort | Pressing `E-STOP` (`E`), `Reset controls` (`Shift+R`), or losing the link immediately cancels the countdown with zero output actuation | N/A |

- **LIVE sequence:** `FILL ON → (fill_seconds timer) → auto FILL OFF → press SRV+IGN → (countdown_seconds on-screen countdown) → SRV+IGN ON/PWR → press SRV+IGN → SRV+IGN OFF/SAFE`
- **TEST sequence:** `FILL ON → (fill_seconds timer) → auto FILL OFF → SRV ON → IGN PWR → IGN SAFE → SRV OFF`

## Safety interlocks

Every gate is evaluated in `gse-app` (greying out buttons with human-readable
reasons and rejecting API/keyboard requests) **and** re-evaluated independently
on the STM32H753ZI vehicle board. There is **no override** in either layer.

| # | Rule | Where enforced |
|---|------|----------------|
| 1 | `FILL` and `SRV` can never be `ON` simultaneously | `safety.rs`, `fsm.rs`, `vehicle.rs` |
| 2 | `FILL` closes automatically after `fill_seconds` (default `60 s`, `1..=600 s`) and sets `FILL_SEQ_COMPLETE`; closing `FILL` early sets `FILL_SEQ_ABORTED` | `engine.rs` auto-off on board timer + independent 2 s vehicle hardware backstop in `vehicle.rs` |
| 3 | `IGN PWR` (`TEST`) and `SRV+IGN ON` (`LIVE`) are impossible unless a full timed fill completed without abort | `safety.rs` + `fsm.rs` + `vehicle.rs` |
| 4 | In `TEST` mode, `IGN PWR` requires `SRV = ON`, and `SRV OFF` is refused while `IGN = PWR` | `safety.rs` + `fsm.rs` + `vehicle.rs` |
| 5 | In `LIVE` mode, `SRV+IGN` starts `OFF/SAFE`, runs a host countdown (`countdown_seconds`, default `5 s`) with continuous safety re-checking, and safes `IGN` before `SRV` on `OFF` | `safety.rs` + `engine.rs` + `fsm.rs` + `vehicle.rs` |
| 6 | Operating mode (`LIVE` ↔ `TEST`) cannot be changed while any output is active, targeted active, or counting down | `http.rs` (`HTTP 400`) + `engine.rs` |
| 7 | Losing the RF link forces all outputs `OFF/SAFE` after 5 s and latches `INHIBITED` | `vehicle.rs` (`check_link` + `LINK_LOSS_SAFE`) |
| 8 | `ALL SAFE · E-STOP` always wins immediately, cancelling any active countdown or in-flight command and latching `ESTOP` | `engine.rs` + `vehicle.rs` |
| 9 | Hardware watchdog timeout (~4 s) or MCU fault resets/de-energises every pin | STM32H753 `IWDG1` + `panic` safe-park handler |
| 10 | Every session and reconnect starts in `INIT` with all controls locked until the vehicle confirms `FILL=OFF SRV=OFF IGN=SAFE` | `engine.rs` startup safe handshake |
| 11 | `Reset controls` always sends `SAFE_ALL` followed by `RESET_FILL`, returning the system to initial `OFF/SAFE` with latches cleared | `engine.rs` + `fsm.rs` + `vehicle.rs` |
| 12 | Shutting down `gse-app` commands `ALL SAFE` first and waits up to 2.5 s for the vehicle `ACK` before exiting | `engine.rs` (`begin_shutdown`) |

## Deterministic control state machine

`FILL`, `SRV` and `IGN` are governed by a single pure transition table in
[`crates/gse-proto/src/fsm.rs`](crates/gse-proto/src/fsm.rs), shared verbatim by
`gse-app`, `gse-sim` and both STM32 firmware images.

Confirmed states are `FILL OFF|ON`, `SRV OFF|ON` and `IGN SAFE|PWR`, starting at
`FILL OFF · SRV OFF · IGN SAFE`. Every event returns one of three outcomes:

| Outcome | Meaning | Console | Vehicle |
|---|---|---|---|
| `Allowed(next)` | legal transition to `next` | button enabled | actuates output, waits settle time, sends `ACK` (`Ok`) |
| `NoOp(same)` | already in target state | button disabled with `"already …"` | sends `ACK` (`Ok`), no actuation |
| `Blocked(reason)` | safety gate refuses move | button disabled with blocking reason | sends `ACK` (`RejectedSafety`, `reason`) |

Confirmed state in the console updates **only when the vehicle `ACK` lands**.
While a command is in flight, the console displays the target after `→` without
changing confirmed state badges.

| Event | Preconditions | Result |
|---|---|---|
| `FILL ON` | `FILL=OFF, SRV=OFF, IGN=SAFE` | `FILL=ON` (`Blocked` if `SRV=ON` or `IGN=PWR`) |
| `FILL OFF` | `FILL=ON` | `FILL=OFF` (`NoOp` if already `OFF`) |
| `SRV ON` (`TEST`) | `FILL=OFF, IGN=SAFE` | `SRV=ON` (`Blocked` while `FILL=ON` or `IGN=PWR`) |
| `SRV OFF` (`TEST`) | `SRV=ON, IGN=SAFE` | `SRV=OFF` (`Blocked` while `IGN=PWR`) |
| `IGN PWR` (`TEST`) | `FILL=OFF, SRV=ON`, `fill_seq_complete`, `!fill_seq_aborted`, `!inhibited`, `!estop` | `IGN=PWR` (`Blocked` otherwise) |
| `IGN SAFE` (`TEST`) | `IGN=PWR` | `IGN=SAFE` (`NoOp` if already `SAFE`) |
| `SRV+IGN ON` (`LIVE`) | `FILL=OFF`, `fill_seq_complete`, `!fill_seq_aborted`, `!inhibited`, `!estop` | `SRV=ON, IGN=PWR` (`NoOp` if already `ON/PWR`, `Blocked` otherwise) |
| `SRV+IGN OFF` (`LIVE`) | `SRV=ON` or `IGN=PWR` | `SRV=OFF, IGN=SAFE` (`NoOp` if already `OFF/SAFE`) |
| `SAFE ALL` / `RESET` | any state | `FILL=OFF, SRV=OFF, IGN=SAFE` |

## Quick start with the simulator

```bash
cargo build --workspace
cargo run -p gse-app -- --sim   # starts gse-sim automatically; console at http://127.0.0.1:8765/
```

Use `--no-browser` on headless machines and `--host 0.0.0.0` when serving to a
remote browser or container preview:

```bash
./target/debug/gse-app --sim --no-browser --host 0.0.0.0 --http-port 8765
```

To simulate degraded RF conditions (latency, packet drops and bit errors):

```bash
cargo run -p gse-sim -- --latency 60 --noise   # terminal 1
cargo run -p gse-app -- --sim                  # terminal 2
```

## Running against real hardware

1. Flash `gse-ground` and `gse-vehicle` (see [Firmware](#firmware) and
   [docs/HARDWARE.md](docs/HARDWARE.md)).
2. Plug the ground NUCLEO-H753ZI USB cable into the laptop.
3. Launch the console:

```bash
gse-app                        # auto-detects the ground board on any serial port
gse-app --port /dev/ttyACM0    # Linux explicit port
gse-app --port COM5 --verbose  # Windows
gse-app --port /dev/tty.usbmodem14201
```

Linux users need serial group permission once:

```bash
sudo usermod -aG dialout "$USER"   # log out and back in
```

### Command line

```text
gse-app [OPTIONS]
  --sim                 use the built-in TCP simulator (tcp:127.0.0.1:8787)
  --port <PORT>         serial port, tcp:host:port, or 'auto' (default)
  --baud <RATE>         serial baud rate (default 115200)
  --config <PATH>       path to configuration JSON file
  --host <ADDR>         HTTP bind address (default 127.0.0.1)
  --http-port <PORT>    HTTP console port (default 8765)
  --no-browser          do not open a browser window on startup
  --open                force opening a browser even in headless environments
  --verbose             print protocol diagnostics on stderr
  -h, --help / -V, --version
```

### Configuration

Persisted atomically as pretty-printed JSON on every change in **Settings**:

| OS | Default path |
|----|--------------|
| Linux | `~/.config/gse/config.json` (honours `XDG_CONFIG_HOME`) |
| macOS | `~/Library/Application Support/GSE/config.json` |
| Windows | `%APPDATA%\GSE\config.json` |

```json
{
  "op_mode": "live",
  "fill_seconds": 60,
  "countdown_seconds": 5,
  "port": "auto",
  "baud": 115200,
  "confirm_ign": true,
  "sound": true,
  "host": "127.0.0.1",
  "http_port": 8765,
  "shortcuts": {
    "fill_toggle": "F",
    "srv_ign_toggle": "L",
    "srv_toggle": "S",
    "ign_toggle": "I",
    "estop": "E",
    "reset_controls": "Shift+R",
    "reset_fill": "R",
    "clear_faults": "Shift+C"
  }
}
```

Session logs are written alongside state files under `logs/session-<timestamp>.log`.

## Using the console

- **Top bar**: shows live `GROUND`, `VEHICLE`, `LINK` RTT, `SETUP` (`LIVE` or
  `TEST`, clickable to open **Settings**), and `TARGET` (`SIMULATOR` or
  `HARDWARE`) pills, plus `connect`, `disconnect` and `settings`.
- **`SYS // MONITOR` left rail**: displays process health (`setup`, `gse-app`,
  `gse-sim`, `sx1281-rf`), live interlock gates (`init safe`, `fill ↔ srv`,
  `fill seq`, `srv+ign` / `ign gate`), and bus dispatch state.
- **Control toggles**: horizontal terminal switches (`FILL[ON/OFF]`,
  `SRV+IGN[ON/OFF]` in `LIVE`; `FILL[ON/OFF]`, `SRV[ON/OFF]`, `IGN[PWR/SAFE]`
  in `TEST`). Whenever a gate is closed, the button is disabled and displays the
  exact reason underneath.
- **On-screen countdown (`LIVE` mode)**: pressing `SRV+IGN` after a completed
  fill reveals `#countdown-banner` (`T - X.X s` + progress bar) counting down to
  `0.0 s` before energizing `SRV+IGN`.
- **Settings modal**: configures **Operating Mode** (`LIVE` / `TEST`), serial
  port & baud rate, **fill duration (seconds)** (`1..=600`), **`srv+ign`
  countdown (seconds)** (`1..=600`), `ask before IGN PWR` (`TEST` mode),
  `audible alerts`, and click-to-record keyboard shortcuts.

## HTTP & SSE API

All endpoints require the per-session `X-GSE-Token` header (or `?token=` query
parameter on `GET /api/events` and `GET /api/log.csv`), except `GET /`, static
assets, and `GET /api/health`.

| Endpoint | Method | Purpose |
|---|---|---|
| `/api/health` | `GET` | Liveness check (`{"ok":true,"subscribers":N}`) |
| `/api/state` | `GET` | Full JSON snapshot (`config`, `ground`, `vehicle`, `controls`, `fill`, `countdown`, `actions`, `stats`) |
| `/api/events` | `GET` | SSE stream emitting `state` and `log` events |
| `/api/command` | `POST` | Dispatch action (`fill_on`, `fill_off`, `srv_on`, `srv_off`, `ign_pwr`, `ign_safe`, `srv_ign_on`, `srv_ign_off`, `reset_controls`, `reset_fill`, `clear_faults`, `estop`) |
| `/api/config` | `POST` | Patch configuration (`op_mode`, `fill_seconds`, `countdown_seconds`, `port`, `baud`, `confirm_ign`, `sound`) |
| `/api/shortcuts` | `POST` | Rebind `{action, combo}` or reset `{reset: true}` |
| `/api/ports` | `GET` | Enumerate available serial ports |
| `/api/connect` / `/api/disconnect` | `POST` | Open/rescan or close the serial/TCP link |
| `/api/log` / `/api/log.csv` / `/api/log/clear` | `GET` / `POST` | Query recent log entries, export CSV, or clear in-memory log |
| `/api/shutdown` | `POST` | Command `ALL SAFE`, await `ACK`, and stop `gse-app` |

## Firmware

`firmware/stm32h753` contains the `no_std` STM32H753ZI package with two
binaries:

- `gse-ground` — USB CDC (`USART3`, `PD8`/`PD9`) ↔ SX1281 (`USART6`,
  `PG14`/`PG9`) transparent frame bridge + `LINKSTAT` diagnostics.
- `gse-vehicle` — SX1281 transceiver + `TIM1` 50 Hz servo PWM (`PE9` `FILL`,
  `PE11` `SRV`), push-pull `IGN` output (`PE13`), 250 ms telemetry, 5 s RF
  link-loss failsafe, and 4 s hardware watchdog (`IWDG1`).

```bash
rustup target add thumbv7em-none-eabihf
cd firmware/stm32h753
cargo build --release --bin gse-ground
cargo build --release --bin gse-vehicle
# flash via probe-rs:
cargo run --release --bin gse-vehicle
```

See [docs/HARDWARE.md](docs/HARDWARE.md) for wiring, fail-safe pull-downs, and
the bench bring-up procedure.

## Project layout

```text
crates/gse-proto      no_std wire framing (COBS, CRC-16) + shared control FSM
crates/gse-fw-core    no_std ground bridge & vehicle state/timer execution
crates/gse-app        host service, safety gates, HTTP/SSE server & embedded web UI
crates/gse-sim        two-board TCP RF link simulator using real firmware cores
crates/gse-fw-check   host unit tests for STM32 register & ring-buffer helpers
firmware/stm32h753    NUCLEO-H753ZI hardware abstraction & firmware binaries
scripts/              end-to-end smoke test (smoke.sh / smoke.py) & SSE state trace
docs/                 protocol spec, hardware/wiring guide & recorded state trace
```

## Testing

```bash
cargo test --workspace --lib --bins --tests
cargo clippy --workspace --all-targets -- -D warnings
cargo fmt --all -- --check
```

End-to-end verification against compiled `gse-sim` + `gse-app` binaries:

```bash
cargo build --workspace
./scripts/smoke.sh                           # 51 end-to-end checks (LIVE + TEST modes, custom countdown, E-STOP, reconnect & shutdown ALL SAFE)
python3 scripts/state_trace.py               # full SSE transition trace across LIVE & TEST sequences
```

See [docs/state-machine-trace.md](docs/state-machine-trace.md) for a recorded
trace and [docs/PROTOCOL.md](docs/PROTOCOL.md) for the byte-level specification.

## Troubleshooting

| Symptom | Check |
|---|---|
| `No serial ports found` | Use a data-capable USB cable, power the board, verify ST-LINK VCP driver |
| `No ground board responded …` | Confirm `gse-ground` is flashed, baud is `115200`, user is in `dialout` on Linux |
| Ground `online`, vehicle stays `pinging` | Check `PG9`/`PG14` ↔ SX1281 `TX`/`RX`, `9600` Bd transparent UART config, antennas |
| `SRV+IGN` greyed out in `LIVE` mode | Run a full timed `FILL` cycle to completion (closing `FILL` early aborts the fill latch; press `Reset fill seq` first) |
| Mode switch refused in Settings | Return all controls to `OFF/SAFE` and cancel any active countdown before switching `LIVE` ↔ `TEST` |
| Shortcut binding rejected | Key combination is already bound to another action (named in the error toast) |

## License

MIT OR Apache-2.0.
