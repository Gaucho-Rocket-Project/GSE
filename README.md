# GSE — Ground Support Equipment

Operator console, wire protocol and flight-hardware firmware for a two-board
rocket ground support station:

- **Ground board** — STMicroelectronics **`NUCLEO-H753ZI`** ([DigiKey `21348937`](https://www.digikey.com/en/products/detail/stmicroelectronics/NUCLEO-H753ZI/21348937)) connected to the operator laptop over USB / ST-LINK Virtual COM Port (`USART3`, configurable **`9600–460800 Bd`**) and bridged to either:
  - **DX-SMART `DX-LR42-24T12D`** 2.4 GHz Semtech SX1281 UART transceiver ([Amazon `B0H8NR5QPY`](https://www.amazon.com/dp/B0H8NR5QPY?plpRedirect=mhFallback&ref=clp_cat_related_p_0)) over `USART6` (configurable **`2400–115200 Bd`**), or
  - **CDEBYTE [`E28-2G4M27S`](https://www.cdebyte.com/products/E28-2G4M27S/4#Downloads)** 2.4 GHz Semtech `SX1280`/`SX1281` **`500 mW` (`+27 dBm`)** SPI + PA/LNA LoRa transceiver over **`SPI1` (`4 MHz`)**.
- **Vehicle board** — paired **`NUCLEO-H753ZI`** + **`DX-LR42-24T12D`** or **`E28-2G4M27S`** on the rocket driving the fill valve servo (`FILL`), vehicle servos (`SRV`) and igniter relay (`IGN`).

The host application (`gse-app`) is a single dependency-free Rust binary. It
serves an embedded terminal-style browser console on port `8765`, speaks a
compact COBS + CRC-16 framed protocol over USB serial (or TCP against the
built-in simulator), and enforces deterministic launch interlocks across two
selectable operating modes (**`LIVE`** and **`TEST`**). Every safety rule is
independently re-checked in the `no_std` vehicle firmware — the vehicle never
trusts the ground station.

<img width="8192" height="4465" alt="image" src="https://github.com/user-attachments/assets/0d05bd30-e619-4844-92db-64ce1e42b475" />

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
- **Configurable sequence timers & pause/resume/abort controls** in **Settings** (`1..=600 s`):
  - **Fill duration** (`fill_seconds`, default `60 s`) — auto-closes `FILL` at
    expiry and marks the fill sequence complete. Turning `FILL -> OFF` prior to
    expiry **pauses** the fill timer (`fill_remaining_ms` preserved), unlocks
    `SRV` (`TEST`) / `SRV+IGN` (`LIVE`), and lets subsequent `FILL -> ON`
    **resume** from the paused remaining time.
  - **Ignition countdown** (`countdown_seconds`, default `5 s`) — displayed live
    on screen before `SRV+IGN -> ON` (`LIVE`) or `IGN -> PWR` (`TEST`), and can
    be aborted mid-countdown at any time (cancelling ignition immediately,
    returning `SRV -> OFF` if powered, and keeping `FILL` progress intact).
- **Live dropdown selectors & flash persistence for baud rates and radio modules** in **Settings**:
  - **USB CDC baud rate** (`USART3` host ↔ ground board: `9600`, `19200`, `38400`, `57600`, `115200`, `230400`, `460800` baud).
  - **UART baud rate** (`USART6` MCU ↔ UART radio: `2400`, `4800`, `9600`, `19200`, `38400`, `57600`, `115200` baud — live `AT+BAUDx` reconfiguration).
  - **Radio module selector** (**`DX-LR42-24T12D`** UART LoRa vs **`E28-2G4M27S`** `500 mW` `+27 dBm` `SPI1` LoRa) — non-destructively probes `SX1280`/`SX1281` over `SPI1`, orchestrates an atomic two-phase `Vehicle → Ground → Host` handover with zero lost ACKs, and persists settings to STM32 Bank 2 Sector 7 internal flash (`0x081E_0000`) across power cycles.
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
| Fill sequence & pause/resume | `FILL ON` → auto `FILL OFF` after `fill_seconds` (`FILL_SEQ_COMPLETE`), or manual `FILL OFF` prior to expiry pauses `fill_remaining_ms` (`FILL_PAUSED`) so `FILL ON` resumes | Identical (`FILL ON` → auto `FILL OFF`, or manual `FILL OFF` pauses timer and `FILL ON` resumes) |
| Pre-fill lock | `SRV+IGN` starts `OFF/SAFE` and unlocks once `FILL` completes or is paused `OFF` | `SRV` and `IGN` start locked and `SRV` unlocks once `FILL` completes or is paused `OFF`; `IGN` additionally requires `SRV = ON` |
| Firing sequence | Pressing `SRV+IGN` runs an on-screen countdown (`countdown_seconds`, default `5 s`), then holds at `T-0.0 s` with outputs `OFF/SAFE`; operator must press `L` to command `Control::SrvIgn` (`ON`) → `ON/PWR` | Operator enables `SRV ON`, then pressing `IGN` (optional confirm dialog) runs the countdown and holds at `T-0.0 s`; operator must press `L` to command `Control::Ign` (`ON`) → `PWR` |
| Safe shutoff | Pressing `SRV+IGN` while `ON/PWR` commands `Control::SrvIgn` (`OFF`) → safes `IGN` first, then `SRV` → `OFF/SAFE` | Operator commands `IGN SAFE`, then `SRV OFF` (`SRV OFF` is blocked while `IGN` is `PWR`) |
| Countdown abort | Pressing `Shift+L`, clicking `ABORT` / the safe toggle, or posting `abort_countdown` cancels the countdown with outputs `OFF/SAFE` and preserves `FILL` state | Pressing `Shift+L`, clicking `ABORT` / `IGN SAFE`, or posting `abort_countdown` cancels `IGN`, commands `SRV -> OFF`, and preserves `FILL` state |

- **LIVE sequence:** `FILL ON → (auto FILL OFF, or early FILL OFF pause / FILL ON resume) → press SRV+IGN → (abortable countdown) → HOLD at 0.0 s → manually press L to trigger SRV+IGN ON/PWR → SRV+IGN OFF/SAFE`
- **TEST sequence:** `FILL ON → (auto FILL OFF, or early FILL OFF pause / FILL ON resume) → SRV ON → press IGN → (abortable countdown) → HOLD at 0.0 s → manually press L to trigger IGN PWR → IGN SAFE → SRV OFF`
- **No automatic ignition:** countdown completion never energises IGN; it stays at `0.0 s` with outputs `OFF/SAFE` until the operator explicitly presses `L`. `Shift+L` cancels the active countdown; the abort button and safe-toggle also remain available.

## Safety interlocks

Every gate is evaluated in `gse-app` (greying out buttons with human-readable
reasons and rejecting API/keyboard requests) **and** re-evaluated independently
on the STM32H753ZI vehicle board. There is **no override** in either layer.

| # | Rule | Where enforced |
|---|------|----------------|
| 1 | `FILL` and `SRV` can never be `ON` simultaneously | `safety.rs`, `fsm.rs`, `vehicle.rs` |
| 2 | `FILL` closes automatically after `fill_seconds` (`FILL_SEQ_COMPLETE`), while manual `FILL OFF` before expiry pauses `fill_remaining_ms` (`FILL_PAUSED`) for later resume; only uncommanded fault closes (`E-STOP` / RF timeout mid-fill) set `FILL_SEQ_ABORTED` | `engine.rs` auto-off on board timer + vehicle `close_fill` & 2 s hardware backstop in `vehicle.rs` |
| 3 | `SRV ON` (`TEST`), `IGN PWR` (`TEST`), and `SRV+IGN ON` (`LIVE`) stay locked until `FILL` has completed or been paused `OFF` without an uncommanded abort | `safety.rs` + `fsm.rs` + `vehicle.rs` |
| 4 | In `TEST` mode, `IGN PWR` requires `SRV = ON`, runs an abortable `countdown_seconds` countdown, and `SRV OFF` is refused while `IGN = PWR` | `safety.rs` + `engine.rs` + `fsm.rs` + `vehicle.rs` |
| 5 | Aborting an active countdown (`IGN` or `SRV+IGN`) immediately cancels ignition, closes `SRV` if powered, and keeps `FILL` progress intact | `safety.rs` + `engine.rs` |
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
| `FILL ON` | `FILL=OFF, SRV=OFF, IGN=SAFE` | `FILL=ON` (resumes saved `fill_remaining_ms` if `FILL_PAUSED`, else starts `fill_seconds`) |
| `FILL OFF` | `FILL=ON` | `FILL=OFF` (pauses `fill_remaining_ms` + unlocks downstream channels if closed before expiry) |
| `SRV ON` (`TEST`) | `FILL=OFF, IGN=SAFE`, `(fill_seq_complete \|\| fill_paused)`, `!fill_seq_aborted`, `!inhibited`, `!estop` | `SRV=ON` (`Blocked` otherwise) |
| `SRV OFF` (`TEST`) | `SRV=ON, IGN=SAFE` | `SRV=OFF` (`Blocked` while `IGN=PWR`) |
| `IGN PWR` (`TEST`) | `FILL=OFF, SRV=ON`, `(fill_seq_complete \|\| fill_paused)`, `!fill_seq_aborted`, `!inhibited`, `!estop` | `IGN=PWR` (`Blocked` otherwise) |
| `IGN SAFE` (`TEST`) | `IGN=PWR` | `IGN=SAFE` (`NoOp` if already `SAFE`) |
| `SRV+IGN ON` (`LIVE`) | `FILL=OFF`, `(fill_seq_complete \|\| fill_paused)`, `!fill_seq_aborted`, `!inhibited`, `!estop` | `SRV=ON, IGN=PWR` (`NoOp` if already `ON/PWR`, `Blocked` otherwise) |
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
  "uart_baud": 9600,
  "radio": "dx-lr42-24t12d",
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
  `gse-sim`, `dx-lr42-rf`), live interlock gates (`init safe`, `fill ↔ srv`,
  `fill seq`, `srv+ign` / `ign gate`), and bus dispatch state.
- **Control toggles**: horizontal terminal switches (`FILL[ON/OFF]`,
  `SRV+IGN[ON/OFF]` in `LIVE`; `FILL[ON/OFF]`, `SRV[ON/OFF]`, `IGN[PWR/SAFE]`
  in `TEST`). Whenever a gate is closed, the button is disabled and displays the
  exact reason underneath.
- **On-screen countdown & abort (`LIVE` & `TEST` modes)**: pressing `SRV+IGN`
  (`LIVE`) or `IGN` (`TEST`) displays `#countdown-banner` (`T - X.X s`, progress
  bar, and **`ABORT`** button) counting down to `0.0 s`. Aborting during the
  countdown immediately cancels ignition, closes `SRV` if powered, and keeps
  `FILL` progress intact.
- **Settings modal**: configures **Operating Mode** (`LIVE` / `TEST`), serial
  port & baud rate, **fill duration (seconds)** (`1..=600`), **ignition
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
| `/api/command` | `POST` | Dispatch action (`fill_on`, `fill_off`, `srv_on`, `srv_off`, `ign_pwr`, `ign_safe`, `srv_ign_on`, `srv_ign_off`, `abort_countdown`, `reset_controls`, `reset_fill`, `clear_faults`, `estop`) |
| `/api/config` | `POST` | Patch configuration (`op_mode`, `fill_seconds`, `countdown_seconds`, `port`, `baud`, `confirm_ign`, `sound`) |
| `/api/shortcuts` | `POST` | Rebind `{action, combo}` or reset `{reset: true}` |
| `/api/ports` | `GET` | Enumerate available serial ports |
| `/api/connect` / `/api/disconnect` | `POST` | Open/rescan or close the serial/TCP link |
| `/api/log` / `/api/log.csv` / `/api/log/clear` | `GET` / `POST` | Query recent log entries, export CSV, or clear in-memory log |
| `/api/shutdown` | `POST` | Command `ALL SAFE`, await `ACK`, and stop `gse-app` |

## Firmware

`firmware/stm32h753` contains the `no_std` STM32H753ZI package with two
binaries:

- `gse-ground` — USB CDC (`USART3`, `PD8`/`PD9`) ↔ `DX-LR42-24T12D` (`USART6`,
  `PG14`/`PG9` + `PG10` `AUX` + `PG12`/`PG13` `M0`/`M1`) transparent frame
  bridge + AT self-recovery + `LINKSTAT` diagnostics.
- `gse-vehicle` — `DX-LR42-24T12D` transceiver + `TIM1` 50 Hz servo PWM (`PE9`
  `FILL`, `PE11` `SRV`), push-pull `IGN` output (`PE13`), 250 ms telemetry,
  5 s RF link-loss failsafe, and 4 s hardware watchdog (`IWDG1`).

```bash
rustup target add thumbv7em-none-eabihf
cd firmware/stm32h753
cargo build --release --bin gse-ground
cargo build --release --bin gse-vehicle
# flash via probe-rs:
cargo run --release --bin gse-vehicle
```

Two factory-fresh `DX-LR42-24T12D` modules work out of the box (`9600 8N1`,
`2424 MHz`, `LEVEL 3` `10 kbps`, `+13 dBm` on `5V`). To inspect or provision a
custom channel / `3.7 km` long-range `LEVEL 1` preset over a `3.3 V` USB-UART
adapter:

```bash
python3 scripts/configure_dx_lr42.py --port /dev/ttyUSB0 --inspect
python3 scripts/configure_dx_lr42.py --port /dev/ttyUSB0 --preset long-range --channel 2440MHz --key 27182 --lock-pins
```

See [docs/HARDWARE.md](docs/HARDWARE.md) for the full 7-pin wiring table, `5.0 V`
`VCC` / `3.3 V` CMOS rules, fail-safe pull-downs, and bench bring-up checklist.

## Project layout

```text
crates/gse-proto      no_std wire framing (COBS, CRC-16), DX-LR42 constants & shared FSM
crates/gse-fw-core    no_std ground bridge, vehicle FSM & DX-LR42 UART stream adapter
crates/gse-app        host service, safety gates, HTTP/SSE server & embedded web UI
crates/gse-sim        two-board TCP RF link simulator using real firmware cores
crates/gse-fw-check   host unit tests for STM32 register, ring-buffer & AUX helpers
firmware/stm32h753    NUCLEO-H753ZI hardware abstraction & firmware binaries
scripts/              smoke tests, SSE state trace & DX-LR42 AT provisioning helper
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
| Ground `online`, vehicle stays `pinging` | Check `VCC` (`5V`, `CN8` pin 9), `PG9`/`PG14` ↔ `DX-LR42-24T12D` Pin 4 `TX` / Pin 3 `RX` (`3.3 V` TTL), matching `AT+CHANNEL` / `AT+LEVEL` / `AT+KEY`, `M0`/`M1` (`PG12`/`PG13`), and `2.4 GHz` antennas |
| `SRV+IGN` (`LIVE`) or `SRV` (`TEST`) greyed out | Run `FILL` and let it auto-complete or click `FILL` again to pause it `OFF` (if an earlier fault set `FILL_SEQ_ABORTED`, press `Reset fill seq` first) |
| Mode switch refused in Settings | Return all controls to `OFF/SAFE` and cancel any active countdown before switching `LIVE` ↔ `TEST` |
| Shortcut binding rejected | Key combination is already bound to another action (named in the error toast) |

## License

MIT OR Apache-2.0.
