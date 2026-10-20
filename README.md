# GSE — Ground Support Equipment

Operator console and flight-hardware firmware for a two-board rocket ground
station:

- **Ground board** — NUCLEO-H753ZI connected to the laptop over USB-C →
  micro-USB, bridged to an SX1281 LoRa module over UART.
- **Vehicle board** — NUCLEO-H753ZI on the rocket, with the same SX1281 module,
  driving the fill valve, the vehicle servos and the igniter.

The host application is a single dependency-free Rust binary. It serves a
local operator console (HTML/JS embedded in the binary), speaks a compact
framed protocol over the serial port and enforces the launch interlocks. The
same interlocks are re-checked in the vehicle firmware — the vehicle never
trusts the ground station.

```
┌──────────────────────┐  USB CDC   ┌───────────────┐  UART   ┌──────────┐
│  gse-app (laptop)    │ 115200 bd  │ ground board  │ 9600 bd │ SX1281   │
│  HTTP console + API  │◄──────────►│ NUCLEO-H753ZI │◄───────►│ (LoRa)   │
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

- **Operator console** in the browser, served from `127.0.0.1` with a session
  token; no cloud, no external assets.
- **Rebindable keyboard shortcuts** (defaults `F` fill, `S` servos, `I` igniter,
  `E` all-safe, `Shift+R` reset controls, `R` reset fill sequence, `Shift+C`
  clear faults). Bindings are captured in Settings and persisted.
- **Live telemetry**: ground/vehicle link state, firmware versions, round-trip
  time, decoded state flags, latched faults and link counters.
- **Session log** in memory and on disk (CSV export from the console).
- **Works on Windows, Linux and macOS** — serial handling is implemented with
  the native APIs (Win32 comm API; POSIX termios/poll) and needs no drivers
  beyond the ST-LINK Virtual COM Port.
- **Simulator** (`gse-sim`) that runs the real ground-bridge and vehicle state
  machines over a simulated radio with latency, packet loss and bit errors.
- **Self-contained protocol crate** (`gse-proto`) with COBS framing, CRC-16,
  typed messages and an exhaustive test suite.

## Safety interlocks

Every gate is evaluated in the app (buttons greyed out, API/keyboard refused)
*and* in the vehicle firmware. There is **no override** in either place.

| # | Rule | Where enforced |
|---|------|----------------|
| 1 | FILL and SRV can never be ON at the same time | `safety.rs` gates; `vehicle.rs` rejects the command |
| 2 | FILL closes automatically after the configured time (default 60 s) and the operator is notified | the board's countdown (reported in `STATE`) triggers `FILL OFF` with `AUTO` in `engine.rs`; the vehicle also closes it itself 2 s after its deadline and emits `FillAutoOff` |
| 3 | IGN PWR is impossible until the fill sequence has completed | `safety.rs` (greyed out) + vehicle state machine |
| 4 | IGN PWR is impossible unless SRV is ON | `safety.rs` (greyed out) + vehicle state machine |
| 5 | Losing the link forces all outputs safe after 5 s and inhibits ignition | vehicle `check_link` and `LINK_LOSS_SAFE` |
| 6 | The emergency stop always wins, even while a command is in flight | host sends `ESTOP` immediately; the vehicle clears pending commands |
| 7 | A watchdog reset de-energises every output | IWDG1 (4 s) + reset-default GPIO states |
| 8 | Every session starts with FILL OFF, SRV OFF and IGN SAFE; controls stay locked until the vehicle confirms it | startup handshake in `engine.rs` + shared state machine |
| 9 | *Reset controls* always returns every output to OFF/SAFE and clears latches | `SAFE_ALL` + `RESET_FILL` from the app; shared state machine |
| 10 | Shutting the app down commands ALL SAFE first and waits for the ACK | shutdown sequence in `engine.rs` |

The fill sequence is: **FILL ON → (timer) → FILL OFF → SRV ON → IGN PWR**.
`IGN SAFE` removes power; a new fill sequence can then be started with
*Reset fill sequence*.

## Deterministic control state machine

FILL, SRV and IGN are not independent switches. Every operator action is an
event in one transition table, implemented once in
[`crates/gse-proto/src/fsm.rs`](crates/gse-proto/src/fsm.rs) and used by the
console, the simulator and both firmware images. That gives the same answer for
a button press, a hand-written HTTP request and a corrupted-but-valid radio
frame.

States are `FILL OFF|ON`, `SRV OFF|ON` and `IGN SAFE|PWR`; the initial state is
`FILL OFF · SRV OFF · IGN SAFE`. Every transition is one of:

| Outcome | Meaning | Console | Vehicle |
|---|---|---|---|
| Allowed | legal move to the returned state | tile enabled | command applied after the settle time, then ACK |
| No-op | already in the requested state | tile greyed with "already …" | ACK OK, no actuation |
| Blocked | an interlock refuses the move | tile greyed with the reason | `ACK` with `RejectedSafety` and the same reason |

The confirmed state only moves when the vehicle **acknowledges**. Sending a
command records a *target* (shown in the console as `→`) but never changes the
confirmed state, and the console keeps displaying that old state even if an
earlier `STATE` frame already reports the servo moving during its settle time.
A rejected command or an ACK timeout leaves the machine exactly where it was.
The vehicle enforces the identical table for every command, so an unsafe
transition cannot happen even if the console is bypassed.

| Event | From | Result |
|---|---|---|
| `FILL ON` | FILL OFF, SRV OFF, IGN SAFE | FILL ON |
| `FILL ON` | SRV ON | Blocked: FILL and SRV are interlocked |
| `FILL ON` / `SRV ON` / `IGN PWR` | IGN PWR (for FILL/SRV) | Blocked: SAFE the igniter first |
| `SRV ON` | FILL open | Blocked: close FILL first |
| `SRV OFF` | IGN PWR | Blocked: SAFE the igniter first |
| `IGN PWR` | fill complete, SRV ON, no latch | IGN PWR |
| `IGN PWR` | SRV OFF | Blocked: IGN PWR requires SRV ON |
| `IGN PWR` | incomplete/aborted fill, inhibited or E-STOP latched | Blocked with the reason |
| `FILL OFF` / `IGN SAFE` / `SRV OFF` | output already off | No-op |
| `SAFE ALL` / `RESET` | any state | FILL OFF · SRV OFF · IGN SAFE |

### Lifecycle

- **Initialisation.** A new link starts in `INIT`; every control tile is greyed
  until the vehicle reports `FILL OFF · SRV OFF · IGN SAFE`. If the board was
  left with an output on (for example after a console crash), the app commands
  `SAFE_ALL` and only unlocks the UI once the vehicle acknowledges the safe
  state. The same handshake runs after every reconnect.
- **Command → acknowledgement.** Pressing a tile sends the command and marks the
  target. The transition is committed only when the ACK arrives with the
  resulting state; until then every other control is greyed with *Command in
  progress* (E-STOP excepted).
- **Reset controls** (`Shift+R`, or the button) sends `SAFE_ALL` followed by
  `RESET_FILL`, returning the machine to its initial state and clearing the
  fill-sequence and E-STOP latches. It is always enabled while the link is up,
  even when everything is already safe, so the operator can always force a known
  state.
- **Shutdown** (Ctrl+C, SIGTERM or *Shutdown* in the API) sends `SAFE_ALL` and
  waits up to 2.5 s for the ACK before the process exits. If the vehicle cannot
  be reached the shutdown still proceeds but the log records that OFF/SAFE could
  not be confirmed. If an output was on, the vehicle also has its independent
  link-loss failsafe.

## Quick start with the simulator

```bash
cargo build --workspace
cargo run -p gse-sim          # terminal 1: simulated boards on 127.0.0.1:8787
cargo run -p gse-app -- --sim # terminal 2: console at http://127.0.0.1:8765/
```

`gse-app --sim` also starts `gse-sim` automatically if the binary sits next to
`gse-app` (as it does in `target/debug`), so a single command is enough. The
console opens in the default browser; use `--no-browser` on headless machines.

To exercise recovery paths:

```bash
cargo run -p gse-sim -- --latency 60 --noise   # 60 ms each way, drops/bit errors
```

## Running against the real hardware

1. Flash the boards (see [Firmware](#firmware)).
2. Plug the ground board's USB into the laptop.
3. Run:

```bash
gse-app                        # scans every serial port for the ground board
gse-app --port /dev/ttyACM0    # or name it explicitly
gse-app --port COM5 --verbose  # Windows
gse-app --port /dev/tty.usbmodem14201
```

The application opens the port, sends a `PING`, waits for the ground board's
`PONG`, then pings the vehicle. The header shows **Ground board** and
**Vehicle** going online; controls stay greyed out until both answer.

Linux users need access to the serial device:

```bash
sudo usermod -aG dialout "$USER"   # then log out and back in
```

### Command line

```
gse-app [OPTIONS]
  --sim                 use the simulator (tcp:127.0.0.1:8787)
  --port <PORT>         serial port, tcp:host:port, or 'auto' (default)
  --baud <RATE>         baud rate (default 115200)
  --config <PATH>       configuration file
  --host <ADDR>         console bind address (default 127.0.0.1)
  --http-port <PORT>    console port (default 8765, next free if busy)
  --no-browser          do not open a browser
  --open                open a browser even when headless
  --verbose             diagnostics on stderr and in the log
  -h, --help / -V, --version
```

### Configuration

Stored as JSON, created on first run:

| OS | Path |
|----|------|
| Linux | `~/.config/gse/config.json` (respects `XDG_CONFIG_HOME`) |
| macOS | `~/Library/Application Support/GSE/config.json` |
| Windows | `%APPDATA%\GSE\config.json` |

```json
{
  "fill_seconds": 60,
  "port": "auto",
  "baud": 115200,
  "confirm_ign": true,
  "sound": true,
  "host": "127.0.0.1",
  "http_port": 8765,
  "shortcuts": {
    "fill_toggle": "F",
    "srv_toggle": "S",
    "ign_toggle": "I",
    "estop": "E",
    "reset_controls": "Shift+R",
    "reset_fill": "R",
    "clear_faults": "Shift+C"
  }
}
```

Session logs are written next to the configuration under `logs/` as
append-only `session-<timestamp>.log` files.

## Using the console

- The action tiles are **greyed out with the blocking reason** whenever a gate
  is closed — those reasons also appear as tooltips and in the API.
- `IGN PWR` asks for confirmation unless *Ask before IGN PWR* is disabled in
  Settings.
- The state strip above the tiles shows the confirmed machine state (`FILL OFF ·
  SRV OFF · IGN SAFE`), the target while a command is in flight, and `INIT`
  while the startup handshake is running.
- *Reset controls* forces every output OFF/SAFE and clears latches; *Reset fill
  sequence* only clears the fill latch.
- The *Settings* dialog edits the fill duration, serial port, baud rate,
  confirmation/audio options and every key binding. Click a binding, press the
  new combination, `Esc` cancels. The API rejects duplicate bindings with a
  readable error.
- The log filter, follow mode, CSV export and *Clear* button act on the
  in-memory log; the on-disk log is never truncated by the UI.
- The console binds to `127.0.0.1` by default. If you change `--host`, anyone
  who can reach the port still needs the per-session token embedded in the
  served page, and the API sends no CORS headers.

## Firmware

`firmware/stm32h753` contains a `no_std` package with two binaries:

- `gse-ground` — USB CDC (USART3, PD8/PD9) ↔ SX1281 (USART6, PG14/PG9) bridge.
- `gse-vehicle` — SX1281 receiver plus TIM1 servo PWM (PE9/PE11), IGN output
  (PE13) and the watchdog.

They share the control transition table in `crates/gse-proto/src/fsm.rs`, the
protocol glue in `crates/gse-fw-core` (also used by the simulator) and the board
layer in `firmware/stm32h753/src/hw.rs`.

```bash
rustup target add thumbv7em-none-eabihf
cd firmware/stm32h753
cargo build --release --bin gse-ground
cargo build --release --bin gse-vehicle
# flashing, if probe-rs is installed:
cargo run --release --bin gse-vehicle
```

The two images differ only in their board bring-up; flash `gse-ground` on the
board wired to the laptop and `gse-vehicle` on the rocket board. See
[docs/HARDWARE.md](docs/HARDWARE.md) for the pin map, power notes and the
SX1281 configuration expected by the firmware.

The board layer is compiled and unit-tested on the host through
`crates/gse-fw-check`, so register maths and the transmit ring buffer are
covered by `cargo test` even without a Cortex-M toolchain.

## Project layout

```
crates/gse-proto      wire format + the shared deterministic control state machine
crates/gse-fw-core    ground bridge + vehicle logic (no_std)
crates/gse-app        host binary: engine, safety, HTTP console, config
crates/gse-sim        two-board TCP simulator (uses the real state machines)
crates/gse-fw-check   host compile/tests for the firmware board layer
firmware/stm32h753    NUCLEO-H753ZI board support and the two firmware images
docs/                 protocol and hardware reference
```

See [docs/PROTOCOL.md](docs/PROTOCOL.md) for the message layout, timing and
error handling.

## Testing

```bash
cargo test --workspace --lib --bins --tests
cargo clippy --workspace --all-targets -- -D warnings
cargo fmt --all -- --check
```

The suite covers framing (including truncated, corrupt and oversized frames),
every state-machine transition and safety rejection, the host interlocks, the
configuration round-trip and the HTTP request parsing. The firmware package is
built separately in CI and on a machine with the Cortex-M target installed.

An end-to-end smoke test drives the real binaries over HTTP:

```bash
./scripts/smoke.sh   # builds: cargo build --workspace
```

It starts the simulator and the console, verifies the OFF/SAFE initialisation,
runs a short fill sequence, checks every interlock, fires IGN, hits E-STOP,
resets the controls, reconnects with SRV ON to prove the startup handshake
forces OFF/SAFE, then shuts down with an output active and checks in the session
log that ALL SAFE was commanded and acknowledged.

To watch the machine itself, against a console you already have running:

```bash
scripts/state_trace.py http://127.0.0.1:8765 <token> [--shutdown]
```

It prints one line per transition — `pending`, `state`, `ack` — with the
confirmed state and the in-flight target, then verifies that RESET CONTROLS
returns to OFF/SAFE with the latches cleared. A recorded run is in
[`docs/state-machine-trace.md`](docs/state-machine-trace.md).

## Troubleshooting

| Symptom | Check |
|---|---|
| `No serial ports found` | USB data cable (not charge-only), board powered, ST-LINK drivers installed |
| `No ground board responded …` | the flashed image is `gse-ground`; baud rate 115200; Linux `dialout` group |
| Ground online, vehicle stays `pinging` | radio wiring (PG9/PG14 ↔ module RX/TX), module baud 9600, antennas, range |
| Buttons greyed out with *Command in progress* | wait for the ACK; if it never arrives the engine retries five times and logs a timeout |
| Console rejects a shortcut | the combination is already bound; the error names the other action |
| `Address already in use` | another instance is running, or pass `--http-port` |

## License

MIT OR Apache-2.0.
