# Ground controller binary (TEST / LIVE)

A native desktop application that sends on/off commands to the controller
ESP32 over USB serial. The controller ESP32 forwards them to the actuator ESP32
over ESP-NOW. The actuator acknowledges every command (`ACK token cmd accepted`).

```
PC controller binary -- USB serial --> ESP32 sender -- ESP-NOW --> ESP32 receiver
                                      (GPIO18/5/32) actuators
```

The firmware sketches are `../ESPNOW/servo_ign/espnow_sender/espnow_sender.ino`
and `../ESPNOW/servo_ign/espnow_receiver/espnow_receiver.ino`. **Re-flash both**
when upgrading from the earlier 4-command version: the protocol now includes
latched commands and acknowledgements.

## Build / binary

| Platform | Script | Output |
| --- | --- | --- |
| Linux x86_64 | `./build_linux.sh` | `dist/ground-controller` |
| Windows x64 | `./build_windows.ps1` | `dist\ground-controller.exe` |
| macOS Apple Silicon | `./build_macos_arm.sh` | `dist/` (.app/binary) |

Each script creates its own `.venv`, installs `requirements.txt`, and runs
PyInstaller. A Linux x86_64 binary is built in this repository's workspace at
`dist/ground-controller` (build artifacts are gitignored). Cross-platform
binaries must be produced on their own OS; PyInstaller cannot cross-compile.
On Debian/Ubuntu, PyInstaller needs the shared Python library; install
`python3-dev` if the build reports a missing `libpython3.x.so.1.0`.

Run from source instead with:

```bash
python3 -m venv .venv && . .venv/bin/activate
pip install -r requirements.txt
python src/ground_controller.py
```

The graphical window needs normal desktop OpenGL libraries (`libGL.so.1` on
Linux); these are present on standard desktop distributions but are absent in
the headless build sandbox, so the GUI itself could not be launched here.

## Operating

1. Flash sender and receiver sketches; put each board's printed MAC into the
   other sketch as documented in `../ESPNOW/servo_ign/README.md`.
2. Connect the sender ESP32 by USB. In the app click **Refresh**, select the
   port, click **Connect**. **PING** should reply `PONG`.
3. Choose **TEST** or **LIVE** from the mode selector. Switching modes sends
   `ALL_OFF` to the actuator before the new mode becomes active.

### TEST mode

Immediate operator control, for bench checks:

- `FILL OPEN` / `FILL CLOSE` → `SV0_ON` / `SV0_OFF`
- `VEH OPEN` / `VEH CLOSE` → `SV1_ON` / `SV1_OFF`
- `BOTH PULSE` → timed both-servo pulse (`3`); `BOTH CLOSE` → `BOTH_OFF`
- `IGN PULSE` → timed igniter pulse (`4`); `IGN OFF / RESET` → `IGN_OFF`

The actuator still enforces its hard interlocks in TEST mode (see below), so a
command that would open VEH while FILL is open is rejected with an ACK flag of
`0` and shown in the log.

### LIVE mode safety gates

| Control | Gate / behavior |
| --- | --- |
| FILL (servo0) | `FILL OPEN` starts a configurable countdown (60 s default). At zero, the app beeps and shows a warning, but **FILL stays OPEN**. Operator may close early with `FILL CLOSE` or postpone with `EXTEND` (10 s default, configurable). |
| VEH (servo1) | `VEH OPEN` is greyed out while FILL is open (also while VEH is already open). Only enabled once FILL is CLOSED. |
| BOTH | `BOTH OPEN` is greyed out in LIVE mode: FILL must be closed for VEH, so a simultaneous open is not permitted. `BOTH CLOSE` is always available. |
| IGN | `IGN ARM` starts a configurable countdown (5 s default). `IGN ON` is **locked out** until it completes. At completion the app beeps and shows `ARMED`; the operator may then trigger `IGN ON` or reset with `IGN OFF / RESET`. |
| ALL OFF | Always available: closes FILL and VEH and forces IGN OFF. |

Countdown durations are edited in the spin boxes and apply to the next
countdown started. State changes are local and latched on the actuator; the
timers are advisory to the operator, not automatic actuators (except that IGN
ON is impossible before its countdown completes).

### Firmware-side interlocks (defense in depth)

The receiver independently refuses to:

- open VEH while FILL is open;
- arm/fire IGN while FILL is open or VEH is open;
- accept `IGN ON` before the requested arm delay has elapsed;
- open FILL while VEH is open or IGN is armed/on;
- accept `BOTH ON` (VEH must never open together with FILL).

Rejected commands are still ACKed with `accepted=0` so the GUI can show the
rejection.

## Protocol (USB serial commands)

One ASCII line per command, 115200 baud:

```
1 / 2 / 3 / 4    timed test pulses
SV0_ON SV0_OFF   FILL open / close
SV1_ON SV1_OFF   VEH open / close
BOTH_OFF         close both servos
IGN_ARM:<ms>     start igniter countdown
IGN_ON IGN_OFF   fire / reset igniter
ALL_OFF          all outputs safe
PING             expects PONG
```

The sender replies with `SENT <cmd> <token>` and `ACK <token> <cmd> <0|1>`.

## Tests

```bash
python3 -m unittest discover -s tests -v
```

These unit tests cover FILL early-close/extend/timeout notification, VEH/IGN
lockout while FILL is open, IGN no-premature-ON, cancellation, and `ALL OFF`.

## Safety

Do not treat software gating as a safety interlock. Use a physical arming
switch and series interlock in the igniter circuit, keep the igniter
disconnected while bench-testing, and verify the servo directions with the
horns detached.
