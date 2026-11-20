# GSE ESP-NOW ground / vehicle link

Two ESP32 Arduino sketches and a Qt desktop controller operate FILL, SRV and IGN
and relay vehicle telemetry to the ground computer:

```
telemetry source -- UART2 --> vehicle ESP32 <-- ESP-NOW --> ground ESP32 <-- USB --> desktop
                           GPIO16 RX / 17 TX              GPIO18 / 5 / 32 outputs on vehicle
```

- [`vehicle/vehicle.ino`](vehicle/vehicle.ino): actuator and UART telemetry input (formerly receiver).
- [`ground/ground.ino`](ground/ground.ino): ESP-NOW / USB serial bridge (formerly sender).
- [`controller/`](controller/README.md): desktop app and tests.

**Safety:** These are starting points, not flight-qualified controls. Do not
rely on software, MAC filtering, CRC, or ESP-NOW as a safety interlock. Use a
physical arming switch, series ignition interlock and GPIO32 pulldown. Test with
the igniter disconnected and servo horns detached. The desktop displays
optimistic commanded state, not verified output position; rejected or lost
commands can leave it out of sync with the vehicle. A successful serial write
or radio send is not proof of actuation. Verify behavior on hardware before use.

## Setup and wiring

1. Install Arduino-ESP32 3.x (2.x is also supported). Open the sketches from
   their matching directories in Arduino IDE; reflash **both** boards.
2. Flash each board and note its STA MAC from the serial monitor (115200 baud).
   Set `VEHICLE_MAC` in the ground sketch and `GROUND_MAC` in the vehicle
   sketch to the other board's MAC and reflash. Keep `ENFORCE_GROUND_MAC`
   enabled. Both ESP32s must use the same Wi-Fi channel. Never use an all-zero
   MAC. Telemetry always goes to the configured ground MAC.
3. Connect vehicle GPIO18 to FILL servo signal, GPIO5 to SRV servo signal, and
   GPIO32 to a suitable active-high ignition **driver**, not an igniter
   directly. GPIO32 is forced LOW on boot; use a 10 kΩ pulldown as well.
4. Use a separate 5–6 V BEC rated for at least 6 A peak for two DS3225 servos;
   do not power them from the ESP32 5 V pin. Common the BEC, driver, telemetry
   source, and ESP32 grounds. Check 3.3 V signal compatibility and mechanical
   endpoints. Home is 2500 µs in vehicle firmware; the desktop open targets
   default to 1860 µs and can be set independently from 500–2500 µs. Calibrate
   before attaching the mechanism. Changing a target affects the next command,
   not an already-open servo.
5. Connect telemetry source TX to vehicle RX2 GPIO16; connect source RX to
   vehicle TX2 GPIO17 only if needed. Use 3.3 V TTL UART, not RS-232 voltages.
   Serial2 is configured 115200 baud, 8N1; change `TELEMETRY_BAUD` if needed.

## Telemetry protocol and storage

The UART source must emit newline-terminated, nonempty **printable ASCII**
records (1–200 bytes, excluding CR/LF), for example `pressure,12.5`. CRLF is
accepted. Overlong and non-printable records are discarded whole. No schema is
assumed: a record is stored as one CSV payload field, including commas and
quotes. Binary, UTF-8 non-ASCII, and partial lines are not supported.

Vehicle samples the latest complete record at most once per
`TELEMETRY_INTERVAL_MS` (default 500 ms, set in `vehicle.ino` before flashing).
Intermediate lines are overwritten if the UART is faster; no repeated/stale
record is sent when the source is idle. ESP-NOW telemetry is best-effort and
not retried, so it is not a complete data logger. Its sequence and vehicle
uptime (32-bit milliseconds, wrapping) help detect gaps. The packed TLM1
packet contains magic, sequence, send uptime, length, 200-byte zero-padded
payload and CRC-16/CCITT-FALSE. Ground verifies source MAC, length and CRC and
forwards `TLM <sequence> <uptime_ms> <payload>` over USB. Its bounded receive
queue can drop records under load.

While connected, the desktop app appends rows to `data/telemetry.csv` under the
repository root, regardless of whether the telemetry panel is open. Columns:
`received_utc,sequence,vehicle_uptime_ms,payload`. UTC timestamps are desktop
receive times; sequence resets on vehicle reboot. CSV is flushed on each row,
is gitignored, and write errors are shown in the link log. For a frozen binary,
`data` is relative to the launch working directory; set `GSE_DATA_DIR` to an
absolute writable directory to override it. Ground ESP32 itself has no storage;
the desktop must be connected and running to capture data.

## Controls and interlocks

See [controller/README.md](controller/README.md) for installation and operation.
In LIVE, there are no manual SRV controls; SRV opens only when IGN is triggered.
FILL must be closed before IGN, and IGN arming requires both servos closed. Once the IGN
countdown completes, `IGN_ON` opens SRV to the commanded pulse **and** latches
IGN on in one vehicle-side command, after validating the interlocks, countdown,
and pulse range. `IGN_OFF` resets ignition but leaves SRV open; use `ALL_OFF`
to close it in LIVE. FILL remains open at its timer expiration until
manually closed. The combined-servo controls are TEST-only; test pulse `3`
remains available there.

Ground accepts one line per USB command at 115200 baud: `1`, `2`, `3`, `4`,
`SV0_ON`, `SV0_OFF`, `SV1_ON`, `SV1_OFF`, `BOTH_OFF`, `IGN_ARM:<ms>`, `IGN_ON`,
`IGN_OFF`, `ALL_OFF`, `PING`. The desktop adds `:<fill_us>,<srv_us>` to servo
actuation commands and `IGN_ON`, e.g. `IGN_ON:500,1500`; ground validates both
values and embeds them in the existing command packet. Unadorned commands use
the last targets (1800 µs after ground boot). Command and ACK packets use CRC and MAC
checks, not encryption. `SV1_ON` remains available over the raw serial protocol
for TEST and bypasses the desktop's LIVE-only restriction; the firmware does not
track desktop mode, so do not use raw commands as a LIVE safety interlock. The
vehicle checks commands and sends `ACK <token>
<cmd> <accepted>`; duplicate command tokens are ignored. A missing ACK does not
prove that no action occurred.
