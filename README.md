# ESP-NOW servo / ignition actuator link

Two Arduino sketches (`.ino`, Arduino IDE compatible) plus a desktop controller
binary turn commands from a controller ESP32 into servo and ignition actions on
a second ESP32.

```
PC controller binary -- USB serial --> ESP32 sender -- ESP-NOW --> ESP32 receiver
                                      (GPIO18 / GPIO5 / GPIO32 actuators)
```

There are two command families:

| Command | Mode | Action on receiver |
| --- | --- | --- |
| `1` / `2` / `3` / `4` | TEST | Timed pulse on servo0 / servo1 / both / IGN |
| `SV0_ON` / `SV0_OFF` | both | FILL (servo0) latched open / closed |
| `SV1_ON` / `SV1_OFF` | both | VEH (servo1) latched open / closed |
| `BOTH_OFF` | both | Close FILL and VEH |
| `IGN_ARM:<ms>` | LIVE | Start IGN countdown; IGN ON stays locked out |
| `IGN_ON` / `IGN_OFF` | both | Fire / reset latched IGN |
| `ALL_OFF` | both | Close FILL and VEH, force IGN OFF |

The TEST/LIVE operator interface, countdowns and greyed-out buttons are in
[`../../controller/README.md`](../../controller/README.md).

## Files

- `espnow_sender/espnow_sender.ino` - controller bridge: parses USB serial text
  commands and forwards them as ESP-NOW packets; prints receiver ACKs.
- `espnow_receiver/espnow_receiver.ino` - actuator board: owns the servo and IGN
  pins, enforces interlocks, and ACKs every command.

Both sketches are self-contained: no third-party libraries are required (only
the Arduino-ESP32 core and its bundled `WiFi`/`esp_now`/LEDC APIs). Folder names
match sketch names so Arduino IDE can open them directly.

## Servo specifications used

The servo is the DS3225 (52Pi D-0001),
[datasheet](https://wiki.52pi.com/index.php?title=D-0001):

- Operating voltage: 4.8-6.8 V (5 V / 6 V nominal)
- Control: PWM, 500-2500 us pulse width, 50-330 Hz (sketches use 50 Hz / 20 ms)
- Stall current: 2.1 A @ 5 V, 2.9 A @ 6.8 V **per servo**
- 180 deg or 270 deg variant exists: verify the mechanical endpoints on your unit

Consequences for the build:

- **Do not power servos from the ESP32 5 V / VIN pin.** Use a separate 5-6 V BEC
  rated for at least 6 A peak (two servos can stall simultaneously), with a
  wide-enough bulk capacitor near the servo connector.
- **Connect the BEC ground, both servos, the IGN driver, and the ESP32 ground
  together.**
- ESP32 outputs 3.3 V logic; the servo signal pin is usually tolerant of this,
  but verify on your specific servo or use a level shifter if it misbehaves.
- `SERVO_HOME_US` (1000) and the `*_TARGET_US` values (2000) are starting points,
  not calibrated endpoints. Set them with the servo horn detached so a wrong
  value cannot jam the mechanism.

## Wiring

| Signal | ESP32 (receiver) | Notes |
| --- | --- | --- |
| servo0 PWM | GPIO 18 | FILL in live mode; 50 Hz, 500-2500 us, 220 ohm series resistor optional |
| servo1 PWM | GPIO 5 | VEH in live mode |
| IGN | GPIO 32 | active HIGH, drives a logic-level MOSFET / relay driver, never an igniter directly |
| GND | any ESP32 GND | common with BEC and igniter driver |

IGN output is forced LOW at boot. In TEST mode it is a timed pulse (clamped to
2000 ms); in LIVE mode it stays latched until `IGN_OFF` / `ALL_OFF`. Add a
10 kohm pulldown at GPIO 32 so the igniter stays off during ESP32 reset/boot,
plus a physical arming switch and a series safety interlock.

## Setup

1. Install the ESP32 board package in Arduino IDE (Arduino-ESP32 3.x
   recommended; 2.x is also supported by both sketches).
2. Open `espnow_receiver/espnow_receiver.ino`, flash the actuator board, and
   open the Serial Monitor at 115200. Note the **Receiver MAC** it prints.
3. Put that MAC into `RECEIVER_MAC` at the top of `espnow_sender.ino`, flash the
   controller, and note the **Sender MAC** it prints.
4. Put the sender's MAC into `SENDER_MAC` in the receiver sketch and reflash it.
   (Keep `ENFORCE_SENDER_MAC = true`; set it to `false` only for a bench test
   with a throwaway controller.)
5. Bench-test with servo horns detached and the igniter disconnected: send
   `1`, `2`, `3`, then `4` from the controller's Serial Monitor, then test the
   latched commands from the desktop binary.

### If the MAC prints as 00:00:00:00:00:00

On Arduino-ESP32 3.x, `WiFi.macAddress()` can return all zeros when the STA
interface has not started yet. Both sketches therefore read the STA MAC via
`esp_wifi_get_mac()` (waiting briefly for `WiFi.STA.started()` on core 3.x),
falling back to `esp_read_mac(ESP_MAC_WIFI_STA)`. If you still see zeros, check
that the factory MAC is programmed (`esptool.py chip_id` / `efuse_summary`) and
that you are flashing a genuine ESP32 with a supported board package. Never put
the printed zeros into `RECEIVER_MAC` or `SENDER_MAC`: the sender refuses to
start with an all-zero peer MAC, and the receiver rejects every packet when
`ENFORCE_SENDER_MAC` is enabled.

## How a command works

1. The controller (desktop binary or Serial Monitor) sends one ASCII line over
   USB at 115200 baud. The sender ESP32 maps it to a command ID.
2. The sender builds an 18-byte packet with a magic word, random token, command
   ID, mode flag, hold/arm time, both target pulse widths, and CRC-16.
3. The packet is transmitted 3 times, 120 ms apart, to survive a lost frame.
4. The receiver checks sender MAC, packet length, magic, CRC, token (replay
   filter), and pulse ranges. Subsequent copies of the same command are ignored.
5. Timed test commands return the servo(s) to home after the hold; latched
   commands hold their position/state until an OFF command is received.
6. The receiver ACKs each command with the token, command ID and an `accepted`
   flag. A `0` means an interlock or countdown refused the action; the desktop
   binary shows the rejection.

### Firmware-side interlocks

Even if the desktop binary is bypassed, the receiver refuses to:

- open VEH while FILL is open;
- arm or fire IGN while FILL is open or VEH is open;
- accept `IGN ON` before the requested arm delay has elapsed;
- open FILL while VEH is open or IGN is armed/on;
- accept a combined `BOTH ON` (VEH must never open together with FILL).

### Command packet layout (must match in both sketches)

| Offset | Type | Field | Meaning |
| --- | --- | --- | --- |
| 0-3 | uint32 | `magic` | `0x314B4352` ("RCK1" over the air) |
| 4-7 | uint32 | `token` | random ID; receiver remembers the last 8 |
| 8 | uint8 | `cmd` | see command table above |
| 9 | uint8 | `reserved` | 0 = timed test pulse, 1 = latched live state |
| 10-11 | uint16 | `holdMs` | test hold / `IGN_ARM` countdown in ms |
| 12-13 | uint16 | `servo0PulseUs` | 500-2500 |
| 14-15 | uint16 | `servo1PulseUs` | 500-2500 |
| 16-17 | uint16 | `crc16` | CRC-16/CCITT-FALSE over bytes 0-15 |

### ACK packet (receiver to sender, 12 bytes)

| Offset | Type | Field | Meaning |
| --- | --- | --- | --- |
| 0-3 | uint32 | `magic` | `0x314B4341` ("ACK1") |
| 4-7 | uint32 | `token` | copied from the command |
| 8 | uint8 | `cmd` | command ID |
| 9 | uint8 | `accepted` | 1 = actuated, 0 = rejected |
| 10-11 | uint16 | `crc16` | CRC-16/CCITT-FALSE over bytes 0-9 |

There is no encryption: MAC filtering and CRC only guard against unintended
senders and corrupted frames. `esp_now_send()` returning `ESP_OK` only means the
frame was handed to the radio, not that the receiver acted on it. Do not treat
this radio link as a safety interlock for pyrotechnics.
