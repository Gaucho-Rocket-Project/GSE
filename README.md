# ESP-NOW servo / ignition actuator link

Two Arduino sketches (`.ino`, Arduino IDE compatible) that turn commands from a
controller ESP32 into servo and ignition actions on a second ESP32:

| Command | Action on receiver |
| --- | --- |
| `1` | Actuate **servo0** (GPIO 18): move to target pulse, hold, return home |
| `2` | Actuate **servo1** (GPIO 5): move to target pulse, hold, return home |
| `3` | Actuate **servo0 + servo1** together |
| `4` | **IGN** (GPIO 32, active HIGH): drive HIGH for the on-time, then LOW |

## Files

- `espnow_sender/espnow_sender.ino` - controller; type `1`, `2`, `3`, or `4` in the Serial Monitor.
- `espnow_receiver/espnow_receiver.ino` - actuator board; owns the servo and IGN pins.

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
| servo0 PWM | GPIO 18 | 50 Hz, 500-2500 us, via 220 ohm series resistor optional |
| servo1 PWM | GPIO 5 | same |
| IGN | GPIO 32 | active HIGH, drives a logic-level MOSFET / relay driver, never an igniter directly |
| GND | any ESP32 GND | common with BEC and igniter driver |

IGN output is forced LOW at boot and is only HIGH during a validated,
time-limited command (clamped to 2000 ms). Add a 10 kohm pulldown at GPIO 32 so
the igniter stays off during ESP32 reset/boot, plus a physical arming switch
and a series safety interlock in the igniter wiring.

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
   `1`, `2`, `3`, then `4` from the controller's Serial Monitor and watch the
   receiver log and the PWM outputs.

## How a command works

1. Controller builds an 18-byte packet with a magic word, random token, command
   ID, hold time, both target pulse widths, and CRC-16.
2. The packet is transmitted 3 times, 120 ms apart, to survive a lost frame.
3. Receiver checks sender MAC, packet length, magic, CRC, token (replay filter),
   and pulse ranges. Subsequent copies of the same command are ignored.
4. For servo commands, the selected servo(s) move to the requested pulse
   immediately, hold for `holdMs` (max 5000 ms), then return to `SERVO_HOME_US`.
   A new command while an action is running is ignored.
5. For IGN, GPIO 32 goes HIGH for `holdMs` (max 2000 ms), then LOW. Servos stay
   at home during ignition.

### Packet layout (must match in both sketches)

| Offset | Type | Field | Meaning |
| --- | --- | --- | --- |
| 0-3 | uint32 | `magic` | `0x314B4352` ("RCK1" over the air) |
| 4-7 | uint32 | `token` | random ID; receiver remembers the last 8 |
| 8 | uint8 | `cmd` | 1=servo0, 2=servo1, 3=both, 4=IGN |
| 9 | uint8 | `reserved` | must be 0 |
| 10-11 | uint16 | `holdMs` | target hold / IGN on-time in ms |
| 12-13 | uint16 | `servo0PulseUs` | 500-2500 |
| 14-15 | uint16 | `servo1PulseUs` | 500-2500 |
| 16-17 | uint16 | `crc16` | CRC-16/CCITT-FALSE over bytes 0-15 |

There is no encryption: MAC filtering and CRC only guard against unintended
senders and corrupted frames. `esp_now_send()` returning `ESP_OK` only means the
frame was handed to the radio, not that the receiver acted on it. Do not treat
this radio link as a safety interlock for pyrotechnics.
