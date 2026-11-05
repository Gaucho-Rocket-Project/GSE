# ESP32-S3 dual SG90 servo firmware

C firmware (ESP-IDF v5.x) that drives **two SG90 micro servos** on an ESP32-S3
board and lets you switch them on and off — individually or together — from a
serial console.

```
servo> help
servo> on servo1          -> servo 1 starts pulsing, centred at 1.5 ms
servo> on servo2          -> servo 2 starts pulsing
servo> on both            -> both at once
servo> servo1 45deg       -> aim servo 1 at 45 degrees
servo> both 1.8ms         -> aim both (explicit pulse width)
servo> off servo2         -> servo 2 stops pulsing (goes limp)
servo> sweep both         -> slow 0 -> 180 -> 0 movement test
servo> status             -> state + pulse width + approximate angle
```

Two ways to get it onto the board:

* **A — flash the prebuilt image** (no toolchain needed, ~1 minute)
* **B — build from source and flash** (ESP-IDF, ~15 minutes first time)

**Wiring: see [WIRING.md](WIRING.md)** (diagram: [docs/wiring.svg](docs/wiring.svg) ·
[docs/wiring.png](docs/wiring.png)) — read §4 about power before you connect anything.

| Signal | ESP32-S3 | SG90 wire |
|---|---|---|
| Servo 1 PWM | **GPIO4** | orange |
| Servo 2 PWM | **GPIO5** | orange |
| Power | **5 V** (external supply ≥ 1 A recommended) | red |
| Ground | **GND** (common with the supply) | brown |

### Supported servos

| Servo | Build / image | Power per servo |
|---|---|---|
| **SG90 / MG90S** (micro) | default, `SERVO_TYPE=1` | ~0.25 A moving, 0.36 A stall |
| **DSSERVO DS3225MG 25 kg** (180° or 270°) | `SERVO_TYPE=2`, images `ds3225-*` | **1–2 A moving, 2.1–2.9 A stall** |

Both use the same 50 Hz / 3-wire interface, so the wiring is identical — but the
25 kg servos need a **separate 5–6.8 V supply of ≥ 6 A for two units** and much
bigger bulk capacitance. Full details: **[WIRING.md §9](WIRING.md)**.

Pick the travel to match your servo — the 180° and 270° DS3225 variants share the
same 500–2500 µs pulse range, so this only changes the angle maths:

```bash
make build SERVO_TYPE=2 SERVO_TRAVEL_DEG=180    # DS3225 180 deg version
make build SERVO_TYPE=2 SERVO_TRAVEL_DEG=270    # DS3225 270 deg version
make servo-type                                 # show what is selected
```

---

## What's in here

```
esp32s3-servo-fw/
├── README.md                  <- you are here
├── WIRING.md                  <- wiring + power + calibration guide
├── docs/
│   ├── wiring.svg             <- wiring diagram (vector)
│   └── wiring.png             <- wiring diagram (raster)
├── flash.sh                   <- flash the prebuilt image with esptool
├── monitor.sh                 <- find the serial port and open the console
├── prebuilt/
│   ├── esp32s3_servo_usb_full.bin                <- SG90/micro, native USB
│   ├── esp32s3_servo_uart_full.bin               <- SG90/micro, UART0
│   ├── esp32s3_servo_ds3225_180_usb_full.bin     <- DS3225MG 180 deg, native USB
│   ├── esp32s3_servo_ds3225_180_uart_full.bin    <- DS3225MG 180 deg, UART0
│   ├── esp32s3_servo_ds3225_270_usb_full.bin     <- DS3225MG 270 deg, native USB
│   └── esp32s3_servo_ds3225_270_uart_full.bin    <- DS3225MG 270 deg, UART0
├── firmware/                  <- the ESP-IDF project
│   ├── CMakeLists.txt
│   ├── Makefile               <- make build / flash / monitor wrappers
│   ├── sdkconfig.defaults     <- USB-console build config
│   ├── sdkconfig.defaults.uart<- UART-console build config
│   └── main/
│       ├── app_main.c         <- command console (the "on/off" interface)
│       ├── servo.c/.h         <- SG90 driver on the LEDC PWM peripheral
│       ├── position.c/.h      <- position/pulse parser (host-testable)
│       ├── console_io.c/.h    <- line input over USB-Serial-JTAG or UART
│       └── CMakeLists.txt
└── tests/
    ├── test_position.c        <- unit tests for the parser (runs on a PC)
    └── Makefile               <- make -C tests
```

---

## A. Flash the prebuilt image (fastest)

The images in `prebuilt/` are complete: **bootloader + partition table +
application** merged into one file, flashed at offset `0x0`. They were built
with ESP-IDF v5.4.1 for the ESP32-S3, 8 MB flash, 80 MHz.

### A.1 Install esptool

```bash
pip install esptool
# or, if you already have ESP-IDF:  . $IDF_PATH/export.sh
```

### A.2 Plug in the board

Two different USB ports exist on ESP32-S3 boards:

| Port on the board | What it is | Port name you get | Which build to use |
|---|---|---|---|
| **USB** (native, labelled "USB" / "OTG" / "USB-Serial-JTAG") | the chip's built-in USB peripheral | `ttyACM0` · `cu.usbmodem*` · `COMx` | `usb` (default) |
| **UART / COM** (labelled "COM" / "UART", next to it) | a USB-UART bridge chip (CP210x / CH340) | **`ttyUSB0`** · `cu.usbserial*` · `COMx` | **`uart`** |

**The port name tells you which build you need:**

* `ttyACM*` / `cu.usbmodem*` → this is the native USB port → flash the **`usb`** image.
* `ttyUSB*` / `cu.usbserial*` / `cu.wchusbserial*` → this is the UART bridge → either
  plug into the native USB port instead, or flash the **`uart`** image.

A mismatch here is the classic "the port opens but nothing prints, and `screen`
exits immediately" symptom. `./monitor.sh` now detects it for you.

### A.3 Flash

```bash
./flash.sh                     # SG90 build, USB console, auto-detect port
./flash.sh /dev/ttyACM0        # explicit port
./flash.sh /dev/ttyUSB0 uart   # SG90 build, UART console

# DS3225MG 25 kg servos (see WIRING.md section 9)
./flash.sh /dev/ttyACM0 ds3225-180        # 180 deg version
./flash.sh /dev/ttyACM0 ds3225-270        # 270 deg version
./flash.sh /dev/ttyUSB0 ds3225-180-uart   # ... UART console variants exist too
```

Or do it by hand (same thing):

```bash
esptool.py --chip esp32s3 -b 460800 write_flash \
    --flash_mode dio --flash_freq 80m --flash_size 8MB \
    0x0 prebuilt/esp32s3_servo_usb_full.bin

# (that is exactly what flash.sh runs; add "-p /dev/ttyACM0" to pick the port)
```

No button presses are needed on a normal board: esptool resets the chip into its
downloader automatically (DTR/RTS). If it still can't connect: **hold BOOT, tap
RESET, release BOOT**, then run the command again.

> **4 MB board?** The image is built for 8 MB flash (the usual N2R8/N8R8 module).
> The image header says 8 MB; on a 4 MB part, erase and rebuild from source with
> `CONFIG_ESPTOOLPY_FLASHSIZE_4MB=y` — see part B. The app itself is ~205 KB, so
> it fits anywhere; it's only the header's flash-size field that matters.

### A.4 Open the console

**First, find the port** — `/dev/ttyACM0` is only a *guess*; on macOS it is
`/dev/cu.usbmodemXXXX`, and on some boards it is `/dev/ttyUSB0`:

```bash
./monitor.sh --list      # show which serial ports actually exist
./monitor.sh             # open the one it finds
./monitor.sh /dev/ttyACM0
```

Or open it yourself:

```bash
# Linux (USB-Serial-JTAG) - baud rate is irrelevant, it is a virtual COM port
screen /dev/ttyACM0 115200              # quit: Ctrl-A then K
# macOS:  screen /dev/cu.usbmodem1234 115200
# Windows: PuTTY -> Serial -> COMx -> 115200
# no screen?  python3 -m serial.tools.miniterm /dev/ttyACM0 115200
```

You should see:

```
==================================================
 ESP32-S3  -  dual SG90 servo test firmware
==================================================
  servo1 : signal on GPIO4   (state: off)
  servo2 : signal on GPIO5   (state: off)
--------------------------------------------------
 Type 'help' for commands.
 ...
servo>
```

Type `1` + Enter → servo 1 pulses to centre. Type `0` + Enter → everything off.

---

## B. Build from source

### B.1 Install ESP-IDF (once)

```bash
git clone -b v5.4.1 --recursive https://github.com/espressif/esp-idf.git
cd esp-idf
./install.sh esp32s3
. ./export.sh          # do this in every new terminal
```

(ESP-IDF v5.0 – v5.4 should also work; the code only uses stable public APIs.)

### B.2 Build and flash

```bash
cd esp32s3-servo-fw/firmware

make build             # console on USB Serial/JTAG (default)
make flash             # auto-detect port
make monitor

# or, straight idf.py:
idf.py set-target esp32s3
idf.py build flash monitor
```

For the UART-console variant, everything is the same but with `-uart`:

```bash
make build-uart
make flash-uart PORT=/dev/ttyUSB0
make monitor-uart
```

Each variant keeps its own config and build directory (`build/`,
`build-uart/`), so they never overwrite each other.

### B.3 Changing pins or travel limits

| What | Where |
|---|---|
| Servo model / travel | `make build SERVO_TYPE=2 SERVO_TRAVEL_DEG=180` (no source edit) |
| PWM pins (default GPIO4 / GPIO5) | `main/servo.c` → `s_servos[]` |
| Pulse range (default 500–2500 µs) | `main/servo.h` → `SERVO_MIN/MAX_PULSE_US` |
| Sweep step/delay | `main/app_main.c` → `SWEEP_STEP_US`, `SWEEP_DELAY_MS` |

Rebuild and re-flash after editing.

---

## Commands

| Command | Effect |
|---|---|
| `on servo1`, `on servo2`, `on both` | start the 50 Hz pulse train, centred (1.5 ms ≈ 90°) |
| `off servo1`, `off servo2`, `off both` | stop pulses, park the signal line low → servo goes limp |
| `servo1 90`, `servo2 45deg` | aim by angle (0–180) |
| `servo1 1.5ms`, `both 1500us`, `both 1500` | aim by pulse width |
| `center both` | back to 1.5 ms |
| `sweep servo1` / `sweep both` | slow 0 → 180 → 0, ~6 s |
| `status` or `s` | state, pulse width, approximate angle |
| `help` or `?` | the menu |

**One-key shortcuts** (just press the key + Enter):

| Key | Action |
|---|---|
| `1` | servo 1 on (centre) |
| `2` | servo 2 on (centre) |
| `3` | both on (centre) |
| `0` | both off |
| `9` | sweep both |
| `s` | status |
| `?` | help |

Commands are case-insensitive and tolerate extra words, e.g.
`please turn on servo 1` works. Values are clamped to the safe pulse range
instead of being rejected, so a typo can't drive the servo into its end stop.

### What "off" means

`off` stops the pulse train and holds the signal pin **low**. A servo with no
valid pulse stops driving — it goes **limp**, it does **not** return to a park
position, and it draws almost nothing. If you want a "parked at 90° and
holding", use `on both` (or `center both`) — it keeps holding torque, which
costs a little current and a little gear wear.

---

## How it works (for the curious)

* **SG90 control**: a pulse every 20 ms, width 1.0–2.0 ms → 0–180°.
* **Hardware PWM**: the ESP32-S3's LEDC peripheral generates it in hardware —
  no timer jitter, nothing for the CPU to babysit. One 50 Hz timer feeds two
  channels (LEDC channels 0 and 1 on timer 0).
* **Resolution**: LEDC on the S3 is 14-bit → 16384 steps per 20 ms →
  **1.22 µs per step ≈ 0.1°**, far finer than the SG90's own slop.
* **Duty maths**: `duty = pulse_us / 20000.0 * 16384`, so 1500 µs → 1229.
* **Start/stop**: `off` calls `ledc_stop(..., idle_level=0)`; `on` re-applies
  `ledc_channel_config()` and sets the duty. The LEDC timer keeps running in
  both cases, so re-enabling is instant.
* **Console input** is read through the driver layer
  (`usb_serial_jtag_read_bytes` / `uart_read_bytes`) with a 20 ms timeout, so the
  parser can tell "no line yet" from "empty line", and left/right-arrow escape
  sequences don't corrupt commands.
* **No dynamic allocation** in the command path — it all runs on `app_main`'s
  task with static buffers.

### Tests

The position parser is pure C with no ESP-IDF dependencies and has a host-side
test suite:

```bash
make -C tests
```

---

## Troubleshooting

| Symptom | Likely cause / fix |
|---|---|
| `screen /dev/ttyACM0 115200` → `cannot exec ...: No such file or directory` | `/dev/ttyACM0` does not exist on your machine. Run `./monitor.sh --list` to see the real name (macOS: `/dev/cu.usbmodemXXXX`, some boards: `/dev/ttyUSB0`). |
| Port is `/dev/ttyUSB0` and `screen` exits immediately with `[screen is terminating]` | You are on the board's **USB-UART bridge** (UART0), but flashed the `usb` image whose console is on the native USB port. Run `./flash.sh /dev/ttyUSB0 uart` — or plug into the board's **USB/OTG** port and use the default build. |
| No serial port shows up at all | Try the **other USB port** on the board; use a **data** cable; check `dmesg \| tail -20` on Linux (`cdc_acm` = native USB, `ch341`/`cp210x` = UART bridge) or install the CH340 driver on macOS/Windows. |
| `esptool` can't find the port (USB build) | Try the other USB port on the board. On Linux your user may need the `dialout` group, or run with `sudo`. |
| `Failed to connect ... Wrong boot mode` | Hold **BOOT**, tap **RESET**, release **BOOT**, retry. |
| Console prints but ignores my typing (USB build) | Close any other program that has the serial port open (Arduino IDE, another monitor). Only one reader at a time. |
| Nothing on the UART port | You flashed the `usb` build. Use 115200 8N1 **and** the `uart` image for the COM port. |
| DS3225 twitches, buzzes, or the board resets when it moves | Almost always **power**: these stall at 2.1–2.9 A each. Use a 5–6.8 V supply ≥ 6 A for two, thick ground, ≥ 2200 µF at the servos. [WIRING.md §9.3](WIRING.md) |
| DS3225 stops 90° short at one end | You flashed the 180° image for a **270°** servo (or vice versa). Flash the other travel build — [WIRING.md §9.2](WIRING.md) |
| DS3225 draws current and gets warm while idle | It is holding torque against a load (normal) or fighting a mechanical stop (bad). Use `off` to unload it. |
| Servo twitches or the board resets when it moves | Power. The servos are browning out the supply → see [WIRING.md](WIRING.md) §4: external 5 V ≥ 1 A, common ground, 470–1000 µF cap. |
| Servo is silent but wired | Run the bench test in [WIRING.md](WIRING.md) §6. Check the orange wire is on GPIO4/GPIO5 and the firmware was built with the same pins. |
| Servo buzzes/hunts at one end | Narrow `SERVO_MIN/MAX_PULSE_US` — [WIRING.md](WIRING.md) §7. |
| Servo turns the wrong way / offset | That's mechanical, not a bug: re-mount the horn at the position you want while the servo is at `center`. |
| Servo gets warm when idle | It's holding torque against a load, or fighting its end stop. Narrow the range or use `off`. |
| Console garbled on the UART build | Wrong baud rate, or the wrong port selected. |

---

## Notes / limits

* SG90 is an **analog, non-feedback** servo: the firmware commands an angle, it
  can never read one back. `status` shows what was *commanded*.
* Servo horns are keyed but can be re-seated: centre the servo (`center both`)
  before pressing a horn on, so "90°" really looks like the middle.
* Only two servos are configured, but `servo.c` is table-driven — add entries to
  `s_servos[]` (and bump `SERVO_COUNT`) to drive up to 8 on the S3's LEDC
  channels, sharing the same timer.
