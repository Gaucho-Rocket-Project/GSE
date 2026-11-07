# Mojave Sphinx GSE — laptop-driven radio link (ECR-01)

Replacement for the FS-i6X transmitter + FS-iA10B receiver in Section VI of the Half Cat Rocketry
*Mojave Sphinx* guidebook (HCR-5100).

```
laptop (Python GUI) --USB--> ESP32 TX node --UART--> DX-LR41 ))) 2.4 GHz LoRa (((
                                     DX-LR41 --> ESP32 pad node --> 3 servo PWM + igniter relay
```

The nodes are ESP32 devkits. **ESP32-S3-DevKitC-1 (or N16R8 "SuperMini") is the default**, and the classic
**"ESP-32S Development Board 2.4 GHz Dual-Mode WiFi+Bluetooth"** (ESP32-WROOM-32, 30/38-pin, CP2102/CH340
USB) is supported as an *optional* board variant for either node — see
[Board variants](#board-variants-esp32-s3-or-esp-32s).

## Read these in this order

| File | What it is |
|---|---|
| `ECR-01_GSE_radio_replacement.pdf` | The engineering change report: rationale, architecture, safety analysis, BOM delta, risks, and the exact handbook text to replace. **Start here.** |
| `docs/INSERT_SECTION_VI.pdf` | Insert-ready replacement pages 236–255, laid out to match HCR-5100. |
| `docs/TEST_PLAN.md` | Bench and range test gates. Nothing flies until G1–G4 are signed. |
| `docs/OPERATOR_CARD.md` | One page for the toolbox lid. |
| `hardware/wiring.md` | Pinouts, keep-alive circuit, bench bring-up checklist. |

## The one design point to understand

The FS-iA10B only listens on FlySky **AFHDS 2A**, a closed proprietary protocol. A DX-LR41 (Semtech SX1281,
LoRa/FLRC) cannot transmit it, so **the receiver cannot be kept** unless a second transmitter is added (a 4-in-1
multiprotocol module). The chosen architecture therefore drops the receiver and puts the pad node directly on the
servos — which means the pad node inherits responsibility for the receiver's failsafes. That responsibility is the
core of the ECR.

Channel numbers are unchanged from the handbook: **FILL Ch5, VEHICLE Ch6, PURGE Ch8, IGNITER Ch3.**

## Quick start

```bash
# firmware  (needs PlatformIO)
cd firmware
pio run -e tx-node  -t upload      # first ESP32-S3   (or -e tx-nucleo, or -e tx-node-esp32s)
pio run -e pad-node -t upload      # second ESP32-S3  (or -e pad-node-esp32s for an ESP-32S)

# laptop app (tkinter + pyserial)
python software/run_gui.py --sim               # no hardware: simulated pad
python software/run_gui.py --port /dev/ttyACM0 # real TX node
python software/run_gui.py --port COM7
```

## Board variants: ESP32-S3 or ESP-32S

Two ESP32 board families are supported, and the choice is a build flag rather than a code fork. The
protocol, the failsafes, the watchdog, the GUI and every procedure are identical; only the pin map and
the USB/LED plumbing differ. **A pad node on either board talks to a TX node on either board.**

| | ESP32-S3 (default) | ESP-32S (optional) |
|---|---|---|
| Board | ESP32-S3-DevKitC-1, N16R8 "SuperMini" | "ESP-32S Development Board 2.4 GHz Dual-Mode WiFi+Bluetooth" (ESP32-WROOM-32, 30/38-pin) |
| Build | `pio run -e tx-node` / `-e pad-node` | `pio run -e tx-node-esp32s` / `-e pad-node-esp32s` |
| Flag | `-D GSE_BOARD_ESP32_S3` (or nothing) | `-D GSE_BOARD_ESP32S` |
| Laptop port | native USB-CDC (`/dev/ttyACM*`) | CP2102/CH340 bridge on UART0 (`/dev/ttyUSB*`, `COM5`-style) |
| Laptop driver | none on Linux/macOS; may need the ESP32 USB driver on Windows | CP210x/CH340 driver, which every OS ships |
| Radio UART | UART1, D16/D17, AUX D18 | UART1, D16/D17 (RX2/TX2), AUX D18 |
| Status LED | WS2812 on GPIO48, five colours | on-board blue LED on GPIO2, blink pattern |
| Wiring that moves | -- | PURGE D6→**D15**, IGNITER D7→**D13**, heartbeat D8→**D33** (GPIO6–11 are SPI flash) |

### Module wiring: two things that bite

* **M0 and M1 → GND on both modules.** They are module outputs while the factory-default `AT+SWITCH=0` is in force,
  but the moment a module is in `AT+SWITCH=1` they become weakly-pulled-up *mode inputs*, and a floating pin parks the
  module in AT mode, wake-on-radio or sleep — it provisions cleanly and never receives. See
  [`hardware/wiring.md` §1.2](hardware/wiring.md#12-m0--m1-do-not-leave-them-floating).
* **LR42 and LR41 are not the same on VCC.** LR42-24T12D takes 3.3–5.5 V (5 V typical, ≥5 V for full power);
  LR41-24T12S is 3.3 V only. UART levels are 3.3 V on both. Check the silkscreen before you power the board.

Both nodes print the module's own `AT+HELP` dump at provisioning (`LOG RADIO ... SWITCH:0 ...`), so the
configuration that decides whether the link works is in the log instead of assumed.

The three pad-node pins that move do so because GPIO6..11 are wired to the SPI flash on every
ESP32-WROOM-32; an ESP32-S3 devkit can drive GPIO6, a WROOM-32 cannot. Everything else is the same wire.
`src/config.h` refuses to build a pad node that puts a servo, the relay or the heartbeat on a flash pin
or on an input-only pin (GPIO34–39), and refuses to put the igniter relay on a strapping pin. The pin
map and the full wiring table are in [`hardware/wiring.md`](hardware/wiring.md).

Neither flag is required: with no flag at all the build targets the ESP32-S3, so every existing command
and every existing board keeps working unchanged.

## If upload fails

`Failed to connect to ESP32-S3: No serial data received` is an esptool problem, not a build problem. (An
ESP-32S is easier here: its single micro-USB socket is a USB-UART bridge, so this failure basically cannot
happen -- see the board-variant section above.) See
**`docs/TROUBLESHOOTING_UPLOAD.md`** — in order: use the **UART** socket (not the native USB one), the upload
speed is already pinned to 115200, and BOOT+RST forces download mode.

## If the link won't come up

`NO PAD TELEMETRY` with only `dir=tx` rows in the CSV means nothing has ever come back from the pad. See
**`docs/TROUBLESHOOTING_LINK.md`** — the pad node now prints a boot banner, a `RADIO OK` / `RADIO FAULT` verdict
and 1 Hz counters (`bytes`, `sof`, `valid`) on its USB port, which separates a dead radio from a dead link in
about two seconds. On a Nucleo TX node, LD1 flashes per packet sent and LD2 per packet received, so the same
diagnosis takes one glance at the board.

## Tests

```bash
make -C tests run                 # 6 C tests on the wire protocol + 29 on the portable TX core
sh tests/esp32_syntax_check.sh    # every ESP32 board variant compiles (host g++, no toolchain)
python software/tests/test_protocol.py   # 9 host tests, incl. byte-identical golden frames
python software/tests/test_sim_loop.py   # 4 closed-loop tests against the pad simulator
```

All 48 currently pass, and all three firmware targets build (`tx-node`, `pad-node`, `tx-nucleo`), plus
the two optional ESP-32S variants (`tx-node-esp32s`, `pad-node-esp32s`).

## Repo map

```
firmware/                  PlatformIO project, five environments
  src/common/gse_protocol.h   wire protocol + failsafe rules (single source of truth)
  src/config.h                all builder-editable constants (board variant, pins, channels, limits)
  src/dxlr41.h                AT-command driver with verification
  src/pad/pad_node.cpp        pad node firmware
  src/common/gse_hal.h        seven platform services a TX node supplies
  src/common/gse_tx_core.c    portable laptop-side bridge (pacing, stale reject, SAFE stream)
  src/tx/tx_platform_esp32.cpp  ESP32 platform layer (ESP32-S3 and ESP-32S)
  src/stm32/tx_platform_nucleo.cpp  Nucleo-H753ZI platform layer (env tx-nucleo)
  stm32/tx_platform_stm32.c   STM32Cube HAL port + README for CubeIDE users
software/
  run_gui.py                  launcher
  gse_control/protocol.py     host mirror of the protocol (golden vectors)
  gse_control/link.py         serial transport, CSV logging, simulator link
  gse_control/sim.py          pad-node model for rehearsal and CI
  gse_control/controller.py   GUI logic (headless-testable)
  gse_control/app.py          tkinter GUI
  tests/                      protocol + closed-loop tests
tests/                        C unit tests + the ESP32 board-variant syntax check
tools/make_figures.py         regenerates the documentation figures
tools/build_ecr_pdf.py        renders ECR-01 markdown -> PDF (source of truth is the .md)
tools/finish_insert.py        stamps the HCR-5100 page chrome onto the Section VI insert
hardware/wiring.md            bench wiring sheet
docs/                         PDFs, figures, test plan, operator card
```

## Status

Draft for review. Not flight hardware until the test gates are signed and a reviewer has accepted the safety
analysis in §6 of the ECR.
