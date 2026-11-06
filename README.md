# Mojave Sphinx GSE — laptop-driven radio link (ECR-01)

Replacement for the FS-i6X transmitter + FS-iA10B receiver in Section VI of the Half Cat Rocketry
*Mojave Sphinx* guidebook (HCR-5100).

```
laptop (Python GUI) --USB-C--> ESP32-S3 TX node --UART--> DX-LR41 ))) 2.4 GHz LoRa (((
                                    DX-LR41 --> ESP32-S3 pad node --> 3 servo PWM + igniter relay
```

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
pio run -e tx-node  -t upload      # first ESP32-S3 (or: pio run -e tx-nucleo -t upload)
pio run -e pad-node -t upload      # second ESP32-S3

# laptop app (tkinter + pyserial)
python software/run_gui.py --sim               # no hardware: simulated pad
python software/run_gui.py --port /dev/ttyACM0 # real TX node
python software/run_gui.py --port COM7
```

## If upload fails

`Failed to connect to ESP32-S3: No serial data received` is an esptool problem, not a build problem. See
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
python software/tests/test_protocol.py   # 9 host tests, incl. byte-identical golden frames
python software/tests/test_sim_loop.py   # 4 closed-loop tests against the pad simulator
```

All 48 currently pass, and all three firmware targets build (`tx-node`, `pad-node`, `tx-nucleo`).

## Repo map

```
firmware/                  PlatformIO project, three environments
  src/common/gse_protocol.h   wire protocol + failsafe rules (single source of truth)
  src/config.h                all builder-editable constants (pins, channels, limits)
  src/dxlr41.h                AT-command driver with verification
  src/pad/pad_node.cpp        pad node firmware
  src/common/gse_hal.h        seven platform services a TX node supplies
  src/common/gse_tx_core.c    portable laptop-side bridge (pacing, stale reject, SAFE stream)
  src/tx/tx_platform_esp32.cpp  ESP32-S3 platform layer
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
tests/                        C unit tests
tools/make_figures.py         regenerates the documentation figures
tools/build_ecr_pdf.py        renders ECR-01 markdown -> PDF (source of truth is the .md)
tools/finish_insert.py        stamps the HCR-5100 page chrome onto the Section VI insert
hardware/wiring.md            bench wiring sheet
docs/                         PDFs, figures, test plan, operator card
```

## Status

Draft for review. Not flight hardware until the test gates are signed and a reviewer has accepted the safety
analysis in §6 of the ECR.
