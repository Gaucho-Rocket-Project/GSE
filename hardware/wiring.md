# Wiring and bench sheet

Everything needed to build and bench-test the two nodes before they go anywhere near the GSE box.
Pin numbers refer to `firmware/src/config.h`; change them there, not here.

---

## 1. Node ↔ DX-LR41-24T12S

Both nodes use the same five-wire connection. The TX node may be either an ESP32-S3 or a NUCLEO-H753ZI —
the pad does not know or care which.

| DX-LR41 pin | ESP32-S3 | Notes |
|---|---|---|
| VCC (3.3 V) | 3V3 | **3.3 V only.** If your carrier board has a regulator and an "5 V" pad, use the 3V3 pad. |
| GND | GND | |
| TXD | GPIO 16 (RX) | crossed |
| RXD | GPIO 17 (TX) | crossed |
| AUX | GPIO 18 | input, pull-up; high = module busy |
| M0 / M1 | tied low | only relevant if `AT+SWITCH=1`; the firmware uses AT mode |

Add 100 µF electrolytic + 100 nF ceramic across VCC/GND at the module. Antenna **must** be fitted before power-up.

Serial parameters after provisioning: **9600 8N1**, AT commands end with `\r\n`, `+++` toggles AT mode.

> **Settings do not survive a reset.** `AT+RESET` is a reboot, not a save. Every parameter the firmware sets is
> verified with `AT+HELP` before *and after* leaving AT mode, and a mismatch is reported as a named
> `RADIO FAULT`. Never issue `AT+RESET` from a terminal and expect the module to keep its configuration; re-flash
> or re-run provisioning instead.

### Same module on a Nucleo-H753ZI TX node

| DX-LR41 pin | Nucleo pin | Morpho | Notes |
|---|---|---|---|
| VCC (3.3 V) | 3V3 | CN11 pin 5 | 3.3 V only |
| GND | GND | CN11 pin 8 | |
| TXD | PD6 | CN11 pin 43 | USART2_RX — crossed |
| RXD | PD5 | CN11 pin 41 | USART2_TX — crossed |
| AUX | PD7 | CN11 pin 45 | input, pull-up |
| M0 / M1 | tied low | — | as above |

Everything else is unchanged: the same six wires, the same 100 µF + 100 nF across the module, the same antenna
rule. The laptop connects through the ST-LINK USB-C connector (USART3, PD8/PD9, via SB5/SB6 at their default ON
position) and appears as an ordinary COM port. Full detail in `firmware/stm32/README.md`.

---

## 2. Pad node pinout

| Function | GPIO | Direction | Load |
|---|---|---|---|
| FILL servo | 4 | out, 50 Hz PWM | DS3225, 5 V |
| VEHICLE servos (Y-harness) | 5 | out, 50 Hz PWM | 2 × DS3225, 5 V |
| PURGE servo | 6 | out, 50 Hz PWM | DS3225, 5 V |
| Igniter relay (Q2 gate) | 7 | out | 10 kΩ to GND, gate of IRLZ44N |
| Keep-alive heartbeat (Q1 gate) | 8 | out, 25 Hz | through the RC pump below |
| UART RX / TX / AUX | 16 / 17 / 18 | see above | module |
| Status LED (optional) | 48 | out | WS2812 on most devkits |

### Keep-alive pump (Q1)

```
D8 ── 100 kΩ ──┬── 10 µF ── GND
               └── gate Q1 (IRLZ44N)
```
Each D8 edge tops up the capacitor; with D8 stuck high or low the gate falls below Vth in ≈ 50 ms and Q1 opens,
removing the relay coil ground. This protects against a *hung* MCU, not against a commanded-off state — that is the
firmware's job.

### Igniter chain

```
+12 V ── F1 (5 A) ── SW1 ──┬── DROK 12→5 V ── logic
                           └── SW2 (arm key) ── K1 contact (NO) ── IGN outlet +
K1 coil + ── +12 V (after SW2)
K1 coil − ── Q2 drain        Q2 gate = D7, source = Q1 drain
Q1 drain                     Q1 source = battery −
1N5819 across the coil, cathode to +12 V
```

---

## 3. Flashing the nodes

ESP32-S3 devkits have two USB-C sockets and they are **not** interchangeable for flashing:

| Task | Use |
|---|---|
| `pio run -e pad-node -t upload` | the **UART** socket (`/dev/ttyUSB*`, `COM5`-style) |
| `pio device monitor`, the GUI, `INFO` | the **native USB** socket (`/dev/ttyACM*`, `COM7`-style) |

Upload speed is pinned to 115200 in `platformio.ini` because that is the only speed that survives every cable
we have tried. If the upload fails with **"No serial data received"**, that is esptool failing to enter the ROM
bootloader — see `docs/TROUBLESHOOTING_UPLOAD.md`. Nothing needs changing in the firmware.

Unplug the DX-LR41 and every servo before flashing a pad node, and never flash one with an igniter on the IGN
outlet or a pressure source connected.

## 4. Bench bring-up checklist

Do this with a **lamp or resistor** on the IGN outlet and **no pressure source** connected.

1. **Power the TX node only** → open a terminal at 115200 → expect `LOG TX esp32 fw 1.0 ready`, or
   `LOG TX stm32 fw 1.0 ready` on a Nucleo (or `RADIO FAULT …` — if so, the module wiring or its baud rate is
   wrong). Typing `INFO` re-prints the same line with counters.
2. **Power the pad node** → expect `LOG` lines on its own USB port; the pad prints nothing on success, so check
   the GUI instead.
3. **Start the GUI** → `python software/run_gui.py` → select the TX port → Connect → banner should turn amber
   (`LINK OK — NOT ARMED`) within a second.
4. **Read the diagnostics** in the read-back panel: `valid CMDs` climbing, `rejected frames` 0, `age` < 100 ms.
5. **T-04 failsafe matrix** — see `docs/TEST_PLAN.md`. Do not skip.
6. **Servo sweep** — connect one servo at a time, set travel, confirm the mechanical stops.
7. **Igniter test** — lamp/resistor only, ARM on/off, hold-to-fire, one pulse per press.

### If the pads don't talk

| Symptom | Most likely cause |
|---|---|
| `AT entry failed` in the boot log | module not powered, UART not crossed, or module already in AT mode at a different baud |
| `AT+LEVEL rejected` | module is a different DX-LR41 revision; read `AT+HELP` and adjust `dxlr41.h` |
| GUI never leaves `NO PAD TELEMETRY` | LEVEL/KEY/channel mismatch between the two modules |
| `rejected frames` climbing fast | RF interference, or one module provisioned at a different LEVEL |
| Works at 1 m, dies at 20 m | antenna not fitted, or modules sharing a channel with something else — run T-05 |

### USB power bank

If the laptop's port is unreliable, power the TX node from a 5 V / 2 A USB power bank and keep the *data* path on a
short cable. The node draws ≈ 90 mA idle, ≈ 200 mA while transmitting.

---

## 5. Provisioning a spare module

1. Connect the spare to the pad node's UART header (module only, no other changes).
2. From the GUI's terminal or any serial console at 115200:
   ```
   SET KEY 51042
   RADIO CONFIG
   ```
3. The boot log must print `RADIO OK`. Power-cycle and confirm with `AT+HELP` that `MAC=00,02` and
   `Frequency:2424MHz`.
4. Label the module: address, channel, key, date.
