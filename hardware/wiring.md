# Wiring and bench sheet

Everything needed to build and bench-test the two nodes before they go anywhere near the GSE box.
Pin numbers refer to `firmware/src/config.h`; change them there, not here.

---

## 0. Do you need to wire a module at all?

Only for the optional **LoRa** transport (`-DGSE_LINK_LORA` on both nodes). With default ESP-NOW builds
(`pio run -e pad-node` / `-e tx-node`), the ESP32's own Wi-Fi radio carries the link and this
whole section — module, five wires, decoupling, M0/M1, AUX, antennas — is **not needed**. Keep the two
boards' USB ports, the servo/relay wiring of §2 and the pad's safety chain; delete the rest.

| | LoRa build (optional) | ESP-NOW build (default) |
|---|---|---|
| Module on pins 3/4/5 + M0/M1 | required | none |
| Antennas | required on both modules | n/a (on-board antenna) |
| `LOG … link:` line | `lora -- LoRa UART1 RX<-D16 …` | `espnow -- ESP-NOW ch 1, local … -> peer …` |
| Before first use | provision the module with AT commands | put both boards' MACs in `src/config.h`, reflash both |
| Boot verdict | `RADIO OK` / `RADIO FAULT` | `LINK OK` / `LINK FAULT` |
| Range test | T-05 | T-11 (§7 of [docs/LINK_OPTIONS.md](../docs/LINK_OPTIONS.md)) |

Everything in §2 onwards (servos, igniter relay, keep-alive) is identical in both builds. See
[docs/LINK_OPTIONS.md](../docs/LINK_OPTIONS.md) for provisioning and the trade-off.

---

## 1. Node ↔ radio module (DX-LR42-24T12D / DX-LR41-24T12S)

Both nodes use the same five-wire connection, on every board we support. The TX node may be an ESP32-S3,
an **ESP-32S** (ESP32-WROOM-32 devkit) or a NUCLEO-H753ZI — the pad does not know or care which, and the
pad node itself may be built on either ESP32 board. See
[§1.3 ESP-32S variant](#13-esp-32s-variant-optional-board) for the three pins that move.

| Module pin (LR42) | ESP32-S3 | ESP-32S | Notes |
|---|---|---|---|
| 6 — VCC | 5V **or** 3V3 | 5V **or** 3V3 | **LR42: 3.3–5.5 V, 5 V typical, ≥5 V for full power. LR41: 3.3 V only.** UART is 3.3 V either way — read the paragraph below before powering |
| 7 — GND | GND | GND | common ground, including the servo/12 V return |
| 3 — RX (data **in**) | **GPIO 17** (TX) | **GPIO 17** = TX2 | crossed: module in ← ESP32 out |
| 4 — TX (data **out**) | **GPIO 16** (RX) | **GPIO 16** = RX2 | crossed: module out → ESP32 in |
| 5 — AUX | GPIO 18 (or leave off, see §1.1a) | GPIO 18 | input; high = module busy. **Never** tied to GND/3V3 |
| 1 — M0 | **tie to GND** | **tie to GND** | never leave floating — see [§1.2](#12-m0--m1-do-not-leave-them-floating) |
| 2 — M1 | **tie to GND** | **tie to GND** | ditto |

Add 100 µF electrolytic + 100 nF ceramic across VCC/GND at the module. Antenna **must** be fitted before power-up.

### 1.1 The vendor's own reference circuit (LR42 manual, Figure 6)

Worth comparing against your harness, because it is the picture of "correct":

```
                          DX-LR42-24T12D                     ESP32
   VIN ──┬── 100 µF ──┐   1  M0  <────────────────────── IO   (or GND)
         ├── 0.1 µF ──┴── 6  VCC  2  M1  <────────────── IO   (or GND)
         │               7  GND  3  RX  <────────────── TXD
        GND                     4  TX  ──────────────> RXD
                                5  AUX ──────────────> IO
```

Four things this picture makes explicit, and they are the four things that go wrong:

1. **The serial lines are crossed.** Module `RX` (pin 3, "serial port data **input**") is fed from the MCU's TX; module
   `TX` (pin 4, "data **output**") feeds the MCU's RX. On our boards that is **module RX ← D17** and **module TX → D16**.
   Getting this backwards is the single most common wiring error, and it looks like a dead link because the module
   receives nothing while happily sending.
2. **AUX goes to an input**, and is only a status output (high = transmitting / receiving / switching mode). The
   vendor also feeds M0/M1 from the MCU; on our nodes they are **not** driven — tie them to GND (see §1.2).
3. **Both capacitors go at the module**, not at the regulator: 100 µF electrolytic **and** 100 nF ceramic, close to VCC
   and GND pins, because the RF burst pulls a current spike the supply impedance has to absorb.
4. **VIN is 5 V for an LR42** (3.3–5.5 V; ≥5 V for full power) — but the UART is 3.3 V, which is what the ESP32 gives.

### 1.1a If AUX is not wired

AUX is optional in practice. Leave GPIO18 unconnected, or build with `-DGSE_LORA_AUX_NONE` (or set `PIN_LORA_AUX` to
`-1`) and the firmware stops polling it. Both nodes print which is the case at boot:

```
LOG PAD radio wiring: RX<-D16 TX->D17 AUX=NOT WIRED (pacing only) M0/M1=tie to GND at the module
```

What you give up is flow control: the driver can no longer wait for "module busy" to clear, so the 20 Hz command
stream relies on pacing alone. At the default LEVEL 3 a 22-byte frame is ~27 ms of a 50 ms slot, so this is fine; at
LEVEL 8/FLRC it is still fine (air time is shorter). It is *not* fine for anything that bursts — do not change the
command rate without wiring AUX. **Never** tie AUX to GND or 3V3: it is a push-pull output from the module.

> **3.3 V or 5 V depends on which module you actually have — check the silkscreen.**
> * **DX-LR42-24T12D** (SX1281, 7-pin stamp module): VCC accepts **3.3–5.5 V, 5 V typical**, and ≥5 V is what guarantees
>   the full +13 dBm. UART levels are still **3.3 V** and the datasheet says 5 V TTL "risks burning" it — so power it
>   from 5 V if you like, but keep D16/D17/D18 at 3.3 V (they are, on both devkits).
> * **DX-LR41-24T12S** (what this ECR specifies): **3.3 V only.**
>
> The board you are holding may say either. Feeding an LR41 5 V is the fastest way to make it answer AT commands and
> then behave erratically; feeding an LR42 3.3 V works but gives up output power. Neither mistake is recoverable by
> re-flashing.

Serial parameters after provisioning: **9600 8N1**, AT commands end with `\r\n`, `+++` toggles AT mode.

> **Settings do not survive a reset.** `AT+RESET` is a reboot, not a save. Every parameter the firmware sets is
> verified with `AT+HELP` before *and after* leaving AT mode, and a mismatch is reported as a named
> `RADIO FAULT`. Never issue `AT+RESET` from a terminal and expect the module to keep its configuration; re-flash
> or re-run provisioning instead.

### 1.2 M0 / M1 do not leave them floating

**Short answer: unconnected is only safe if you are certain the module is (and stays) in `AT+SWITCH=0`.** Tie both pins
to GND and the question disappears. This costs two pieces of wire and is the difference between a link that works and
one that "provisions fine but never receives".

From the vendor documentation (LR42 technical manual v2.0 §2.3.4, and the same text in the LR41 application guide):

* `AT+SWITCH=0` (the **factory default**): M0/M1 are **outputs** from the module, driven low. Mode switching happens
  over the UART with `AT+SLEEP` / `+++`. This is the scheme the firmware uses, so the pins are electrically irrelevant
  *in this state* and leaving them unconnected appears to work.
* `AT+SWITCH=1`: M0/M1 become **inputs with a weak pull-up**, and they select the operating mode:

  | M0 | M1 | Mode |
  |---|---|---|
  | low | low | high-timeliness (normal transmit/receive) — **what you want** |
  | low | high | air-wakeup (WOR) — **receives nothing on a normal frame** |
  | high | low | AT command mode — **receives nothing** |
  | high | high | sleep — **receives nothing** |

  The manual is explicit: with `SWITCH=1` the pins are weakly pulled up, and *"you must never leave the pins floating,
  or mode switching will malfunction."* Floating inputs sit at the weak pull-up level, which is **M0=1, M1=1 = sleep**,
  or they pick up noise from the servo harness and land in any of the four states. And the mode is re-sampled **at every
  power-up and every `AT+RESET`**, of which the firmware's provisioning does several.

So the failure mode that matches your symptom exactly is: the module sits in AT or sleep mode while the firmware is
happily writing command frames into its UART. `AT+HELP` still answers, provisioning still verifies, `RADIO OK` still
prints — and the pad's `bytes=` counter stays at 0 forever.

**What to do**

1. Wire **M0 → GND** and **M1 → GND** on both modules (directly at the module; a 10 kΩ to GND is fine if you prefer
   not to solder to the pin, but a direct link is better).
2. Optionally let the MCU hold them: `firmware/src/config.h` has `PIN_LORA_M0` / `PIN_LORA_M1`, currently `-1`
   (not driven). Set them to spare GPIOs (e.g. `-1` → `4`/`5` on a TX node, which uses no servos) and the driver drives
   them low in `begin()` — belt and braces, and it survives a module that was left in `SWITCH=1`.
3. Confirm the module's own view of it in the boot log. The firmware now prints the module's `AT+HELP` dump:

   ```
   LOG RADIO ...
   LOG | SWITCH:0 (modes set by AT command)      <- from a separate AT+SWITCH query
   LOG PAD RADIO OK  (waiting for the TX node)
   ```

   `SWITCH:` must be `0`. If it reads `1`, clear it with `AT+SWITCH0` (it reboots), then tie the pins low anyway.

> **Do not** try to drive M0/M1 from the MCU while the module is in `AT+SWITCH=0`: in that state the pins are the
> module's **outputs**, and an ESP32 driving them low/high against an output is a contention path you do not want on a
> radio that decides whether the igniter can fire. GND (or a GPIO held low, which is the same electrical state) is safe;
> toggling them is not.

### 1.3 ESP-32S variant (optional board)

"ESP-32S Development Board 2.4 GHz Dual-Mode WiFi+Bluetooth" is the classic 30/38-pin ESP32-WROOM-32
devkit. It runs the same firmware, from the same source, on the same wires — with three exceptions, all of
them because **GPIO6…GPIO11 are the SPI flash** on a WROOM-32 and cannot be used:

| Function | ESP32-S3 build | ESP-32S build | Why it moved |
|---|---|---|---|
| FILL servo | GPIO 4 | GPIO 4 (D4) | unchanged |
| VEHICLE servo | GPIO 5 | GPIO 5 (D5) | unchanged |
| PURGE servo | GPIO 6 | **GPIO 15 (D15)** | GPIO6 is flash |
| Igniter relay | GPIO 7 | **GPIO 13 (D13)** | GPIO7 is flash; D14 emits a PWM burst at boot |
| Heartbeat (Q1) | GPIO 8 | **GPIO 33 (D33)** | GPIO8 is flash |
| Radio RX / TX / AUX | 16 / 17 / 18 | 16 / 17 / 18 (RX2/TX2/D18) | unchanged |
| Laptop | native USB-CDC | CP2102/CH340 on the micro-USB socket | the board has a bridge chip |
| Status LED | WS2812 GPIO 48 | on-board blue LED GPIO 2 | the board has no WS2812 |

Notes for this board:

* **Build it with the variant flag**: `pio run -e pad-node-esp32s -t upload` (or `-e tx-node-esp32s`). The
  flag is `-D GSE_BOARD_ESP32S`; `src/config.h` then picks the pin map above.
* **D15 (PURGE), D13 (IGN) and D33 (heartbeat) are near the S3 board's GPIO6/7/8** on the header, so the
  harness is relabelled, it is not redesigned.
* **The igniter is deliberately on D13, not "the next pin along".** D14 emits a short PWM burst at boot,
  D5 and D15 are strapping pins (and D15 emits the ROM boot log), D0/D2/D12 strap too. A PWM burst on the
  relay gate at power-up is hazard H4 in the ECR; D13 has no boot function, no strap and no reset artefact.
  `config.h` fails the build if the relay lands on GPIO0/2/12. A servo shrugs off a boot glitch, which is
  why PURGE may sit on D15 — the relay may not.
* **GPIO34–39 are input-only** and cannot drive a servo or the relay; `config.h` rejects them too.
* **Power:** the same 3.3 V rule. Feed the DX-LR41 from the board's 3V3 pin, never from 5 V, and keep the
  100 µF + 100 nF across the module — the ESP32's Wi-Fi/BT radio is idle in this firmware (never
  started), so the supply is actually quieter than on the S3, but the LR41's own TX burst still needs it.
* **USB:** the ESP-32S has one micro-USB socket behind a CP2102/CH340, so it behaves like the S3's
  *"UART"* socket all the time: no `Failed to connect … No serial data received` class of problem, and a
  normal `COM5`-style port for both flashing and the GUI. Install the CP210x or CH340 driver once if
  Windows does not offer one.
* **Status LED:** one blue LED, so the colours of the S3 build become blink patterns —
  5 Hz = **RADIO FAULT**, 2 Hz = **no link**, 1 Hz = **link OK, not armed**, 10 Hz = **igniter
  energised**, solid = booting. If your clone's LED is inverted (lit when it should be dark), build with
  `-D GSE_STATUS_LED_ACTIVE_LOW=1`.
* **Nothing else changes.** Same protocol, same failsafes, same 1.0 s pad watchdog, same 300 ms laptop
  dead-man, same channel map, same GUI, same test plan. A mixed pair (ESP-32S pad + ESP32-S3 TX, or the
  reverse) is a supported configuration and is what the syntax check builds.

### 1.4 Same module on a Nucleo-H753ZI TX node

| DX-LR41 pin | Nucleo pin | Morpho | Notes |
|---|---|---|---|
| VCC (3.3 V) | 3V3 | CN11 pin 5 | LR41: 3.3 V only. LR42 accepts 3.3–5.5 V, but keep it on 3V3 here |
| GND | GND | CN11 pin 8 | |
| TXD | PD6 | CN11 pin 43 | USART2_RX — crossed |
| RXD | PD5 | CN11 pin 41 | USART2_TX — crossed |
| AUX | PD7 | CN11 pin 45 | input, pull-up |
| M0 / M1 | **tie low** | — | as above, §1.2 — do not leave floating |

Everything else is unchanged: the same six wires, the same 100 µF + 100 nF across the module, the same antenna
rule. The laptop connects through the ST-LINK USB-C connector (USART3, PD8/PD9, via SB5/SB6 at their default ON
position) and appears as an ordinary COM port. Full detail in `firmware/stm32/README.md`.

---

## 2. Pad node pinout

| Function | GPIO (ESP32-S3) | GPIO (ESP-32S) | Direction | Load |
|---|---|---|---|---|
| FILL servo | 4 | 4 (D4) | out, 50 Hz PWM | DS3225, 5 V |
| VEHICLE servos (Y-harness) | 5 | 5 (D5) | out, 50 Hz PWM | 2 × DS3225, 5 V |
| PURGE servo | 6 | **15 (D15)** | out, 50 Hz PWM | DS3225, 5 V |
| Igniter relay (Q2 gate) | 7 | **13 (D13)** | out | 10 kΩ to GND, gate of IRLZ44N |
| Keep-alive heartbeat | 8 | **33 (D33)** | out, 25 Hz | input to an independently designed and tested edge-sensitive watchdog |
| UART RX / TX / AUX | 16 / 17 / 18 | 16 / 17 / 18 | see above | module |
| Status LED | 48 (WS2812) | 2 (plain blue) | out | on-board indicator |

Both columns are compiled in `firmware/src/config.h`: the S3 map is the default, the ESP-32S map is
selected by `-D GSE_BOARD_ESP32S` (envs `*-esp32s`). The firmware will not build a pad node that puts a
servo, the relay or the heartbeat on GPIO6–11 (flash) or GPIO34–39 (input-only), or that puts the igniter
on a strapping pin — so a pin mistake is a compile error, not a bench surprise.

### Independent hardware keep-alive

**Do not use the previously illustrated GPIO → 100 kΩ → gate with a 10 µF
capacitor to ground as a missing-heartbeat detector.** A GPIO stuck high can
charge the capacitor and keep the gate on. That RC circuit does not prove a
safe state for a hung MCU.

The heartbeat pin toggles approximately every 20 ms while the pad loop runs.
A qualified hardware reviewer must design and verify an independent
edge-sensitive watchdog and fail-safe ignition power disconnect, including
stuck-high, stuck-low, power-loss and startup behavior. No circuit in this
repository has been verified to satisfy that requirement. Keep the igniter
and hazardous systems disconnected until the independent safety gates pass.

### Igniter chain

```
+12 V ── F1 (5 A) ── SW1 ──┬── DROK 12→5 V ── logic
                           └── SW2 (arm key) ── K1 contact (NO) ── IGN outlet +
K1 coil + ── +12 V (after SW2)
K1 coil − ── qualified, independently fail-safe driver and heartbeat interlock
GPIO7 (ESP-32S: GPIO13) → driver control, not directly to the 12 V coil
Coil suppression and ratings must match the actual relay and driver
```

---

## 3. Flashing the nodes

ESP32-S3 devkits have two USB-C sockets and they are **not** interchangeable for flashing:

| Task | Use |
|---|---|
| `pio run -e pad-node -t upload` | the **UART** socket (`/dev/ttyUSB*`, `COM5`-style) |
| `pio device monitor`, the GUI, `INFO` | the **native USB** socket (`/dev/ttyACM*`, `COM7`-style) |
| `pio run -e pad-node-esp32s -t upload` | the ESP-32S's only socket — flash *and* GUI (`COM5`/`/dev/ttyUSB*`) |

An **ESP-32S** has a single micro-USB socket behind a CP2102/CH340, and it is both the flash port and the
GUI port — there is no second socket to get wrong, and uploads at 115200 are routine. Use
`pio run -e pad-node-esp32s -t upload` (or `-e tx-node-esp32s`) so the pin map and the board definition
match the hardware.

Upload speed is pinned to 115200 in `platformio.ini` because that is the only speed that survives every cable
we have tried. If the upload fails with **"No serial data received"**, that is esptool failing to enter the ROM
bootloader — see `docs/TROUBLESHOOTING_UPLOAD.md`. Nothing needs changing in the firmware.

Unplug the DX-LR41 and every servo before flashing a pad node, and never flash one with an igniter on the IGN
outlet or a pressure source connected.

## 4. Bench bring-up checklist

Do this with a **lamp or resistor** on the IGN outlet and **no pressure source** connected.

### Wiring check before anything else

The checklist that catches this class of fault in a minute, on both modules:

| # | Check | Right answer |
|---|---|---|
| 1 | Module RX (3) ← ESP32 **D17** | crossed, module pin 3 |
| 2 | Module TX (4) → ESP32 **D16** | crossed, module pin 4 |
| 3 | VCC at the module | LR42: 5 V (3.3–5.5 V); LR41: 3.3 V |
| 4 | M0 (1) and M1 (2) | **tied to GND** on both modules |
| 5 | AUX (5) | wired to D18, or left floating + `-DGSE_LORA_AUX_NONE` |
| 6 | Antenna | fitted on **both** modules before power-up |
| 7 | Boot log | `link: lora …`, `AUX=wired` (or "NOT WIRED"), `RADIO OK`, `SWITCH=0` |

1. **Power the TX node only** → open a terminal at 115200 → expect `LOG TX esp32s3 fw 1.0 ready`
   (`LOG TX esp32s fw 1.0 ready` on an ESP-32S, `LOG TX stm32 fw 1.0 ready` on a Nucleo), or
   `RADIO FAULT …` — if so, the module wiring or its baud rate is wrong. Typing `INFO` re-prints the same
   line with counters. The banner names the board the firmware was built for; if it does not match the
   board in your hand you flashed the wrong environment.
2. **Power the pad node** → expect `LOG` lines on its own USB port, starting with
   `LOG PAD boot fw 1.0 board=esp32s3 pins FILL=4 VEH=5 PRG=6 IGN=7 HB=8`
   (`board=esp32s … PRG=15 IGN=13 HB=33` on an ESP-32S), then
   `LOG PAD link: lora -- LoRa UART1 RX<-D16 TX->D17 AUX=wired M0/M1=tie to GND at the module`,
   then `RADIO OK` / `RADIO FAULT`, then the module's own `LOG RADIO` banner. (An ESP-NOW build prints
   `LOG PAD link: espnow -- … local MAC …` there instead and has no module banner — see §0. On the very
   first flash that line says `peer MAC NOT CONFIGURED`, which is how you read the MACs; see
   [docs/LINK_OPTIONS.md](../docs/LINK_OPTIONS.md).) Check that the printed pins are
   where your harness actually is before you connect a servo, and that `AUX=` matches reality.
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
