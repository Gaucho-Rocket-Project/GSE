# Link options — LoRa module or Bluetooth

*Two ways to connect the TX node to the pad node. Same protocol, same frames, same failsafes. Pick one at
build time and build **both** nodes the same way.*

---

## 1. The short version

| | **LoRa** (default) | **Bluetooth (BLE)** |
|---|---|---|
| Hardware | 2× DX-LR42-24T12D / DX-LR41-24T12S + antennas | none — the ESP32's own radio |
| Wiring | 5 wires per node + 100 µF/100 nF at the module | none |
| Provisioning | AT commands, verified at boot | none |
| Range, line of sight | hundreds of metres to ~2 km at `LEVEL 3` | roughly 10–50 m (see §4) |
| Goes through brush/terrain/vehicles | yes (sub-GHz-like 2.4 GHz LoRa at 10 kbit/s) | poorly — BLE hates obstructions |
| Interference with Wi-Fi | shares 2.4 GHz, but LoRa tolerates it well | shares 2.4 GHz, weaker link margin |
| Pairing | none | none (see §5) |
| Needs a laptop at the pad | no | no |
| Status in this repo | **default**, what ECR-01 specifies | optional, bench/short-range |
| Test plan gate | G1–G4 | T-11 (§7) takes the place of T-01/T-03 |
| Firmware flag | *(none)* | `-DGSE_LINK_BLE` |

**Default stays LoRa.** Nothing about the shipped configuration changes: the `pad-node`, `tx-node`,
`pad-node-esp32s` and `tx-node-esp32s` environments build exactly as before.

Bluetooth exists so the pair works **with no radio module at all** — a spare pad, a bench run on a
kitchen table, a launch where the modules are lost or delayed, or a quick check that a protocol change
broke nothing on the air.

---

## 2. What changes between the two, and what does not

The seam is one file: [`firmware/src/link.h`](../firmware/src/link.h). Above it, everything is
transport-agnostic; below it, two tiny adapters.

**Unchanged — byte for byte, in both builds**

- the 7E frame format, CRC-16, sequence numbers and version byte (`gse_protocol.h`);
- the pad's 1.0 s watchdog (`wdg_ds`), the failsafe output set, and the "recompute from the local clock
  every 20 ms" model;
- the arming rule: igniter only while `arm_code == 0xA5`, the link is fresh, and the on-time limit holds;
- the on-time limiter (0.5 s default), the one-pulse-per-press latch, and the hardware keep-alive;
- the TX node's pacing, its 200 ms stale-frame rejection and its 20 Hz synthetic-SAFE stream;
- the GUI, the text console (`PING`, `INFO`, `SAFE`, `RADIO CONFIG`, …) and the log format.

**Different — and only this**

- how many bytes leave one board and arrive at the other, and how they are addressed;
- the boot-banner line that names the transport;
- what `RADIO CONFIG` means (LoRa: re-write and verify the module's AT parameters; BLE: drop the
  connection and rescan).

Because the frame is the same, the pad cannot tell which transport is underneath. That is deliberate: a
fault in the safety logic cannot hide behind a change of radio, and the host tests exercise both paths.

---

## 3. Building and running it

### 3.1 Environments

| Board | Pad | TX node |
|---|---|---|
| ESP32-S3-DevKitC-1 | `pad-node-ble` | `tx-node-ble` |
| ESP-32S / WROOM-32 | `pad-node-ble-esp32s` | `tx-node-ble-esp32s` |
| NUCLEO-H753ZI | *(not supported — no Bluetooth)* | *(not supported)* |

```sh
cd firmware
pio run -e tx-node-ble         -t upload     # laptop side
pio run -e pad-node-ble        -t upload     # pad side
```

or, if you would rather keep one environment and flip a flag, `-D GSE_LINK_BLE` on both nodes. Either
way, **both ends must be BLE**. A BLE pad cannot hear a LoRa TX node and vice versa — see §6.

The BLE environments only change one thing about the build besides the flag: `lib_ldf_mode = chain`,
because the base environments set it to `off` and the `<BLEDevice.h>` include would otherwise not
resolve. Nothing else about the toolchain differs.

**Verified in this repo:** all four BLE environments pass the host syntax check against both
arduino-esp32 2.x and 3.x (`sh tests/esp32_syntax_check.sh`), and the frame-chunking rules are covered
by `make -C tests run`. The Espressif toolchain is not available in this workspace, so
**`pio run -e …-ble` has not actually been executed here** — that is the first thing to do on a machine
with PlatformIO installed (§7, T-11 step 1).

### 3.2 What the boot log says

Pad:

```
LOG PAD boot fw 1.0 board=esp32s3 pins FILL=4 VEH=5 PRG=6 IGN=7 HB=8
LOG PAD link: ble -- BLE peripheral "GSE-PAD" (svc ff00, rx ff01, tx ff02), no module, no pairing
LOG PAD LINK OK  (waiting for the TX node)
LOG PAD BLE: advertising as "GSE-PAD"; the TX node must also be built with -DGSE_LINK_BLE
```

TX node:

```
LOG link: ble -- BLE central, scanning for service ff00 for "GSE-PAD", no module, no pairing
LOG TX esp32s3 fw 1.0 ready
LOG tx fwd=812 state=79 safe=0 dropped=0 radio=ok
```

LoRa builds print `PAD link: lora -- LoRa UART1 RX<-D16 TX->D17 AUX=wired M0/M1=tie to GND at the
module` in the same place. **Read that line first when a link will not come up**: it is the one thing
that tells you two boards were built differently.

`RADIO FAULT` on a BLE build means the Bluetooth stack did not start (`BLE stack not started`) — on an
ESP32 that means the wrong board (e.g. a chip without Bluetooth), or an out-of-memory condition, not a
wiring problem.

---

## 4. Range, and why "it worked on the bench" is not range

BLE at the ESP32's default transmit power with no antenna is a **short** link: expect a working range of
tens of metres in the open, less through a vehicle, a fuselage, brush or a person. LoRa at `LEVEL 3`
(10 kbit/s) is designed for exactly the opposite regime.

Treat the numbers as a starting point for a range test, not as a specification:

| Situation | LoRa `LEVEL 3` | BLE |
|---|---|---|
| Bench, 2 m through the room | fine | fine |
| Pad at the end of a 30 m hose, line of sight | fine | usually fine |
| Through a truck / behind a berm | fine | unreliable |
| 300 m across flat desert | fine | no |
| 2 km | fine | no |

The link budget is not the only difference: BLE uses frequency hopping, connection intervals and its own
retry scheme, so its **failure** mode is not "nothing arrives" but "some notifications are dropped and
arrive late". That is harmless here — a dropped frame costs 20 ms and the pad's watchdog is 1.0 s — but
it is why the operating distance should be set by test (§7, T-11 step 5) and not by hope.

**Do not use BLE as the link for a live fill unless T-11 has been signed off for that build and the
measured range covers the actual separation with margin.** The ECR's answer for flight is still LoRa.

---

## 5. Security — read before using this off the bench

The GSE protocol has **no authentication**. Anyone in radio range who can speak it can command a pad.
That is already true of the LoRa link, which mitigates it with a private channel, a shared KEY word and
the arm code; the physical SW2 arm key and the igniter interlock are the real interlocks.

BLE changes the radio layer, so it changes what "in range" means:

- the pad **does not advertise** in a way that invites pairing; it is a peripheral that only ever
  accepts a central that subscribes to the right characteristics;
- the TX node scans for the service UUID **and** checks the advertised name, so it will not silently
  attach to a neighbour's device;
- **no pairing or bonding is used.** Enabling it would add a key store and an operator step in exchange
  for protecting a payload that already carries a CRC and a dead-man switch — and it would break the
  "flash both boards and go" property that is the whole reason this option exists.

The honest summary: on BLE, someone who deliberately targets this pad within a few tens of metres can
talk to it. On LoRa, someone who deliberately targets this pad with a compatible module and the right
channel/KEY can talk to it. Neither link is a barrier against a determined attacker; **the barrier is
procedural** — arm key out, someone on the pad, IGN unloaded until the last moment.

If that is not acceptable for a given launch, use LoRa, or use neither and wire the pad.

---

## 6. Fault-finding, BLE-specific

| Symptom | Look at |
|---|---|
| TX node logs `scanning for the pad` for ever | pad powered? pad built with `-DGSE_LINK_BLE`? both flashed from the same repo version? |
| Pad logs `advertising (no TX node connected)` for ever | is the TX node's scan running (`LOG link: ble …` at boot)? are they within range? |
| Pad connects then immediately shows NO-LINK again | TX node rebooted (USB re-enumeration) — it reconnects within ~5 s by itself |
| `ble: re-scanning for "GSE-PAD"` after `RADIO CONFIG` | expected: that is what `RADIO CONFIG` does on this transport |
| Link works, GUI shows `rejected frames` climbing | not a link fault — that is stale-frame rejection, i.e. the laptop, not the radio |
| Everything connects, pad never moves a servo | SW2 arm key, then ARM in the GUI — the link being up is not arming |
| Both nodes boot but nothing ever arrives | one of them is a LoRa build. Read the first `LOG … link:` line on each |

The pad's `PAD st=… bytes=… sof=…` heartbeat works identically on both transports; the only change is
the wording of the "nothing is arriving" hint, which names the BLE state instead of the UART pins.

---

## 7. Test plan delta (T-11)

The LoRa gate tests T-00…T-10 do not apply to a BLE build: there is no module to provision (T-01), no
channel/KEY to match (T-03), no M0/M1 (T-00). Replace them with the following, and keep T-04 (failsafe),
T-06 (igniter), T-08 and T-09 (operational) exactly as written — they are transport-independent and are
the ones that matter.

### T-11 · Bluetooth transport (B, then R)

1. **Builds.** `sh tests/esp32_syntax_check.sh` passes; `pio run -e pad-node-ble` and
   `pio run -e tx-node-ble` complete with no warnings in `src/`.
2. **Boot banners.** Pad prints `PAD link: ble …` and `PAD BLE: advertising as "GSE-PAD"`. TX node
   prints `link: ble …` and `TX … fw … ready`. Neither prints `RADIO FAULT` / `LINK FAULT`.
3. **Connect.** Within 5 s of both being powered the pad's `PAD st=…` line reads `LINK OK`, `bytes`
   climbs, `sof` climbs, `valid` climbs, and the TX node's `state=` counter climbs. GUI banner reads
   **LINK OK — NOT ARMED**.
4. **Reconnect.** Power-cycle the TX node. The pad must show `NO-LINK (safe)` (amber / 2 Hz) within
   1 s, hold all outputs safe, and return to `LINK OK` on its own within ~10 s of the TX node coming
   back — with no reset of the pad.
5. **Range (R).** Walk the TX node away until the pad shows NO-LINK, record the distance and the
   conditions (line of sight / through what). Repeat twice. **Pass:** the recorded distance exceeds the
   planned separation by at least 2×. If it does not, this transport is not usable for that launch.
6. **Watchdog (H).** With the link up and FILL commanded open, power off the TX node. FILL must close
   and VEHICLE open within `wdg_ds` (1.0 s default) — the same acceptance as T-04.
7. **Igniter (H).** With the link up, arm and fire with the IGN outlet **unloaded** (LED/scope on the
   relay contacts): one pulse, ≤ `ign_max_ds`, no repeat while the button is held. Release and re-press
   for a second pulse. Same acceptance as T-06.
8. **Wrong-pair check.** Build one node LoRa and one BLE. Neither must ever report LINK OK, and neither
   may move a servo. Reflash both to the same transport.
9. **Sign-off.** Record board serials, firmware commit, measured range, and who ran steps 5–7.

---

## 8. Where each transport appears in the repo

| File | What it says about the transport |
|---|---|
| `firmware/src/link.h` | the seam itself; both adapters; the BLE topology and its API-compatibility notes (`GseLink gseLink;` is named that way on purpose — see the note in `pad_node.cpp`: the core's `Arduino.h` brings in POSIX `link()` via lwIP, and `tests/esp32_stub/Arduino.h` now declares the same names so the host check catches it) |
| `firmware/src/config.h` | `GSE_LINK_BLE` / `GSE_LINK_LORA`, the "exactly one" rule, BLE names/UUIDs, the security note |
| `firmware/platformio.ini` | the four `-ble` environments |
| `firmware/src/pad/pad_node.cpp`, `firmware/src/tx/tx_platform_esp32.cpp` | one code path; `link.send()` / `link.read()` / `link.poll()` |
| `tests/esp32_syntax_check.sh` | both roles × both boards × both cores, plus the three new negative cases |
| `tests/esp32_stub/BLEDevice.h` | the 2.x/3.x-compatible BLE stand-in for that check |
| `tests/test_protocol.c` | chunking and lost-chunk behaviour of the BLE framing |
| `docs/TEST_PLAN.md` | T-11 (this document, §7) |
| `docs/TROUBLESHOOTING_LINK.md` | the BLE symptom table (§6) |
| `docs/OPERATOR_CARD.md` | the transport line to check at power-up |
| `hardware/wiring.md` | what you *do not* have to wire in a BLE build |
