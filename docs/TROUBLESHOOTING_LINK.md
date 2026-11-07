# No pad telemetry — the link is up but no STATE frames arrive

```
banner: NO PAD TELEMETRY
CSV:    only rows with dir=tx  (no rx STATE, no rx EVENT, not even rx HELLO)
```

The TX node is transmitting, the port is open, and the laptop is happily sending 20 CMD/s. **Nothing has ever
come back.** That narrows it fast: the laptop↔TX half is proven, so the fault is in the TX→pad→TX half.

Before anything else, check the TX node's own log. It prints these lines even when the GUI is connected:

```
LOG TX esp32s3 fw 1.0 ready                       <- radio_provision() succeeded
                                                  (TX esp32s on an ESP-32S board,
                                                   TX stm32 on a Nucleo)
LOG RADIO AT+HELP dump: MODE:1 LEVEL:3 ...         <- what the module says it is
LOG | Frequency:2424MHz >> 0e
LOG | MAC:00,01  SLEEP:2
LOG | SWITCH:0 (modes set by AT command)          <- the AT+SWITCH answer
LOG tx fwd=812 state=0 safe=0 dropped=0 radio=ok  <- once a second
```

The `LOG RADIO` block is new and is the first thing to read: it is the module's own configuration as it reported it
during provisioning, printed on **both** nodes whether or not the verification passed. The `SWITCH:` line at the end
of it is the module's answer to a separate `AT+SWITCH` query -- v2.0's `AT+HELP` banner does not list SWITCH at all, so
the driver asks and appends it. `SWITCH:1` is a fault (see §2a).

* `radio=FAULT` on the TX node → **stop here**, it is the TX module's wiring or baud (see §1).

**No terminal handy?** On the Nucleo TX node the three user LEDs tell the same story at a glance: **LD1 green
flashes per packet sent, LD2 yellow per packet received, LD3 red solid = radio fault**. LD1 flickering with LD2
dark is this exact symptom. See `firmware/stm32/README.md`.
* `state=0` climbing forever → the TX node has never received anything. §2 or §3.
* `state=` climbing but the GUI still says NO PAD TELEMETRY → the frames are arriving corrupt. §4.

---

## The bug this firmware used to have (now fixed)

`DxLr41::configure()` finished with:

```c
if (!at("AT+RESET", "Power On", 900)) return F("AT+RESET rejected");
```

`AT+RESET` is a **reboot**, not a save, and the receive parameters (`MODE`, `LEVEL`, `CHANNEL`, `MAC`, `KEY`) are
not written to the module's non-volatile store by the set commands. So the sequence

1. `AT+MODE1`, `AT+CHANNEL0C`, `AT+MAC00,02` … all answer `OK`
2. `AT+RESET` → `Power On`
3. module comes back up on whatever was last **saved**

…silently undid the provisioning. Each node then transmitted on its *own* channel (`0x0C` for the pad, `0x0E` for
the TX) and in *transparent* mode, so the two never met. Everything looked configured; nothing was.

**If you flashed before this fix, re-flash both boards.** The firmware now verifies with `AT+HELP` before leaving
AT mode **and again after the reboot**, and refuses to continue with a named fault:

```
LOG PAD RADIO FAULT: CHANNEL did not stick
LOG PAD RADIO FAULT: reset wiped MODE (config not saved)
```

The GUI banner shows the same string instead of the generic `NO PAD TELEMETRY`.

---

## 0. Which transport is this build? (read this first)

There are two links now — the LoRa module (default) and Bluetooth Low Energy
(`-DGSE_LINK_BLE`). **Everything below §1 assumes LoRa.** If either node is a BLE build, §1 (modules),
§2 (UART), §2-0 (crossed wires), §2a (M0/M1) and §3 (AT mode) do not apply to it at all: there is no
module and no UART.

The very first line each node prints says which link it has:

```
LOG PAD link: lora -- LoRa UART1 RX<-D16 TX->D17 AUX=wired M0/M1=tie to GND at the module
LOG link: lora -- LoRa UART1 RX<-D16 TX->D17 AUX=wired M0/M1=tie to GND at the module
```

```
LOG PAD link: ble -- BLE peripheral "GSE-PAD" (svc ff00, rx ff01, tx ff02), no module, no pairing
LOG link: ble -- BLE central, scanning for service ff00 for "GSE-PAD", no module, no pairing
```

**If the two lines disagree, nothing below will help — reflash both boards from the same configuration.**
A LoRa pad and a BLE TX node cannot hear each other, by construction, and they will both look perfectly
healthy while the pad sits in `NO-LINK` for ever.

For BLE-specific fault-finding see [LINK_OPTIONS.md](LINK_OPTIONS.md) §6; the short form:

| Symptom on a BLE build | Verdict |
|---|---|
| TX node logs `scanning for the pad` for ever | pad not powered, not a BLE build, or out of range |
| Pad logs `advertising (no TX node connected)` for ever | mirror of the above; check the TX node's scan line |
| Pad cycled from `LINK OK` back to `NO-LINK` about every reconnect | TX node rebooted (USB re-enumeration or power glitch); it re-scans on its own within ~5 s |
| `LINK FAULT: BLE stack not started` at boot | Bluetooth did not initialise on that board — wrong chip, or the `-ble` env built for a board without BT |
| `ble: re-scanning for "GSE-PAD"` after `RADIO CONFIG` | expected — that is what `RADIO CONFIG` means on this transport |

---

## 1. Prove each module independently (two minutes, no firmware)

This is the test that isolates a module fault from a link fault. Do it with a USB-TTL adapter, or by using the
board itself as a serial bridge — **unplug the other node first**.

On the *pad* board, open its **native USB** port at 115200 in any terminal and watch `LOG` lines, or open the
module's UART directly at **9600** and type:

```
+++                ->  Entry AT
AT+HELP            ->  a banner; read these five fields
```

The banner must show, on the pad:

| Field | Pad must say | TX must say |
|---|---|---|
| `MODE:` | `1` (fixed point) | `1` |
| `LEVEL:` | `3` | `3` |
| `Frequency:` | `2424MHz >> 0c` | `2428MHz >> 0e` |
| `MAC:` | `00,02` | `00,01` |
| `Openkey:` | `1` | `1` |

Any mismatch is the whole fault. Two modules on the **same** channel is also wrong — with `MODE:0` (transparent)
both ends transmit and receive on one channel and the 20 Hz CMD stream collides with the 10 Hz STATE stream, which
reads as "sometimes telemetry". Set `MODE:1` and the channels above.

Leave AT mode with `+++` (the module reboots):

```
+++                ->  Exit AT / Power On
```

## 2-0. Is the UART crossed the right way? (check this first)

The vendor's own reference circuit (LR42 technical manual, Figure 6) is unambiguous, and it is the most common wiring
error on this link:

```
module pin 3 "RX  — serial port data INPUT"   <── ESP32 D17 (TX)
module pin 4 "TX  — serial port data OUTPUT"  ──> ESP32 D16 (RX)
```

So **module RX goes to the ESP32's TX**, and **module TX goes to the ESP32's RX**. "RX to RX, TX to TX" is the mistake
that looks most natural and is wrong.

What it costs you: the module's transmitter still fires (it does not care what is on its RX pin), so the link *looks*
alive, but the module never receives a byte of configuration or data. Provisioning then fails at the first AT command
(`AT entry failed`) because the module never heard `+++`.

Quick test that does not need the firmware: unplug the two signal wires, and with the module powered, jumper **module
RX to the ESP32's D17** only (leave TX off), then start a terminal at 9600 on the module's UART and type `AT`. If it
answers `OK`, the direction you have is right. If it answers nothing, swap to the other pin.

## 2a. M0 / M1 floating (a module that provisions but never receives)

Check this *before* the wiring table below, because it produces the exact symptom and touches no wire you have
already checked. Full background in `hardware/wiring.md` §1.2.

The failure: with the vendor's factory default `AT+SWITCH=0`, M0/M1 are **outputs** from the module, so leaving them
unconnected looks fine and everything passes. If the module is (or ever was) in `AT+SWITCH=1`, they are **inputs with
a weak pull-up**, and they pick the operating mode:

| M0 | M1 | Mode | Receives? |
|---|---|---|---|
| low | low | high-timeliness | **yes** |
| low | high | air-wakeup (WOR) | only WOR frames — i.e. no |
| high | low | AT command mode | **no** |
| high | high | sleep | **no** |

Floating inputs sit at the weak pull-up (M0=1, M1=1 → **sleep**) or wander with harness noise, and the mode is
re-sampled on every power-up and every `AT+RESET` — which provisioning performs several times. The module then answers
`AT+HELP`, provisioning reports `RADIO OK`, the TX node keeps transmitting, and the pad's `bytes=` stays at 0 forever.

**Fix, in order:**

1. `grep` the boot log for `SWITCH:`. If it says `1`, send `AT+SWITCH0` (the module reboots) from a terminal on that
   module's UART at 9600, and re-check.
2. Tie **M0 → GND** and **M1 → GND** at the module on *both* modules. This is the actual fix; step 1 only proves the
   diagnosis.
3. Re-power both nodes and watch the pad's 1 Hz line. `bytes>` 0 means the module is receiving again.

> Tie them to GND, do not "leave them for the MCU to drive" while `SWITCH=0`: in that state they are the module's
> outputs. GND (or a GPIO held low, which is the same electrical state) is safe; a GPIO that toggles is not.

## 2. Check the three wires again, in this order

Fixed-point silence is almost always wiring, and always in the same three places.

| Suspect | Why it bites | How to check |
|---|---|---|
| **TX/RX crossed the wrong way** | The module's pins are labelled from *its* point of view: `RX` is an input and must come from the MCU's TX. | See §2-0. The pad's 1 Hz line reports `bytes=0` if it never receives; `bytes>0` if it does. |
| **AUX not wired but firmware polls it** | If AUX is left off, GPIO18 reads whatever the floating pin picks up, and a spurious "busy" stalls every write. | Boot log says `AUX=wired` or `NOT WIRED`; if not wired, build with `-DGSE_LORA_AUX_NONE`. |
| **Module fed the wrong voltage** | **DX-LR42** is specified 3.3–5.5 V (5 V typical) and needs ≥5 V for full power; **DX-LR41** is 3.3 V only, and 5 V makes it answer AT commands then behave erratically. Both take 3.3 V UART levels. | Measure VCC at the module. LR41: 3.2–3.4 V. LR42: 3.3–5.5 V, and note that 3.3 V costs you output power. Check which module the silkscreen says you have. |
| **No common ground** | Two separately-powered boards with no GND reference read each other's UART as noise. | Continuity from pad GND to module GND to the 12 V return. |

The pad node now prints, once a second for the first 10 s, the numbers that separate these:

```
LOG PAD st=1 valid=0 bad=0 bytes=0 sof=0 age=65535 NO-LINK (safe)
LOG PAD nothing on the radio UART: check TXD/RXD cross, 3V3, and that both modules are Level3 Ch0C
```

| Reading | Meaning |
|---|---|
| `bytes=0` | the pad's module is not receiving anything at all → §1 (channel / level) or wiring |
| `bytes>0`, `sof=0` | bytes arrive but no frame start → the other end is not speaking our protocol (wrong baud: 9600 vs 115200) |
| `sof>0`, `valid=0` | frames start but fail the CRC/length check → corrupt RF, or the two ends disagree on `KEY` |
| `valid>0`, `bad` climbing | working link with interference — check antennas, distance, `AT+POWE` |

## 3. The AT-mode trap after a long uptime

`+++` needs **silence before it**. `enterAt()` flushes and waits 20 ms, but a module that is actively receiving
(or still transmitting a queued packet) may answer nothing. The pad node then reports:

```
LOG PAD RADIO FAULT: AT entry failed
```

Power-cycle the pad (not just reset) and look again. If it is reproducible, raise the pre-delay in
`DxLr41::enterAt()` from 20 ms to 200 ms.

## 4. Frames arrive but the GUI shows nothing

If the TX node reports `state=` climbing while the GUI says NO PAD TELEMETRY, the binary stream is reaching the
laptop but not parsing. In order of likelihood:

* **The monitor is open as well.** Two readers on one port steal each other's bytes. Close `pio device monitor`
  before connecting the GUI.
* **Text and binary interleaved.** Both nodes print `LOG` lines on the same port they send frames on. The parser
  resyncs on `0x7E`, so this costs a frame, never a connection — but if it costs *every* frame, the node is in a
  print loop. Check for a `RADIO FAULT` banner repeating.
* **Version mismatch.** A pad flashed from this package and a TX node flashed from an older copy (or the
  Nucleo build) will both parse — the version byte is checked — but a payload-length change between revisions
  rejects silently. Flash both from the same commit.

## 5. When it is the radio and not the software

| Observation | Verdict |
|---|---|
| `AT+HELP` echoes but `MODE:` stays `0` after `AT+MODE1` | the command is being rejected — check for `ERROR=` and look up the code (`5.4 错误码一览表` in the module manual) |
| `AT+HELP` is silent but the module powers up | wrong baud. Try 115200 (`AT+BAUD` index 6) — some modules ship reconfigured |
| `AT+HELP` is silent at every baud | module dead, or TXD not connected. Swap in the spare. |
| Everything checks out but `bytes=0` | antennas missing. The SX1281 will not receive without a load on the RF port; fit both antennas before concluding anything |
| Everything checks out, `SWITCH:1` in the banner | **M0/M1 are mode inputs and at least one is floating** → §2a. Tie both to GND |
| Both nodes `RADIO OK`, `bytes=0`, `SWITCH:0`, wiring correct | the two ends disagree on something the code does not verify: compare the two `LOG RADIO` banners field by field — `LEVEL`, `Frequency`, `KEY`/`Openkey` must match, `MAC` must differ (`00,01` TX / `00,02` pad) |

---

## Order of operations, condensed

0. Read the first `LOG … link:` line on **both** nodes. If one says `ble` and the other `lora`, stop and
   reflash — §0. Everything below this point is for the LoRa transport.
1. Re-flash **both** boards from this package (the `AT+RESET` fix).
2. Power-cycle both, then look at the two nodes' `LOG` lines.
3. Read the `LOG RADIO` block on both: `SWITCH:0`? `MODE:1`? `LEVEL` equal? `Frequency` `0c` on the pad, `0e` on
   the TX? If `SWITCH:1` → tie M0/M1 to GND (§2a).
4. Read the pad's banner: `RADIO OK` or a named `RADIO FAULT`.
5. If `RADIO OK`, read the 1 Hz line: `bytes=` / `sof=` / `valid=`.
6. If `valid>0` and the GUI is still blank, close every other program holding the port.

If steps 1–4 all look right and the GUI is still silent, the problem is on the laptop side, and the CSV will show
it: `dir=rx` rows appear as soon as any frame parses at all.
