# No pad telemetry — the link is up but no STATE frames arrive

```
banner: NO PAD TELEMETRY
CSV:    only rows with dir=tx  (no rx STATE, no rx EVENT, not even rx HELLO)
```

The TX node is transmitting, the port is open, and the laptop is happily sending 20 CMD/s. **Nothing has ever
come back.** That narrows it fast: the laptop↔TX half is proven, so the fault is in the TX→pad→TX half.

Before anything else, check the TX node's own log. It prints these lines even when the GUI is connected:

```
LOG TX esp32 fw 1.0 ready                         <- radio_provision() succeeded
LOG tx fwd=812 state=0 safe=0 dropped=0 radio=ok  <- once a second
```

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

## 2. Check the three wires again, in this order

Fixed-point silence is almost always wiring, and always in the same three places.

| Suspect | Why it bites | How to check |
|---|---|---|
| **TX/RX not crossed** | Module `TXD` must go to the MCU's **RX**. Both ends are labelled from the module's point of view. | The pad's 1 Hz line reports `bytes=0` if it never receives, and `bytes>0` if it does. Crossed-wrong gives 0. |
| **Module fed 5 V** | The DX-LR41 is 3.3 V. 5 V on VCC does not always kill it — it makes it answer AT commands and then behave erratically. | Measure VCC at the module: 3.2–3.4 V, not 5. |
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

---

## Order of operations, condensed

1. Re-flash **both** boards from this package (the `AT+RESET` fix).
2. Power-cycle both, then look at the two nodes' `LOG` lines.
3. Read the pad's banner: `RADIO OK` or a named `RADIO FAULT`.
4. If `RADIO OK`, read the 1 Hz line: `bytes=` / `sof=` / `valid=`.
5. If `valid>0` and the GUI is still blank, close every other program holding the port.

If steps 1–4 all look right and the GUI is still silent, the problem is on the laptop side, and the CSV will show
it: `dir=rx` rows appear as soon as any frame parses at all.
