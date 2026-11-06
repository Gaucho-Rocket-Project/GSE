# Cannot upload to the ESP32-S3 pad node — "No serial data received"

```
ERROR: A fatal error occurred: Failed to connect to ESP32-S3: No serial data received.
```

This message is **not** a firmware error. It comes from `esptool`, before any of our code runs: esptool asked the
chip to enter its ROM bootloader and got silence back. The build is fine (it compiled — you got to `[upload]`), so
nothing in `pad_node.cpp` needs changing.

Work through the list in order. Step 1 fixes this for most people.

---

## 1. Check that you are on the right USB socket

An ESP32-S3-DevKitC-1 has **two USB-C sockets**, and only one of them is a serial port.

| Socket (silkscreen) | What it is | Device name | Uploads reliably? |
|---|---|---|---|
| **UART** / **COM** | CP2102 / CH340 / CH343 bridge chip → the ESP32's UART0 | `/dev/ttyUSB0`, `COM5` | **Yes, always** — the bridge chip owns the BOOT/RST lines |
| **USB** / **USB-OTG** | wired straight to the ESP32-S3's own USB peripheral | `/dev/ttyACM0`, `COM7`, "USB JTAG/serial debug unit" | Usually, *if* the chip is running and esptool can reset it |

The **UART** socket works unconditionally, because a small bridge chip does the reset for you. The **native**
socket is a conversation with the chip itself: esptool has to reset it into download mode over USB, which only
works while the USB peripheral is alive and cooperative. A blank chip, a crashed program, an interrupted flash or
a firmware that has taken the peripheral over all break that hand-shake — and the failure is reported as
**"No serial data received"**.

**So: try the other socket first.** It is a 10-second test and it resolves this exact message most of the time.
If `pio device list` shows only `/dev/ttyACM0` / "USB Serial Device", you are on the native socket and there is a
`/dev/ttyUSB*` waiting on the other port.

> A "SuperMini" / N16R8 clone with only **one** USB socket has no bridge chip at all. There, the native socket is
> the only option — use steps 2 and 3.

---

## 2. Drop the upload speed

The default for this board definition is **460800 baud**. Cheap cables, long cables, USB hubs and some clones
cannot sustain it, and the failure mode is exactly "no serial data received".

`firmware/platformio.ini` now ships with a conservative speed that works on everything:

```ini
upload_speed = 115200
```

It costs about 30 s per upload. Once your board is talking reliably you can raise it to `460800` (or `921600`)
for speed. To try a speed without editing the file:

```bash
pio run -e pad-node -t upload --upload-port /dev/ttyUSB0
pio run -e pad-node -t upload -v          # shows the exact esptool command line
```

---

## 3. Force download mode by hand

This bypasses every autoboot question. It is the universal fallback and it always works if the board is healthy.

1. Hold down **BOOT** (the `IO0` button, usually labelled `BOOT`, next to `RST`).
2. Press and release **RST** (sometimes `EN`), keeping BOOT held.
3. Release BOOT.
4. Immediately run the upload, within a few seconds.

If the upload still fails with BOOT held, the port or the cable is wrong — not the board.

---

## 4. Confirm the port with esptool directly, outside PlatformIO

This separates "PlatformIO picked the wrong port" from "the board will not talk".

```bash
# PlatformIO ships its own esptool; this uses exactly the same binary
~/.platformio/packages/tool-esptoolpy/esptool.py --chip esp32s3 --port /dev/ttyUSB0 chip_id
```

Expect a line like `Chip is ESP32-S3 (QFN56) (revision v0.2)` and `MAC: ...`.

| What you see | What it means |
|---|---|
| `Chip is ESP32-S3` | The board and cable are fine — the problem was the port or speed PlatformIO used. Set `upload_port` in `platformio.ini`. |
| `No serial data received` | The board is not entering download mode, or this is not a UART port → step 1 and step 3. |
| `could not open port ... Permission denied` | On Linux you are not in the `dialout` group: `sudo usermod -aG dialout $USER`, then log out and back in. |
| `could not open port ... busy` | Something else holds the port — close the PlatformIO monitor, the Arduino IDE, a serial terminal, or a second VS Code window. |

Pin the port so PlatformIO stops guessing (find it with `pio device list`):

```ini
[env:pad-node]
upload_port  = /dev/ttyUSB0     ; Windows: COM5
monitor_port = /dev/ttyUSB0
```

---

## 5. Things specific to *this* GSE

These catch out pads that are already wired up:

* **Unplug the DX-LR41 and the servos before flashing.** Not for the upload's sake — for safety. During flashing,
  and for the ~200 ms after reset, GPIO4/5/6/7 are floating. The hardware keep-alive (Q1 + RC pump) holds the
  igniter relay off whenever D8 is not toggling, so the igniter cannot fire, but do not rely on that while a live
  igniter is on the outlet. **No igniter, no pressure source, no exceptions.**
* **Both nodes on the bench at once.** If the TX node is also plugged in you have two ports, and the one
  PlatformIO auto-detects may be the TX board. Flashing the pad firmware onto the TX board produces a board that
  boots, then prints `pad_node.cpp must be built with -DGSE_ROLE_PAD`-style behaviour — confusing, not dangerous.
  Unplug the other node.
* **The power switch.** If the pad node is powered from the GSE 12 V rail through the DROK 5 V buck, the board may
  be powered while the USB cable is plugged in — that is fine, but if the buck is *off* the ESP32-S3 can
  back-power from USB and sit in a brown-out reset loop. Unplug the 12 V, or make sure SW1 is on, but not both
  half-way.

---

## 6. When it is neither of those

| Symptom | Likely cause | Try |
|---|---|---|
| Port never appears in `pio device list` | cable is charge-only, or the socket is dead | a different USB-C cable (many are power-only), then a different socket |
| Port appears and disappears every ~1 s | brown-out reset loop | power the board from a known-good 5 V supply, disconnect the servos and module |
| `Invalid head of packet` / `Failed to leave Flash mode` | flaky link at speed | `upload_speed = 115200` (step 2) |
| Uploads succeed but the board says nothing | you are looking at the wrong port | the firmware prints on the **native** USB socket (see below) |
| Everything works, then stops after wiring the servos | 5 V rail collapses when the servos move | separate 5 V supply for servos, common ground; see `hardware/wiring.md` |

**A note on which port prints the log.** Because the firmware is built with `ARDUINO_USB_MODE=1` +
`ARDUINO_USB_CDC_ON_BOOT=1`, `Serial` is the **native USB** port. So:

* **upload** over the **UART** socket (`/dev/ttyUSB0`, `COM5`),
* **`pio device monitor` / the GUI** over the **native USB** socket (`/dev/ttyACM0`, `COM7`).

That is expected and correct — it is the same split the TX node uses. If you only have one socket, both go
through it and you must close the monitor before uploading.

---

## Quick checklist

1. Cable in the **UART** socket, not the USB-OTG one.
2. `upload_speed = 115200`.
3. Nothing else has the port open.
4. BOOT + RST, then upload.
5. `esptool.py chip_id` to prove the board answers.
6. `upload_port = ...` in `platformio.ini` so it stops guessing.
7. Igniter and pressure source disconnected — always.
