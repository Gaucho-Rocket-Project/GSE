# GSE control — operator card

*Print, laminate, tape inside the toolbox lid.*

## Know which link you are carrying
- [ ] Both nodes are the **same transport**. The first `LOG ... link:` line on each says so:
      `link: lora -- LoRa UART1 ...` or `link: espnow -- ESP-NOW ch 1, local ... -> peer ...`
      (ESP-NOW envs `pad-node` / `tx-node`, LoRa envs `pad-node-lora` / `tx-node-lora`.)
      **A LoRa pad and an ESP-NOW TX node will never link.**
- [ ] If it is the ESP-NOW build, there are **no external radio modules** to pack — but it is a 2.4 GHz
      802.11 radio, so it competes with phones and hotspots, and T-11 (range test) has to be signed for
      that vehicle.
- [ ] If it is the ESP-NOW build, the two nodes are **MAC-paired**: swapping either board for a spare
      needs both MACs in `config.h` and a reflash of both. A spare pad flashed with the old MAC pair
      cannot communicate with the old peer configuration; it may still print a local setup-success
      banner. Confirm fresh telemetry and re-provision both nodes.

## Before you leave the prep area
- [ ] Laptop charged, GSE application installed, TX-node cable in the box
- [ ] Pad powered: SW1 on, **SW2 (arm key) OFF and in your pocket**
- [ ] IGN outlet **unloaded**, leads insulated from each other
- [ ] **LoRa build only:** antennas fitted to both modules
- [ ] **LoRa build only:** **M0 and M1 tied to GND on both modules** (not left floating — see wiring sheet §1.2)
- [ ] **LoRa build only:** UART **crossed**: module RX (pin 3) ← D17, module TX (pin 4) → D16
- [ ] **LoRa build only:** AUX wired to D18, or firmware built with `-DGSE_LORA_AUX_NONE` (boot log says which)

## Start-up (at the pad, before anyone goes downrange)
1. Start the application → select the TX-node port → **Connect**
2. Banner must read **LINK OK — NOT ARMED**
3. `last valid CMD` < 100 ms, `rejected frames` 0
4. Checkouts: FILL open/close, VEHICLE open/close (listen for the servos), PURGE if fitted
   *(purge valve fitted? tick the box in the left panel and press **Apply to pad** — the pad ignores
   PURGE until it has been told `allow_purge`, and the GUI blocks it too. One tick, one Apply.)*
5. Spark check: leads touching, ARM off → no spark; ARM on, hold IGN → spark within 0.1 s; release → no spark
6. **ARM off. SW2 open. Both keys with the operator.**

## Arming
- Only when the launch window is certain and everyone is clear.
- Turn the **SW2 key**, then press **ARM** in the application.
- Banner turns green: **LINK OK — ARMED**
- Releasing ARM or pressing `Esc` requests disarm; loss of commands is subject to the pad watchdog delay.
  Opening SW2 removes ignition power only if the independent hardware chain is correctly built and tested.
  Do not assume a lost heartbeat cuts power: the former RC example was not a valid stuck-high detector.

## Firing
- **Hold** the fire control. It is hold-to-fire with a hard limit (0.5 s default) and one pulse per press.
- If nothing happens: release, check `rejected frames`, `armed`, `age`, then try again. Do **not** hold the button down.

## Status light (no terminal needed)
| ESP32-S3 (WS2812) | ESP-32S (blue LED) | Meaning |
|---|---|---|
| blue solid | solid on | booting |
| red solid | 5 Hz fast blink | **LINK FAULT** — LoRa: module did not answer. ESP-NOW: Wi-Fi did not start, or the peer MAC is not configured. Do not fly |
| amber blink | 2 Hz blink | radio fine, **no link** from the TX node |
| green blink | 1 Hz blink | link OK, **not armed** |
| purple | 10 Hz fast blink | igniter energised |

## If the pad hears nothing
Banner `NO PAD TELEMETRY` and the pad's line says `bytes=0`: the modules are not talking at all.
1. **LoRa build:** boot log must say `SWITCH=0` (pad line `PAD radio mode select: SWITCH=0 ...`). If it says
   `1`, tie M0/M1 to GND and power-cycle. **ESP-NOW build:** the pad line must say
   `PAD link: espnow ... -> peer ...` (not `peer MAC NOT CONFIGURED`), both nodes must print the same
   channel, and the TX node must be powered and within range.
2. **LoRa build**, then check in order: antennas fitted, TXD/RXD crossed, 3V3 on both, common ground.
   **ESP-NOW build:** check nothing else is using that channel (phone hotspot, another AP) and back away
   from the pad — 2.4 GHz does not go through a vehicle.
3. `bytes>0` but `valid=0` is a different fault (LEVEL / KEY / channel, or interference) — see
   TROUBLESHOOTING_LINK.md.

## If the link drops
- Banner turns red, pad side goes to **FILL closed / PURGE closed / VEHICLE open / IGN off** on its own.
- Do not restart the application in a hurry. Reconnect, confirm the banner, and continue only after a deliberate re-arm.

## If you must abandon the attempt
1. `Esc` — igniter released and disarmed
2. Confirm VEHICLE OPEN, IGN off on the read-back
3. Wait for the pad to be safed before approaching (handbook §7.5.2)
4. Close the SW2 key only when the next attempt is imminent

## Never
- Never leave the application armed while walking to the pad
- Never connect an e-match before the T-06 gate has been signed off
- Never operate the GUI from a laptop that is not the one that ran today's checkouts
