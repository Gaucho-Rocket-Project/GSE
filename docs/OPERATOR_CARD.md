# GSE control — operator card

*Print, laminate, tape inside the toolbox lid.*

## Before you leave the prep area
- [ ] Laptop charged, GSE application installed, TX-node cable in the box
- [ ] Pad powered: SW1 on, **SW2 (arm key) OFF and in your pocket**
- [ ] IGN outlet **unloaded**, leads insulated from each other
- [ ] Antennas fitted to both modules
- [ ] **M0 and M1 tied to GND on both modules** (not left floating — see wiring sheet §1.2)
- [ ] UART **crossed**: module RX (pin 3) ← D17, module TX (pin 4) → D16
- [ ] AUX wired to D18, or firmware built with `-DGSE_LORA_AUX_NONE` (boot log says which)

## Start-up (at the pad, before anyone goes downrange)
1. Start the application → select the TX-node port → **Connect**
2. Banner must read **LINK OK — NOT ARMED**
3. `last valid CMD` < 100 ms, `rejected frames` 0
4. Checkouts: FILL open/close, VEHICLE open/close (listen for the servos), PURGE if fitted
5. Spark check: leads touching, ARM off → no spark; ARM on, hold IGN → spark within 0.1 s; release → no spark
6. **ARM off. SW2 open. Both keys with the operator.**

## Arming
- Only when the launch window is certain and everyone is clear.
- Turn the **SW2 key**, then press **ARM** in the application.
- Banner turns green: **LINK OK — ARMED**
- Any of the following drops the arm instantly: releasing ARM, `Esc`, losing the link, losing the heartbeat, opening SW2.

## Firing
- **Hold** the fire control. It is hold-to-fire with a hard limit (0.5 s default) and one pulse per press.
- If nothing happens: release, check `rejected frames`, `armed`, `age`, then try again. Do **not** hold the button down.

## Status light (no terminal needed)
| ESP32-S3 (WS2812) | ESP-32S (blue LED) | Meaning |
|---|---|---|
| blue solid | solid on | booting |
| red solid | 5 Hz fast blink | **RADIO FAULT** — module did not answer, do not fly |
| amber blink | 2 Hz blink | radio fine, **no link** from the TX node |
| green blink | 1 Hz blink | link OK, **not armed** |
| purple | 10 Hz fast blink | igniter energised |

## If the pad hears nothing
Banner `NO PAD TELEMETRY` and the pad's line says `bytes=0`: the modules are not talking at all.
1. Boot log must say `SWITCH:0`. If it says `1`, tie M0/M1 to GND and power-cycle.
2. Then check, in order: antennas fitted, TXD/RXD crossed, 3V3 on both, common ground.
3. `bytes>0` but `valid=0` is a different fault (LEVEL / KEY / channel) — see TROUBLESHOOTING_LINK.md.

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
