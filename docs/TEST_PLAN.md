# Test plan — GSE LoRa control link

**Companion to ECR-01.** Nothing in this plan can be skipped by "it worked on the bench last time". Each test has a written acceptance criterion and a sign-off line. Copy this file per build (`TEST_PLAN_<vehicle>_<date>.md`) and record serial numbers.

Legend — **B** bench, **R** range, **H** hardware-only (no firmware change needed to pass).

---

## Gate summary

| Gate | Tests | Blocks |
|---|---|---|
| G1 — modules alive | T-00, T-01, T-01a, T-02 | anything |
| G2 — link | T-03, T-05, T-07 | servo integration |
| G3 — safety | T-04, T-06 | connecting a real e-match |
| G4 — flight worthiness | T-08, T-09, T-10 | the first live oxidizer fill |

---

## T-01 · Module provisioning (B)

1. Connect one DX-LR41 to a USB-UART adapter at 9600 8N1 with its antenna fitted.
2. Send `+++`, expect `Entry AT`.
3. Send `AT+HELP`; record the banner (version, MODE, LEVEL, SLEEP, Frequency, MAC, CRC, Openkey, Power).
4. Confirm: `MODE:1`, `LEVEL:3`, `Frequency:2424MHz >> 0c`, `Power:13dBm` after the firmware's `configure()` has run; `MAC=00,02` (pad) or `00,01` (TX).
5. Repeat for the second module; it must show the mirrored channel (`0e`) and `MAC=00,01`.

**Pass:** both banners match the values in `firmware/src/config.h`. Attach the banners to the test record.

---

## T-00 · Mode-select pins and module identity (B) — do this first

Purpose: two silent, "everything looks configured" failures are excluded before any other test.

1. With the module powered, read its `AT+HELP` banner (terminal at 9600, or the node's own `LOG RADIO` block).
2. Confirm `SWITCH:0`. If it reads `1`, the module is in hardware-mode-select and **M0/M1 must not be floating** —
   send `AT+SWITCH0`, then tie M0 and M1 to GND and re-power. A floating pin in that state parks the module in AT,
   WOR or sleep mode: it answers commands and never receives.
3. Record **which module** is fitted — `DX-LR41-24T12S` (3.3 V only) or `DX-LR42-24T12D` (3.3–5.5 V, 5 V typical) —
   and measure VCC against it. UART signalling is 3.3 V for both.

**Pass:** `SWITCH:0` on both modules, M0/M1 tied to GND on both, VCC within the datasheet range of the module
actually fitted (LR41: 3.2–3.4 V; LR42: 3.3–5.5 V). Record the two module part numbers in the log book.

---

## T-01a · Board variant identification (B) — required if either node is not an ESP32-S3

Purpose: prove the board in the hand matches the firmware in it. The ESP-32S variant moves three pad-node
pads (PURGE D6→D15, IGNITER D7→D13, heartbeat D8→D33) because GPIO6–11 are SPI flash on a WROOM-32.

1. Open each node's USB port at 115200 and power-cycle it.
2. TX node must print `LOG TX esp32s3 fw 1.0 ready`, `LOG TX esp32s fw 1.0 ready` or
   `LOG TX stm32 fw 1.0 ready`; the board name must match the board.
3. Pad node must print `LOG PAD boot fw 1.0 board=<board> pins FILL=… VEH=… PRG=… IGN=… HB=…`.
   ESP32-S3: `PRG=6 IGN=7 HB=8`. ESP-32S: `PRG=15 IGN=13 HB=33`.
4. With a lamp (never an igniter) on IGN: confirm the relay clicks only on the pad listed in the banner.

**Pass:** banner board and pin numbers match the physical harness; no probing needed to find out which
pad does what. Record board type and environment name for both nodes.

---

## T-02 · Cross-check against the stock radio (B) — *optional but strongly recommended*

Purpose: prove the servo behaviour the pad node produces is the behaviour OP 8 produced.

1. Bench: one servo, a 5 V supply, and the FS-iA10B.
2. Using the stock FS-i6X, set Ch5 endpoints per OP 8 steps 3–6; record OPEN/CLOSED pulse widths with a scope or a servo tester that reads pulse width.
3. Swap to the pad node on D4 with the same servo. Set FILL travel until the pulse widths match within ±20 µs.
4. Repeat for Ch6 (both valves) and Ch8.

**Pass:** pad-node pulse widths within ±20 µs of the stock configuration at the same mechanical positions.

---

## T-03 · Link integrity, static (B)

1. Both nodes powered, 2 m apart, antennas fitted. GUI connected.
2. Run for 30 minutes at 20 Hz.
3. Record from the GUI: `valid CMDs`, `rejected frames`, `last valid CMD` max/mean, and the CSV log.

**Pass:** zero rejected frames; `last valid CMD` ≤ 150 ms at all times; no `COMMS_LOSS` event except those deliberately provoked in T-04.

---

## T-04 · Failsafe matrix (B) — **gate for connecting anything to the IGN outlet**

Set up: servos connected, IGN outlet loaded with a 12 V lamp (never an e-match).

| # | Action | Expected within | Expected result |
|---|---|---|---|
| 4.1 | Boot the pad with no TX node present | immediately | FILL closed, PURGE closed, VEHICLE open, lamp dark, banner `PAD SILENT` |
| 4.2 | Start the GUI, set FILL open, VEHICLE closed | 200 ms | read-back matches the GUI |
| 4.3 | Kill the GUI process (Task Manager) | ≤ 0.4 s | TX node begins sending safe frames; pad reports safe set |
| 4.4 | Unplug the TX node from the laptop | ≤ 1.5 s | pad reports `COMMS LOSS`, safe set, lamp dark |
| 4.5 | Switch off the pad node and back on while the GUI runs | ≤ 1.5 s | pad returns safe, then follows commands again |
| 4.6 | Hold the fire control with ARM on, then switch ARM off while holding | ≤ 50 ms | lamp dark, `ARM` false in read-back |
| 4.7 | Hold the fire control, then unplug the TX node | ≤ 1.5 s | lamp dark |
| 4.8 | Set the watchdog to 0.3 s, repeat 4.4 | ≤ 0.5 s | pad declares loss earlier — proves the parameter is live |
| 4.9 | Power-cycle the pad node while the GUI is armed | immediately on boot | lamp stays dark; read-back `NOT ARMED` until the GUI re-arms |

**Pass:** every row as stated, three repetitions each, video evidence for any row that fails once.

---

## T-05 · Range and link quality (R)

1. Set up the GSE box in its flight configuration at the pad. Take the laptop + TX node to the operating position (100 ft standoff, and again from the furthest bunker in use).
2. Open the GUI; log for 10 minutes with all valves idle.
3. Repeat with the fill valve cycling every 5 s (servo current noise), and again with the pad node powered from the same 12 V battery as the servos.
4. Record `rejected frames`, `valid CMDs`, worst `last valid CMD`, and any `COMMS_LOSS` events.

**Pass:** no `COMMS_LOSS`; network loss (rejected/(valid+rejected)) on the link < 0.1 %; margin demonstrated by repeating with the TX antenna replaced by a 50 Ω terminator (must fail) and restored (must pass).

*If failures occur:* change `AT+LEVEL` to 5 or 7 (re-provision BOTH modules: `SET LEVEL n` + `RADIO CONFIG` on each), re-run T-03 and T-05.

---

## T-06 · Igniter integrity with a real relay (B) — **gate before an e-match**

1. IGN outlet loaded with a 12 V 21 W lamp.
2. ARM on; fire 20 times with 5 s between presses, one press per cycle.
3. Verify the pad cuts at the configured limit every time (`IGN_OFF arg=1` in the log).
4. Verify a continuously held button produces exactly one pulse.
5. With SW2 **open**, hold fire: the lamp must never light, even though the GUI reports `ENERGISED` (the software does not know about SW2 — this is the point of the hardware interlock). Record this as an *expected* mismatch.
6. Wiggle-test: flex the servo/relay wiring while firing; no spurious pulses, no MCU resets (`uptime_s` must not restart).

**Pass:** 20/20 correct pulses; never more than one pulse per press; no resets.

---

## T-07 · Two-channel behaviour (B)

1. With both nodes running, change ONLY the pad's `AT+CHANNEL` to match the TX node's channel via the serial console (`SET ...`), so both modules listen on the same channel (single-channel fallback).
2. Observe the state rate actually achieved and the rejected-frame count at 20 Hz commands / 10 Hz states.
3. Restore the two-channel configuration.

**Pass:** document which mode your module revision supports. If single-channel is required, rebuild both nodes with `-DGSE_SINGLE_CHANNEL`, set the state rate to 5 Hz in `config.h`, and re-run T-03.

---

## T-08 · Dry rehearsal at the pad (R) — **gate for the first live fill**

Full dress rehearsal with **no oxidizer and no e-match**:

1. Walk the handbook §7.2/§7.3 sequence with the new procedure.
2. Operate FILL, VEHICLE and IGN from the operating position; a second person at the pad confirms each valve audibly/visually.
3. Deliberately drop the link mid-sequence (close the laptop lid) and confirm the pad safes; recover.
4. Time the whole sequence; compare with the handbook's §7.4 load-and-go timing (fill ≈ 60 s ± 10 s).

**Pass:** all valve movements confirmed by the second operator; timing unchanged; a written note of anything the operator had to look up.

---

## T-09 · Static fire with the new GSE (R)

Same as the handbook's static test procedure (Appendix A), with the new control system. The operator runs the fill and ignition from the GUI. Record the CSV log with the test record.

**Pass:** identical fill behaviour and ignition timing to the previous static fire; log attached.

---

## T-10 · Configuration diff (H)

Before every launch day, record the configuration actually in force (read from the pad's STATE frames — travel per channel, watchdog, igniter limit, purge enable, firmware versions) and diff against the previous launch.

**Pass:** every difference is explained in the log.

---

## Sign-off

| Gate | Tester | Date | Signature | Notes / evidence file |
|---|---|---|---|---|
| G1 | | | | |
| G2 | | | | |
| G3 | | | | |
| G4 | | | | |
