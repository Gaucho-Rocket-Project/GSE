# Wiring — two DS3225MG servos to a NUCLEO-H753ZI

Everything here is for the **NUCLEO-H753ZI** (STM32H753ZIT6) and the
**DSSERVO DS3225MG 25 kg** digital servo from the Amazon listing
([B09SB8RMTT](https://www.amazon.com/dp/B09SB8RMTT) — 4-pack, 180° version).

Diagram: [`docs/wiring-stm32h753.svg`](docs/wiring-stm32h753.svg) ·
[`docs/wiring-stm32h753.png`](docs/wiring-stm32h753.png)

> Using SG90 / MG90S micro servos instead? The wiring is **identical** — only the
> power supply is smaller (see §5) and you may want to narrow the pulse range
> (§7). The firmware drives any standard 50 Hz 3-wire servo.

---

## 1. What you need

| Item | Notes |
|---|---|
| NUCLEO-H753ZI | The board you linked. Comes with the ST-LINK/V2-1 on board. |
| 2 × DS3225MG servo | 3-wire: brown/black = GND, red = V+, orange/white = signal |
| **5–6.8 V supply, ≥ 5 A (6 A recommended)** | **Separate from the NUCLEO.** See §5. A 5 A bench supply or a 6 V/10 A UBEC both work. |
| 2200 µF electrolytic capacitor | ≥ 16 V rating, across the servo supply rails |
| 6–8 jumper wires | female-to-male is handy for the servo connectors |
| Thick ground wire (20 AWG or better) | supply − → NUCLEO GND |

You do **not** need an external programmer: the NUCLEO's ST-LINK flashes the
chip over the same USB cable that gives you the serial console.

---

## 2. Pin map (the three wires per servo)

| Signal | Where on the NUCLEO | MCU pin | Firmware constant |
|---|---|---|---|
| **Servo 1 signal** (orange) | **CN10, pin 4** — silkscreen **D6** | **PE9** | `TIM1_CH1`, `servo.c` |
| **Servo 2 signal** (orange) | **CN10, pin 6** — silkscreen **D5** | **PE11** | `TIM1_CH2`, `servo.c` |
| **Servo power** (red, both) | external supply **+** | — | — |
| **Ground** (brown, both) | external supply **−** **and** NUCLEO **GND** | — | — |
| **Console** (built in) | ST-LINK USB port → PC | PD8 / PD9 | `USART3`, 115200 8N1 |

Both signals are on the Arduino/Zio connector **CN10** (the lower-right header
block), which is convenient: the two servo plugs sit next to each other.

```
CN10 pin 1  · · ·  pin 2
CN10 pin 3  · · ·  pin 4   <- D6  = PE9  = servo 1 signal
CN10 pin 5  · · ·  pin 6   <- D5  = PE11 = servo 2 signal
CN10 pin 7  · · ·  pin 8
...
```

Any **GND** pin on the board works for the common ground — CN10 has several, as
does the CN8 power header. Do **not** use the board's `5V` pin for the servos
(§5).

### Why these two pins

PE9 and PE11 are **TIM1_CH1 and TIM1_CH2**: the same hardware timer, two
independent compare channels. One timer means both servos get the *same* 50 Hz
frame (they are never commanded at different instants, which keeps supply
inrush tidy), while each channel's pulse width — and therefore each servo's
position — is set completely independently. Commands like `on both` write both
channels; `servo1 45` touches only channel 1.

They are also free of every peripheral that matters on this board (USB, UART,
QSPI, SDRAM, LEDs, user button).

---

## 3. Step-by-step

1. **Power off everything.** Do not have the servo supply plugged in yet.
2. Servo 1: orange → **CN10-4**; brown → supply **−** rail; red → supply **+** rail.
3. Servo 2: orange → **CN10-6**; brown → supply **−** rail; red → supply **+** rail.
4. Solder/plug the **2200 µF** capacitor across the +/− rails at the servos
   (stripe = negative leg). Get this the right way round.
5. Run a **thick** wire from supply **−** to a **GND** pin on the NUCLEO. This
   common ground is mandatory: the PWM signal has no reference without it.
6. Double-check against the checklist in §4, then switch the servo supply on
   **and only then** plug the NUCLEO's USB into your PC.

---

## 4. Checklist before you apply power

- [ ] Both servo **red** wires go to the external supply, **not** to the NUCLEO.
- [ ] All grounds joined: supply − ↔ NUCLEO GND ↔ both servo brown wires.
- [ ] Servo 1 signal on **CN10-4 (PE9)**, servo 2 on **CN10-6 (PE11)**.
- [ ] Supply voltage between **4.8 V and 6.8 V** (never 7.4 V / 2S LiPo direct).
- [ ] Capacitor polarity correct.
- [ ] Servo horns can turn freely through their full travel — nothing blocking
      the arm and nothing fighting a mechanical stop.
- [ ] Nothing else is driving PE9 / PE11.

---

## 5. Power — read this before connecting the red wires

This is the part that decides whether the rig works or resets randomly.

| | DS3225MG (yours) | SG90 / MG90S (for comparison) |
|---|---|---|
| Idle | 4–5 mA | 10 mA |
| Moving | **≈ 1–2 A** | 100–250 mA |
| **Stall** | **2.1 A @ 5 V · 2.9 A @ 6.8 V** | 0.36 A |

**Two DS3225MG can pull 4–6 A.** The NUCLEO's `5V` pin comes from the ST-LINK
USB (≈ 0.5 A) and the chip's own LDO — it cannot do this. The failure mode is
not subtle: the MCU browns out mid-command and reboots, or the servo buzzes and
hunts because its supply sags.

| Supply | Verdict for two DS3225MG |
|---|---|
| NUCLEO `5V` pin from USB (≈0.5 A) | **No.** Reboot loop, likely. |
| USB 3.0 port (≈0.9 A) | **No.** |
| 5 V 3 A adapter | Marginal — gentle, unloaded movement only. |
| **5 V 5 A adapter + 2200 µF** | Works for light loads. |
| **5 V ≥ 6 A supply (recommended)** | Comfortable, including stall moments. |
| 6.0–6.8 V ≥ 6 A | Best: more torque and speed, still in spec. |

Practical rules:

1. **Separate supply for the servos.** Only ground is shared with the NUCLEO.
2. **Thick ground** (20 AWG+) back to the supply. Servo return current on a
   thin dupont lead is the classic cause of "the firmware randomly reboots".
3. **2200 µF minimum** for two DS3225MG, placed at the servos, not at the MCU.
4. **Never** power them from `3V3`, and never exceed **6.8 V**.
5. Power the servo supply **first**, then the MCU. If the MCU is already on and
   the servos' supply comes up later, that is fine too — but never leave the
   signal wires connected with the servo grounds floating.
6. Servos are electrically noisy. Random resets *with* a good supply usually
   mean a long thin ground or a missing capacitor.

---

## 6. Bench test the servos before you blame the firmware

With **only** the external supply (no MCU connected):

1. Brown → supply −, red → supply +.
2. Touch a wire from supply **+** to the **orange** wire for a moment — the horn
   should jerk to one end. Touch it to **−** and it should swing the other way.
   *(A crude full-swing test; don't hold it there and don't repeat it.)*
3. Silent servo = wiring/power problem, not this firmware.

---

## 7. Pulse range, travel and calibration

| Parameter | Value | Where |
|---|---|---|
| Frame | 50 Hz (20 ms) | `SERVO_FRAME_HZ` |
| Pulse range | 500 … 2500 µs | `SERVO_MIN_PULSE_US` / `SERVO_MAX_PULSE_US` |
| Centre | 1500 µs | `SERVO_DEFAULT_PULSE_US` |
| Travel | 180° (this listing) | `SERVO_TRAVEL_DEG`, or `make TRAVEL=270` |
| Resolution | **1 µs** (= 1 timer tick) | hardware — far finer than the servo's 3 µs dead band |

* **180° vs 270°**: the listing you linked is the *180°* version, which is what
  the default build assumes (500 µs = 0°, 1500 µs = 90°, 2500 µs = 180°). If
  yours sweeps further than 180°, build with `make TRAVEL=270`
  (or flash `prebuilt/nucleo_h753zi_servo_270.bin`).
* **Buzzing or grinding at the ends** = the mechanical limit of *your* unit is
  inside 500–2500 µs. Narrow it, no source edit needed:

  ```bash
  make MIN_US=700 MAX_US=2300
  ```

  Repeat until a full `sweep both` runs quietly.
* **Centre the servo before pressing a horn on** (`center both`), so "90°"
  really looks like the middle.

---

## 8. Pin conflicts (NUCLEO-H753ZI)

| Pin / peripheral | Why to leave it alone |
|---|---|
| **PD8 / PD9** | USART3 = the ST-LINK virtual COM port. Using them for a servo kills the terminal. |
| **PA13 / PA14** | SWDIO / SWCLK — the debug and flash interface. |
| **PC13 / PC14 / PC15** | user button, LSE, and PC14/15 are on the oscillator circuit |
| **PE2** | QSPI_BK1_IO2 (shared with the ST Zio connector) |
| **PC0 / PC1 / PC4 / PC5 / PA1 / PA7 / PG11 / PG13** | RMII Ethernet, if you ever add the PHY |
| **PB0 / PB7 / PB14** | LD1 / LD2 / LD3 LEDs (PB14 is used by the fault handler) |

Moving the servos to different pins is easy: `servo.c` → `gpio_init()` sets
PE9/PE11 and `AF1_TIM1`; `servo_pin_name()` is only used for display. Keep both
servos on **TIM1** channels (PE9/PE11, or PE13=CH3 / PE14=CH4) so they keep
sharing one frame.

---

## 9. Bring-up sequence

1. Wire per §3, capacitor in place, external supply off.
2. Flash: `make flash` (or drag `prebuilt/nucleo_h753zi_servo_180.bin` onto the
   NUCLEO drive — see the firmware README).
3. Open the console: `make monitor` (or `screen /dev/ttyACM0 115200`).
4. Switch the servo supply on. You should see the banner.
5. `status` → both servos `off`, no buzzing, nothing warm.
6. `on servo1` → servo 1 snaps to centre and holds firmly.
7. `on servo2` → servo 2 does the same; servo 1 does not move.
8. `on both`, then `servo1 0`, `servo2 180` → independent positions.
9. `sweep both` → smooth full travel. If the NUCLEO resets here it is **power**
   (§5), not software.
10. `off both` → both go limp (no holding torque, almost no current).
