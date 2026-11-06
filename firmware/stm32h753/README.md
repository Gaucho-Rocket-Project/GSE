# NUCLEO-H753ZI dual servo firmware

Bare-metal **C** firmware that drives **two DS3225MG 25 kg digital servos**
(Amazon [B09SB8RMTT](https://www.amazon.com/dp/B09SB8RMTT)) from a
**NUCLEO-H753ZI**, and lets you switch them on and off — individually or
together — from a serial terminal.

No HAL, no CubeMX, no RTOS, no vendor libraries: `src/` is ~1100 lines of C11
that talks straight to the registers. The whole image is **13 KB**.

```
==================================================
 NUCLEO-H753ZI  -  dual servo test firmware
==================================================
  servo1 : signal on PE9  (CN10-4)  state: off
  servo2 : signal on PE11 (CN10-6)  state: off
  servo profile: 180 deg, 500-2500 us, centre 1500 us @ 50 Hz
  console: USART3 PD8/PD9 (ST-LINK VCP) @ 115200 baud
--------------------------------------------------
servo> on servo1           -> servo 1 pulses, centred at 1.5 ms
servo> on both             -> both at once
servo> servo2 45deg        -> servo 2 to 45 degrees
servo> both 1.8ms          -> both, explicit pulse width
servo> off servo1          -> servo 1 stops pulsing (goes limp)
servo> sweep both          -> slow 0 -> 180 -> 0 test
servo> status              -> state + pulse width + angle
```

**Wiring: [WIRING-STM32H753.md](../../WIRING-STM32H753.md)** — read the power
section before you connect the red wires. Two DS3225MG can draw 4–6 A; the
board's 5 V pin cannot supply that.

| Signal | NUCLEO-H753ZI | Servo wire |
|---|---|---|
| Servo 1 PWM | **PE9** — CN10 pin 4 (D6) | orange |
| Servo 2 PWM | **PE11** — CN10 pin 6 (D5) | orange |
| Power | external 5–6.8 V, ≥ 6 A | red |
| Ground | GND, common with the supply | brown |
| Console | ST-LINK USB (USART3, PD8/PD9) | — |

---

## Flash it — three ways

### A. Prebuilt binary (no toolchain, ~1 minute)

`prebuilt/` holds complete images linked for flash address `0x0800_0000`:

| File | Servo travel |
|---|---|
| `nucleo_h753zi_servo_180.bin` | **180°** — matches the listing you have |
| `nucleo_h753zi_servo_270.bin` | 270° variant of the same servo |

**Option 1 — drag and drop.** With the board plugged in over USB, a drive called
`NODE_H753ZI` (or `NUCLEO`) appears. Copy the `.bin` onto it. The ST-LINK
flashes it and resets the MCU. That is all.

**Option 2 — STM32CubeProgrammer** (free, from ST):

```bash
STM32_Programmer_CLI -c port=SWD \
    -w prebuilt/nucleo_h753zi_servo_180.bin 0x08000000 -v -rst
```

**Option 3 — OpenOCD** (ships with STM32CubeCLT / CubeIDE, and in Linux distros):

```bash
openocd -f interface/stlink.cfg -f target/stm32h7x.cfg \
    -c "program prebuilt/nucleo_h753zi_servo_180.bin 0x08000000 verify reset exit"
```

### B. Build from source

```bash
cd firmware/stm32h753
make                # out/servo.elf, out/servo.bin, out/servo.hex
make flash          # OpenOCD + on-board ST-LINK
make monitor        # serial console on /dev/ttyACM*
```

Needs `arm-none-eabi-gcc` — any of: the Arm GNU Toolchain, the xPack build,
the compiler bundled with **STM32CubeCLT/CubeIDE**, or
`sudo apt install gcc-arm-none-eabi`. If it is not on `PATH`:

```bash
make CROSS=/opt/arm-gnu-toolchain-13.3.rel1-x86_64-arm-none-eabi/bin/arm-none-eabi-
```

No `openocd`? Any of these work just as well:

```bash
make programmer     # STM32_Programmer_CLI -c port=SWD -w out/servo.elf -s -rst
```

...or drag `out/servo.bin` onto the `NODE_H753ZI` drive.

### C. Import into STM32CubeIDE (optional, if you want the GUI)

The project is plain C with a Makefile, so CubeIDE users generally keep using
`make` and just attach the debugger (**Run → Debug Configurations → STM32 C/C++
Application**, point it at `out/servo.elf`). There is no `.ioc` file to import
because nothing here is generated from CubeMX — that is deliberate (see
[What is deliberately *not* here](#what-is-deliberately-not-here)).

---

## Open the terminal

The ST-LINK exposes **USART3 as a virtual COM port**, so one USB cable gives you
flashing *and* the console.

```bash
make monitor              # find the ST-LINK port, check it answers, connect
make ports                # list serial ports, marking the ST-LINK/NUCLEO one
make sniff SECS=3         # just dump whatever arrives for 3 s
make send CMD='status'    # send one command, print the reply
```

`make monitor` runs `tools/monitor.py`, which is more useful than `screen`
because it tells you *why* nothing happens:

| What it prints | What it means |
|---|---|
| `Port /dev/ttyACM0 is already open by: screen (pid 5192)` | something already owns the port - close it, or `killall screen`. On Linux, ModemManager also grabs ttyACM ports: `sudo systemctl stop ModemManager`. |
| `Checking /dev/ttyACM0 for output ... nothing.` | the port opened but the firmware did not answer a CR: wrong port (another board can own ttyACM0), or the image is not flashed. It prints the next steps. |
| a banner + `servo>` | you are in. Ctrl-] disconnects, Ctrl-C goes to the board. |

`tools/find_port.py` identifies the port by **USB IDs**, not by guessing: the
NUCLEO's ST-LINK reports VID `0483` (STMicroelectronics), while the ESP32-S3
board in this repo also enumerates as `ttyACM*` and would happily eat your
commands if you picked the wrong one.

Manual alternatives still work:

```bash
screen /dev/ttyACM0 115200    # quit: Ctrl-A then K
python3 -m serial.tools.miniterm /dev/ttyACM0 115200
```

* Linux: `/dev/ttyACM0`; add yourself to `dialout` if access is denied.
* macOS: `/dev/cu.usbmodem*`. Windows: `COMx` (PuTTY/Tera Term, 115200 8N1).

## Commands

| Command | Effect |
|---|---|
| `on servo1`, `on servo2`, `on both` | start the 50 Hz pulse train, centred (1500 µs) |
| `off servo1`, `off servo2`, `off both` | stop pulses, signal held low → servo goes **limp** |
| `servo1 90`, `servo2 45deg` | aim by angle (0–180) |
| `servo1 1.5ms`, `both 1500us`, `both 1500` | aim by pulse width |
| `center both` | back to 1500 µs |
| `sweep servo1` / `sweep both` | slow 0 → 180 → 0, ~6 s |
| `status` / `s` | state, pulse width, angle |
| `help` / `?` | the menu |

**One-key shortcuts** (one character + Enter):

| Key | Action |
|---|---|
| `1` | servo 1 on (centre) |
| `2` | servo 2 on (centre) |
| `3` | both on (centre) |
| `0` | both off |
| `9` | sweep both |
| `s` | status |
| `?` | help |

Commands are case-insensitive and tolerant of extra words
(`please turn on servo 1` works). Values are **clamped** to the safe pulse range
rather than rejected, so a typo cannot drive a servo into its end stop.

**`off` vs `center`:** `off` removes the pulses, so the servo stops driving and
goes limp — it does *not* return to a park position, and it draws almost
nothing. `center both` keeps the pulses at 1.5 ms: the servo holds position with
holding torque, which costs a little current and gear wear.

---

## How it works

### Clock tree (no crystal, no jumper to set)

```
HSI 64 MHz ──/32──> 2 MHz ──x200──> VCO 400 MHz ──/2──> SYSCLK 200 MHz
                                                        ├── AHB  200 MHz  (core, GPIO)
                                                        ├── APB1 100 MHz  (USART3)
                                                        └── APB2 100 MHz  (TIM1)
```

Everything comes from the internal RC oscillator, so the rig does not depend on
the ST-LINK's MCO output or on a 25 MHz crystal being fitted. The H753's HSI is
trimmed to ±1 %, which is ~50x better than a hobby servo needs (its dead band is
3 µs = 0.2 % of the 1500 µs centre pulse).

### Servo timing

One hardware timer, **TIM1**:

| Register | Value | Meaning |
|---|---|---|
| `PSC` | 199 | 200 MHz / 200 = **1 MHz → 1 tick = 1 µs** |
| `ARR` | 19999 | 20 000 ticks = 20 ms = **50 Hz** |
| `CCMR1` | PWM mode 1, preloaded | CH1 and CH2 |
| `CCR1` / `CCR2` | 500…2500 | **the pulse width in µs, directly** |
| `BDTR.MOE` | 1 | TIM1 is an advanced timer; without MOE the outputs stay dead |
| `CCER.CC1E/CC2E` | 1 / 0 | per-servo enable — this is what `on`/`off` toggles |

Because the compare value *is* the pulse width, there is no conversion, no
rounding and no per-pulse CPU work: the timer generates both pulse trains in
hardware forever, with zero jitter. `off` simply clears that channel's CCER bit,
which returns the pin to its idle low level while the other servo keeps running.

### Console

`USART3` on PD8/PD9 at 115200 8N1, polled — no interrupts, no ring buffer, no
libc `printf` (there is a ~90-line formatter in `console.c`). For a human typing
commands this is the right trade: the firmware stays small and every byte path is
obvious.

### Memory layout

| Region | Address | Use |
|---|---|---|
| Flash (image) | `0x0800_0000` | vectors + code, ~13 KB of 2 MB |
| AXI SRAM | `0x2400_0000` | `.data`, `.bss`, 16 KB stack at the top |

---

## Source map

```
firmware/stm32h753/
├── Makefile              build / flash / monitor / test
├── link.ld               memory map + vector-table placement
├── prebuilt/
│   ├── nucleo_h753zi_servo_180.bin    ready to flash (180 deg)
│   └── nucleo_h753zi_servo_270.bin    ready to flash (270 deg)
├── tools/
│   ├── flash.sh          ST-LINK programming (CubeProgrammer/OpenOCD/drag-drop)
│   ├── monitor.py        console with liveness + busy-port diagnostics
│   └── find_port.py      find the ST-LINK port by USB ID
└── src/
    ├── board.h           register map (cross-checked against stm32h753xx.h)
    ├── board.c           clocks, GPIO, DWT-based delays, static asserts
    ├── startup.c         vector table, reset handler, fault handler
    ├── servo.h/.c        TIM1 50 Hz dual pulse generator
    ├── console.h/.c      USART3 driver + tiny printf + line editor
    ├── position.h/.c     "90deg" / "1.5ms" / "1500us" parser (host-testable)
    ├── main.c            the command interpreter (the terminal app)
    └── syscalls.c        freestanding stubs + memcpy/memset/memmove
```

### Changing things

| What | Where |
|---|---|
| Servo pins | `servo.c` → `gpio_init()` and the `s_servos[]` table |
| Pulse range / centre | `servo.h` → `SERVO_MIN/MAX_PULSE_US`, `SERVO_DEFAULT_PULSE_US` |
| Travel (180 vs 270) | `make TRAVEL=270`, or `SERVO_TRAVEL_DEG` in `servo.h` |
| Frame rate | `servo.h` → `SERVO_FRAME_HZ` (must stay 50 Hz for analogue/standard digital servos) |
| Console baud | `board.h` → `BOARD_CONSOLE_BAUD` |
| Sweep step/speed | `main.c` → `SWEEP_STEP_US`, `SWEEP_DELAY_MS` |
| CPU clock | `board.c` → `pll1_init()` + `flash_latency_init()` + `BOARD_SYSCLK_HZ` |

Adding a third/fourth servo: put it on PE13 (TIM1_CH3) or PE14 (TIM1_CH4), add
an entry to `s_servos[]`, bump `SERVO_COUNT`, and use `TIM1->CCR3/CCR4`. Keep
them on TIM1 so all servos share one frame.

---

## Tests

The position parser is pure C with no hardware dependency, so the exact source
that runs on the STM32 is compiled and executed on your PC:

```bash
make -C tests test-stm32      # 40 checks
make -C firmware/stm32h753 test
```

---

## What is deliberately *not* here

* **No STM32 HAL / LL / CubeMX `.ioc`.** The register subset needed for a
  200 MHz clock, a 50 Hz PWM and a UART is ~200 lines, and it is all in
  `board.h`/`board.c` with the datasheet values in comments. This also means the
  project builds from a plain `git clone` with nothing to regenerate.
* **No USB stack.** USART3 already reaches the PC through the ST-LINK — a CDC
  implementation would add thousands of lines and zero capability.
* **No interrupts.** The servo pulses are pure hardware; the only thing that
  would need an interrupt is the UART, and polling is fine for typing.
* **No dynamic memory.** No heap, no `malloc` (`_sbrk` deliberately fails).

---

## Troubleshooting

| Symptom | Likely cause / fix |
|---|---|
| `Device or resource busy` when opening the port | Another program owns it (screen/minicom/PuTTY/CubeIDE), or on Linux ModemManager is probing it. Run `make monitor` - it names the process - then `killall screen` and/or `sudo systemctl stop ModemManager`. |
| The port opens, but no data comes out | Run `make monitor` instead of guessing: it sends a CR and reports what it heard. Usually it is the *other* board's ttyACM port, or the image was overwritten. |
| Console prints a `FATAL FAULT` block | The MCU crashed and rebooted. The report gives CFSR/HFSR and the faulting PC; resolve it with `arm-none-eabi-addr2line -e out/servo.elf <PC>`. Brown-outs from insufficient servo power are the usual cause. |
| Banner says `boot : BROWN-OUT` | The supply sagged - power problem, not software. |
| No drive appears / flash tool can't connect | Use the USB port **next to the ST-LINK** (not the user-USB one, which is unconnected on this board). Re-seat the two jumpers CN4 (both in the `ST-LINK` / `U5V` positions). |
| Flashed OK, but the terminal is silent | Wrong port. `make list-ports`; on macOS it is `/dev/cu.usbmodem*`, on Windows `COMx`. Close other programs holding the port. |
| Terminal prints garbage | Baud rate not 115200, or the terminal is on the UART bridge of a different board. |
| Servo twitches / board resets when a servo moves | **Power.** 2200 µF at the servos, thick ground, ≥6 A supply — [WIRING-STM32H753.md §5](../../WIRING-STM32H753.md). |
| Servo silent, firmware looks fine | Signal wire on the wrong pin (CN10-4 / CN10-6 = PE9 / PE11), ground not shared with the supply, or the servo supply is off. Run the bench test in the wiring doc §6. |
| Servo stops 90° short at one end | 270° servo with the 180° build (or vice versa). Flash the other image, or `make TRAVEL=270`. |
| Servo buzzes / hunts at an end | Narrow the range: `make MIN_US=700 MAX_US=2300`. Mechanical stop or a travel mismatch. |
| Servo gets warm while idle | It is holding torque against a load (normal) or fighting a mechanical stop (bad). `off` unloads it completely. |
| Red LED blinking fast, nothing on the console | A hard fault was caught by `Fault_Handler` in `startup.c`. Almost always power; check the supply before suspecting the code. |
| Turns the wrong way / offset | Mechanical, not a bug: run `center both`, then re-seat the horn where you want "90°" to be. |
