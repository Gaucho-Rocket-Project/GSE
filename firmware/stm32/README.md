# STM32H753ZI (Nucleo-144, MB1364E) as the TX / laptop node

This directory makes the **Nucleo-H753ZI the TX node** and leaves the ESP32-S3 as the pad node.
It is the same protocol, the same frames, and the same GUI — only the USB bridge changes.

```
laptop --USB--> NUCLEO-H753ZI --USART2(PD5/PD6)--> DX-LR41 ))) 2.4 GHz ((( DX-LR41 --> ESP32-S3 pad node
        (ST-LINK VCP)                                                                    servos + igniter
```

**Nothing on the pad changes.** The ESP32-S3 pad node keeps every safety decision — the 1.0 s link watchdog,
the hardware keep-alive RC pump on Q1, the igniter one-shot and hard cut, the task watchdog. Swapping the laptop
side cannot make the pad less safe, because the pad never trusts the laptop further than "a frame arrived
recently, and its arm code is valid".

## Why this is a good swap for the TX side

| | ESP32-S3 as TX node | Nucleo-H753ZI as TX node |
|---|---|---|
| Laptop link | native USB-CDC (driver can be fiddly on Windows) | ST-LINK VCP, enumerates as a plain COM port, no driver hunting |
| Supply | 5 V from the laptop | 5 V from the ST-LINK USB, plus **E5V / VIN** for a bench supply |
| Debug | USB serial only | SWD, breakpoints, live variable watch — invaluable while range-testing a link |
| Headroom | 512 KB RAM | 1 MB RAM, 2 MB flash |
| Firmware size | 335 KB flash / 22.6 KB RAM | 33.9 KB flash / 2.5 KB RAM |
| Risk if it dies | loses the link | loses the link — identically; the pad goes safe either way |

## Do you have the right board?

MB1364**E** is the Nucleo-144 revision with the **STLINK-V3EC** (USB-C debug connector) and Ethernet.
Two consequences:

* The VCP still comes from **USART3 (PD8/PD9)**. Do **not** follow F411/F446 tutorials that say USART2.
* The relevant solder bridges are **SB5/SB6** (USART3 ↔ ST-LINK). They are ON by default, and we do not
  touch them.

## Pin map (Nucleo-H753ZI → DX-LR41)

| Signal | Nucleo pin | ST morpho | Notes |
|---|---|---|---|
| DX-LR41 RXD (module RX) | **PD5** | CN11 pin 41 | USART2_TX, AF7 |
| DX-LR41 TXD (module TX) | **PD6** | CN11 pin 43 | USART2_RX, AF7 |
| AUX (module busy) | **PD7** | CN11 pin 45 | GPIO in, pull-up, read after every transmit |
| 3V3 to the module | — | CN11 pin 5 (VDD) | **3.3 V only** — the LR41 is not 5 V tolerant |
| GND | — | CN11 pin 8 | |
| ST-LINK VCP → laptop | PD8 / PD9 | (through SB5/SB6) | USART3, leave bridges at default |
| Debug console (optional) | **PA9 / PA10** | morpho | USART1, 115200 |
| Status LEDs | PB0 / PE1 / PB14 | on board | LD1 green / LD2 yellow / LD3 red |

The Arduino headers are left alone. Note that **D0/D1 on this board are PG9/PG14 and are not a UART** — a
very common trap when porting ESP32 habits over.

## User LEDs — the link, visible without a terminal

The three on-board LEDs report traffic continuously, so you can diagnose the link with nothing but the board.

| LED | Pin | Meaning |
|---|---|---|
| **LD1 green** | PB0 | one 10 ms flash per packet **sent** to the air (20 Hz when running) |
| **LD2 yellow** | PE1 | one 10 ms flash per packet **received** from the air (10 Hz when the pad answers) |
| **LD3 red** | PB14 | steady **on** = `RADIO FAULT` (the module did not provision) |

Both LEDs show a double-flash sweep at boot, so a dead LED is obvious before you read anything into the pattern.

| What you see | What it means |
|---|---|
| LD1 flickering, LD2 dark | the Nucleo is transmitting, nothing is coming back — pad, channel or antenna |
| LD1 dark | nothing is being sent: the laptop side is not connected (300 ms dead-man still emits SAFE at 20 Hz, so LD1 should flicker even with no GUI) |
| LD1 + LD2 both flickering, GUI still blank | frames arrive but the laptop is not reading them — usually a second program holding the port |
| LD3 solid red | module did not answer `AT+…` — wiring or baud, see `docs/TROUBLESHOOTING_LINK.md` |

> **Do not use Arduino's `LED_GREEN` / `LED_BLUE` / `LED_RED` macros on this board.** Its variant header defines
> `LED_BLUE` as **PB7**, but PB7 is `USART1_RX` / `LPUART1_RX` here. UM2407 §7.6.1 is the authority: LD1 = PB0,
> **LD2 = PE1**, LD3 = PB14. The firmware uses the pins directly for this reason.

LD3 is also the on-board red LED that the ST-LINK blinks during a debug session — if you see it flashing rather
than solid, that is the debugger, not a fault.

## Three ways to build it

There are two independent implementations of `gse_hal.h`. Both feed the **same portable core**
(`firmware/src/common/gse_tx_core.c`), so the laptop and the pad cannot tell which one is running.

| Build | File | Command |
|---|---|---|
| PlatformIO + Arduino core (**fastest to try**) | `firmware/src/stm32/tx_platform_nucleo.cpp` | `pio run -e tx-nucleo -t upload` |
| STM32CubeIDE / CubeMX project | `firmware/stm32/tx_platform_stm32.c` | add to your project, see below |
| Host simulation | `tests/host_hal_stub.c` | `make -C tests host-hal` |

### 1. PlatformIO (tested — `pio run -e tx-nucleo` succeeds, 33.9 KB)

```bash
cd firmware
pio run -e tx-nucleo              # build
pio run -e tx-nucleo -t upload    # flash through the on-board ST-LINK
pio device monitor -e tx-nucleo   # the same text console the ESP32 has
```

The env expands to:

```ini
platform  = ststm32
board     = nucleo_h753zi
framework = arduino
build_flags = -D GSE_ROLE_TX -D GSE_PLATFORM_STM32 -D ENABLE_HWSERIAL1 -D ENABLE_HWSERIAL2
build_src_filter = +<common/> +<stm32/>
```

`ENABLE_HWSERIAL1/2` are required: the STM32 Arduino core only instantiates the console UART (USART3) unless
asked, and `Serial1`/`Serial2` would otherwise not exist at link time. Port assignment in that build:

```
Serial   -> USART3  PD8/PD9   115200   laptop (ST-LINK VCP)
Serial2  -> USART2  PD5/PD6     9600   DX-LR41          (setRx(PD_6), setTx(PD_5))
Serial1  -> USART1  PA9/PA10  115200   debug console
PD7                                    AUX, INPUT_PULLUP
```

### 2. STM32CubeIDE / CubeMX (the auditable C port)

`tx_platform_stm32.c` is plain STM32Cube HAL C with no Arduino dependency. In CubeMX for NUCLEO-H753ZI, enable:

```
USART3  Asynchronous  115200 8N1   (Virtual COM Port — do not uncheck the VCP box)
USART2  Asynchronous    9600 8N1   PD5 = TX, PD6 = RX, AF7
USART1  Asynchronous  115200 8N1   optional debug
PD7     GPIO_Input, pull-up
SysTick 1 kHz
```

then add `tx_platform_stm32.c` and `gse_tx_core.c` to the project and call from `main()`:

```c
gse_stm32_start();                  /* binds huart3/huart2/huart1 + AUX, then gse_tx_init() */
while (1) { gse_stm32_loop(); }     /* one call per pass; 1 kHz SysTick is plenty           */
```

If your handles are not the CubeMX defaults `huart1/huart2/huart3`, call `gse_stm32_bind()` yourself and the
weak autobinder is skipped.

Check it compiles without generating a project:

```bash
sh firmware/stm32/syntax_check.sh     # stands in a stub main.h, compiles with -Wall -Wextra
```

### 3. Host simulation (no hardware at all)

```bash
make -C tests host-hal                # 29 assertions against the unmodified core
```

## Behaviour — identical to the ESP32 build

| Behaviour | Value |
|---|---|
| Frame rate into the link | 20 Hz (guaranteed by the core, not the platform) |
| Stale-frame reject | 200 ms since the previous host frame |
| Laptop dead-man | 300 ms of host silence → synthetic SAFE at 20 Hz |
| Radio provisioning | MODE1 / LEVEL3 / CHAN0E / KEY51042 / MAC 00,01 / PACKET1 / POWE13 / SLEEP2, each reply verified |
| AT timeout | 400 ms, extended while bytes keep arriving |

The two platforms differ only in plumbing: the ESP32 uses the `DxLr41` driver class, the STM32 builds the AT
commands itself; both wait for AUX to fall after a transmit so a slow air rate cannot overflow the module's
64-byte chunk buffer.

## Bringing it up

```bash
python software/run_gui.py --port /dev/ttyACM0     # Linux (ST-LINK VCP)
python software/run_gui.py --port COM5             # Windows
python software/run_gui.py --port /dev/cu.usbmodemXXXX   # macOS
```

The GUI does not care which node is on the other end. If the banner goes amber (**LINK OK — NOT ARMED**) within a
second of connecting, the Nucleo is doing its job.

### Bench checks

| Check | Expected |
|---|---|
| Console | open the VCP at 115200, send `INFO` → `TX stm32 fw 1.0 level=3 key=51042 …` |
| Module echo | send `+++` at 9600 from a spare terminal → module replies `Entry AT` |
| Provisioning | `AT+HELP` on the module shows `MODE:1`, `LEVEL:3`, `Frequency:2428MHz >> 0e`, `MAC:00,01` |
| Bridge alive | unplug the laptop USB → pad reports **COMMS LOSS** within 1.5 s (the watchdog is on the pad) |
| Laptop dead-man | kill the GUI but keep the port open → the Nucleo's 300 ms watchdog starts emitting SAFE |
| Stale frames | `SIGSTOP` the GUI for 1 s → pad goes safe, recovers when the GUI resumes |

## Spares / rollback

The ESP32 TX firmware is unchanged and still builds (`pio run -e tx-node`, or `-e tx-node-esp32s` if the
spare board is an ESP-32S ESP32-WROOM-32 devkit). Keep the flashed ESP32-S3 in the
box: if the Nucleo is lost, re-flash the ESP32-S3, plug the same DX-LR41 into it (same four wires), and the
laptop side only needs a different `--port`.

## Files

| File | What it is |
|---|---|
| `firmware/src/stm32/tx_platform_nucleo.cpp` | Arduino-core platform layer, `env tx-nucleo` |
| `firmware/stm32/tx_platform_stm32.c` | STM32Cube HAL platform layer, for CubeIDE |
| `firmware/stm32/syntax_check.sh` | header-only compile check of the HAL port |
| `firmware/src/common/gse_hal.h` | the seven services a platform must supply |
| `firmware/src/common/gse_tx_core.c` | the portable bridge logic both platforms run |
| `tests/host_hal_stub.c` | desktop stub HAL + 29 assertions |
