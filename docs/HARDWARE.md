# Hardware and firmware

Two identical **STMicroelectronics `NUCLEO-H753ZI`** boards, two **DX-SMART `DX-LR42-24T12D`** (`DX-LR42-24T12D(T0)`) 2.4 GHz Semtech SX1281 UART transceivers, and one operator laptop.

## Bill of materials (core electronics)

| Qty | Component | Part number | Supplier reference |
|---|---|---|---|
| 2 | Ground & vehicle MCU development boards | STMicroelectronics **`NUCLEO-H753ZI`** (`STM32H753ZIT6`, board rev `MB1364`) | [DigiKey `21348937`](https://www.digikey.com/en/products/detail/stmicroelectronics/NUCLEO-H753ZI/21348937) |
| 2 | Ground & vehicle 2.4 GHz LoRa/FLRC transceivers | DX-SMART **`DX-LR42-24T12D`** (`DX-LR42-24T12D(T0)`, Semtech SX1281, `+13 dBm`, 7-pin `2.54 mm` header, 2.4 GHz IPEX/SMA antenna) | [Amazon ASIN `B0H8NR5QPY`](https://www.amazon.com/dp/B0H8NR5QPY?plpRedirect=mhFallback&ref=clp_cat_related_p_0) |

## Critical electrical rules (`DX-LR42-24T12D` + `NUCLEO-H753ZI`)

1. **Power `DX-LR42-24T12D` `VCC` (Pin 6) from `5V` (`CN8` pin 9), with `100 µF + 0.1 µF` decoupling near the module:**
   - Per the `DX-LR42-24T12D` Datasheet V2.0 (Table 2 & §5.1), `VCC` accepts **`3.3 V – 5.5 V`** (`5.5 V` absolute maximum), with **`5.0 V` typical recommended** — `VCC ≥ 5.0 V` is required for the PA/LDO to deliver the full **`+13 dBm`** RF output power (`32 mA` peak TX, `4 mA` continuous RX, `9 mA` standby).
   - Place a **`100 µF` bulk + `0.1 µF` ceramic capacitor** across `VCC` and `GND` close to Pins 6–7 (`3V3` on `CN8` pin 7 also works for short-range bench tests at slightly reduced output power).
2. **All signal pins (`M0`, `M1`, `RX`, `TX`, `AUX`) are strictly `3.3 V` CMOS logic (`3.6 V` max):**
   - Never connect `RX`, `M0`, or `M1` to a `5 V` TTL signal or the module's onboard MCU will be damaged.
   - `STM32H753ZIT6` GPIOs run natively at **`3.3 V` CMOS**, so `PG14`, `PG9`, `PG10`, `PG12`, and `PG13` wire **directly** to the `DX-LR42-24T12D` with **no level shifter** — and when configuring a module on a laptop with a USB-UART dongle, set the dongle's logic jumper to **`3.3 V`**.
3. **Always attach the `2.4 GHz` antenna before applying `VCC`**, and keep the antenna element clear of ground planes, metal airframe tubes, and battery packs.

## Pinout & wiring

### `DX-LR42-24T12D` 7-pin header reference

Looking at the `35.60 × 18.00 mm` `DX-LR42-24T12D` breakout module (`2.54 mm` pitch header, Pins `1 → 7`):

| Module pin | Silkscreen | Direction (module view) | Level | Role |
|---|---|---|---|---|
| **Pin 1** | `M0` | Input (weak pull-up when `SWITCH=1`; low out when `SWITCH=0`) | `3.3 V` CMOS | Hardware operating mode bit 0 (`PG12`, `CN9` pin 65) |
| **Pin 2** | `M1` | Input (weak pull-up when `SWITCH=1`; low out when `SWITCH=0`) | `3.3 V` CMOS | Hardware operating mode bit 1 (`PG13`, `CN9` pin 68) |
| **Pin 3** | `RX` (`RXD`) | Input | `3.3 V` TTL **only** | UART data in **from** `NUCLEO` `PG14` (`USART6_TX`) |
| **Pin 4** | `TX` (`TXD`) | Output | `3.3 V` TTL | UART data out **to** `NUCLEO` `PG9` (`USART6_RX`) |
| **Pin 5** | `AUX` | Output | `3.3 V` CMOS | `HIGH` = RF TX/RX/reset busy; `LOW` = idle & ready (`PG10`, `CN9` pin 66) |
| **Pin 6** | `VCC` | Supply input | **`3.3 – 5.5 V` (`5.0 V` typ.)** | Module supply (`CN8` pin 9 `5V` + `100 µF` / `0.1 µF` bypass) |
| **Pin 7** | `GND` | Ground | `0 V` | Common power & signal return (`CN8` pin 11/13 or `CN10` pin 63) |

> **Full 7-wire flight harness vs minimal 4-wire bench hookup:**
> `DxLr42Port` in `firmware/stm32h753/src/hw.rs` enables an internal pull-down on `PG10` (`AUX`), an internal pull-up on `PG9` (`USART6_RX`), and open-drain outputs with weak pull-ups driven `LOW` on `PG12`/`PG13` (`M0`/`M1`). That means:
> - **Flight harness (recommended, all 7 pins wired):** gives hardware `AUX` half-duplex packet pacing (`AUX` pre-asserts `HIGH` `2–3 ms` before RX data and throughout RF TX, with a `250 ms` stuck-high failsafe) plus contention-free `(M0=0, M1=0)` hardware mode locking.
> - **Bench quick-test (4 wires: `VCC`, `GND`, `RX`, `TX`):** works immediately out of the box with zero firmware or jumper changes (`PG10` pulls gently `LOW = ready`; factory `AT+SWITCH=0` ignores `M0`/`M1`).

### Ground board (`NUCLEO-H753ZI` `MB1364` — `gse-ground`)

| Function | STM32 pin | NUCLEO header | Mode / electrical configuration |
|---|---|---|---|
| USB CDC TX → laptop | `PD8` | Internal ST-LINK VCP | `USART3_TX`, `AF7`, `115200 Bd 8N1` |
| USB CDC RX ← laptop | `PD9` | Internal ST-LINK VCP | `USART3_RX`, `AF7`, internal pull-up |
| `DX-LR42` Pin 3 (`RX`) ← MCU TX | `PG14` | **`CN10` pin 12** (`D2`) | `USART6_TX`, `AF7`, `3.3 V`, `9600 Bd 8N1` |
| `DX-LR42` Pin 4 (`TX`) → MCU RX | `PG9` | **`CN9` pin 63** | `USART6_RX`, `AF7`, internal pull-up |
| `DX-LR42` Pin 5 (`AUX`) → MCU busy | `PG10` | **`CN9` pin 66** | GPIO input, internal pull-down (`HIGH` = busy) |
| `DX-LR42` Pin 1 (`M0`) ← MCU mode 0 | `PG12` | **`CN9` pin 65** | Open-drain output + pull-up, driven `LOW` (`0`) |
| `DX-LR42` Pin 2 (`M1`) ← MCU mode 1 | `PG13` | **`CN9` pin 68** | Open-drain output + pull-up, driven `LOW` (`0`) |
| `DX-LR42` Pin 6 (`VCC`) | `5V` | **`CN8` pin 9** | `5.0 V` output rail (`+ 100 µF / 0.1 µF` decoupling) |
| `DX-LR42` Pin 7 (`GND`) | `GND` | **`CN8` pin 11 / `CN10` pin 63** | Common ground |

The integrated ST-LINK-V3E on the `NUCLEO-H753ZI` exposes `USART3` over the USB micro-B connector (`CN1`) as a standard USB CDC Virtual COM Port — no external FTDI adapter is needed between the ground board and laptop.

### Vehicle board (`NUCLEO-H753ZI` `MB1364` — `gse-vehicle`)

| Function | STM32 pin | NUCLEO header | Mode / electrical configuration |
|---|---|---|---|
| `DX-LR42` Pin 3 (`RX`) ← MCU TX | `PG14` | **`CN10` pin 12** (`D2`) | `USART6_TX`, `AF7`, `3.3 V`, `9600 Bd 8N1` |
| `DX-LR42` Pin 4 (`TX`) → MCU RX | `PG9` | **`CN9` pin 63** | `USART6_RX`, `AF7`, internal pull-up |
| `DX-LR42` Pin 5 (`AUX`) → MCU busy | `PG10` | **`CN9` pin 66** | GPIO input, internal pull-down (`HIGH` = busy) |
| `DX-LR42` Pin 1 (`M0`) ← MCU mode 0 | `PG12` | **`CN9` pin 65** | Open-drain output + pull-up, driven `LOW` (`0`) |
| `DX-LR42` Pin 2 (`M1`) ← MCU mode 1 | `PG13` | **`CN9` pin 68** | Open-drain output + pull-up, driven `LOW` (`0`) |
| `DX-LR42` Pin 6 (`VCC`) | `5V` | **`CN8` pin 9** | `5.0 V` rail (`+ 100 µF / 0.1 µF` decoupling) |
| `DX-LR42` Pin 7 (`GND`) | `GND` | **`CN8` pin 11 / `CN10` pin 63** | Common ground |
| `FILL` valve servo | `PE9` | **`CN10` pin 4** (`D6`) | `TIM1_CH1`, `50 Hz`, `1.0 ms` closed / `2.0 ms` open |
| `SRV` servo power | `PE11` | **`CN10` pin 6** (`D5`) | `TIM1_CH2`, `50 Hz`, `1.0 ms` off / `2.0 ms` on |
| `IGN` relay / MOSFET | `PE13` | **`CN10` pin 10** (`D3`) | Push-pull output, `HIGH = PWR`, `LOW = SAFE` |
| Green LED (link) | `PB0` | Onboard `LD1` | Active high |
| Yellow LED (activity) | `PE1` | Onboard `LD2` | Active high |
| Red LED (fault / `IGN`) | `PB14` | Onboard `LD3` | Active high |

**Fail-safe wiring is mandatory.** Fit `10 kΩ` pull-downs on `PE13` (`IGN`), on the servo signal inputs (`PE9`, `PE11`), and directly from the igniter MOSFET gate to source; wire any pyro relay contacts **normally open** so an unpowered coil is physically `SAFE`. During power-up, brownout, panic, or watchdog reset (`IWDG1`), STM32 pins are high-impedance until `VehicleBoard::init()` executes — external pull-downs keep hardware safe throughout that window.

Servo pulses use `TIM1`'s `1 MHz` counter directly (`CCR = pulse width in µs`, clamped to `500..=2500 µs`). `SERVO_OFF_US` (`1000 µs`) and `SERVO_ON_US` (`2000 µs`) in `firmware/stm32h753/src/hw.rs` set the endpoints.

## `DX-LR42-24T12D` configuration & RF parameters

### Factory defaults (plug-and-play compatible)

Two unconfigured `DX-LR42-24T12D` modules work straight out of the box with `gse-ground` and `gse-vehicle`:

| Parameter | AT command | Factory default | Meaning |
|---|---|---|---|
| UART baud & parity | `AT+BAUD` / `AT+PARI` | `2` / `0` | **`9600 bps, 8N1`** (matches `gse_proto::RADIO_BAUD`) |
| Transmission framing | `AT+MODE` | `0` | **Transparent UART mode** on power-up |
| Power / wake state | `AT+SLEEP` | `2` | **High-efficiency continuous RX** (`4 mA` RX, automatic RX↔TX switching) |
| Hardware pin switch | `AT+SWITCH` | `0` (`1` optional) | `0` = AT mode switch; `1` = `PG12`/`PG13` `(M0=0, M1=0)` hardware lock |
| Air data rate | `AT+LEVEL` | `3` | **LoRa `10 kbps`** (`SF8, BW 406 kHz, CR 4/5`, ~`2.2 km` LOS) |
| Carrier channel | `AT+CHANNEL` | `0C` (`00..=32` hex) | **`2424 MHz`** (`2400 + 2 × ch` MHz across `2400–2500 MHz`) |
| Output power | `AT+POWE` | `13` (`0..=13`) | **`+13 dBm`** (full rated output with `VCC = 5.0 V`) |
| Sub-packet length | `AT+PACKET` | `3` | **`230 B`** max packet (all GSE COBS frames are `13..=62 B`) |
| Appended packet RSSI | `AT+DRSSI` | `0` | Disabled (`0` = raw payload bytes only) |
| Encryption key | `AT+OPENKEY` / `AT+KEY` | `1` / `12345` | Enabled (must match on ground & vehicle modules) |
| Listen-before-talk | `AT+LBT` | `0` | Disabled (`0`) |

> **Warning on `AT+SWITCH=1`:** When `AT+SWITCH=1` is saved on a module, `M0` and `M1` become inputs with weak internal pull-ups (`(0,0)` = continuous transparent, `(0,1)` = 4 s CAD wake, `(1,0)` = AT command mode, `(1,1)` = deep sleep). **Never enable `AT+SWITCH=1` unless `PG12`/`PG13` (or `GND` jumpers) are connected to `M0`/`M1`**, otherwise floating pull-ups put the module into `(1,1)` sleep mode.

### Air-rate levels (`AT+LEVEL`)

| `LEVEL` | Modulation | Parameters | Air rate | Open-air LOS | Recommended use |
|---|---|---|---|---|---|
| `1` | LoRa | `SF11, BW 406 kHz, CR 4/8` | `1 kbps` | **`3.7 km`** | Distant bunker / maximum link margin (`--preset long-range`) |
| **`3`** *(default)* | LoRa | `SF8, BW 406 kHz, CR 4/5` | **`10 kbps`** | **`2.2 km`** | Standard pad operations (`--preset pad`) |
| `6` | LoRa | `SF6, BW 1625 kHz, CR 4/6` | `100 kbps` | Medium | High-rate close-in testing (`--preset fast`) |
| `8` | FLRC | `BW 1.2 MHz, CR 1` | `1.04 Mbps` | Short | Bench/short-range FLRC (`127 B` max packet; `OPENKEY` disabled) |

*(Note: `DX-LR42-24T12D` uses a standard crystal rather than a TCXO; never use custom bandwidths narrower than `203 kHz`, i.e., stick to built-in `LEVEL 1..8`.)*

### Provisioning modules for multi-team launch sites (`scripts/configure_dx_lr42.py`)

At crowded launch sites (e.g., FAR), assign your ground and vehicle pair a dedicated `2.4 GHz` channel, private `AT+KEY`, and optional `LEVEL 1` (`3.7 km`) + `AT+SWITCH1` hardware pin lock using a **`3.3 V`** USB-UART dongle:

```bash
# Inspect stored configuration and live 2.4 GHz channel noise floor (AT+ERSSI)
python3 scripts/configure_dx_lr42.py --port /dev/ttyUSB0 --inspect

# Provision BOTH ground and vehicle modules identically (e.g. 2440 MHz [0x14], 3.7 km LEVEL 1, hardware M0/M1 lock)
python3 scripts/configure_dx_lr42.py --port /dev/ttyUSB0 --preset long-range --channel 2440MHz --key 27182 --lock-pins
```

### Firmware resilience features (`DxLr42Receiver` & `DxLr42Port`)

- **Clean boot-banner filtering:** On power-up or `AT+RESET`, each `DX-LR42-24T12D` outputs `"Power On\r\n"` locally on `TX`. `DxLr42Receiver` (`crates/gse-fw-core/src/radio.rs`) absorbs local status banners cleanly before COBS decoding so `LINKSTAT.uart_errors` stays `0` and the ground bridge never mistakes a local boot banner for remote RF link activity.
- **Automatic AT-mode self-recovery:** If a `DX-LR42-24T12D` module on either board is accidentally left in AT command mode (`"Entry AT\r\n"` / `"ERROR=104\r\n"`), both `gse-ground` and `gse-vehicle` automatically pause UART TX for `80 ms`, assert `(M0=0, M1=0)`, send `"+++"` to reboot the module into transparent mode, flag `RADIO_ERROR` for operator visibility, and resume normal framing within `200 ms`.
- **Inter-frame `AUX` pacing with failsafe:** `DxLr42Port` pumps bytes within each COBS frame back-to-back without splitting packets across UART idle windows, pauses at `0x00` frame boundaries while `AUX` (`PG10`) is `HIGH`, and falls back after `250 ms` if `AUX` is ever shorted high so safety commands can never stall.

## Build and flash

Prerequisites: Rust 1.75 or newer, the Cortex-M target, and a flashing utility:

```bash
rustup target add thumbv7em-none-eabihf
cargo install probe-rs-tools   # or use STM32CubeProgrammer / openocd
```

```bash
cd firmware/stm32h753
cargo build --release --bin gse-ground
cargo build --release --bin gse-vehicle
```

`cargo run --release --bin gse-vehicle` flashes via `probe-rs` (`STM32H753ZITx` configured in `firmware/stm32h753/.cargo/config.toml`), or load `target/thumbv7em-none-eabihf/release/gse-ground` and `gse-vehicle` directly in `STM32CubeProgrammer`.

Architecture notes:
- Clocking uses the `64 MHz` internal `HSI` out of reset (no PLL or crystal bring-up failure modes).
- `1 ms` `SysTick` timebase + polled `USART3`/`USART6` hardware FIFOs; zero peripheral ISRs.
- `IWDG1` hardware watchdog (`~4 s` timeout on `LSI/64`) + `HardFault`/panic handler (`force_safe_outputs()`) that drives `IGN` low and both servos to `1000 µs` before halting.
- `crates/gse-fw-check` compiles `firmware/stm32h753/src/hw.rs` on the host during `cargo test` to verify baud dividers, ring buffers, servo clamping, and `AUX` flow-control state transitions.

## Bring-up checklist

1. Verify `DX-LR42-24T12D` wiring on both `NUCLEO-H753ZI` boards (`VCC` → `CN8` pin 9 `5V`, `GND` → `CN8` pin 11, `RX` → `PG14` [`CN10` pin 12], `TX` → `PG9` [`CN9` pin 63], `AUX` → `PG10` [`CN9` pin 66], `M0` → `PG12` [`CN9` pin 65], `M1` → `PG13` [`CN9` pin 68]) and confirm both `2.4 GHz` antennas are attached.
2. Flash `gse-ground`, connect USB (`CN1`), and run `gse-app`. Confirm **GROUND** turns `online` (`115200 Bd` VCP) while **VEHICLE** stays `pinging` with the ground board's red fault LED lit until the vehicle powers up.
3. Flash and power `gse-vehicle` on the second board (igniter disconnected). Confirm **VEHICLE** turns `online`, the ground red LED extinguishes, `STATE` frames arrive every `250 ms`, and `LINKSTAT.uart_errors` stays `0`.
4. **Verify `TEST` mode on the bench (igniter disconnected):** open **Settings**, switch `Control setup` to **`TEST`**, shorten `Fill duration` (e.g. `5 s`), and confirm `FILL[ON/OFF]` → auto `OFF` completes the fill, `SRV[ON/OFF]` drives `PE11` (`2.0 ms`) and unlocks `IGN[PWR/SAFE]`, `FILL` stays greyed out while `SRV` is `ON`, and `SRV OFF` stays blocked while `IGN` is `PWR`.
5. **Verify `LIVE` mode on the bench (igniter disconnected):** reset controls (`Shift+R`), switch `Control setup` to **`LIVE`** (e.g. `5 s` countdown), confirm `SRV+IGN[ON/OFF]` starts greyed out (`OFF/SAFE`), run a full timed `FILL` cycle to unlock `SRV+IGN`, verify pressing `SRV+IGN` runs the on-screen countdown before energising `PE11` + `PE13`, verify `E-STOP` (`E`) mid-countdown aborts with zero actuation, and verify pressing `SRV+IGN` while `ON/PWR` cleanly drops `PE13` (`IGN`) then `PE11` (`SRV`) to `OFF/SAFE`.
6. Unplug power or the radio from the vehicle board while an output is active and verify all outputs drop within `~5 s`, the vehicle red LED lights, and `gse-app` logs the link loss.
7. Restore the link, press *Reset controls* (`Shift+R`), and confirm `All outputs confirmed OFF/SAFE` clears latches and re-arms the console.
8. Stop `gse-app` (`Ctrl+C`) with an output active and confirm the session log ends with `ALL SAFE acknowledged` before powering hardware down.
