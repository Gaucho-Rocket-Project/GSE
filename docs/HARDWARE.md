# Hardware and firmware

Two identical NUCLEO-H753ZI boards, two SX1281 UART LoRa modules, one laptop.

## Wiring

### Ground board (NUCLEO-H753ZI)

| Function | Pin | Configuration |
|---|---|---|
| USB CDC TX to laptop | PD8 | USART3_TX, AF7 (ST-LINK VCP) |
| USB CDC RX from laptop | PD9 | USART3_RX, AF7 |
| SX1281 RX ← module TX | PG9 | USART6_RX, AF7 |
| SX1281 TX → module RX | PG14 | USART6_TX, AF7 |
| SX1281 GND / 3V3 | — | common ground, 3.3 V |

The ST-LINK on the NUCLEO provides the virtual COM port; no FTDI adapter is
needed. Do **not** power the SX1281 from 5 V. PG14 is the module's RX.

### Vehicle board (NUCLEO-H753ZI)

| Function | Pin | Configuration |
|---|---|---|
| SX1281 RX ← module TX | PG9 | USART6_RX, AF7 |
| SX1281 TX → module RX | PG14 | USART6_TX, AF7 |
| FILL valve servo | PE9 | TIM1_CH1, 50 Hz, 1.0 ms closed / 2.0 ms open |
| SRV servo power | PE11 | TIM1_CH2, 50 Hz, 1.0 ms off / 2.0 ms on |
| IGN relay / MOSFET | PE13 | push-pull, high = PWR |
| Green LED (link) | PB0 | active high |
| Yellow LED (activity) | PE1 | active high |
| Red LED (fault / IGN) | PB14 | active high |

**Fail-safe wiring matters.** Fit 10 kΩ pulldowns on PE13 (ignition) and on
the servo signal inputs, and a pulldown on the igniter MOSFET gate. On reset,
panic or watchdog the MCU leaves every pin high-impedance until the firmware
configures it; those resistors are what keep the igniter safe in that window.
The ignition relay should also be wired so that an unpowered coil means SAFE.

Servo pulses use the timer's 1 MHz counter directly: `CCR = pulse width in
µs`, clamped to 500…2500 µs. The "off" position is 1.0 ms and "on" 2.0 ms; trim
the mechanics or adjust `SERVO_OFF_US`/`SERVO_ON_US` in
`firmware/stm32h753/src/hw.rs` if your linkage needs different endpoints.

## SX1281 modules

The firmware expects the module to be configured once (vendor tool or your own
AT/serial utility) as a **transparent UART bridge**:

- UART 9600 Bd, 8N1, no flow control.
- One module as "ground"/master, the other as "vehicle"/slave is fine, but both
  must use the same RF parameters (frequency, spreading factor, bandwidth,
  coding rate, sync word, network ID).
- Transparent mode: bytes written to the module's UART are transmitted as-is
  and every received packet is emitted on its UART. The protocol's COBS framing
  makes packet boundaries irrelevant.
- If the module exposes an AUX/busy line, leave it unconnected; the firmware
  relies on airtime back-pressure through its transmit queue (`radio_ready()`
  turns false when the queue is nearly full).

The protocol tolerates the module's latency (typically 20–60 ms each way) and
its occasional losses: every command is acknowledged and retried.

## Build and flash

Prerequisites: Rust 1.75 or newer, the Cortex-M target and a flashing tool.

```bash
rustup target add thumbv7em-none-eabihf
cargo install probe-rs-tools   # or use STM32CubeProgrammer / openocd
```

```bash
cd firmware/stm32h753
cargo build --release --bin gse-ground
cargo build --release --bin gse-vehicle
```

`cargo run --release --bin gse-vehicle` uses the `probe-rs` runner configured
in `.cargo/config.toml`. With STM32CubeProgrammer, flash
`target/thumbv7em-none-eabihf/release/gse-ground` (or `gse-vehicle`) directly.

The images are deliberately conservative:

- Clocking uses the 64 MHz HSI exactly as it comes out of reset: no PLL, no
  external crystal, no voltage-scaling changes. USART3/6 kernel clocks stay on
  their reset muxes (`pclk1`/`pclk2`).
- SysTick provides a 1 ms time base; all UARTs are polled in the main loop
  (the USART FIFO gives 8 bytes of RX slack). There are no peripheral
  interrupts, so the loop can never be delayed by an interrupt storm.
- IWDG1 runs from the LSI with a ~4 s timeout. Any hang resets the MCU, which
  returns all pins to their reset state — and therefore all outputs safe.
- On a panic or hard fault the panic handler forces IGN low and both servo
  pulses to "off" before parking in `wfi`.

LED behaviour (both boards): green = link to the host, yellow = recent traffic,
red = fault (or, on the vehicle, ignition powered).

## Firmware layout

```
firmware/stm32h753/src/hw.rs          register-level board layer, HAL impls
firmware/stm32h753/src/lib.rs         `gse_board` no_std library
firmware/stm32h753/src/bin/ground.rs  bridge image
firmware/stm32h753/src/bin/vehicle.rs vehicle image
```

`hw.rs` is also compiled on the host by `crates/gse-fw-check`, which runs its
portable unit tests (baud dividers, the transmit ring buffer, servo clamping).
This catches register-map typos and borrow errors without hardware; it cannot
catch wiring mistakes or a wrong pin mux, so a bring-up with a scope/logic
analyser is still recommended.

## Bring-up checklist

1. Flash `gse-ground`, connect USB. The ST-LINK VCP appears; open it at
   115200 Bd and send a `PING` frame. The green LED turns on after the first
   command and the board answers `PONG` (see `docs/PROTOCOL.md`).
2. Flash `gse-vehicle` on the second board. With both radios powered, the
   vehicle green LED blinks as `STATE` frames go out every 250 ms.
3. Run `gse-app`, pick the port (or leave on `Auto-detect`), and verify the
   header pills show **GROUND** and **VEHICLE** `online` with a plausible RTT.
4. **Verify `TEST` mode on the bench (igniter disconnected):** open **Settings**,
   switch `Control setup` to **`TEST`**, and shorten `Fill duration` (e.g. `5 s`).
   Confirm `FILL[ON/OFF]` → auto `OFF` completes the fill, `SRV[ON/OFF]` enables
   PE11 PWM and unlocks `IGN[PWR/SAFE]`, `FILL` stays greyed out while `SRV` is
   `ON`, and `SRV OFF` stays blocked while `IGN` is `PWR`.
5. **Verify `LIVE` mode on the bench (igniter disconnected):** reset controls
   (`Shift+R`), switch `Control setup` back to **`LIVE`** (with e.g. `5 s`
   `SRV+IGN` countdown), confirm `SRV+IGN[ON/OFF]` starts greyed out
   (`OFF/SAFE`), complete a timed `FILL` cycle to unlock `SRV+IGN`, verify
   pressing `SRV+IGN` runs the on-screen countdown before energising PE11 + PE13
   (`ON/PWR`), verify pressing `E-STOP` (`E`) mid-countdown aborts before
   actuation, and verify pressing `SRV+IGN` again cleanly drops PE13 (`IGN`)
   followed by PE11 (`SRV`) back to `OFF/SAFE`.
6. Pull the radio from the vehicle board and verify all outputs drop within
   ~5 s, the red LED comes on and the host logs the link loss.
7. Press *Reset controls* (`Shift+R`) after a fault and confirm the app logs
   `All outputs confirmed OFF/SAFE`, clears the latches and re-arms the controls.
8. Stop the app (Ctrl+C) with an output active and confirm the session log ends
   with `ALL SAFE acknowledged`; the vehicle must be confirmed safe before
   powering the ground station down.
