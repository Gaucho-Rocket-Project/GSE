# ESP32-S3 ESP-NOW firmware

This is the ESP-IDF Rust firmware for the `main` branch's ESP32-S3 variant. It
keeps the current Rust protocol and safety state machines while replacing the
NUCLEO radio link with ESP-NOW:

```text
host -- USB serial --> ESP32-S3 ground bridge
                         )) ESP-NOW (COBS + CRC frame payloads)
                      ESP32-S3 vehicle controller -- servos + igniter
```

The transport adapter is `crates/gse-espnow`; the board layer is in
`src/bin/platform.rs`. The two binaries construct:

- `gse_fw_core::ground::Ground<EspNowGroundHal<...>>` for the ground board;
- `gse_fw_core::vehicle::Vehicle<EspNowVehicleHal<...>>` for the vehicle board.

Each ESP-NOW datagram contains exactly one existing `gse-proto` frame. The
receive callback only queues the datagram; the main loop drains it and runs the
same state machine used by the STM32 vehicle. This avoids doing servo, relay,
or protocol work in the Wi-Fi callback.

## Link configuration

Both boards must be built with the same settings. They are read at build time
by `build.rs`:

| Variable | Default | Meaning |
|---|---|---|
| `GSE_ESPNOW_PMK` | none, required | 16-byte ESP-NOW primary key, 32 hex characters |
| `GSE_ESPNOW_LMK` | none, required | 16-byte ESP-NOW peer key, 32 hex characters |
| `GSE_GROUND_MAC` | `02:47:53:45:00:01` | Station MAC the ground board assigns itself |
| `GSE_VEHICLE_MAC` | `02:47:53:45:00:02` | Station MAC the vehicle board assigns itself |
| `GSE_ESPNOW_CHANNEL` | `1` | Wi-Fi channel, 1 to 13 |

Each board sets its own station MAC to the configured address, so the factory
MACs of the boards do not need to be looked up. The factory MAC is still used
for the `device_id` reported in HELLO frames. Change the MACs if another team
nearby runs this firmware.

Generate the keys once and keep them out of git:

```bash
export GSE_ESPNOW_PMK=$(openssl rand -hex 16)
export GSE_ESPNOW_LMK=$(openssl rand -hex 16)
```

The build stops with an error if either key is missing.

## Build and flash

ESP-IDF Rust needs the Xtensa toolchain. ESP-IDF v5.3.3 itself is downloaded
by the first build into `.embuild/`:

```bash
cargo install espup espflash ldproxy
espup install --targets esp32s3
source "$HOME/export-esp.sh"

cd firmware/esp32
cargo +esp build --release --bin gse-esp32-vehicle
espflash flash target/xtensa-esp32s3-espidf/release/gse-esp32-vehicle
cargo +esp build --release --no-default-features --features ground \
  --bin gse-esp32-ground
espflash flash target/xtensa-esp32s3-espidf/release/gse-esp32-ground
```

If `cargo +esp` reports `no such command: +esp`, the `cargo` on `PATH` is not
rustup's (for example Homebrew's); run `rustup run esp cargo ...` instead.

## Board wiring

| Function | ESP32-S3 GPIO |
|---|---:|
| Vehicle FILL servo PWM | 7 |
| Vehicle SRV servo PWM | 8 |
| Igniter output | 6 |
| Vehicle LEDs link/activity/fault | 1/2/3 |
| Ground host UART TX/RX (UART0, 115200 8N1) | 43/44 |
| Ground LEDs link/activity/fault | 1/2/3 |

FILL and SRV use 50 Hz LEDC PWM with 1 ms safe and 2 ms active pulses. The
vehicle drives IGN low and both servos to the safe pulse before it starts
Wi-Fi, then the shared state machine applies its boot-safe state. IGN still
needs an external pull-down, because the pin floats between reset and that
first write. An external heartbeat/enable circuit is still required for a real
igniter installation; firmware cannot replace that independent power cut.

## Serial ports

On the ground board, UART0 (GPIO43/44, the "UART" USB port on most DevKitC-1
boards) carries only `gse-proto` frames for `gse-app`. ESP-IDF's console,
logs and panic messages are sent to the native USB-Serial/JTAG port ("USB")
instead, as set in `sdkconfig.defaults`. The ROM bootloader still prints a
short banner on UART0 at reset, which `gse-app` discards.

## Behaviour notes

- ESP-NOW is encrypted, and each board accepts packets only from the other
  board's configured MAC.
- The main loop yields for 1 ms per pass and is registered with the task
  watchdog; a loop stalled for 2 s resets the board.
- Up to four datagrams may wait for ESP-NOW's send callback before the cores
  see the radio as not ready.
- These boards have no DX-LR42 or E28 radio. A radio switch from Settings is
  answered with `HardwareFault` (and the vehicle sets its radio fault flag).
  `LinkState` still reports the protocol default, so the app labels the link
  DX-LR42 until `gse-proto` gets an ESP-NOW radio type.
