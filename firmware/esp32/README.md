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

## Build and flash

This package is intentionally standalone because ESP-IDF Rust requires an
Xtensa toolchain and an ESP-IDF installation. Install `espup`, source the
generated export file, and install `espflash`:

```bash
cargo install espup espflash cargo-espflash
espup install
source "$HOME/export-esp.sh"

cd firmware/esp32
cargo +esp build --release --bin gse-esp32-vehicle
cargo +esp espflash flash --release --bin gse-esp32-vehicle
cargo +esp build --release --no-default-features --features ground \
  --bin gse-esp32-ground
cargo +esp espflash flash --release --no-default-features --features ground \
  --bin gse-esp32-ground
```

The current host cannot complete this build without the ESP-IDF toolchain; a
normal macOS host build fails during `esp-idf-sys` setup with
`Unsupported target 'aarch64-apple-darwin'`. That is an environment
limitation, not a Rust type-check result.

## Board wiring

The default `src/bin/platform.rs` mapping is deliberately explicit:

| Function | ESP32-S3 GPIO |
|---|---:|
| Vehicle FILL servo PWM | 7 |
| Vehicle SRV servo PWM | 8 |
| Igniter output | 6 |
| Vehicle LEDs link/activity/fault | 1/2/3 |
| Ground host UART RX/TX | 43/44 |
| Ground LEDs link/activity/fault | 1/2/3 |

FILL and SRV use 50 Hz LEDC PWM with 1 ms safe and 2 ms active pulses.
The relay output is forced low during construction; the shared vehicle state
machine then applies its boot-safe state before accepting commands. An
external heartbeat/enable circuit is still required for a real igniter
installation; ESP-NOW firmware cannot replace that independent power cut.

## Peer and security configuration

The ESP-NOW MAC addresses should be explicit build-time configuration, not
discovered from arbitrary nearby devices. The vehicle must accept packets only
from the configured ground MAC, and the ground bridge must accept packets only
from the configured vehicle MAC. ESP-NOW encryption/PMK/LMK configuration is
also a deployment requirement. The sample currently uses unencrypted ESP-NOW
with fixed peer MACs to make bench bring-up possible; it is not flight-ready
until peer filtering, ESP-NOW encryption, and the hardware keep-alive circuit
are validated.
