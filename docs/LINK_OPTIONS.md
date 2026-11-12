# ESP-NOW (default) and optional DX-LR42/LoRa

The two ESP32 nodes use ESP-NOW by default. The ESP32-S3 and classic ESP-32S
(ESP32-WROOM-32) can be used on either side. The external DX-LR42/LR41 UART
link remains available with `-D GSE_LINK_LORA` on **both** ESP32 nodes. The
STM32 TX node has no Wi-Fi and remains LoRa-only. Unlike the current default,
the older ECR and insert PDFs describe the original LoRa configuration; they
are historical design documents, not evidence that ESP-NOW has been approved.

## Build selection

| Board | Default ESP-NOW pad | Default ESP-NOW TX | LoRa pad | LoRa TX |
|---|---|---|---|---|
| ESP32-S3 | `pad-node` | `tx-node` | `pad-node-lora` | `tx-node-lora` |
| ESP-32S | `pad-node-esp32s` | `tx-node-esp32s` | `pad-node-lora-esp32s` | `tx-node-lora-esp32s` |

The `-now` environments are aliases of the default ESP-NOW builds. Both ends
must use the same transport. LoRa selection requires two compatible modules,
wiring, and provisioning; see [wiring.md](../hardware/wiring.md). The link
layer changes, not the GUI protocol, FILL/SRV outputs or IGN interlocks.

## Provisioning ESP-NOW

1. With outputs and igniter disconnected, flash each node once using the
   appropriate default environment. Monitor each node's own USB serial port at
   115200 baud and record the **station** `local MAC` from the `link: espnow`
   banner. Placeholder peers intentionally cause `peer MAC NOT CONFIGURED`.
2. Put both station addresses in `GSE_NOW_PAD_MAC` and `GSE_NOW_TX_MAC` in
   `firmware/src/config.h`. Set the same channel on both boards. Replace the
   published `GSE_NOW_KEY_TEXT` and `GSE_NOW_PMK_TEXT` defaults with distinct,
   secret 16-byte values, identical on both nodes; protect builds containing
   these secrets. Rebuild and reflash both boards. Repeat when replacing one.
3. Read both boot logs. `PAD LINK OK (waiting for the TX node)` means **local
   radio configuration succeeded**, not that the other node is reachable. The
   pad heartbeat and GUI must show fresh telemetry before a link is considered
   established. A placeholder, initialization failure, or peer registration
   failure leaves the link unready and pad outputs in their safe state.

There is no Wi-Fi association or router. One protocol frame is sent in one
ESP-NOW datagram. The adapters check source MAC and use ESP-NOW's encrypted
unicast (CCMP); the arm byte and MAC address are not secrets or substitutes for
physical interlocks. The published default keys are not secret and must not be
used as a security guarantee. Key mismatch or interference results in lost
traffic; safety depends on the existing watchdog and independently verified
hardware, not on a delivery callback.

## Fault finding

- `peer MAC NOT CONFIGURED`: check the peer address and reflash that node;
  first-boot local MAC is available after the radio starts.
- Radio setup failure: read the named `LINK FAULT` on that node's own USB port.
- No telemetry after local setup: check that both nodes use ESP-NOW, each has
  the other's station MAC, both have matching channel and keys, and both are
  powered. The GUI only receives the TX node's text logs; the pad log is on its
  own USB port.
- A registered peer or send success is not proof that application frames are
  being accepted. Inspect the pad's valid-frame counter and GUI telemetry.

## Validation and safety gates

`make -C tests run` includes host integration tests using the actual TX core,
ESP-NOW framing adapter and pad loop with mocked I/O and radio. It covers FILL,
SRV/VEH, IGN arming, release, the one-shot limit, telemetry and link loss on
both pin maps. Syntax stubs cover ESP-IDF 4.4 and 5.1/5.4/5.5 callback APIs.
These tests do not prove a PlatformIO build, radio reception, wiring, actuator
movement or electrical safety. Run the physical bench and range gates in
[TEST_PLAN.md](TEST_PLAN.md), especially T-11, with igniter disconnected and
hazardous systems isolated. Do not use this change for live operations until
those gates and the independent safety review are complete.
