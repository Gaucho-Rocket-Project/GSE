# Link options — LoRa module or ESP-NOW

*Two ways to connect the TX node to the pad node. Same protocol, same frames, same failsafes. Pick one at
build time and build **both** nodes the same way.*

*ESP-NOW replaces the Bluetooth (BLE) transport that appeared in an earlier commit. The reason is the
failure mode, not fashion: BLE needed a connection — advertise, scan, connect, retry — and a connection
is extra state that can be lost, so it added a reconnect path and a slower recovery than anything the
protocol does. ESP-NOW is connectionless: a frame is a datagram addressed to a MAC, and if it is lost
the next frame 20 ms later simply goes out. Fewer moving parts underneath a dead-man switch is the
whole argument.*

---

## 1. The short version

| | **LoRa** (default) | **ESP-NOW** |
|---|---|---|
| Hardware | 2× DX-LR42-24T12D / DX-LR41-24T12S + antennas | none — the ESP32's own Wi-Fi radio |
| Wiring | 5 wires per node + 100 µF/100 nF at the module | none |
| Provisioning | AT commands, verified at boot | two MAC addresses in `config.h`, once per pair (§3.3) |
| Range, line of sight | hundreds of metres to ~2 km at `LEVEL 3` | a few hundred metres at best, on-board antenna (see §4) |
| Goes through brush/terrain/vehicles | yes (2.4 GHz LoRa at 10 kbit/s, sub-GHz-like robustness) | poorly — 2.4 GHz, direct sequence, no retries |
| Interference with Wi-Fi | shares 2.4 GHz, tolerates it well | shares 2.4 GHz and *is* Wi-Fi; it is the weaker neighbour |
| Pairing / connection | none | none — connectionless, nothing to connect *or* lose |
| Crypto | module KEY word (obfuscation, not encryption) | CCMP (AES-128) on every frame, key compiled in |
| Needs a laptop at the pad | no | no |
| Status in this repo | **default**, what ECR-01 specifies | optional, bench/short-range |
| Test plan gate | G1–G4 | T-11 (§7) takes the place of T-00, T-01, T-01a, T-02, T-03, T-07 |
| Firmware flag | *(none)* | `-DGSE_LINK_ESPNOW` |

**Default stays LoRa.** Nothing about the shipped configuration changes: the `pad-node`, `tx-node`,
`pad-node-esp32s` and `tx-node-esp32s` environments build exactly as before.

ESP-NOW exists so the pair works **with no radio module at all** — a spare pad, a bench run on a
kitchen table, a launch where the modules are lost or delayed, or a quick check that a protocol change
broke nothing on the air. It is also the better of the two *optional* transports for anything the pad
has to survive: it is encrypted, and it has no connection to drop.

---

## 2. What changes between the two, and what does not

The seam is one file: [`firmware/src/link.h`](../firmware/src/link.h). Above it, everything is
transport-agnostic; below it, two tiny adapters.

**Unchanged — byte for byte, in both builds**

- the 7E frame format, CRC-16, sequence numbers and version byte (`gse_protocol.h`);
- the pad's 1.0 s watchdog (`wdg_ds`), the failsafe output set, and the "recompute from the local clock
  every 20 ms" model;
- the arming rule: igniter only while `arm_code == 0xA5`, the link is fresh, and the on-time limit holds;
- the on-time limiter (0.5 s default), the one-pulse-per-press latch, and the hardware keep-alive;
- the TX node's pacing, its 200 ms stale-frame rejection and its 20 Hz synthetic-SAFE stream;
- the GUI, the text console (`PING`, `INFO`, `SAFE`, `RADIO CONFIG`, …) and the log format.

**Different — and only this**

- how many bytes leave one board and arrive at the other, and how they are addressed;
- the boot-banner line that names the transport;
- what `RADIO CONFIG` means (LoRa: re-write and verify the module's AT parameters; ESP-NOW: verify the
  radio started and the peer is registered — there is nothing to re-negotiate);
- how a frame is *lost*: over LoRa it is a byte stream, over ESP-NOW it is a whole datagram.

Because the frame is the same, the pad cannot tell which transport is underneath. That is deliberate: a
fault in the safety logic cannot hide behind a change of radio, and the host tests exercise both paths.

---

## 3. Building and running it

### 3.1 Environments

| Board | Pad | TX node |
|---|---|---|
| ESP32-S3-DevKitC-1 | `pad-node-now` | `tx-node-now` |
| ESP-32S / WROOM-32 | `pad-node-now-esp32s` | `tx-node-now-esp32s` |
| NUCLEO-H753ZI | *(not supported — no Wi-Fi radio)* | *(not supported)* |

```sh
cd firmware
pio run -e tx-node-now         -t upload     # laptop side
pio run -e pad-node-now        -t upload     # pad side
```

or, if you would rather keep one environment and flip a flag, `-D GSE_LINK_ESPNOW` on both nodes. Either
way, **both ends must be ESP-NOW**. An ESP-NOW pad cannot hear a LoRa TX node and vice versa — see §6.

The `-now` environments change one thing beyond the flag: nothing else, in fact — ESP-NOW is part of the
Arduino-ESP32 core (`<esp_now.h>`, `<esp_wifi.h>`, `<esp_netif.h>`), not an external library, so the base
environments' `lib_ldf_mode = off` is fine and no extra dependency is pulled in. The Wi-Fi library is part
of every ESP32 core build; nothing has to be enabled for it.

**Verified in this repo:** all four `-now` environments pass the host syntax check against both
arduino-esp32 2.x (ESP-IDF 4.4) and 3.x (ESP-IDF 5.x) — the receive-callback signature is the only
thing that differs between them and `link.h` shims it — and the "one frame = one datagram" rule is
covered by `make -C tests run`. The Espressif toolchain is not available in this workspace, so
**`pio run -e …-now` has not actually been executed here** — that is the first thing to do on a machine
with PlatformIO installed (§7, T-11 step 1).

### 3.2 What the boot log says

First flash, with the placeholder MACs still in `config.h`:

```
LOG PAD link: espnow -- ESP-NOW ch 1, local MAC 24:6F:28:AA:BB:02, peer MAC NOT CONFIGURED (set GSE_NOW_TX_MAC and rebuild)
LOG PAD LINK FAULT: peer MAC not configured: flash once, read this node's local MAC from the banner above, ...
```

That is deliberate. A pad that cannot possibly hear the TX node must not report `LINK OK` and must not
sit there looking healthy — it holds its outputs safe and says what is missing.

After both MACs are in and both boards reflashed:

```
LOG PAD link: espnow -- ESP-NOW ch 1, local 24:6F:28:AA:BB:02 -> peer 24:6F:28:AA:BB:01, CCMP on, no Wi-Fi association
LOG PAD LINK OK  (waiting for the TX node)
LOG PAD ESP-NOW: ch 1 -> peer 24:6F:28:AA:BB:01; the TX node must be built with -DGSE_LINK_ESPNOW ...
```

TX node:

```
LOG link: espnow -- ESP-NOW ch 1, local 24:6F:28:AA:BB:01 -> peer 24:6F:28:AA:BB:02, CCMP on, no Wi-Fi association
LOG TX esp32s3 fw 1.0 ready
LOG espnow: ch 1 -> peer registered
LOG tx fwd=812 state=79 safe=0 dropped=0 radio=ok
```

LoRa builds print `PAD link: lora -- LoRa UART1 RX<-D16 TX->D17 AUX=wired M0/M1=tie to GND at the
module` in the same place. **Read that line first when a link will not come up**: it is the one thing
that tells you two boards were built differently.

`LINK FAULT` on an ESP-NOW build has three causes and the banner names which: `esp_now_init failed`
(the Wi-Fi driver did not start — wrong chip or a core built without Wi-Fi), `peer MAC not configured`
(the placeholders are still in `config.h`), or `esp_now_add_peer failed` (the peer table is full, which
only happens if the same MAC was added twice with different keys). None of them is a wiring problem.

### 3.3 Provisioning the MAC addresses (once per pair of boards)

ESP-NOW addresses frames with the **station** MAC of the receiving board's Wi-Fi interface, so each
node needs the *other* node's MAC compiled in. A board's own MAC is not something a build can fill in,
so this is a two-pass step and it is the only extra work this transport asks for:

1. Flash **both** boards with the placeholders in `config.h` and open each board's serial monitor.
   The pad says it in `describe()`: `local MAC 24:6F:28:AA:BB:02`. The TX node prints the same line
   with `local 24:6F:28:AA:BB:01`. Write both down.
2. Put them in `firmware/src/config.h`:

   ```c
   #define GSE_NOW_PAD_MAC   { 0x24, 0x6F, 0x28, 0xAA, 0xBB, 0x02 }   /* the pad  */
   #define GSE_NOW_TX_MAC    { 0x24, 0x6F, 0x28, 0xAA, 0xBB, 0x01 }   /* the TX   */
   ```

   (or pass them as `-DGSE_NOW_PAD_MAC='{...}'` build flags, which is what the second pair of boards on
   a second build environment would do).
3. Rebuild and reflash **both** boards. One build cannot do this for the other: the MAC is a property
   of the radio, not of the firmware.

The MAC to use is the **station** MAC (`WIFI_IF_STA`), which is what the ESP32 prints for
`Wi-Fi MAC address` in its Arduino-ESP32 `STA MAC` boot line — not the Bluetooth MAC, and not the
Wi-Fi *access point* MAC, which is the station MAC plus one in the last byte on most ESP32s. If in
doubt, trust the firmware's own `local MAC` line: it is read with `esp_wifi_get_mac(WIFI_IF_STA, …)`
after the driver starts, i.e. exactly the address the peer must target.

Note the placeholder check: `02:00:00:00:00:0x` is not a real Espressif address, so `link.h` treats it
as "not configured" rather than as a peer. Two boards flashed with the placeholders will each report
`peer MAC NOT CONFIGURED` and refuse to link, which is what you want — a link that silently accepts
"whoever spoke first" would be a pad that anyone in range could command.

## 4. Range, and why "it worked on the bench" is not range

ESP-NOW runs on the ESP32's Wi-Fi radio at its default transmit power with the on-board antenna. That is
a *good* 2.4 GHz link for a few hundred metres in the open with clean line of sight, and a *poor* one
through anything: a vehicle, a berm, a fuselage, wet brush, or a person standing between the two
antennas.

Treat the numbers as a starting point for a range test, not as a specification:

| Situation | LoRa `LEVEL 3` | ESP-NOW |
|---|---|---|
| Bench, 2 m through the room | fine | fine |
| Pad at the end of a 30 m hose, line of sight | fine | fine |
| Through a truck / behind a berm | fine | unreliable |
| 300 m across flat desert | fine | usually fine, antenna-height dependent |
| 2 km | fine | no |

Two things make ESP-NOW's failure mode *kinder* than the BLE transport it replaces, even though both
share the same 2.4 GHz band:

- **No connection to lose.** There is no advertising, no scan, no connection interval and no
  supervision timeout, so there is no "link looks up but nothing arrives" state and no recovery
  sequence to wait through. Every frame is independently delivered or lost, and the next one is 20 ms
  behind it.
- **Delivery is reported.** `esp_now_send()` returns a per-packet status, so the TX node counts an
  outright failure separately from a pad that has gone quiet (`GSE_NOW_QUIET_MS`), and both numbers
  are visible in its 1 Hz line.

But the 1.0 s pad watchdog does not care about any of that, which is the point: a lost frame costs one
20 ms tick; a lost *link* opens FILL and VEHICLE and drops IGN within `wdg_ds`.

**Do not use ESP-NOW as the link for a live fill unless T-11 has been signed off for that build and the
measured range covers the actual separation with margin.** The ECR's answer for flight is still LoRa.

---

## 5. Security — read before using this off the bench

The GSE protocol has **no authentication**. Anyone in radio range who can speak it can command a pad.
That is already true of the LoRa link, which mitigates it with a private channel, a shared KEY word and
the arm code; the physical SW2 arm key and the igniter interlock are the real interlocks.

ESP-NOW changes the radio layer, so it changes what "in range" means — and here it is a genuine
improvement over both other options:

- **Every frame is encrypted.** The peer is added with `encrypt = true`, a 16-byte local master key
  (`GSE_NOW_KEY_TEXT`) and a 16-byte primary master key (`GSE_NOW_PMK_TEXT`), both compiled in from
  `config.h`. ESP-IDF uses CCMP (AES-128), so a third party with a plain ESP-NOW sniffer sees ciphertext
  rather than GSE frames. The key text is baked into the firmware image, though: it is a link-layer
  privacy measure between these two boards, **not** a secret an attacker holding your firmware binary
  cannot have. Change the text for a launch if you want, and rebuild both ends — a mismatch is silent
  at the receiver (the TX node logs failed sends, the pad simply hears nothing).
- **The peer is pinned by MAC.** `link.h` compares every received frame's source address against
  `GSE_NOW_PAD_MAC` / `GSE_NOW_TX_MAC` and drops anything else (the `rx foreign` counter in the 1 Hz
  line shows it). A neighbour's ESP-NOW traffic, or a spare board flashed with the same key, is not
  accepted merely because it is in range.
- **Nothing is associated.** The Wi-Fi station is started and never connected to an access point: no
  `esp_wifi_connect()`, no SSID, no DHCP. There is no network to reach through this board, and no
  credentials stored on it (`WIFI_STORAGE_RAM`).
- **Nothing to pair or bond.** Pairing would add a key store and an operator step in exchange for
  protecting a payload that already carries a CRC and a dead-man switch — and it would break the
  "flash both boards and go" property this option exists for.

The honest summary: with ESP-NOW, someone who has your firmware (and so your keys) and is within radio
range can talk to the pad; without the keys they can jam it but cannot command it. On LoRa, anyone with
a compatible module and the right channel/KEY word can talk to it, in the clear. Neither link is a
barrier against a determined attacker; **the barrier is procedural** — arm key out, someone on the pad,
IGN unloaded until the last moment.

If that is not acceptable for a given launch, use LoRa, or use neither and wire the pad.

---

## 6. Fault-finding, ESP-NOW-specific

| Symptom | Look at |
|---|---|
| Pad prints `peer MAC NOT CONFIGURED` | the placeholders are still in `config.h`. Flash once, read each board's `local MAC`, set both, reflash **both** (§3.3) |
| Pad prints `LINK FAULT: esp_now_init failed` | the Wi-Fi driver did not start — wrong board, or a core built without the Wi-Fi stack. On a NUCLEO this is expected: there is no ESP-NOW for STM32 |
| Both banners look right, pad's `bytes=` stays 0 | one of them is a LoRa build. Read the first `LOG … link:` line on each. Then check the **channel** — it is one constant on both, so a mismatch means one board was not reflashed |
| Everything links, TX node logs failed sends | the two ends have different key/PMK text, or the pad's MAC is stale after a board swap. Rebuild both from the same `config.h` |
| Link worked, then stopped the moment a phone hotspot started nearby | channel collision; move `GSE_NOW_CHANNEL` (and rebuild both) or turn the hotspot off. There is no automatic channel selection here, on purpose |
| `rx foreign` climbing while the link is up | a third ESP-NOW device on the same channel — ignored by design, harmless, but worth knowing about at a launch |
| `RADIO CONFIG` says `peer registered` but nothing arrives | that is not a configuration fault: `RADIO CONFIG` has nothing to re-negotiate on this transport. Check range (§4) |
| Link works, GUI shows `rejected frames` climbing | not a link fault — that is stale-frame rejection, i.e. the laptop, not the radio |
| Everything links, pad never moves a servo | SW2 arm key, then ARM in the GUI — the link being up is not arming |

The pad's `PAD st=… bytes=… sof=… valid=…` heartbeat works identically on both transports. On ESP-NOW
`sof` should track `bytes / 19` closely: every datagram that arrives is a whole frame, so a `sof` count
well below the byte count means someone else is transmitting on that channel, while `valid` below `sof`
means corrupted frames — interference, not a wrong baud rate.

---

## 7. Test plan delta (T-11)

The LoRa gate tests do not apply to an ESP-NOW build: there is no module to provision (T-01), no
channel/KEY to match (T-03), no M0/M1 (T-00), no UART pins (T-01a), no AUX line (T-02) and no baud rate
(T-07). Replace T-00, T-01, T-01a, T-02, T-03 and T-07 with the following, and keep T-04 (failsafe),
T-06 (igniter), T-08 and T-09 (operational) exactly as written — they are transport-independent and are
the ones that matter.

### T-11 · ESP-NOW transport (B, then R)

1. **Builds.** `sh tests/esp32_syntax_check.sh` passes (both roles, both boards, both Arduino-ESP32
   major versions); `pio run -e pad-node-now` and `pio run -e tx-node-now` complete with no warnings
   from `src/`.
2. **Provisioning.** The first flash of each board prints its own `local MAC`. Both MACs go into
   `config.h`, both boards are rebuilt and reflashed, and only then does either stop printing
   `peer MAC NOT CONFIGURED`. **Record both MACs here** — that pair is the link identity for the launch.
3. **Boot banners.** Pad prints `PAD link: espnow … -> peer …` and `PAD ESP-NOW: ch 1 -> peer …`. TX
   node prints `link: espnow …` and `TX … fw … ready`. Neither prints `RADIO FAULT` / `LINK FAULT`.
4. **Link.** Within 2 s of both being powered the pad's `PAD st=…` line reads `LINK OK`, `bytes`
   climbs, `sof` climbs, `valid` climbs, and the TX node's `state=` counter climbs. GUI banner reads
   **LINK OK — NOT ARMED**. `rx foreign` is 0 with nothing else powered on that channel.
5. **Kill and recover.** Power-cycle the TX node. The pad must show `NO-LINK (safe)` (amber / 2 Hz)
   within 1 s, hold all outputs safe, and return to `LINK OK` on its own within ~1 s of the TX node
   coming back — no reset of the pad, and no reconnect delay to wait through.
6. **Range (R).** Walk the TX node away until the pad shows NO-LINK, record the distance and the
   conditions (line of sight / through what). Repeat twice. **Pass:** the recorded distance exceeds the
   planned separation by at least 2×. If it does not, this transport is not usable for that launch.
7. **Key mismatch (B).** Rebuild **one** end with a different `GSE_NOW_KEY_TEXT`. Neither node may
   report LINK OK and no servo may move. Rebuild both from the same `config.h` afterwards.
8. **Watchdog (H).** With the link up and FILL commanded open, power off the TX node. FILL must close
   and VEHICLE open within `wdg_ds` (1.0 s default) — the same acceptance as T-04.
9. **Igniter (H).** With the link up, arm and fire with the IGN outlet **unloaded** (LED/scope on the
   relay contacts): one pulse, ≤ `ign_max_ds`, no repeat while the button is held. Release and re-press
   for a second pulse. Same acceptance as T-06.
10. **Wrong-pair check.** Build one node LoRa and one ESP-NOW. Neither must ever report LINK OK, and
    neither may move a servo. Reflash both to the same transport.
11. **Sign-off.** Record board serials, **both MACs**, firmware commit, measured range, and who ran
    steps 6–9.

---

## 8. Where each transport appears in the repo

| File | What it says about the transport |
|---|---|
| `firmware/src/link.h` | the seam itself; both adapters. The ESP-NOW adapter owns the receive ring, the peer setup and the IDF-4.4/5.x receive-callback shim; the LoRa adapter is the DX-LR41 driver, unchanged. (`GseLink gseLink;` is named that way on purpose — the core's `Arduino.h` brings in POSIX `link()` via lwIP, and `tests/esp32_stub/Arduino.h` declares the same names so the host check catches it) |
| `firmware/src/config.h` | `GSE_LINK_ESPNOW` / `GSE_LINK_LORA`, the "exactly one" rule, the STM32 guard, the peer MACs, channel, key/PMK text, the quiet and queue constants |
| `firmware/platformio.ini` | the four `-now` environments |
| `firmware/src/pad/pad_node.cpp`, `firmware/src/tx/tx_platform_esp32.cpp` | one code path; `gseLink.send()` / `.read()` / `.poll()` |
| `tests/esp32_syntax_check.sh` | both roles × both boards × both cores, the MAC-override case, and the negative cases (two links at once, ESP-NOW on STM32, a 15-character key) |
| `tests/esp32_stub/esp_now.h`, `esp_idf_version.h`, `esp_wifi.h` | the 2.x/3.x-compatible ESP-NOW stand-ins for that check; `esp_wifi.h` deliberately declares no connect/scan API |
| `tests/test_protocol.c` | one-frame-one-datagram sizing, and torn-frame behaviour |
| `docs/TEST_PLAN.md` | T-11 (this document, §7) |
| `docs/TROUBLESHOOTING_LINK.md` | the ESP-NOW symptom table (§6) |
| `docs/OPERATOR_CARD.md` | the transport line to check at power-up |
| `hardware/wiring.md` | what you *do not* have to wire in an ESP-NOW build |
