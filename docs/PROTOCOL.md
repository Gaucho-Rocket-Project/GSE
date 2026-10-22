# GSE link protocol — version 1

This document specifies the byte-level wire contract between the host
application (`gse-app`), the ground bridge board (`gse-ground`) and the vehicle
board (`gse-vehicle`), plus the shared control state machine and host snapshot
contract. The reference implementation is `crates/gse-proto`, which is
`no_std`, allocation-free and compiled identically into all targets.

## Physical layer

| Link | Rate | Format / signals |
|------|------|------------------|
| Laptop ↔ ground board (`NUCLEO-H753ZI` ST-LINK VCP, `PD8`/`PD9`) | `115 200 Bd` | `8N1`, `3.3 V` CMOS |
| Ground board ↔ `DX-LR42-24T12D` (`USART6`, `PG14`/`PG9` + `PG10`/`PG12`/`PG13`) | `9 600 Bd` | `8N1`, `3.3 V` TTL UART, `AUX` frame-boundary flow control (`250 ms` failsafe), `(M0=0, M1=0)` open-drain |
| Vehicle board ↔ `DX-LR42-24T12D` (`USART6`, `PG14`/`PG9` + `PG10`/`PG12`/`PG13`) | `9 600 Bd` | `8N1`, `3.3 V` TTL UART, `AUX` frame-boundary flow control (`250 ms` failsafe), `(M0=0, M1=0)` open-drain |
| Over-the-air link (`DX-LR42-24T12D`, Semtech SX1281, `2400–2500 MHz`) | `10 kbps` default (`AT+LEVEL3`, `2424 MHz`, `+13 dBm`) | Half-duplex LoRa (`LEVEL 0..7`, `230 B` max packet) or FLRC (`LEVEL 8`, `6..=127 B` packet) |

Both `DX-LR42-24T12D` modules operate in continuous transparent mode
(`AT+MODE0`, `AT+SLEEP2`, `AT+DRSSI0`). Every valid wire frame is `13..=62`
bytes long (compile-time checked in `gse_proto::radio`), fitting inside a single
air packet across all nine modulation levels (`0..=8`) without sub-packet
splitting. On receive, `DxLr42Receiver` (`crates/gse-fw-core/src/radio.rs`)
absorbs local module status lines (`"Power On\r\n"`, `"Exit AT\r\n"`) before COBS
decoding and automatically exits accidental local AT command mode (`"Entry AT"`,
`"ERROR=104"`) via `"+++"`.

## Frame

```
┌──────┬──────────────────────────────────────────────────────────┬──────┐
│ 0x00 │ COBS( header ‖ body ‖ crc16 )                            │ 0x00 │
└──────┴──────────────────────────────────────────────────────────┴──────┘
```

| Offset | Size | Field | Notes |
|---|---|---|---|
| 0 | 2 | `MAGIC` | `0xA5 0x5A` |
| 2 | 1 | `VERSION` | `1` |
| 3 | 1 | `TYPE` | message type (table below) |
| 4 | 1 | `SRC` | sender endpoint |
| 5 | 1 | `DST` | destination endpoint |
| 6 | 1 | `SEQ` | free-running per-sender counter (diagnostics only) |
| 7 | 1 | `LEN` | body length, `0..=48` |
| 8 | `LEN` | `BODY` | message payload, little-endian |
| 8+`LEN` | 2 | `CRC` | CRC-16/CCITT-FALSE, little-endian |

- **CRC** covers `MAGIC` through the last body byte (`poly = 0x1021`,
  `init = 0xFFFF`, no reflection, no final XOR; check value `0x29B1` for
  `"123456789"`).
- **COBS** removes every `0x00` inside the frame so `0x00` delimiters uniquely
  mark frame boundaries. Maximum wire frame length is 62 bytes.
- **Resynchronisation**: invalid frames (short payload, bad magic, unknown
  version, length > 48, bad COBS, CRC mismatch) are discarded and decoding
  resumes cleanly at the next `0x00` delimiter.
- **Deduplication** uses `cmd_id` inside `COMMAND`, not `SEQ`: retransmitting a
  command with the same `cmd_id` replays the cached `ACK` without actuating
  hardware twice.

## Endpoints

| Value | Name | Description |
|---|---|---|
| `0x01` | `Host` | laptop ground-station application |
| `0x10` | `Ground` | ground USB ↔ LoRa bridge board |
| `0x20` | `Vehicle` | onboard vehicle control board |
| `0xFF` | `Broadcast` | accepted as destination only, never valid as source |

## Messages

| Type | Name | Direction | Body |
|---|---|---|---|
| `0x01` | `HELLO` | board → host | 15 B |
| `0x02` | `PING` | host → board | 8 B |
| `0x03` | `PONG` | board → host | 24 B |
| `0x04` | `COMMAND` | host → vehicle | 7 B |
| `0x05` | `ACK` | vehicle → host | 24 B |
| `0x06` | `STATE` | vehicle → host | 16 B |
| `0x07` | `EVENT` | vehicle → host | 22 B |
| `0x08` | `LINKSTAT` | ground → host | 20 B |
| `0x09` | `HEARTBEAT` | host → vehicle | 5 B |

All multi-byte integers are little-endian.

### HELLO (15 B)

| Offset | Size | Field |
|---|---|---|
| 0 | 1 | protocol version |
| 1 | 3 | firmware version `major.minor.patch` |
| 4 | 1 | role (endpoint value) |
| 5 | 4 | uptime ms |
| 9 | 4 | device id (UID-derived) |
| 13 | 2 | capability bits |

Capabilities: `RADIO` 1, `SERVOS` 2, `IGNITER` 4, `WATCHDOG` 8,
`FILL_TIMER` 16, `USB_BRIDGE` 32.

### PING (8 B) / PONG (24 B)

`PING`: `nonce: u32`, `host_ms: u32`.

`PONG`: `nonce: u32` (echoed), `uptime_ms: u32`, `state: StateSnapshot` (16 B).

### COMMAND (7 B)

| Offset | Size | Field |
|---|---|---|
| 0 | 2 | `cmd_id` (dedupe key) |
| 2 | 1 | `control` (`SYSTEM` 0, `FILL` 1, `SRV` 2, `IGN` 3, `SRV+IGN` 4) |
| 3 | 1 | `action` (`0` = `OFF`/`SAFE`, `1` = `ON`/`PWR`; `SYSTEM` actions below) |
| 4 | 2 | `arg_s` (fill duration in seconds for `FILL ON`, else `0`) |
| 6 | 1 | `flags` |

- **`SYSTEM` actions (`control = 0`)**:
  - `SAFE_ALL` (`0`) — de-energise `IGN`, `SRV`, and `FILL` immediately.
  - `RESET_FILL` (`1`) — clear `FILL_SEQ_COMPLETE`, `FILL_SEQ_ABORTED`,
    `ESTOP`, `INHIBITED`, and `LINK_LOSS_SAFE` (requires `FILL = OFF` and
    `IGN = SAFE`).
  - `CLEAR_FAULTS` (`2`) — clear latched fault bits and inhibit/E-STOP flags.
- **`SRV+IGN` combined launch control (`control = 4`)**:
  - `ACTION_ON` (`1`) — used by `LIVE` launch mode after the host countdown
    reaches `T-0`: re-verifies `!fill_open && fill_seq_complete &&
    !fill_seq_aborted && !inhibited && !estop`, drives `SRV = true` and
    `IGN = true`, waits `SETTLE_SERVO_MS` (`350 ms`), emits `EventCode::SrvOn`
    + `EventCode::IgnPwr`, and replies `ACK` with `SRV_ON | IGN_PWR`.
  - `ACTION_OFF` (`0`) — safes `IGN = false` first, then drives `SRV = false`,
    waits `SETTLE_SERVO_MS` (`350 ms`), emits `EventCode::IgnSafe` +
    `EventCode::SrvOff`, and replies `ACK` with all outputs `OFF/SAFE`.
- **Command flags**: `AUTO` (`1`), `ESTOP` (`2`), `IMMEDIATE` (`4`).

### ACK (24 B)

| Offset | Size | Field |
|---|---|---|
| 0 | 2 | `cmd_id` |
| 2 | 1 | `control` |
| 3 | 1 | `action` |
| 4 | 1 | `result` |
| 5 | 1 | `reason` |
| 6 | 2 | execute time, ms |
| 8 | 16 | `state` after execution |

Results: `Ok` 0, `RejectedSafety` 1, `RejectedState` 2, `Busy` 3,
`UnknownControl` 4, `UnknownAction` 5, `BadArgument` 6, `HardwareFault` 7.

Reasons: `None` 0, `FillSrvExclusive` 1, `FillActive` 2, `SrvRequired` 3,
`FillSeqIncomplete` 4, `IgnPowered` 5, `IgnRequiredSafe` 6, `LinkLost` 7,
`AlreadyInState` 8, `NotActive` 9, `Duplicate` 10, `HardwareFault` 11,
`InvalidFillTime` 12, `Busy` 13, `NotPermitted` 14, `BadFrame` 15.

### STATE (16 B)

| Offset | Size | Field |
|---|---|---|
| 0 | 2 | `flags` |
| 2 | 2 | `faults` (latched fault bitmap) |
| 4 | 4 | `fill_remaining_ms` |
| 8 | 2 | `fill_duration_s` |
| 10 | 2 | `last_cmd_id` |
| 12 | 4 | `uptime_ms` |

Flags:

| Bit | Name | Meaning |
|---|---|---|
| 0 | `FILL_OPEN` | fill valve open |
| 1 | `SRV_ON` | servo power on |
| 2 | `IGN_PWR` | igniter powered |
| 3 | `FILL_TIMER_ACTIVE` | onboard fill countdown running |
| 4 | `FILL_SEQ_COMPLETE` | full timed fill cycle completed |
| 5 | `FILL_SEQ_ABORTED` | fill closed before timer expiry |
| 6 | `LINK_LOSS_SAFE` | outputs forced safe on link timeout |
| 7 | `LAST_REJECTED` | previous command was rejected |
| 8 | `FILL_AUTO_OFF` | last close was triggered by the fill timer |
| 9 | `BOOT_SAFE` | no actuating command accepted since boot |
| 10 | `ESTOP` | emergency stop latched |
| 11 | `INHIBITED` | ignition inhibited until reset |

Faults: `UART_OVERRUN` 0, `CRC_ERROR` 1, `LINK_TIMEOUT` 2, `WATCHDOG_RESET` 3,
`RADIO_ERROR` 4, `CONFIG_ERROR` 5, `OUTPUT_FAULT` 6.

### EVENT (22 B)

`code: u8`, `detail: u8`, `t_ms: u32`, `state: StateSnapshot`.

Codes: `Boot` 1, `FillAutoOff` 2, `FillAborted` 3, `LinkTimeoutSafe` 4,
`SafetyReject` 5, `FillOn` 6, `FillOff` 7, `SrvOn` 8, `SrvOff` 9, `IgnPwr` 10,
`IgnSafe` 11, `Estop` 12, `FillSeqReset` 13, `WatchdogReset` 14,
`RadioFault` 15, `GroundDetected` 16, `GroundLost` 17, `BridgeReady` 18,
`FaultsCleared` 19.

### LINKSTAT (20 B) & HEARTBEAT (5 B)

- `LINKSTAT`: `frames_from_radio: u32`, `frames_to_radio: u32`,
  `crc_errors: u16`, `uart_errors: u16`, `bridge_uptime_ms: u32`,
  `last_radio_age_ms: u16`, `drops: u16`.
- `HEARTBEAT`: `uptime_ms: u32`, `mode: u8` (`0` = normal).

## Behaviour

### Liveness & timers

| Mechanism | Interval / Limit | Consequence |
|---|---|---|
| Host → ground `PING` | every 500 ms (connecting), 3 s timeout | ground marked `offline`, serial port closed |
| Host → vehicle `PING` | every 500 ms (connecting), 2 s (online); 2 s timeout | vehicle marked `pinging`, in-flight command & active countdown aborted |
| Host → vehicle `HEARTBEAT` | every 1 s | resets vehicle's link-loss timer |
| Vehicle `LINK_TIMEOUT_MS` | 5 s without host frame | vehicle forces all outputs `OFF/SAFE`, sets `LINK_LOSS_SAFE` + `INHIBITED`, emits `LinkTimeoutSafe` |
| Vehicle `STATE` telemetry | every 250 ms | updates host snapshot and board-driven fill remaining time |
| Timed `FILL` (`DEFAULT_FILL_SECONDS = 60`) | `1..=600 s` | board counts down `fill_remaining_ms`; at `0 ms` host sends `FILL OFF (AUTO)` (with 5 s host backstop and 2 s autonomous vehicle close backstop) |
| `LIVE` `SRV+IGN` countdown (`DEFAULT_COUNTDOWN_SECONDS = 5`) | `1..=600 s` | host counts down on `snapshot.countdown`, re-validating every safety gate on every tick before sending `COMMAND(SrvIgn, ON)` at `T-0` |

### Control state machine (`gse-proto/src/fsm.rs`)

Both ends evaluate the exact same transition table over `(FILL, SRV, IGN)`
starting at `FILL OFF · SRV OFF · IGN SAFE`:

| Current state / condition | Event | Outcome |
|---|---|---|
| `FILL=OFF, SRV=OFF, IGN=SAFE` | `FILL ON` | `Allowed` → `FILL ON` |
| `FILL=ON` | `FILL ON` | `NoOp` (`AlreadyInState`) |
| `IGN=PWR` | `FILL ON` | `Blocked` (`IgnPowered`) |
| `SRV=ON` | `FILL ON` | `Blocked` (`FillSrvExclusive`) |
| `FILL=OFF` | `FILL OFF` | `NoOp` (`NotActive`) |
| `FILL=ON` | `SRV ON` | `Blocked` (`FillActive`) |
| `IGN=PWR` | `SRV ON` / `SRV OFF` | `Blocked` (`IgnPowered`) |
| `SRV=OFF` | `SRV OFF` | `NoOp` (`NotActive`) |
| `FILL=ON` | `IGN PWR` | `Blocked` (`FillActive`) |
| `SRV=OFF` | `IGN PWR` | `Blocked` (`SrvRequired`) |
| `!fill_seq_complete` or `fill_seq_aborted` | `IGN PWR` | `Blocked` (`FillSeqIncomplete`) |
| `inhibited` or `estop` | `IGN PWR` | `Blocked` (`NotPermitted`) |
| `IGN=SAFE` | `IGN SAFE` | `NoOp` (`NotActive`) |
| `SRV=ON` and `IGN=PWR` | `SRV+IGN ON` | `NoOp` (`AlreadyInState`) |
| `FILL=ON` | `SRV+IGN ON` | `Blocked` (`FillActive`) |
| `!fill_seq_complete` or `fill_seq_aborted` | `SRV+IGN ON` | `Blocked` (`FillSeqIncomplete`) |
| `inhibited` or `estop` | `SRV+IGN ON` | `Blocked` (`NotPermitted`) |
| `FILL=OFF`, `fill_seq_complete`, no latches | `SRV+IGN ON` | `Allowed` → `FILL OFF · SRV ON · IGN PWR` |
| `SRV=OFF` and `IGN=SAFE` | `SRV+IGN OFF` | `NoOp` (`NotActive`) |
| `SRV=ON` or `IGN=PWR` | `SRV+IGN OFF` | `Allowed` → `FILL OFF · SRV OFF · IGN SAFE` |
| any state | `SAFE_ALL` / `RESET` | `Allowed` → `FILL OFF · SRV OFF · IGN SAFE` |

### Host state snapshot (`/api/state` & SSE `state` event)

`gse-app` exposes the combined host + vehicle view at `GET /api/state` and on
every `event: state` frame on `/api/events`:

- `config`: `{op_mode: "live"|"test", fill_seconds, countdown_seconds, port, baud, confirm_ign, sound, host, http_port, shortcuts}`
- `controls`: confirmed state (`fill`, `srv`, `ign`, `safe`, `summary`, `ready`,
  `target`, `last_event`, `last_result`) — updates only when a vehicle `ACK`
  lands.
- `fill`: `{active, duration_s, remaining_ms, ends_at, seq_complete, seq_aborted, auto_off_pending}`
- `countdown`: `{active, duration_ms, remaining_ms, ends_at}` — active during
  the `LIVE` `SRV+IGN` countdown prior to transmitting `Control::SrvIgn` (`ON`).
- `actions`: per-action `{enabled, reason}` map for `fill_on`, `fill_off`,
  `srv_on`, `srv_off`, `ign_pwr`, `ign_safe`, `srv_ign_on`, `srv_ign_off`,
  `reset_controls`, `reset_fill`, `clear_faults`, and `estop`.

## Versioning

`PROTOCOL_VERSION` is incremented on incompatible wire changes; frames with an
unknown version are rejected immediately. New message types or trailing payload
bytes remain backwards-compatible within a version.
