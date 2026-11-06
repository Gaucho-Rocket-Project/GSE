"""
protocol.py  --  host-side mirror of firmware/src/common/gse_protocol.h
--------------------------------------------------------------------------
Every constant and rule here MUST match the C header byte for byte.  The unit
tests (tests/test_protocol.py) compare this implementation against the same
golden vectors the C tests use, so a firmware/protocol drift breaks the build.

GPL-3.0, same licence as HCR-5100.
"""
from __future__ import annotations

import struct
from dataclasses import dataclass, asdict

PROTO_VERSION = 1
SOF = 0x7E
MAX_PAYLOAD = 32
MAX_FRAME = 5 + MAX_PAYLOAD + 2
ARM_CODE = 0xA5

T_HELLO = 0x01
T_CMD = 0x10
T_STATE = 0x11
T_EVENT = 0x12
T_CFG = 0x13
T_ACK = 0x81

NODE_TX, NODE_PAD = 1, 2
PAD_BOOT, PAD_WAIT_HELLO, PAD_RUN, PAD_LATCHED = 0, 1, 2, 3

DEF_WDG_DS = 10
DEF_IGN_MAX_DS = 5
WDG_DS_MIN, WDG_DS_MAX = 3, 30
IGN_DS_MIN, IGN_DS_MAX = 1, 20

ST_LINK_OK = 0x01
ST_LATCHED = 0x02
ST_ARMED = 0x04
ST_IGN_ON = 0x08
ST_COMMS_LOSS = 0x10
ST_HELLO_OK = 0x20

IGN_SRC_CMD = 0x01
IGN_SRC_WDG = 0x02

EV_BOOT, EV_HELLO_OK, EV_COMMS_LOSS, EV_ARM, EV_DISARM = 1, 2, 3, 4, 5
EV_IGN_ON, EV_IGN_OFF, EV_CFG_CHANGED, EV_BAD_CMD = 6, 7, 8, 9

EVENT_NAMES = {
    EV_BOOT: "BOOT",
    EV_HELLO_OK: "HELLO_OK",
    EV_COMMS_LOSS: "COMMS_LOSS",
    EV_ARM: "ARM",
    EV_DISARM: "DISARM",
    EV_IGN_ON: "IGN_ON",
    EV_IGN_OFF: "IGN_OFF",
    EV_CFG_CHANGED: "CFG_CHANGED",
    EV_BAD_CMD: "BAD_CMD",
}

CMD_FMT = "<BBBBBBBBHBB"      # 12 bytes
STATE_FMT = "<6B3HbBH8B"         # 24 bytes
CFG_FMT = "<BBBBBBBB"         # 8 bytes
EVENT_FMT = "<BBB"            # 3 bytes


def crc16(data: bytes, init: int = 0xFFFF) -> int:
    """CRC-16/CCITT-FALSE (poly 0x1021, init 0xFFFF, no reflection)."""
    crc = init
    for b in data:
        crc ^= b << 8
        for _ in range(8):
            crc = ((crc << 1) ^ 0x1021) & 0xFFFF if crc & 0x8000 else (crc << 1) & 0xFFFF
    return crc


# --------------------------------------------------------------- structures
@dataclass
class Cmd:
    arm_code: int = 0
    fill: int = 0
    veh: int = 1
    prg: int = 0
    ign: int = 0
    wdg_ds: int = 0
    ign_max_ds: int = 0
    flags: int = 0
    tx_ms: int = 0
    ign_wdg: int = 0
    reserved: int = 0

    def pack(self) -> bytes:
        return struct.pack(CMD_FMT, self.arm_code, self.fill, self.veh, self.prg,
                           self.ign, self.wdg_ds, self.ign_max_ds, self.flags,
                           self.tx_ms, self.ign_wdg, self.reserved)

    @classmethod
    def unpack(cls, b: bytes) -> "Cmd":
        return cls(*struct.unpack(CMD_FMT, b))


@dataclass
class State:
    flags: int = 0
    out_fill: int = 0
    out_veh: int = 0
    out_prg: int = 0
    out_ign: int = 0
    ign_src: int = 0
    age_ms: int = 0xFFFF
    rx_count: int = 0
    crc_err: int = 0
    rssi_dbm: int = 0x7F
    pad_state: int = PAD_BOOT
    uptime_s: int = 0
    wdg_ds: int = DEF_WDG_DS
    ign_max_ds: int = DEF_IGN_MAX_DS
    fw_major: int = 0
    fw_minor: int = 0
    travel_fill: int = 45
    travel_veh: int = 45
    travel_prg: int = 45
    allow_purge: int = 1

    def pack(self) -> bytes:
        v = asdict(self)
        for k in ("out_veh", "out_prg", "out_ign"):
            v[k] = 1 if v[k] else 0
        v["allow_purge"] = 1 if v["allow_purge"] else 0
        return struct.pack(
            STATE_FMT, v["flags"], v["out_fill"], v["out_veh"], v["out_prg"],
            v["out_ign"], v["ign_src"], v["age_ms"], v["rx_count"], v["crc_err"],
            v["rssi_dbm"], v["pad_state"], v["uptime_s"], v["wdg_ds"],
            v["ign_max_ds"], v["fw_major"], v["fw_minor"], v["travel_fill"],
            v["travel_veh"], v["travel_prg"], v["allow_purge"])

    @classmethod
    def unpack(cls, b: bytes) -> "State":
        f = struct.unpack(STATE_FMT, b)
        return cls(*f)

    @property
    def link_ok(self) -> bool:
        return bool(self.flags & ST_LINK_OK)

    @property
    def armed(self) -> bool:
        return bool(self.flags & ST_ARMED)

    @property
    def ign_on(self) -> bool:
        return bool(self.flags & ST_IGN_ON)


@dataclass
class Cfg:
    travel_fill: int = 45
    travel_veh: int = 45
    travel_prg: int = 45
    wdg_ds: int = DEF_WDG_DS
    ign_max_ds: int = DEF_IGN_MAX_DS
    allow_purge: int = 1
    cfg_seq: int = 0
    reserved: int = 0

    def pack(self) -> bytes:
        return struct.pack(CFG_FMT, self.travel_fill, self.travel_veh,
                           self.travel_prg, self.wdg_ds, self.ign_max_ds,
                           1 if self.allow_purge else 0, self.cfg_seq, 0)

    @classmethod
    def unpack(cls, b: bytes) -> "Cfg":
        return cls(*struct.unpack(CFG_FMT, b))


@dataclass
class Event:
    code: int
    arg: int = 0
    pad: int = 0

    @classmethod
    def unpack(cls, b: bytes) -> "Event":
        return cls(*struct.unpack(EVENT_FMT, b))

    @property
    def name(self) -> str:
        return EVENT_NAMES.get(self.code, f"EV_{self.code}")


# --------------------------------------------------------------- framing
def encode(ftype: int, seq: int, payload: bytes = b"") -> bytes:
    if len(payload) > MAX_PAYLOAD:
        raise ValueError("payload too long")
    body = bytes([PROTO_VERSION, ftype & 0xFF, seq & 0xFF, len(payload)]) + payload
    crc = crc16(body)
    return bytes([SOF]) + body + bytes([crc & 0xFF, crc >> 8])


def encode_cmd(seq: int, c: Cmd) -> bytes:
    return encode(T_CMD, seq, c.pack())


def encode_cfg(seq: int, c: Cfg) -> bytes:
    return encode(T_CFG, seq, c.pack())


def encode_hello(seq: int = 0, node: int = NODE_PAD, major: int = 1, minor: int = 0) -> bytes:
    return encode(T_HELLO, seq, bytes([node, major, minor]))


class FrameError(Exception):
    pass


def decode(frame: bytes):
    """Decode one complete frame.  Raises FrameError on anything suspicious."""
    if len(frame) < 7:
        raise FrameError("short frame")
    if frame[0] != SOF:
        raise FrameError("no SOF")
    if frame[1] != PROTO_VERSION:
        raise FrameError(f"version {frame[1]} != {PROTO_VERSION}")
    ftype, seq, n = frame[2], frame[3], frame[4]
    if len(frame) != 7 + n:
        raise FrameError("length mismatch")
    rx = frame[5 + n] | (frame[6 + n] << 8)
    if crc16(frame[1:5 + n]) != rx:
        raise FrameError("bad CRC")
    payload = frame[5:5 + n]
    if ftype == T_CMD and n == 12:
        return ftype, seq, Cmd.unpack(payload)
    if ftype == T_STATE and n == 24:
        return ftype, seq, State.unpack(payload)
    if ftype == T_CFG and n == 8:
        return ftype, seq, Cfg.unpack(payload)
    if ftype == T_EVENT and n == 3:
        return ftype, seq, Event.unpack(payload)
    return ftype, seq, payload


class Parser:
    """Incremental byte-stream parser; identical state machine to the C one."""

    def __init__(self):
        self.buf = bytearray()
        self.frames_ok = 0
        self.frames_bad = 0

    def _drop(self):
        if self.buf:
            del self.buf[0]
        self.frames_bad += 1
        while self.buf and self.buf[0] != SOF:
            del self.buf[0]

    def push(self, data: bytes):
        out = []
        for b in data:
            if not self.buf:
                if b != SOF:
                    continue
                self.buf.append(b)
                continue
            if len(self.buf) >= MAX_FRAME:
                self.buf.clear()
                continue
            self.buf.append(b)
            while True:
                if not self.buf:
                    break
                if self.buf[0] != SOF:
                    self._drop()
                    continue
                if len(self.buf) >= 2 and self.buf[1] != PROTO_VERSION:
                    self._drop()
                    continue
                if len(self.buf) < 5:
                    break
                n = self.buf[4]
                if n > MAX_PAYLOAD:
                    self._drop()
                    continue
                if len(self.buf) < 7 + n:
                    break
                if self.buf[1] != PROTO_VERSION:
                    self._drop()
                    continue
                rx = self.buf[5 + n] | (self.buf[6 + n] << 8)
                if crc16(bytes(self.buf[1:5 + n])) != rx:
                    self._drop()
                    continue
                self.frames_ok += 1
                out.append(bytes(self.buf[:7 + n]))
                self.buf.clear()
                break
        res = []
        for fr in out:
            try:
                res.append(decode(fr))
            except FrameError:
                self.frames_bad += 1
        return res


# --------------------------------------------------------------- failsafe
@dataclass
class Outputs:
    fill: int = 0
    veh: int = 1
    prg: int = 0
    ign: int = 0
    armed: bool = False
    comms_loss: bool = True
    ign_src: int = 0


def clamp(v: int, lo: int, hi: int) -> int:
    return lo if v < lo else hi if v > hi else v


def failsafe(last_cmd: Cmd | None, age_ms: int, wdg_ds: int,
             ign_max_ds: int) -> Outputs:
    """Mirror of gse_failsafe() in the firmware.  Used by the GUI to preview
    and by the tests to prove host and firmware agree."""
    o = Outputs()
    armed = bool(last_cmd and last_cmd.arm_code == ARM_CODE)
    fresh = bool(last_cmd is not None and age_ms != 0xFFFF
                 and age_ms <= wdg_ds * 100)
    o.comms_loss = not fresh
    o.armed = armed and fresh
    if not fresh:
        o.fill, o.prg, o.veh, o.ign = 0, 0, 1, 0
        o.ign_src = 0
        return o
    o.fill = 1 if last_cmd.fill else 0
    o.veh = 1 if last_cmd.veh else 0
    o.prg = 1 if last_cmd.prg else 0
    cmd_ign = bool(last_cmd.ign) and armed and fresh
    o.ign = 1 if cmd_ign else 0
    o.ign_src = IGN_SRC_CMD if cmd_ign else 0
    return o
