"""--sim: a stand-in for the ground ESP32's USB port, the vehicle ESP32 behind it
and the DAQ board on the vehicle's UART2.  No hardware needed.

It answers the same text lines as ground/ground.ino:
    banner, "SENT <cmd> <token>", "ACK <token> <cmd> <0|1>", "PONG", "ERR ...",
    "TLM <seq> <uptime_ms> <payload>"
applies the vehicle.ino interlocks to decide each ACK, and forwards the DAQ's
NEWEST UART line once every TELEMETRY_INTERVAL_MS (500 ms by default), exactly
as vehicle.ino does.  IGN (test '4' or live IGN_ON) starts a simulated burn.

SimSerial wraps it in the few QSerialPort methods the window uses.
"""
from __future__ import annotations

import random
import time
from collections import deque

from daq import SimDaq

SEND_DELAY_S = 0.24            # ground: 3 copies, 120 ms apart, then "SENT"
ACK_DELAY_S = 0.03
SERVO_MIN_US, SERVO_MAX_US = 500, 2500
SERVO_HOLD_MS, IGN_ON_MS = 500, 1000          # ground defaults for test pulses
MAX_IGN_ON_MS, MAX_IGN_ARM_MS = 2000, 60000

BANNER = [
    "ESP-NOW servo/IGN ground",
    "Ground MAC (copy into vehicle sketch): A4:F0:0F:62:0C:B8",
    "Ready. Test: 1/2/3/4. Live: SV0_ON, SV1_ON, IGN_ARM, IGN_ON, ALL_OFF.",
    "TEST WITH IGNITION DISCONNECTED FIRST.",
]
LIVE_CMDS = {"SV0_OFF": 10, "SV0_ON": 11, "SV1_OFF": 12, "SV1_ON": 13, "BOTH_OFF": 14,
             "BOTH_ON": 15, "IGN_OFF": 16, "IGN_ARM": 17, "IGN_ON": 18, "ALL_OFF": 19}
TEST_CMDS = {"1": 1, "2": 2, "3": 3, "4": 4}


class SimVehicle:
    """vehicle.ino's actuator logic and interlocks (no PWM, just state)."""

    def __init__(self, clock=time.monotonic):
        self.clock = clock
        self.fill_open = self.srv_open = self.ign_on = self.ign_armed = False
        self.arm_start = 0.0
        self.arm_required_s = 0.0
        self.busy_until = 0.0
        self.on_ignite = None

    def clear_latched(self):
        self.fill_open = self.srv_open = self.ign_on = self.ign_armed = False

    def apply(self, cmd: int, hold_ms: int, s0: int, s1: int) -> bool:
        now = self.clock()
        if now < self.busy_until:
            return False                                   # "BUSY: command ignored"
        valid = SERVO_MIN_US <= s0 <= SERVO_MAX_US and SERVO_MIN_US <= s1 <= SERVO_MAX_US
        if cmd in (1, 2, 3):
            if not valid:
                return False
            self.clear_latched()
            self.busy_until = now + min(hold_ms, 5000) / 1000
            return True
        if cmd == 4:
            if self.fill_open or self.srv_open:
                return False
            self.clear_latched()
            self.busy_until = now + min(hold_ms or 1, MAX_IGN_ON_MS) / 1000
            self._ignite()
            return True
        if cmd == 11:
            if self.srv_open or self.ign_on or self.ign_armed or not valid:
                return False
            self.fill_open = True
            return True
        if cmd == 10:
            self.fill_open = False
            return True
        if cmd == 13:
            if self.fill_open or self.ign_armed or not valid:
                return False
            self.srv_open = True
            return True
        if cmd == 12:
            self.srv_open = False
            return True
        if cmd == 15:
            return False                                   # BOTH ON refused
        if cmd == 14:
            self.fill_open = self.srv_open = False
            return True
        if cmd == 17:
            if self.fill_open or self.srv_open or self.ign_on:
                return False
            self.ign_armed, self.ign_on = True, False
            self.arm_start = now
            self.arm_required_s = min(hold_ms, MAX_IGN_ARM_MS) / 1000
            return True
        if cmd == 18:
            if self.fill_open or self.srv_open or not self.ign_armed:
                return False
            if now - self.arm_start < self.arm_required_s or not valid:
                return False
            self.srv_open, self.ign_armed, self.ign_on = True, False, True
            self._ignite()
            return True
        if cmd == 16:
            self.ign_armed = self.ign_on = False
            return True
        if cmd == 19:
            self.clear_latched()
            return True
        return False

    def _ignite(self):
        if self.on_ignite:
            self.on_ignite()


class SimGround:
    def __init__(self, ambient_f: float = 70.0, interval_ms: int = 500, fmt: str = "combined",
                 clock=time.monotonic, seed: int | None = None):
        self.clock = clock
        self.ambient_f = ambient_f
        self.interval_s = interval_ms / 1000
        self.fmt = fmt
        self.rng = random.Random(seed)
        self._out: deque = deque()                 # (due, line)
        self.written: list[str] = []
        self.s0_us = self.s1_us = 1800             # ground's last targets after boot
        self.vehicle = SimVehicle(clock)
        self.vehicle.on_ignite = self._ignite
        self.daq: SimDaq | None = None
        self.opened_at = 0.0

    # ------------------------------------------------------------ port
    def open(self) -> None:
        now = self.clock()
        self.opened_at = now
        self._out.clear()
        for i, line in enumerate(BANNER):
            self._out.append((now + 0.05 * (i + 1), line))
        self.daq = SimDaq(self.ambient_f, self.fmt, self.clock,
                          seed=self.rng.randrange(1 << 30))
        self._latest = None
        self._pending = False
        self._tlm_seq = 0
        self._next_tlm = now + self.interval_s

    def close(self) -> None:
        self._out.clear()
        self.daq = None

    def set_format(self, fmt: str) -> None:
        """The simulated DAQ switches output format (on hardware the DAQ
        firmware decides; the app's toggle must match it)."""
        self.fmt = fmt
        if self.daq:
            self.daq.fmt = fmt

    def _ignite(self):
        if self.daq:
            self.daq.ignite()

    # ------------------------------------------------------- commands
    def write_line(self, text: str) -> None:
        """One USB line from the app, handled like ground.ino handleCommand()."""
        self.written.append(text)
        now = self.clock()
        line = text.strip().upper()
        if not line:
            return
        if not line.startswith("IGN_ARM") and ":" in line:
            head, _, tail = line.partition(":")
            a, comma, b = tail.partition(",")
            if not comma or not a.isdigit() or not b.isdigit():
                self._out.append((now, "ERR invalid targets"))
                return
            fill, srv = int(a), int(b)
            if not (SERVO_MIN_US <= fill <= SERVO_MAX_US and SERVO_MIN_US <= srv <= SERVO_MAX_US):
                self._out.append((now, "ERR targets out of range"))
                return
            self.s0_us, self.s1_us = fill, srv
            line = head
        hold = 0
        if line in TEST_CMDS:
            cmd = TEST_CMDS[line]
            hold = IGN_ON_MS if cmd == 4 else SERVO_HOLD_MS
        elif line == "PING":
            self._out.append((now, "PONG"))
            return
        elif line.startswith("IGN_ARM"):
            cmd, hold = 17, 5000
            _, colon, ms = line.partition(":")
            if colon and ms.isdigit() and 0 < int(ms) <= 60000:
                hold = int(ms)
        elif line in LIVE_CMDS:
            cmd = LIVE_CMDS[line]
        else:
            self._out.append((now, "ERR unknown command"))
            return
        token = self.rng.randrange(1, 1 << 32)
        sent_at = now + SEND_DELAY_S
        self._out.append((sent_at, f"SENT {cmd} {token}"))
        accepted = self.vehicle.apply(cmd, hold, self.s0_us, self.s1_us)
        self._out.append((sent_at + ACK_DELAY_S, f"ACK {token} {cmd} {1 if accepted else 0}"))

    # ------------------------------------------------------ telemetry
    def read_lines(self) -> list[str]:
        now = self.clock()
        if self.daq is not None:
            for line in self.daq.lines():           # vehicle loop(): keep the newest
                self._latest = line
                self._pending = True
            while self._next_tlm <= now:            # every TELEMETRY_INTERVAL_MS
                if self._pending:
                    self._tlm_seq += 1
                    uptime = int((self._next_tlm - self.opened_at) * 1000) + 3000
                    self._out.append((self._next_tlm, f"TLM {self._tlm_seq} {uptime} {self._latest}"))
                    self._pending = False
                self._next_tlm += self.interval_s
        pending = sorted(self._out, key=lambda e: e[0])
        self._out = deque(e for e in pending if e[0] > now)
        return [line for due, line in pending if due <= now]


class SimSerial:
    """The QSerialPort methods MainWindow uses, backed by SimGround."""

    def __init__(self, ground: SimGround):
        self.ground = ground
        self._open = False
        self._rx = bytearray()

    def setPortName(self, name):
        pass

    def setBaudRate(self, baud):
        pass

    def open(self, mode=None) -> bool:
        self.ground.open()
        self._open = True
        return True

    def close(self) -> None:
        self.ground.close()
        self._open = False

    def isOpen(self) -> bool:
        return self._open

    def errorString(self) -> str:
        return ""

    def write(self, data: bytes) -> int:
        if not self._open:
            return -1
        for line in bytes(data).decode("ascii", "replace").splitlines():
            self.ground.write_line(line)
        return len(data)

    def poll(self) -> bool:
        """Move due lines into the receive buffer; True if there is data."""
        if self._open:
            for line in self.ground.read_lines():
                self._rx += (line + "\r\n").encode()
        return bool(self._rx)

    def readAll(self) -> bytes:
        out = bytes(self._rx)
        self._rx.clear()
        return out
