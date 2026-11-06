"""
link.py  --  transport between the laptop and the TX node

Two implementations:

    SerialLink(port)   real hardware, pyserial, 115200 8N1 over USB-C CDC
    SimLink()          no hardware: runs a virtual pad node (sim.PadModel) so
                       the GUI, the smoke tests and the launch-day rehearsal
                       can be exercised on any laptop.

Both expose the same small surface:

    link.open() / link.close()
    link.send_cmd(Cmd)           20 Hz, called by the app tick
    link.send_cfg(Cfg)
    link.send_hello()
    link.poll() -> list[events]  drains the port, keeps the newest State
    link.state  -> State | None
    link.age_ms -> wall-clock age of the last State received
    link.stats  -> dict for the status bar
    link.log    -> CsvLogger

The link does NOT decide anything: all safety logic lives in the pad node and
is mirrored by protocol.failsafe() for the GUI preview.
"""
from __future__ import annotations

import csv
import os
import threading
import time
from dataclasses import dataclass, field

from . import protocol as P

BAUD = 115200


# --------------------------------------------------------------------- logging
class CsvLogger:
    """Append-only CSV of everything the app commanded and everything the pad
    reported.  One row per tick keeps the file aligned and easy to plot."""

    def __init__(self, path: str | None = None):
        self.path = path
        self._fh = None
        self._w = None
        if path:
            new = not os.path.exists(path)
            self._fh = open(path, "a", newline="", encoding="utf-8")
            self._w = csv.writer(self._fh)
            if new:
                self._w.writerow([
                    "t_unix", "t_mono", "dir", "kind", "seq",
                    "fill", "veh", "prg", "ign", "arm",
                    "age_ms", "rx", "crc_err", "flags", "pad_state", "note"])

    def row(self, direction: str, kind: str, **kw):
        if not self._w:
            return
        self._w.writerow([
            f"{time.time():.3f}", f"{time.monotonic():.3f}", direction, kind,
            kw.get("seq", ""), kw.get("fill", ""), kw.get("veh", ""),
            kw.get("prg", ""), kw.get("ign", ""), kw.get("arm", ""),
            kw.get("age_ms", ""), kw.get("rx", ""), kw.get("crc_err", ""),
            kw.get("flags", ""), kw.get("pad_state", ""), kw.get("note", "")])
        self._fh.flush()                      # survive a laptop crash

    def close(self):
        if self._fh:
            self._fh.close()
            self._fh = None
            self._w = None


# ------------------------------------------------------------------ base link
@dataclass
class Stats:
    tx_cmd: int = 0
    rx_state: int = 0
    rx_event: int = 0
    frames_bad: int = 0
    last_rx_mono: float = 0.0
    opened_mono: float = field(default_factory=time.monotonic)

    def as_text(self) -> str:
        age = (time.monotonic() - self.last_rx_mono) * 1000 if self.last_rx_mono else -1
        return (f"tx {self.tx_cmd} | rx {self.rx_state} | ev {self.rx_event} | "
                f"bad {self.frames_bad} | age {age:6.0f} ms")


class Link:
    name = "abstract"

    def __init__(self, log_path: str | None = None):
        self.parser = P.Parser()
        self.state: P.State | None = None
        self.state_mono: float = 0.0
        self.stats = Stats()
        self.log = CsvLogger(log_path)
        self.events: list[tuple[float, P.Event]] = []
        self.seq = 0
        self.node_log: list[str] = []      # "LOG ..." lines from a real node
        self._lock = threading.Lock()

    # -- to be implemented ------------------------------------------------
    def open(self): raise NotImplementedError
    def close(self): raise NotImplementedError
    def _write(self, data: bytes): raise NotImplementedError
    def _read(self) -> bytes: return b""

    # -- shared -----------------------------------------------------------
    def age_ms(self) -> float:
        if not self.state_mono:
            return float("inf")
        return (time.monotonic() - self.state_mono) * 1000.0

    def send_cmd(self, cmd: P.Cmd) -> None:
        cmd.tx_ms = int(time.monotonic() * 1000) & 0xFFFF
        self.seq = (self.seq + 1) & 0xFF
        self._write(P.encode_cmd(self.seq, cmd))
        self.stats.tx_cmd += 1
        self.log.row("tx", "CMD", seq=self.seq, fill=cmd.fill, veh=cmd.veh,
                     prg=cmd.prg, ign=cmd.ign, arm=cmd.arm_code == P.ARM_CODE)

    def send_cfg(self, cfg: P.Cfg) -> None:
        self.seq = (self.seq + 1) & 0xFF
        self._write(P.encode_cfg(self.seq, cfg))
        self.log.row("tx", "CFG", seq=self.seq,
                     note=f"travel={cfg.travel_fill}/{cfg.travel_veh}/{cfg.travel_prg} "
                          f"wdg={cfg.wdg_ds} ign={cfg.ign_max_ds} purge={cfg.allow_purge}")

    def send_hello(self) -> None:
        self.seq = (self.seq + 1) & 0xFF
        self._write(P.encode_hello(self.seq))

    def poll(self) -> list[P.Event]:
        """Drain the transport; returns any new pad events."""
        with self._lock:
            data = self._read()
            if data:
                for ftype, seq, obj in self.parser.push(data):
                    if ftype == P.T_STATE:
                        self.state = obj
                        self.state_mono = time.monotonic()
                        self.stats.last_rx_mono = self.state_mono
                        self.stats.rx_state += 1
                    elif ftype == P.T_EVENT:
                        self.events.append((time.monotonic(), obj))
                        self.stats.rx_event += 1
                        self.log.row("rx", "EVENT", seq=seq, note=obj.name)
                    elif ftype == P.T_HELLO:
                        self.log.row("rx", "HELLO", seq=seq, note=str(obj))
                    elif ftype == P.T_CMD:
                        pass
                self.stats.frames_bad = self.parser.frames_bad
            new_events = [e for _, e in self.events]
            self.events.clear()
            return new_events

    def text_command(self, line: str) -> None:
        self._write((line.strip().upper() + "\n").encode())


# ----------------------------------------------------------------- serial link
class SerialLink(Link):
    name = "serial"

    def __init__(self, port: str, log_path: str | None = None, baud: int = BAUD):
        super().__init__(log_path)
        self.port = port
        self.baud = baud
        self._ser = None
        self._text = bytearray()
        self.node_log: list[str] = []

    def open(self):
        import serial  # pyserial
        self._ser = serial.Serial(self.port, self.baud, timeout=0)
        time.sleep(0.2)
        self._ser.reset_input_buffer()
        # nudge the node: the hello also tells us the node is listening
        self.text_command("PING")

    def close(self):
        if self._ser:
            self._ser.close()
            self._ser = None

    def _write(self, data: bytes):
        if self._ser:
            self._ser.write(data)

    def _read(self) -> bytes:
        if not self._ser:
            return b""
        return self._ser.read(max(1, self._ser.in_waiting))

    def poll(self) -> list[P.Event]:
        evs = super().poll()
        # collect "LOG ..." text lines from the node.  These carry the pad's
        # own boot banner, RADIO OK / RADIO FAULT verdict and 1 Hz counters --
        # the only way to tell a dead radio from a dead link.  Keep them.
        while self._ser and self._ser.in_waiting:
            b = self._ser.read(1)
            if b == b"\n":
                line = self._text.decode("ascii", "replace").strip()
                self._text.clear()
                if line.startswith("LOG "):
                    text = line[4:]
                    self.log.row("rx", "LOG", note=text)
                    self.last_text = text
                    self.node_log.append(text)
                    del self.node_log[:-40]        # keep the last 40 lines
            elif b != b"\r":
                self._text += b
                if len(self._text) > 400:
                    self._text.clear()
        return evs


def list_ports() -> list[str]:
    """Ports that look like an ESP32-S3 native-USB CDC device."""
    try:
        from serial.tools import list_ports as lp
    except Exception:
        return []
    out = []
    for p in lp.comports():
        desc = (p.description or "") + " " + (p.product or "")
        if any(k in desc for k in ("ESP32", "USB JTAG", "USB Serial", "CDC")):
            out.append(p.device)
    return out or [p.device for p in lp.comports()]


# -------------------------------------------------------------------- sim link
class SimLink(Link):
    """Talk to sim.PadModel instead of hardware.  Same framing on the wire."""

    name = "simulation"

    def __init__(self, log_path: str | None = None):
        super().__init__(log_path)
        from .sim import PadModel
        self.pad = PadModel()
        self.last_text = ""
        self._tx = bytearray()

    def open(self):
        self.pad.reset()

    def close(self):
        pass

    def _write(self, data: bytes):
        self._tx += data

    def _read(self) -> bytes:
        # 1. laptop -> pad
        if self._tx:
            self.pad.feed(bytes(self._tx))
            self._tx.clear()
        # 2. pad -> laptop
        return self.pad.tick()

    def text_command(self, line: str) -> None:
        self.last_text = line.strip().upper()


# --------------------------------------------------------------- pad simulator
def make_link(port: str | None, log_path: str | None = None) -> Link:
    if port in (None, "", "sim", "SIM"):
        return SimLink(log_path)
    return SerialLink(port, log_path)
