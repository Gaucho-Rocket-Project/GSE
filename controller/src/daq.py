"""DAQ board (Hotfire-Board) data carried in vehicle telemetry: parsing, live
state, CSV logging and a simulator.  No Qt here; see daq_panel.py for the UI.

Path:  DAQ --UART2--> vehicle.ino --ESP-NOW, newest line every
TELEMETRY_INTERVAL_MS (500 ms = 2 Hz)--> ground.ino --USB--> "TLM <seq> <uptime_ms> <payload>"

The vehicle forwards only the NEWEST complete UART line at each interval, so
the DAQ's output format matters.  The app supports both, chosen with a toggle:

  "combined"  one line with everything (recommended):
      D,<seq>,<t_us>,<p0>,<p1>,<p2>,<p3>,<tc1>,<tc2>,<tc3>,<tc4>,<f1>,<f2>,<f3>,<f4>
  "separate"  pressure and thermocouple lines; whichever is newest is forwarded:
      P,<seq>,<t_us>,<p0>,<p1>,<p2>,<p3>
      T,<seq>,<t_us>,<tc1>,<tc2>,<tc3>,<tc4>,<f1>,<f2>,<f3>,<f4>
  either:  HB,<n>  or  heartbeat     (the DAQ's 1 Hz heartbeat)

p* = 4-20 mA loop current in uA (integer); tc* = deg C; f* = MAX31856 fault
register as hex (00 = none).  An empty field = no reading.  <seq> counts DAQ
frames, so a gap in it is a frame the vehicle did not forward (normal: newest-
line-wins) or lost; gaps in the TLM sequence are radio losses.  psi = min +
(uA - 4000) * (max - min) / 16000, with each channel's range set in the app.
"""
from __future__ import annotations

import csv
import math
import random
import time
from collections import deque
from dataclasses import asdict, dataclass, field
from datetime import datetime, timezone
from pathlib import Path

N_P = 4
N_T = 4
P_RATE_HZ = 100               # DAQ output rates (before the vehicle's 2 Hz pick)
T_RATE_HZ = 10

LOOP_MIN_UA = 4000
LOOP_MAX_UA = 20000
UNDER_UA = 3800               # below: open loop / no sensor
OVER_UA = 20500               # above: over range / sensor fault
NEAR_MAX = 0.90

STALE_S = 2.0                 # 4 missed telemetry intervals at 2 Hz
HISTORY_S = 30.0              # chart window
RATE_WINDOW_S = 5.0

FORMATS = ("combined", "separate")
FORMAT_LABELS = {
    "combined": "One line (D,...)",
    "separate": "Separate P / T lines",
}

TC_FAULT_BITS = [(0x80, "CJ range"), (0x40, "TC range"), (0x20, "CJ high"),
                 (0x10, "CJ low"), (0x08, "TC high"), (0x04, "TC low"),
                 (0x02, "over/under voltage"), (0x01, "open circuit")]


# ------------------------------------------------------------------ frames
@dataclass
class PressureFrame:
    seq: int
    t_us: int
    ua: list


@dataclass
class ThermoFrame:
    seq: int
    t_us: int
    temp_c: list
    fault: list


@dataclass
class CombinedFrame:
    seq: int
    t_us: int
    ua: list
    temp_c: list
    fault: list


@dataclass
class Heartbeat:
    n: int | None


MALFORMED = "malformed"


def _opt_int(s):
    s = s.strip()
    return int(s) if s else None


def _opt_float(s):
    s = s.strip()
    if not s:
        return None
    v = float(s)
    return v if math.isfinite(v) else None


def _opt_hex(s):
    s = s.strip()
    return int(s, 16) if s else None


def parse_payload(payload: str):
    """A TLM payload -> frame / Heartbeat, MALFORMED for a broken DAQ line, or
    None if the payload is not DAQ data at all."""
    line = payload.strip()
    if line.lower() == "heartbeat":
        return Heartbeat(None)
    parts = line.split(",")
    kind = parts[0]
    if kind not in ("D", "P", "T", "HB"):
        return None
    try:
        if kind == "D" and len(parts) == 3 + N_P + 2 * N_T:
            i = 3 + N_P
            return CombinedFrame(int(parts[1]), int(parts[2]),
                                 [_opt_int(x) for x in parts[3:i]],
                                 [_opt_float(x) for x in parts[i:i + N_T]],
                                 [_opt_hex(x) for x in parts[i + N_T:i + 2 * N_T]])
        if kind == "P" and len(parts) == 3 + N_P:
            return PressureFrame(int(parts[1]), int(parts[2]), [_opt_int(x) for x in parts[3:]])
        if kind == "T" and len(parts) == 3 + 2 * N_T:
            return ThermoFrame(int(parts[1]), int(parts[2]),
                               [_opt_float(x) for x in parts[3:3 + N_T]],
                               [_opt_hex(x) for x in parts[3 + N_T:]])
        if kind == "HB" and len(parts) == 2:
            return Heartbeat(int(parts[1]))
    except ValueError:
        pass
    return MALFORMED


def ua_to_psi(ua, lo, hi):
    return lo + (ua - LOOP_MIN_UA) * (hi - lo) / (LOOP_MAX_UA - LOOP_MIN_UA)


def psi_to_ua(psi, lo, hi):
    return LOOP_MIN_UA + (psi - lo) * (LOOP_MAX_UA - LOOP_MIN_UA) / (hi - lo)


def fault_text(code: int) -> str:
    names = [n for bit, n in TC_FAULT_BITS if code & bit]
    return ", ".join(names) if names else "none"


def c_to_f(c):
    return c * 9 / 5 + 32


# ---------------------------------------------------------------- settings
@dataclass
class PressureChannel:
    name: str
    min_psi: float
    max_psi: float


@dataclass
class DaqSettings:
    """Channel names / ranges (editable in the app).  Defaults follow HCR-5100
    Appendix A: tank line ~1100 psi on hot days -> 1600 psi transducer; chamber
    ~500 psi -> 750-1000 psi.  Set them to the transducers actually fitted."""
    pressure: list = field(default_factory=lambda: [
        PressureChannel("Tank (N2O)", 0.0, 1600.0),
        PressureChannel("Chamber", 0.0, 1000.0),
        PressureChannel("Ox injector inlet", 0.0, 1000.0),
        PressureChannel("Fuel injector inlet", 0.0, 1000.0),
    ])
    tc_names: list = field(default_factory=lambda: [
        "Chamber exterior", "Ox injector inlet", "Fuel injector inlet", "Spare"])
    temp_unit: str = "C"

    def validate(self) -> list[str]:
        errs = []
        if len(self.pressure) != N_P or len(self.tc_names) != N_T:
            errs.append("wrong number of channels")
        for i, ch in enumerate(self.pressure):
            if not ch.name.strip():
                errs.append(f"P{i}: name is empty")
            if not (math.isfinite(ch.min_psi) and math.isfinite(ch.max_psi)):
                errs.append(f"P{i}: range must be numbers")
            elif ch.max_psi <= ch.min_psi:
                errs.append(f"P{i}: max must be greater than min")
        for i, n in enumerate(self.tc_names):
            if not n.strip():
                errs.append(f"TC{i + 1}: name is empty")
        if self.temp_unit not in ("C", "F"):
            errs.append("temperature unit must be C or F")
        return errs

    def to_dict(self) -> dict:
        return {"pressure": [asdict(p) for p in self.pressure],
                "tc_names": list(self.tc_names), "temp_unit": self.temp_unit}

    @classmethod
    def from_dict(cls, d) -> "DaqSettings":
        """Settings from a dict; the defaults if it is missing or invalid."""
        try:
            s = cls(pressure=[PressureChannel(str(p["name"]), float(p["min_psi"]),
                                              float(p["max_psi"])) for p in d["pressure"]],
                    tc_names=[str(n) for n in d["tc_names"]],
                    temp_unit=str(d.get("temp_unit", "C")))
        except (TypeError, ValueError, KeyError):
            return cls()
        return cls() if s.validate() else s


# ------------------------------------------------------------- live state
class _Stream:
    def __init__(self):
        self.frames = 0
        self.gaps = 0
        self.restarts = 0
        self.last_seq: int | None = None
        self.last_at = 0.0
        self._arrivals: deque = deque()

    def add(self, seq: int, now: float) -> str:
        """'new', 'restart' or 'duplicate'.  Telemetry arrives in order (one
        radio hop, newest line only), so a repeated number is a duplicate and
        a smaller one means the sender rebooted and started counting again."""
        status = "new"
        if self.last_seq is not None:
            if seq == self.last_seq:
                return "duplicate"
            if seq > self.last_seq + 1:
                self.gaps += seq - self.last_seq - 1
            elif seq < self.last_seq:
                self.restarts += 1
                status = "restart"
        self.last_seq = seq
        self.frames += 1
        self.last_at = now
        self._arrivals.append(now)
        return status

    def rate(self, now: float) -> float:
        while self._arrivals and self._arrivals[0] < now - RATE_WINDOW_S:
            self._arrivals.popleft()
        return len(self._arrivals) / RATE_WINDOW_S

    def age(self, now: float) -> float:
        return now - self.last_at if self.last_at else float("inf")


class DaqState:
    def __init__(self, settings: DaqSettings | None = None, fmt: str = "combined",
                 data_dir: Path | None = None, clock=time.monotonic):
        self.settings = settings or DaqSettings()
        self.fmt = fmt if fmt in FORMATS else "combined"
        self.data_dir = data_dir
        self.clock = clock
        self._logger: DaqLogger | None = None
        self.reset()

    def reset(self) -> None:
        self.close()
        self.p = _Stream()            # DAQ frame seq (pressure / combined)
        self.t = _Stream()            # DAQ frame seq (thermocouples)
        self.hb = _Stream()
        self.tlm = _Stream()          # vehicle TLM seq: gaps = radio losses
        self.malformed = 0
        self.other_format = 0         # DAQ lines of the format NOT selected
        self.non_daq = 0              # TLM payloads that are not DAQ lines at all
        self.non_daq_example = ""
        self.csv_error = ""           # set when the DAQ CSV could not be written
        self.latest_p: PressureFrame | None = None
        self.latest_t: ThermoFrame | None = None
        self.p_hist: deque = deque(maxlen=4000)   # (mono, [uA])
        self.t_hist: deque = deque(maxlen=4000)   # (mono, [C])
        self.p_peak: list = [None] * N_P
        self.t_peak: list = [None] * N_T
        self._tlm_last: tuple[int, int] | None = None

    def set_format(self, fmt: str) -> None:
        if fmt in FORMATS and fmt != self.fmt:
            self.fmt = fmt
            self.other_format = 0

    def set_settings(self, settings: DaqSettings) -> None:
        self.settings = settings
        if self._logger:
            self._logger.settings = settings

    def close(self) -> None:
        if self._logger:
            self._logger.close()
            self._logger = None

    @property
    def csv_path(self) -> Path | None:
        return self._logger.path if self._logger else None

    def feed_tlm(self, seq: int, uptime_ms: int, payload: str):
        """One TLM record from the ground.  Returns the parsed item (or None)."""
        now = self.clock()
        if self.tlm.add(seq, now) == "duplicate":
            return None                              # same packet twice
        self._tlm_last = (seq, uptime_ms)
        item = parse_payload(payload)
        if item is None:
            self.non_daq += 1
            self.non_daq_example = payload[:40]
        else:
            self.feed(item, now)
        return item

    def feed(self, item, now: float | None = None) -> None:
        now = self.clock() if now is None else now
        if item is MALFORMED:
            self.malformed += 1
            return
        if isinstance(item, Heartbeat):
            if item.n is None:
                self.hb.frames += 1
                self.hb.last_at = now
            else:
                self.hb.add(item.n, now)
            return
        wanted = (CombinedFrame,) if self.fmt == "combined" else (PressureFrame, ThermoFrame)
        if not isinstance(item, wanted):
            self.other_format += 1
            return
        if isinstance(item, (CombinedFrame, PressureFrame)):
            self.p.add(item.seq, now)
            self.latest_p = PressureFrame(item.seq, item.t_us, list(item.ua))
            self.p_hist.append((now, list(item.ua)))
            self.p_peak = [v if pk is None else pk if v is None else max(pk, v)
                           for pk, v in zip(self.p_peak, item.ua)]
        if isinstance(item, (CombinedFrame, ThermoFrame)):
            self.t.add(item.seq, now)
            self.latest_t = ThermoFrame(item.seq, item.t_us, list(item.temp_c), list(item.fault))
            self.t_hist.append((now, list(item.temp_c)))
            self.t_peak = [v if pk is None else pk if v is None else max(pk, v)
                           for pk, v in zip(self.t_peak, item.temp_c)]
        if self.data_dir is not None and not self.csv_error:
            try:
                if self._logger is None:
                    self._logger = DaqLogger(self.data_dir, self.settings)
                tlm = self._tlm_last or (None, None)
                self._logger.write(item, now, *tlm)
            except OSError as exc:
                # Keep displaying; stop trying to log until the next connection
                # and report it once (the caller shows it in the link log).
                self.csv_error = str(exc) or exc.__class__.__name__
                self.close()
                raise

    # ------------------------------------------------------------ display
    def psi(self, i: int, ua):
        if ua is None:
            return None
        ch = self.settings.pressure[i]
        return ua_to_psi(ua, ch.min_psi, ch.max_psi)

    def pressure_status(self, i: int) -> tuple[str, str]:
        now = self.clock()
        if self.latest_p is None:
            return "no data", "none"
        if self.p.age(now) > STALE_S:
            return f"STALE ({self.p.age(now):.1f} s)", "bad"
        ua = self.latest_p.ua[i]
        if ua is None:
            return "READ ERROR", "bad"
        if ua < UNDER_UA:
            return "BELOW 4 mA (no sensor / open loop?)", "bad"
        if ua > OVER_UA:
            return "ABOVE 20 mA (over range / fault)", "bad"
        ch = self.settings.pressure[i]
        frac = (ua_to_psi(ua, ch.min_psi, ch.max_psi) - ch.min_psi) / (ch.max_psi - ch.min_psi)
        if frac >= NEAR_MAX:
            return f"NEAR MAX ({100 * frac:.0f}% of range)", "warn"
        return "ok", "ok"

    def tc_status(self, i: int) -> tuple[str, str]:
        now = self.clock()
        if self.latest_t is None:
            return "no data", "none"
        if self.t.age(now) > STALE_S:
            return f"STALE ({self.t.age(now):.1f} s)", "bad"
        f = self.latest_t.fault[i]
        if f:
            return f"FAULT 0x{f:02X}: {fault_text(f)}", "bad"
        if self.latest_t.temp_c[i] is None:
            return "READ ERROR", "bad"
        return "ok", "ok"

    def link_text(self) -> tuple[str, str]:
        """(text, level) summarising the DAQ link; level ok / warn / bad / none."""
        now = self.clock()
        if not (self.tlm.frames or self.p.frames or self.t.frames or self.hb.frames):
            return "DAQ: no telemetry yet", "none"
        if not (self.p.frames or self.t.frames):
            # Telemetry is arriving, but no DAQ data frames (yet).
            text = f"DAQ: TLM {self.tlm.rate(now):.1f}/s, but no DAQ data lines yet"
            if self.hb.frames:
                text += (f"\nDAQ alive (heartbeat {self.hb.age(now):.1f} s ago) - the DAQ "
                         "firmware must send data lines (see controller README) to fill this panel")
            if self.other_format:
                other = "separate P/T" if self.fmt == "combined" else "one-line (D,...)"
                text += (f"\nReceiving {other} lines ({self.other_format}) - "
                         "switch the DAQ line format toggle")
            if self.non_daq:
                text += f"\n{self.non_daq} telemetry lines are not DAQ lines, e.g. '{self.non_daq_example}'"
            if self.malformed:
                text += f"\n{self.malformed} broken DAQ lines"
            return text, "warn"
        parts = [f"TLM {self.tlm.rate(now):.1f}/s"]
        if self.fmt == "combined":
            parts.append(f"frames {self.p.rate(now):.1f}/s")
        else:
            parts.append(f"P {self.p.rate(now):.1f}/s · TC {self.t.rate(now):.1f}/s")
        last = min(self.p.age(now), self.t.age(now))
        parts.append("last frame " + (f"{last:.1f} s ago" if math.isfinite(last) else "never"))
        parts.append(f"radio lost {self.tlm.gaps}")
        skipped = self.p.gaps if self.fmt == "combined" else self.p.gaps + self.t.gaps
        parts.append(f"DAQ lines skipped by the vehicle's newest-line pick {skipped}")
        if self.hb.frames:
            parts.append(f"heartbeat {self.hb.age(now):.1f} s ago")
        if self.malformed:
            parts.append(f"bad lines {self.malformed}")
        if self.non_daq:
            parts.append(f"other telemetry lines {self.non_daq}")
        if self.p.restarts or self.tlm.restarts:
            parts.append(f"restarts DAQ {self.p.restarts} / vehicle {self.tlm.restarts}")
        text = "DAQ: " + " · ".join(parts)
        level = "bad" if last > STALE_S else "ok"
        if self.csv_error:
            text += f"\nDAQ CSV NOT BEING WRITTEN: {self.csv_error} (reconnect to retry)"
            level = "warn" if level == "ok" else level
        if self.other_format:
            other = "separate P/T" if self.fmt == "combined" else "one-line (D,...)"
            text += (f"\nReceiving {other} lines ({self.other_format}) - the format toggle "
                     "does not match what the DAQ sends")
            level = "warn" if level == "ok" else level
        elif self.fmt == "separate" and self.p.frames and self.t.age(now) > STALE_S:
            text += ("\nThermocouple lines are rarely the newest line at 2 Hz - "
                     "one line with everything avoids this")
            level = "warn" if level == "ok" else level
        return text, level


# ------------------------------------------------------------- CSV logging
class DaqLogger:
    """daq_<date>_<time>.csv next to telemetry.csv: one row per DAQ frame,
    raw uA plus psi with the range in force at that moment."""

    def __init__(self, data_dir: Path, settings: DaqSettings):
        data_dir = Path(data_dir)
        data_dir.mkdir(parents=True, exist_ok=True)
        self.path = data_dir / time.strftime("daq_%Y%m%d_%H%M%S.csv")
        self.settings = settings
        self._fh = self.path.open("w", newline="", encoding="utf-8")
        self._w = csv.writer(self._fh)
        self._w.writerow(["received_utc", "tlm_seq", "vehicle_uptime_ms", "kind", "daq_seq",
                          "daq_t_us"]
                         + [f"p{i}_ua" for i in range(N_P)] + [f"p{i}_psi" for i in range(N_P)]
                         + [f"p{i}_range" for i in range(N_P)]
                         + [f"tc{i + 1}_c" for i in range(N_T)]
                         + [f"tc{i + 1}_fault" for i in range(N_T)])

    def write(self, item, now, tlm_seq=None, uptime_ms=None) -> None:
        utc = datetime.now(timezone.utc).isoformat(timespec="milliseconds")
        kind = {CombinedFrame: "D", PressureFrame: "P", ThermoFrame: "T"}[type(item)]
        ua = psi = rng = [""] * N_P
        temps = faults = [""] * N_T
        if isinstance(item, (CombinedFrame, PressureFrame)):
            ua = ["" if v is None else v for v in item.ua]
            psi, rng = [], []
            for i, v in enumerate(item.ua):
                ch = self.settings.pressure[i]
                psi.append("" if v is None else f"{ua_to_psi(v, ch.min_psi, ch.max_psi):.2f}")
                rng.append(f"{ch.min_psi:g}-{ch.max_psi:g}")
        if isinstance(item, (CombinedFrame, ThermoFrame)):
            temps = ["" if v is None else f"{v:.2f}" for v in item.temp_c]
            faults = ["" if v is None else f"{v:02X}" for v in item.fault]
        self._w.writerow([utc, "" if tlm_seq is None else tlm_seq,
                          "" if uptime_ms is None else uptime_ms, kind, item.seq, item.t_us]
                         + ua + psi + rng + temps + faults)
        self._fh.flush()

    def close(self) -> None:
        try:
            self._fh.close()
        except OSError:
            pass


# --------------------------------------------------------------- simulator
# HCR-5100 Table 2.6: ambient F -> (tank psi, chamber psi, burn time s)
TABLE_2_6 = [(40, 425, 220, 5.9), (50, 500, 240, 5.2), (60, 575, 260, 4.7),
             (70, 650, 280, 4.2), (80, 725, 300, 3.9), (90, 825, 310, 3.5),
             (100, 925, 330, 3.3), (110, 1050, 340, 3.2)]


def table_2_6(ambient_f):
    t = TABLE_2_6
    a = max(t[0][0], min(t[-1][0], ambient_f))
    for (a0, p0, c0, b0), (a1, p1, c1, b1) in zip(t, t[1:]):
        if a0 <= a <= a1:
            k = (a - a0) / (a1 - a0)
            return p0 + k * (p1 - p0), c0 + k * (c1 - c0), b0 + k * (b1 - b0)
    return t[-1][1], t[-1][2], t[-1][3]


class SimDaq:
    """The DAQ's UART output for a filled Mojave Sphinx on the stand: tank at the
    Table 2.6 pressure until ignite(), then a burn and venting.  TC4 unplugged
    (open-circuit fault).  fmt selects the output format."""

    def __init__(self, ambient_f=70.0, fmt="combined", clock=time.monotonic, seed=1):
        self.clock = clock
        self.fmt = fmt
        self.ambient_f = ambient_f
        self.p_tank, self.p_chamber, self.burn_s = table_2_6(ambient_f)
        self.rng = random.Random(seed)
        self.ranges = DaqSettings().pressure
        self.ignited_at = None
        t0 = clock()
        self._t0 = t0
        self._next_p = self._next_t = self._next_hb = t0
        self.seq = self.seq_t = self.seq_hb = 0
        self._temps_now = self._temps(t0)

    def ignite(self) -> None:
        if self.ignited_at is None:
            self.ignited_at = self.clock()

    def _pressures(self, t):
        tank, cham = self.p_tank, 0.0
        if self.ignited_at is not None:
            s, b = t - self.ignited_at, self.burn_s
            if 0 <= s < b:
                tank = self.p_tank * (1.0 - 0.55 * s / b)
                cham = self.p_chamber * min(1.0, s / 0.3) * (1.0 - 0.25 * s / b)
            elif s >= b:
                tank = max(0.0, self.p_tank * 0.45 * (1.0 - (s - b) / 6.0))
                cham = max(0.0, self.p_chamber * 0.75 * (1.0 - (s - b) / 0.3))
        burning = cham > 1
        return [tank, cham, tank * 0.92 if burning else tank, tank * 0.95 if burning else tank]

    def _temps(self, t):
        amb = (self.ambient_f - 32) * 5 / 9
        wall = ox = fuel = amb
        if self.ignited_at is not None:
            s, b = t - self.ignited_at, self.burn_s
            heat = min(s, b) / b if s >= 0 else 0.0
            cool = math.exp(-max(0.0, s - b) / 40.0)
            wall, ox, fuel = amb + 160 * heat * cool, amb - 8 * heat * cool, amb - 2 * heat * cool
        return [wall, ox, fuel, None]

    def _ua_fields(self, t):
        out = []
        for psi, ch in zip(self._pressures(t), self.ranges):
            out.append(str(int(round(psi_to_ua(psi, ch.min_psi, ch.max_psi) + self.rng.gauss(0, 6)))))
        return out

    def _tc_fields(self, temps):
        tc = ["" if v is None else f"{v + self.rng.gauss(0, 0.05):.2f}" for v in temps]
        return tc + ["01" if v is None else "00" for v in temps]

    def lines(self) -> list[str]:
        """Every UART line the DAQ has sent since the last call, in order."""
        now = self.clock()
        events = []                                     # (time, order, line)
        while self._next_t <= now:
            t = self._next_t
            self._next_t += 1.0 / T_RATE_HZ
            self._temps_now = self._temps(t)
            if self.fmt == "separate":
                self.seq_t += 1
                events.append((t, 1, f"T,{self.seq_t},{int((t - self._t0) * 1e6)},"
                                     + ",".join(self._tc_fields(self._temps_now))))
        while self._next_p <= now:
            t = self._next_p
            self._next_p += 1.0 / P_RATE_HZ
            self.seq += 1
            t_us = int((t - self._t0) * 1e6)
            if self.fmt == "combined":
                events.append((t, 0, f"D,{self.seq},{t_us}," + ",".join(
                    self._ua_fields(t) + self._tc_fields(self._temps(t)))))
            else:
                events.append((t, 0, f"P,{self.seq},{t_us}," + ",".join(self._ua_fields(t))))
        while self._next_hb <= now:
            t = self._next_hb
            self._next_hb += 1.0
            if self.fmt == "separate":
                self.seq_hb += 1
                events.append((t, 2, f"HB,{self.seq_hb}"))
        events.sort(key=lambda e: (e[0], e[1]))
        return [e[2] for e in events[-2000:]]
