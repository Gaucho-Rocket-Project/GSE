#!/usr/bin/env python3
"""
wokwi_pad_test.py -- drive the REAL pad-node firmware running in Wokwi with the
REAL laptop software, and check that what the software sets is what the pad
does -- both what it reports back and what it puts on the servo pins.

    laptop software (gse_control.Controller)
        |  frames, exactly as the GUI builds them
        v
    this script, standing in for the DX-LR41 radio  <-- rfc2217://localhost:4000
        |  answers the AT set-up dialog in dxlr41.h, then a transparent pipe
        v
    Wokwi ESP32-S3 running .pio/build/pad-node/firmware.bin (unmodified)
        |  GPIO4/5/6 servo PWM, GPIO7 igniter relay, GPIO8 heartbeat
        v
    Wokwi logic analyzer -> wokwi.vcd -> pulse widths -> degrees

The TX node is not in this chain: it forwards host frames to the radio
verbatim (tests/host_hal_stub.c proves that), so its absence changes nothing
about what the pad receives.

Usage (from the GSE folder)
    python wokwi_sims/wokwi_pad_test.py run      # start this first, then the sim
    python wokwi_sims/wokwi_pad_test.py run --travels 5,30,45,60,90 --hold 1.0 -v
    python wokwi_sims/wokwi_pad_test.py analyze --vcd firmware/wokwi.vcd
    python wokwi_sims/wokwi_pad_test.py inspect  # timeline of the last recording

Wokwi set-up: firmware/wokwi.toml + firmware/diagram.json (serial monitor on
GPIO16/17, GPIO18 to GND, logic analyzer on GPIO4/5/6/7/8).  wokwi_bridge.py
in this folder uses the radio stand-in below to run software/run_gui.py live.

Needs: pip install pyserial   (for 'run'; 'analyze' needs nothing)
"""
from __future__ import annotations

import argparse
import json
import os
import re
import statistics
import sys
import time

HERE = os.path.dirname(os.path.abspath(__file__))
GSE_ROOT = os.path.dirname(HERE)                         # this file lives in GSE/wokwi_sims/
DEFAULT_SOFTWARE = os.path.join(GSE_ROOT, "software")
DEFAULT_FIRMWARE = os.path.join(GSE_ROOT, "firmware")    # wokwi.toml / wokwi.vcd live here
DEFAULT_STEPS = os.path.join(HERE, "wokwi_pad_steps.json")


def _software_dir() -> str:
    for i, a in enumerate(sys.argv):
        if a == "--software" and i + 1 < len(sys.argv):
            return sys.argv[i + 1]
    return os.environ.get("GSE_SOFTWARE", DEFAULT_SOFTWARE)


sys.path.insert(0, _software_dir())
from gse_control import protocol as P            # noqa: E402
from gse_control.controller import Controller    # noqa: E402
from gse_control.link import Link                # noqa: E402

# ---------------------------------------------------------------- servo model
# The project's own definition of a degree (firmware/src/config.h):
#   500 us per 45 deg (DS3225 180-degree version), neutral 1500 us,
#   pulses clamped to the servo's 500..2500 us range.
NEUTRAL_US = 1500
US_PER_45DEG = 500                       # SERVO_US_PER_45DEG
US_PER_DEG = US_PER_45DEG / 45
MIN_US, MAX_US = 500, 2500               # SERVO_MIN_US / SERVO_MAX_US
MIN_TRAVEL, MAX_TRAVEL = 5, 90
LEDC_BITS, PWM_HZ = 12, 50
ANGLE_TOL_DEG = 0.75          # LEDC step is 4.88 us = 0.44 deg; allow a bit more
FW_MATCH_TOL_US = 3.0
RUN_TOL_US = 2.0              # pulses within this of each other = same setting
MIN_RUN_PULSES = 10           # 0.2 s; each test position is held far longer

# what the pad's radio set-up must leave in the module (config.h)
EXPECT_RADIO = {"MODE": "1", "LEVEL": "3", "CHANNEL": "0c", "MAC": "00,02", "KEY": "51042"}


def fw_predicted_us(travel: int, open_: bool) -> float:
    """What pad_node.cpp servoUs() puts on the pin, LEDC quantisation included.
    Only used to tell 'firmware does what its code says' apart from 'the code
    does what the operator asked'."""
    deg = max(MIN_TRAVEL, min(MAX_TRAVEL, travel))
    off = (US_PER_45DEG * deg) // 45
    us = NEUTRAL_US + (off if open_ else -off)
    us = max(MIN_US, min(MAX_US, us))
    period = 1_000_000 // PWM_HZ
    duty = (us * (1 << LEDC_BITS)) // period
    return duty * period / (1 << LEDC_BITS)


def clamp_travel(t: int) -> int:
    return max(MIN_TRAVEL, min(MAX_TRAVEL, t))


# ------------------------------------------------------------- radio stand-in
class RadioEmu:
    """Plays the pad's DX-LR41 on its UART.  Answers exactly the dialog that
    DxLr41::configure() runs (+++ / AT+... / AT+HELP / +++), remembers what it
    was told so the read-back banner is honest, then is a transparent pipe."""

    GUARD_S = 0.015

    def __init__(self, verbose: bool = False, force_pipe: bool = True):
        self.verbose = verbose
        self.force_pipe = force_pipe
        self.left_in_at = False      # pad started sending frames without the closing +++
        self.junk_lines = 0
        self.mode = "pipe"
        self.params = {"MODE": "0", "LEVEL": "3", "CHANNEL": "0c", "MAC": "00,00",
                       "KEY": "0", "OPENKEY": "0", "PACKET": "0", "POWE": "13",
                       "SLEEP": "0"}
        self.line = bytearray()
        self.plus = 0
        self.plus_ok = False
        self.last_rx = 0.0
        self.total_rx = 0
        self.at_sessions = 0
        self.exits = 0
        self.helps = 0
        self.log: list[str] = []

    def _say(self, s: str) -> None:
        self.log.append(s)
        if self.verbose:
            print(f"      radio | {s}")

    def banner(self) -> bytes:
        p = self.params
        try:
            mhz = 2400 + 2 * int(p["CHANNEL"], 16)
        except ValueError:
            mhz = 0
        lines = ["LoRa Parameter:",
                 f"MODE:{p['MODE']}",
                 f"LEVEL:{p['LEVEL']}",
                 f"CHANNEL:{mhz}MHz >> {p['CHANNEL']}",
                 f"MAC:{p['MAC']}",
                 f"KEY:{p['KEY']} OPENKEY:{p['OPENKEY']}",
                 f"PACKET:{p['PACKET']} POWE:{p['POWE']} SLEEP:{p['SLEEP']}",
                 "===="]                       # must be last: help() stops on it
        return ("\r\n".join(lines) + "\r\n").encode()

    def _at(self, cmd: str) -> bytes:
        c = cmd.upper()
        if c in ("+++", "AT+RESET"):
            self.mode = "pipe"
            self.exits += 1
            self._say(f"{cmd:<16} -> Power On (back to transparent)")
            return b"Power On\r\n"
        if c == "AT+HELP":
            self.helps += 1
            self._say(f"{cmd:<16} -> parameter banner")
            return self.banner()
        m = re.fullmatch(r"AT\+(MODE|LEVEL|CHANNEL|MAC|KEY|OPENKEY|PACKET|POWE|SLEEP)(.+)", c)
        if m:
            key, val = m.groups()
            self.params[key] = val.lower() if key == "CHANNEL" else val
            self._say(f"{cmd:<16} -> OK")
            return b"OK\r\n"
        self.junk_lines += 1
        if self.junk_lines <= 2:
            self._say(f"{cmd.encode('ascii', 'backslashreplace')!r} -> ERROR (not an AT command)")
        elif self.junk_lines == 3:
            self._say("... more non-AT lines while in AT mode (suppressed)")
        return b"ERROR\r\n"

    def process(self, chunk: bytes, now: float, mid_frame: bool) -> tuple[bytes, bytes]:
        """Returns (bytes for the frame parser, bytes to send back to the pad)."""
        to_parser = bytearray()
        reply = bytearray()
        gap_ok = (now - self.last_rx) >= self.GUARD_S or self.total_rx == 0
        for b in chunk:
            self.total_rx += 1
            if self.mode == "at" and b == P.SOF and self.force_pipe:
                # A protocol frame while still in AT mode: the pad's set-up gave
                # up without the closing "+++".  A real module would now answer
                # every frame with ERROR and the link would never come up.
                self.mode = "pipe"
                self.left_in_at = True
                self.line.clear()
                self._say("frames arriving while still in AT mode: the pad never sent "
                          "the closing '+++' -- forcing transparent mode")
            if self.mode == "at":
                if b == 0x0D:
                    continue
                if b == 0x0A:
                    cmd = self.line.decode("ascii", "replace").strip()
                    self.line.clear()
                    if cmd:
                        reply += self._at(cmd)
                    continue
                self.line.append(b)
                if len(self.line) > 200:
                    self.line.clear()
                continue
            # transparent mode: "+++" after a quiet gap, outside a frame = AT entry
            if b == 0x2B:
                if self.plus == 0:
                    self.plus_ok = gap_ok and not mid_frame and not to_parser
                self.plus += 1
                if self.plus == 3:
                    self.plus = 0
                    if self.plus_ok:
                        self.mode = "at"
                        self.at_sessions += 1
                        self._say("+++              -> Entry AT")
                        reply += b"Entry AT\r\n"
                    else:
                        to_parser += b"+++"
                continue
            if self.plus:
                to_parser += b"+" * self.plus
                self.plus = 0
            to_parser.append(b)
        if chunk:
            self.last_rx = now
        return bytes(to_parser), bytes(reply)

    @property
    def configured(self) -> bool:
        return self.at_sessions >= 2 and self.exits >= 2 and self.mode == "pipe"


class WokwiRadioLink(Link):
    """A gse_control Link whose far end is the simulated pad's radio UART."""

    name = "wokwi-radio"

    def __init__(self, url: str, verbose: bool = False, force_pipe: bool = True):
        super().__init__(None)
        self.url = url
        self.ser = None
        self.radio = RadioEmu(verbose, force_pipe)
        self.all_events: list[P.Event] = []
        self.raw_rx = 0
        self.reconnects = 0
        self._next_try = 0.0

    def _connect(self) -> bool:
        import serial  # pyserial
        try:
            self.ser = serial.serial_for_url(self.url, baudrate=9600, timeout=0)
            return True
        except Exception:
            self.ser = None
            return False

    def open(self, wait_s: float = 300.0):
        t0 = time.monotonic()
        while time.monotonic() - t0 < wait_s:
            if self._connect():
                return
            time.sleep(0.05)
        raise SystemExit(f"could not connect to {self.url} in {wait_s:.0f} s -- "
                         "is the simulation running and its tab visible?")

    def close(self):
        if self.ser:
            try:
                self.ser.close()
            except Exception:
                pass
            self.ser = None

    def _lost(self):
        self.close()
        self.reconnects += 1
        self.radio = RadioEmu(self.radio.verbose, self.radio.force_pipe)  # pad re-runs set-up
        print("    ! connection to the simulator dropped (restarted?) -- reconnecting")

    def _write(self, data: bytes):
        # a module in AT mode does not transmit; neither do we
        if self.ser and self.radio.mode == "pipe":
            try:
                self.ser.write(data)
            except Exception:
                self._lost()

    def _read(self) -> bytes:
        if not self.ser:
            now = time.monotonic()
            if now >= self._next_try:
                self._next_try = now + 0.2
                self._connect()
            return b""
        try:
            n = self.ser.in_waiting
            chunk = self.ser.read(n) if n else b""
        except Exception:
            self._lost()
            return b""
        if not chunk:
            return b""
        self.raw_rx += len(chunk)
        to_parser, reply = self.radio.process(chunk, time.monotonic(), bool(self.parser.buf))
        if reply:
            try:
                self.ser.write(reply)
            except Exception:
                self._lost()
        return to_parser

    def poll(self):
        evs = super().poll()
        self.all_events.extend(evs)
        return evs


# ------------------------------------------------------------------ helpers
# Wokwi rarely runs at real time (a first run measured ~1/3).  The GUI's 20 Hz
# is wall-clock, so against a slow simulation it becomes ~60 Hz of *simulated*
# time -- more than the radio UART's 9600 baud carries -- and commands queue or
# drop.  Everything here is therefore clocked by the pad itself: it sends STATE
# every 100 ms of simulated time, and we answer each with two commands, which
# is the GUI's 20 Hz in simulated time whatever the simulator speed.
STATE_HZ = 10
CMDS_PER_STATE = 2


class Pacer:
    def __init__(self, ctl: Controller):
        self.ctl = ctl
        self.seen = ctl.link.stats.rx_state
        self.last_send = 0.0
        self.sim_s = 0.0                         # simulated time observed, from STATE count
        self.wall0 = time.monotonic()

    def service(self, send: bool = True) -> int:
        link = self.ctl.link
        link.poll()
        n = link.stats.rx_state
        new = n - self.seen
        self.seen = n
        self.sim_s += new / STATE_HZ
        if send:
            now = time.monotonic()
            k = min(new, 2) * CMDS_PER_STATE
            if k == 0 and now - self.last_send > 0.5:
                k = 1                            # keep-alive if STATE frames are lost
            for _ in range(k):
                self.ctl.push_cmd(force=True)
            if k:
                self.last_send = now
        return new

    def pump(self, sim_s: float, send: bool = True) -> None:
        target, got, t0 = sim_s * STATE_HZ, 0, time.monotonic()
        while got < target and time.monotonic() - t0 < sim_s * 10 + 10:
            got += self.service(send)
            time.sleep(0.003)

    def until(self, pred, sim_s: float, send: bool = True):
        """Wait up to sim_s of simulated time for a STATE satisfying pred."""
        target, got, t0 = sim_s * STATE_HZ, 0, time.monotonic()
        while got < target and time.monotonic() - t0 < sim_s * 10 + 10:
            new = self.service(send)
            got += new
            st = self.ctl.pad
            if new and st is not None and pred(st):
                return st, got / STATE_HZ
            time.sleep(0.003)
        return None, got / STATE_HZ

    def speed(self) -> float:
        wall = time.monotonic() - self.wall0
        return self.sim_s / wall if wall > 0 else float("nan")


def set_all(ctl: Controller, on: bool) -> None:
    ctl.set_fill(on)
    ctl.set_veh(on)
    ctl.set_prg(on)


# ---------------------------------------------------------------------- run
def cmd_run(args) -> int:
    fails: list[str] = []
    travels = [int(x) for x in args.travels.split(",") if x.strip()]
    t_start = time.time()

    link = WokwiRadioLink(args.url, verbose=args.verbose, force_pipe=not args.strict_radio)
    print(f"[1/6] connecting to {args.url}")
    print("      start (or restart) the Wokwi simulation now; keep its tab visible")
    link.open()
    print("      connected")

    print("[2/6] waiting for the pad's radio set-up dialog (dxlr41.h configure())")
    t0 = time.monotonic()
    warned = False
    while (time.monotonic() - t0 < args.setup_timeout and not link.radio.configured
           and not link.radio.left_in_at):
        link.poll()
        if not warned and link.raw_rx == 0 and time.monotonic() - t0 > 8:
            warned = True
            print("      ! 8 s and not one byte from the pad's radio UART. Check:\n"
                  "          - diagram.json wires esp:17 -> $serialMonitor:RX and\n"
                  "            esp:16 -> $serialMonitor:TX (the radio UART, not esp:TX/RX)\n"
                  "          - wokwi.toml has rfc2217ServerPort = 4000\n"
                  "          - the simulator tab is visible (hidden tabs pause)\n"
                  "      still waiting...")
        time.sleep(0.003)
    r = link.radio
    if r.configured:
        print(f"      done: {r.at_sessions} AT sessions, {r.helps} read-backs")
        for k, want in EXPECT_RADIO.items():
            got = r.params.get(k)
            ok = got == want
            print(f"        {k:<8} set to {got!s:<6} expected {want:<6} {'ok' if ok else 'MISMATCH'}")
            if not ok:
                fails.append(f"radio {k}={got}, expected {want}")
    elif r.left_in_at:
        print(f"      ! RADIO SET-UP ABORTED after {r.at_sessions} AT sessions / {r.helps} "
              "read-backs: the pad went to normal operation without sending the closing "
              "'+++'.\n        On hardware the module would stay in AT mode and the link "
              "would never come up\n        (dxlr41.h help(): the end-of-banner check runs "
              "on an unterminated buffer, so the second read-back stops after 1 byte).\n"
              "        Forcing the stand-in radio to "
              "transparent mode so the travel test can still run.")
        fails.append("pad aborted radio set-up (radio left in AT mode)")
    else:
        msg = ("never saw the radio set-up finish. The pad only does it at boot: if the "
               "sim was already running, restart it while this script waits. "
               "Carrying on -- the pad still obeys valid frames when its radio check fails.")
        print(f"      ! {msg}")
        fails.append("radio set-up dialog not observed")

    print("[3/6] bringing the link up with the real Controller")
    ctl = Controller()
    ctl.link = link
    ctl.ui.connected = True
    ctl.ui.link_name = link.name
    link.send_hello()
    ctl.push_cmd(force=True)
    pace = Pacer(ctl)
    st, dt = pace.until(lambda s: s.link_ok and not (s.flags & P.ST_COMMS_LOSS), 6.0)
    if st is None:
        print("      ! no fresh STATE from the pad within 6 s of simulated time")
        last = ctl.pad
        print(f"        last STATE: {last}")
        ctl.link.close()
        return 2
    print(f"      link OK after {dt:.1f} s simulated: fw {st.fw_major}.{st.fw_minor}, "
          f"pad_state {st.pad_state}, rx_count {st.rx_count}, crc_err {st.crc_err}")

    print(f"[4/6] travel sweep {travels} (hold {args.hold:.1f} s simulated per position)")
    ctl.ui.purge_enabled = True
    set_all(ctl, False)
    pace.pump(args.hold)
    steps = []
    for t in travels:
        want = clamp_travel(t)
        ctl.apply_cfg(travel_fill=t, travel_veh=t, travel_prg=t, allow_purge=1)
        st, dt = pace.until(lambda s: (s.travel_fill, s.travel_veh, s.travel_prg)
                            == (want, want, want), 2.0)
        st = st or ctl.pad
        echo = {"FILL": st.travel_fill, "VEH": st.travel_veh, "PRG": st.travel_prg}
        cfg_echo = {"wdg_ds": st.wdg_ds, "ign_max_ds": st.ign_max_ds, "allow_purge": st.allow_purge}
        pace.pump(args.hold)                                    # CLOSED at new travel
        set_all(ctl, True)
        st_o, _ = pace.until(lambda s: s.out_fill and s.out_veh and s.out_prg, 1.5)
        pace.pump(args.hold)                                    # OPEN
        set_all(ctl, False)
        st_c, _ = pace.until(lambda s: not (s.out_fill or s.out_veh or s.out_prg), 1.5)
        pace.pump(args.hold)                                    # CLOSED
        steps.append({"travel": t, "echo": echo, "cfg_echo": cfg_echo,
                      "opened": st_o is not None, "closed": st_c is not None,
                      "uptime_s": (st_c or ctl.pad).uptime_s})
        flag = "ok" if all(v == t for v in echo.values()) else "MISMATCH"
        print(f"      travel {t:>3}: pad reports FILL/VEH/PRG = "
              f"{echo['FILL']}/{echo['VEH']}/{echo['PRG']}  {flag}"
              f"{'' if st_o else '  (never reported OPEN)'}")

    print("[5/6] watchdog: stop sending, pad must go safe on its own")
    before = ctl.pad
    st, dt = pace.until(lambda s: bool(s.flags & P.ST_COMMS_LOSS), 6.0, send=False)
    if st is None:
        print("      ! pad never declared COMMS LOSS")
        fails.append("watchdog: no COMMS LOSS")
    else:
        safe = st.out_fill == 0 and st.out_veh == 1 and st.out_prg == 0 and st.out_ign == 0
        wdg_ms = st.wdg_ds * 100
        in_time = wdg_ms < st.age_ms <= wdg_ms + 1000 // STATE_HZ + 50
        print(f"      COMMS LOSS reported at age {st.age_ms} ms (pad's own clock; watchdog "
              f"{wdg_ms} ms, STATE every {1000 // STATE_HZ} ms) "
              f"{'on time' if in_time else 'LATE'}: FILL={st.out_fill} VEH={st.out_veh} "
              f"PRG={st.out_prg} IGN={st.out_ign} {'safe' if safe else 'NOT SAFE'}")
        if not safe:
            fails.append("watchdog: outputs not safe")
        if not in_time:
            fails.append(f"watchdog: COMMS LOSS at age {st.age_ms} ms, limit {wdg_ms} ms")
    seqs = sorted({e.arg for e in link.all_events if e.code == P.EV_CFG_CHANGED})
    last = ctl.pad or before
    print(f"      link health: pad counted {last.rx_count} valid commands, "
          f"{last.crc_err} bad; laptop side dropped {link.parser.frames_bad} bad frames")
    print(f"      simulation ran at {pace.speed():.2f}x real time")
    ctl.disconnect()

    print("[6/6] what the pad reported back (STATE frames)")
    for s in steps:
        bad = [k for k, v in s["echo"].items() if v != s["travel"]]
        if bad:
            fails.append(f"travel {s['travel']}: STATE says "
                         + ", ".join(f"{k}={s['echo'][k]}" for k in bad))
        ce = s["cfg_echo"]
        if (ce["wdg_ds"], ce["ign_max_ds"], ce["allow_purge"]) != \
           (ctl.cfg.wdg_ds, ctl.cfg.ign_max_ds, 1):
            fails.append(f"travel {s['travel']}: cfg echo {ce}")
    print(f"      CFG_CHANGED echoed cfg_seq values: {seqs} "
          f"({'every CFG carried the same cfg_seq' if len(seqs) <= 1 else 'distinct'})")

    with open(args.steps, "w", encoding="utf-8") as fh:
        json.dump({"t_start": t_start, "travels": travels, "steps": steps,
                   "radio_log": link.radio.log}, fh, indent=1)
    print(f"      steps saved to {args.steps}")

    if fails:
        print("      problems on the link / STATE side:")
        for f in fails:
            print(f"        - {f}")
    else:
        print("      every value the software set was reported back unchanged")

    rc = 0
    if not args.no_wait_vcd:
        vcd = wait_for_vcd(args.vcd, t_start)
        if vcd:
            rc = analyze(vcd, steps)
    return 1 if (fails or rc) else 0


def wait_for_vcd(explicit: str | None, newer_than: float) -> str | None:
    cands = [explicit] if explicit else [
        os.path.join(DEFAULT_FIRMWARE, "wokwi.vcd"),
        os.path.join(os.path.dirname(os.path.dirname(DEFAULT_FIRMWARE)), "wokwi.vcd"),
        os.path.join(os.path.dirname(DEFAULT_FIRMWARE), "wokwi.vcd"),
    ]
    print("\n>>> STOP the Wokwi simulation now -- the logic-analyzer file is written on stop.")
    print("    watching: " + " | ".join(cands) + "   (Ctrl-C to skip)")
    try:
        t0 = time.monotonic()
        while time.monotonic() - t0 < 600:
            for c in cands:
                if os.path.exists(c) and os.path.getmtime(c) > newer_than:
                    s1 = os.path.getsize(c)
                    time.sleep(1.0)                    # let the writer finish
                    if os.path.getsize(c) == s1:
                        print(f"    found {c}")
                        return c
            time.sleep(0.5)
    except KeyboardInterrupt:
        pass
    print("    no new VCD; run 'analyze --vcd <file>' later")
    return None


# ------------------------------------------------------------------ VCD
_SCALE = {"s": 1.0, "ms": 1e-3, "us": 1e-6, "ns": 1e-9, "ps": 1e-12, "fs": 1e-15}


def parse_vcd(path: str) -> tuple[dict[str, list[tuple[int, str]]], float]:
    with open(path, "r", encoding="ascii", errors="replace") as fh:
        text = fh.read()
    end = text.find("$enddefinitions")
    if end < 0:
        raise SystemExit(f"{path}: not a VCD file")
    header, body = text[:end], text[end:]
    m = re.search(r"\$timescale\s+(\d+)\s*([a-z]+)\s+\$end", header)
    scale = int(m.group(1)) * _SCALE[m.group(2)] if m else 1e-9
    ids: dict[str, str] = {}
    for vm in re.finditer(r"\$var\s+\S+\s+\d+\s+(\S+)\s+(\S+)[^$]*\$end", header):
        ids[vm.group(1)] = vm.group(2)
    ch: dict[str, list[tuple[int, str]]] = {n: [] for n in ids.values()}
    t = 0
    for tok in body.split():
        c0 = tok[0]
        if c0 == "#":
            t = int(tok[1:])
        elif c0 in "01xXzZ" and tok[1:] in ids:
            ch[ids[tok[1:]]].append((t, c0))
    return ch, scale


def pulses_of(changes: list[tuple[int, str]], scale: float) -> list[tuple[float, float]]:
    """[(rise_time_s, high_width_us)]"""
    out = []
    level = None
    rise = None
    for t, v in changes:
        if v == "1" and level != "1":
            rise = t
        elif v == "0" and level == "1" and rise is not None:
            out.append((rise * scale, (t - rise) * scale * 1e6))
            rise = None
        level = v if v in "01" else None
    return out


def runs_of(pulses: list[tuple[float, float]], dropped: list | None = None) -> list[dict]:
    """Group pulses into settings.  Settings shorter than MIN_RUN_PULSES are
    appended to `dropped` (if given) instead of being returned."""
    runs: list[dict] = []
    for t, w in pulses:
        if runs and abs(w - runs[-1]["mean"]) <= RUN_TOL_US:
            r = runs[-1]
            r["n"] += 1
            r["mean"] += (w - r["mean"]) / r["n"]
            r["t1"] = t
        else:
            runs.append({"mean": w, "n": 1, "t0": t, "t1": t})
    if dropped is not None:            # 1-2 pulses = the edge of a normal change
        dropped.extend(r for r in runs if 3 <= r["n"] < MIN_RUN_PULSES)
    runs = [r for r in runs if r["n"] >= MIN_RUN_PULSES]          # drop glitches
    merged: list[dict] = []
    for r in runs:
        if merged and abs(r["mean"] - merged[-1]["mean"]) <= RUN_TOL_US:
            m = merged[-1]
            tot = m["n"] + r["n"]
            m["mean"] = (m["mean"] * m["n"] + r["mean"] * r["n"]) / tot
            m["n"] = tot
            m["t1"] = r["t1"]
        else:
            merged.append(dict(r))
    for r in merged:
        r["kind"] = ("O" if r["mean"] > NEUTRAL_US + 2 else
                     "C" if r["mean"] < NEUTRAL_US - 2 else "N")
    return merged


CHANNELS = {"FILL": ("FILL", "D0"), "VEH": ("VEH", "D1"), "PRG": ("PRG", "D2"),
            "IGN": ("IGN", "D3"), "HB": ("HB", "D4")}


def _find(ch: dict, names) -> list | None:
    for n in names:
        if n in ch:
            return ch[n]
    return None


def analyze(vcd: str, steps: list[dict]) -> int:
    ch, scale = parse_vcd(vcd)
    print(f"\n=== servo pins ({os.path.basename(vcd)}) ===")
    fails = 0
    hdr = (f"  {'chan':<5}{'set':>5}{'reported':>10}   {'OPEN us':>9} {'deg':>7}   "
           f"{'CLOSED us':>9} {'deg':>7}   {'= fw code?':<11} result")
    for name in ("FILL", "VEH", "PRG"):
        chg = _find(ch, CHANNELS[name])
        if chg is None:
            print(f"  {name}: channel missing from the VCD (logic analyzer wiring?)")
            fails += 1
            continue
        pl = pulses_of(chg, scale)
        if len(pl) < 10:
            print(f"  {name}: only {len(pl)} pulses recorded")
            fails += 1
            continue
        periods = [b[0] - a[0] for a, b in zip(pl, pl[1:]) if b[0] - a[0] < 0.05]
        per_ms = statistics.median(periods) * 1e3 if periods else float("nan")
        short: list[dict] = []
        runs = runs_of(pl, short)
        sel = [i for i in range(1, len(runs) - 1)
               if runs[i]["kind"] == "O" and runs[i - 1]["kind"] == "C"
               and runs[i + 1]["kind"] == "C"]
        print(f"\n  {name}: {len(pl)} pulses, period {per_ms:.3f} ms, {len(runs)} settings seen")
        if short:
            print(f"  ! {len(short)} brief changes (< {MIN_RUN_PULSES} pulses) ignored, at t = "
                  + ", ".join(f"{r['t0']:.2f}s" for r in short[:8])
                  + (" ..." if len(short) > 8 else "")
                  + "\n    (usually the pad's watchdog briefly going safe because commands"
                    " arrived late)")
        if len(sel) != len(steps):
            print(f"  ! expected {len(steps)} OPEN periods between CLOSED ones, found {len(sel)};"
                  " settings seen in order:")
            for r in runs:
                print(f"      {r['kind']} {r['mean']:8.2f} us x{r['n']:<5} "
                      f"t={r['t0']:.2f}..{r['t1']:.2f} s")
            fails += 1
            continue
        print(hdr)
        for step, i in zip(steps, sel):
            t = step["travel"]
            o_us, c_us = runs[i]["mean"], runs[i + 1]["mean"]
            o_deg = (o_us - NEUTRAL_US) / US_PER_DEG
            c_deg = (c_us - NEUTRAL_US) / US_PER_DEG
            fw_ok = (abs(o_us - fw_predicted_us(t, True)) <= FW_MATCH_TOL_US and
                     abs(c_us - fw_predicted_us(t, False)) <= FW_MATCH_TOL_US)
            ok = abs(o_deg - t) <= ANGLE_TOL_DEG and abs(-c_deg - t) <= ANGLE_TOL_DEG
            rep = step["echo"].get(name)
            verdict = "PASS" if ok else f"FAIL: {t} deg set, {o_deg:+.1f}/{c_deg:+.1f} at the pin"
            if rep != t:
                verdict += f" (pad reports {rep})"
            print(f"  {name:<5}{t:>5}{rep!s:>10}   {o_us:9.2f} {o_deg:+7.2f}   "
                  f"{c_us:9.2f} {c_deg:+7.2f}   {'yes' if fw_ok else 'NO':<11} {verdict}")
            if not ok:
                fails += 1

    ign = _find(ch, CHANNELS["IGN"])
    if ign is not None:
        ign = list(ign)
        boot_ms = None
        if ign and ign[0] == (0, "1"):
            # undriven at reset: the diagram has no pull-down; the board has
            # 10k to GND on GPIO7 (hardware/wiring.md), so this is not a firing
            nxt = next((t for t, v in ign if v == "0"), None)
            boot_ms = (nxt or 0) * scale * 1e3
            ign = [(t, v) for t, v in ign if nxt is not None and t >= nxt]
        n_on = len(pulses_of(ign, scale)) + (1 if ign and ign[-1][1] == "1" else 0)
        print(f"\n  IGN relay: energised {n_on} times {'(ok: never armed)' if n_on == 0 else '!!'}"
              + (f"; pin floated high for {boot_ms:.0f} ms at reset before setup() "
                 "drove it low (no pull-down in the diagram)" if boot_ms else ""))
        if n_on:
            fails += 1
    hb = _find(ch, CHANNELS["HB"])
    if hb is not None and len(hb) > 3:
        gaps = [(b[0] - a[0]) * scale * 1e3 for a, b in zip(hb, hb[1:])]
        worst = max(gaps[2:]) if len(gaps) > 2 else max(gaps)
        print(f"  heartbeat: {len(hb)} edges, typical {statistics.median(gaps):.1f} ms, "
              f"longest gap {worst:.1f} ms"
              + ("  <- loop() stalled" if worst > 100 else ""))
    print(f"\n{'ALL PASS' if fails == 0 else f'{fails} FAIL(S)'}")
    return 1 if fails else 0


def inspect(vcd: str) -> int:
    """Free-form timeline of a recording: what each output did, when, for how
    long, in microseconds and degrees.  For manual sessions (e.g. wokwi_gui.py)
    where there is no scripted step list to compare against."""
    ch, scale = parse_vcd(vcd)
    end = max((v[-1][0] for v in ch.values() if v), default=0) * scale
    print(f"=== {os.path.basename(vcd)}: {end:.2f} s of simulated time ===")
    for name in ("FILL", "VEH", "PRG"):
        chg = _find(ch, CHANNELS[name])
        if chg is None:
            print(f"\n{name}: not in the recording")
            continue
        pl = pulses_of(chg, scale)
        short: list[dict] = []
        runs = runs_of(pl, short)
        print(f"\n{name}  ({len(pl)} pulses)")
        print(f"  {'from':>8} {'to':>8} {'held':>7}   {'pulse':>9}  {'angle':>8}  position")
        for r in runs:
            deg = (r["mean"] - NEUTRAL_US) / US_PER_DEG
            pos = {"O": "OPEN", "C": "CLOSED", "N": "neutral"}[r["kind"]]
            print(f"  {r['t0']:7.2f}s {r['t1'] + 0.02:7.2f}s {r['t1'] + 0.02 - r['t0']:6.2f}s"
                  f"   {r['mean']:7.2f}us  {deg:+7.2f} deg  {pos}")
        if short:
            print(f"  + {len(short)} brief changes (<{MIN_RUN_PULSES} pulses) at t = "
                  + ", ".join(f"{r['t0']:.2f}s" for r in short[:10])
                  + (" ..." if len(short) > 10 else ""))
    ign = _find(ch, CHANNELS["IGN"])
    if ign is not None:
        ign = list(ign)
        if ign and ign[0] == (0, "1"):                      # reset float, not a firing
            nxt = next((t for t, v in ign if v == "0"), None)
            ign = [(t, v) for t, v in ign if nxt is not None and t >= nxt]
        on = pulses_of(ign, scale)
        still_on = bool(ign) and ign[-1][1] == "1"
        print(f"\nIGN relay (GPIO7): energised {len(on) + still_on} time(s)")
        for t, w in on:
            print(f"  on at {t:7.2f}s for {w / 1000:7.1f} ms")
        if still_on:
            print(f"  on at {ign[-1][0] * scale:7.2f}s and still on when the recording ended")
    hb = _find(ch, CHANNELS["HB"])
    if hb is not None and len(hb) > 3:
        gaps = [(b[0] - a[0]) * scale * 1e3 for a, b in zip(hb, hb[1:])]
        worst = max(gaps[2:]) if len(gaps) > 2 else max(gaps)
        print(f"\nheartbeat (GPIO8): typical {statistics.median(gaps):.1f} ms, "
              f"longest gap {worst:.1f} ms" + ("  <- loop() stalled" if worst > 100 else ""))
    return 0


def cmd_analyze(args) -> int:
    with open(args.steps, encoding="utf-8") as fh:
        steps = json.load(fh)["steps"]
    return analyze(args.vcd, steps)


# ---------------------------------------------------------------------- main
def main() -> int:
    ap = argparse.ArgumentParser(description=__doc__.split("\n\n")[0])
    ap.add_argument("--software", default=_software_dir(), help="GSE/software directory")
    sub = ap.add_subparsers(dest="cmd", required=True)
    r = sub.add_parser("run", help="drive the simulated pad, then analyse the VCD")
    r.add_argument("--url", default="rfc2217://localhost:4000")
    r.add_argument("--travels", default="5,30,45,60,90")
    r.add_argument("--hold", type=float, default=1.0, help="seconds per position")
    r.add_argument("--setup-timeout", type=float, default=60.0)
    r.add_argument("--steps", default=DEFAULT_STEPS)
    r.add_argument("--vcd", default=None, help="VCD path (default: look next to wokwi.toml)")
    r.add_argument("--no-wait-vcd", action="store_true")
    r.add_argument("-v", "--verbose", action="store_true", help="print the AT dialog")
    r.add_argument("--strict-radio", action="store_true",
                   help="behave like a real module: if the pad leaves it in AT mode, stay there")
    a = sub.add_parser("analyze", help="analyse a saved VCD against saved steps")
    a.add_argument("--vcd", required=True)
    a.add_argument("--steps", default=DEFAULT_STEPS)
    i = sub.add_parser("inspect", help="timeline of any recording (manual sessions)")
    i.add_argument("--vcd", default=os.path.join(DEFAULT_FIRMWARE, "wokwi.vcd"))
    args = ap.parse_args()
    if args.cmd == "inspect":
        return inspect(args.vcd)
    return cmd_run(args) if args.cmd == "run" else cmd_analyze(args)


if __name__ == "__main__":
    sys.exit(main())
