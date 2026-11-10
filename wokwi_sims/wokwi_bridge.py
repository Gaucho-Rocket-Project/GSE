#!/usr/bin/env python3
"""
wokwi_bridge.py -- a separate process that stands in for the TX node ("bridge")
and both DX-LR41 radios, so the unmodified GUI (software/run_gui.py) can drive
the real pad firmware running in Wokwi.

    run_gui.py ──rfc2217://localhost:5000──► wokwi_bridge.py ──rfc2217://localhost:4000──► Wokwi
    (laptop GUI,      "the TX node's USB port"    (TX node + radios)     (pad radio UART)    (pad firmware)
     unmodified)

Port 4000 is Wokwi's serial port (wokwi.toml rfc2217ServerPort); port 5000 is
served here.  The GUI reaches it because link.py opens ports with
serial.serial_for_url(), which accepts rfc2217:// addresses as well as COM7.

TX-node behaviour reproduced from firmware/src/common/gse_tx_core.c
    * host frames: CMD length check, 200 ms stale drop (first CMD after a gap),
      CFG / HELLO forwarded; frames re-encoded with the bridge's own seq
    * laptop silent for 300 ms -> synthetic SAFE commands at 20 Hz
      (FILL closed, PURGE closed, VEHICLE open, IGN off), also before the GUI
      ever connects -- exactly like a powered TX node with no laptop
    * pad frames re-encoded and sent to the laptop verbatim
    * text console: PING, INFO, SAFE, HELP (SET LEVEL / SET KEY / RADIO CONFIG
      are acknowledged but not simulated)
Radio behaviour
    * answers the pad's AT set-up dialog (dxlr41.h configure()); if the pad
      aborts it (bug D2), the radio is forced to normal mode and this is logged
    * commands are paced to the SIMULATION's clock: 2 per STATE received
      (= 20 Hz simulated), latest command wins -- otherwise a slow simulation
      is flooded past what its 9600-baud UART can carry

Usage (from the GSE folder; needs pyserial)
    python wokwi_sims/wokwi_bridge.py                             # 1. leave running
    (start the Wokwi simulation in VS Code, tab visible)          # 2.
    python software/run_gui.py --port rfc2217://localhost:5000    # 3. click Connect
    python wokwi_sims/wokwi_pad_test.py inspect                   # 4. after stopping the sim

Requires the two link.py changes (serial.serial_for_url() in SerialLink.open,
and SerialLink._read() reading until nothing is waiting) so the GUI can open an
rfc2217:// address.
"""
from __future__ import annotations

import argparse
import os
import select
import socket
import sys
import time

HERE = os.path.dirname(os.path.abspath(__file__))
sys.path.insert(0, HERE)
import wokwi_pad_test as W                       # noqa: E402  (also adds GSE/software)
from gse_control import protocol as P            # noqa: E402

# mirrored from firmware/src/common/gse_tx_core.c
LINK_TIMEOUT_S = 0.300
CMD_MAX_AGE_S = 0.200
CMD_PERIOD_S = 0.050

TOKENS_PER_STATE = 2
TOKEN_CAP = 4


def stamp() -> str:
    return time.strftime("%H:%M:%S")


# --------------------------------------------------------------- framing
class RawParser:
    """gse_parser_push() in Python, returning (type, seq, payload) unchanged."""

    def __init__(self):
        self.buf = bytearray()
        self.bad = 0

    def _drop(self):
        del self.buf[0]
        self.bad += 1
        while self.buf and self.buf[0] != P.SOF:
            del self.buf[0]

    def push_byte(self, b: int):
        buf = self.buf
        if not buf:
            if b == P.SOF:
                buf.append(b)
            return None
        if len(buf) >= P.MAX_FRAME:
            buf.clear()
            return None
        buf.append(b)
        while True:
            if not buf:
                return None
            if buf[0] != P.SOF:
                self._drop()
                continue
            if len(buf) >= 2 and buf[1] != P.PROTO_VERSION:
                self._drop()
                continue
            if len(buf) < 5:
                return None
            n = buf[4]
            if n > P.MAX_PAYLOAD:
                self._drop()
                continue
            if len(buf) < 7 + n:
                return None
            if P.crc16(bytes(buf[1:5 + n])) != (buf[5 + n] | (buf[6 + n] << 8)):
                self._drop()
                continue
            frame = (buf[2], buf[3], bytes(buf[5:5 + n]))
            buf.clear()
            return frame


# ------------------------------------------------------------- pad sides
class WokwiPad:
    """The simulated pad's radio UART, through Wokwi's RFC2217 server."""

    name = "Wokwi"

    def __init__(self, url: str, verbose: bool = False):
        self.url = url
        self.verbose = verbose
        self.ser = None
        self.radio = W.RadioEmu(verbose, force_pipe=True)
        self._next_try = 0.0
        self._reported = False
        self._waiting_note = False

    def _connect(self) -> None:
        import serial
        try:
            self.ser = serial.serial_for_url(self.url, baudrate=9600, timeout=0)
            self.radio = W.RadioEmu(self.verbose, force_pipe=True)
            self._reported = False
            print(f"{stamp()} [radio] connected to the simulated pad at {self.url}")
        except Exception:
            self.ser = None
            if not self._waiting_note:
                self._waiting_note = True
                print(f"{stamp()} [radio] waiting for the Wokwi simulation on {self.url} "
                      "(start it; keep its tab visible)")

    def _lost(self) -> None:
        try:
            if self.ser:
                self.ser.close()
        except Exception:
            pass
        self.ser = None
        self._waiting_note = False
        print(f"{stamp()} [radio] simulation connection dropped (stopped/restarted?)")

    def can_send(self) -> bool:
        return self.ser is not None and self.radio.mode == "pipe"

    def send(self, data: bytes) -> None:
        try:
            self.ser.write(data)
        except Exception:
            self._lost()

    def service(self, mid_frame: bool) -> bytes:
        if self.ser is None:
            now = time.monotonic()
            if now >= self._next_try:
                self._next_try = now + 0.5
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
        to_parser, reply = self.radio.process(chunk, time.monotonic(), mid_frame)
        if reply:
            self.send(reply)
        self._report()
        return to_parser

    def _report(self) -> None:
        r = self.radio
        if self._reported:
            return
        if r.left_in_at:
            print(f"{stamp()} [radio] PAD RADIO FAULT: the pad aborted its radio set-up after "
                  "the 2nd AT+HELP (dxlr41.h help() bug, D2). On hardware the radio would stay "
                  "in AT mode; here it is forced to normal mode so the link works.")
        elif r.configured:
            p = r.params
            print(f"{stamp()} [radio] pad radio set-up completed: MODE {p['MODE']} "
                  f"LEVEL {p['LEVEL']} CH {p['CHANNEL']} MAC {p['MAC']}")
        else:
            return
        self._reported = True


class SimPad:
    """sim.PadModel instead of Wokwi -- only for this file's self-test."""

    name = "sim.PadModel"

    def __init__(self):
        from gse_control.sim import PadModel
        self.pad = PadModel()

    def can_send(self) -> bool:
        return True

    def send(self, data: bytes) -> None:
        self.pad.feed(data)

    def service(self, mid_frame: bool) -> bytes:
        return self.pad.tick()


# ----------------------------------------------------- RFC2217 host side
class _VirtualPort:
    """What pyserial's RFC2217 PortManager needs to see: a port to configure."""

    def __init__(self):
        self.baudrate, self.bytesize, self.parity, self.stopbits = 115200, 8, "N", 1
        self.rtscts = self.xonxoff = False
        self.rts = self.dtr = True
        self.break_condition = False
        self.cts = self.dsr = True
        self.ri = self.cd = False

    def reset_input_buffer(self):
        pass

    def reset_output_buffer(self):
        pass


class _Conn:
    def __init__(self, sock):
        self.sock = sock

    def write(self, data: bytes) -> None:
        self.sock.sendall(data)


# ------------------------------------------------------------------ bridge
class Bridge:
    def __init__(self, pad, listen_port: int, quiet: bool = False):
        self.pad = pad
        self.quiet = quiet
        # Listen on IPv4 AND IPv6 loopback: Windows tries "localhost" as ::1
        # first, and an IPv4-only listener costs every connect ~2 s.
        self.srvs = []
        for fam, host in ((socket.AF_INET, "127.0.0.1"), (socket.AF_INET6, "::1")):
            try:
                s = socket.socket(fam, socket.SOCK_STREAM)
                s.bind((host, listen_port))
                s.listen(1)
                s.setblocking(False)
                self.srvs.append(s)
            except OSError:
                pass
        if not self.srvs:
            raise SystemExit(f"cannot listen on port {listen_port} (already in use?)")
        self.listen_port = listen_port
        self.cli = None
        self.pm = None
        self.host_parser = RawParser()
        self.pad_parser = RawParser()
        self.in_binary = False
        self.line = bytearray()
        self.last_host: float | None = None
        self.last_safe = 0.0
        self.last_cmd = P.Cmd(veh=1, wdg_ds=P.DEF_WDG_DS, ign_max_ds=P.DEF_IGN_MAX_DS)
        self.seq_air = 0
        self.seq_host = 0
        self.pending: bytes | None = None
        self.tokens = 2
        self.last_air_send = 0.0
        self.t0 = time.monotonic()
        self.st = dict(forwarded=0, stale=0, safe=0, superseded=0, rx_state=0,
                       rx_event=0, bad_host=0)
        self.last_status = time.monotonic()
        self.was_alive = False

    def note(self, msg: str) -> None:
        if not self.quiet:
            print(f"{stamp()} {msg}")

    # ------------------------------------------------------------ laptop I/O
    def _drop_client(self, why: str) -> None:
        try:
            self.cli.close()
        except Exception:
            pass
        self.cli = None
        self.pm = None
        self.note(f"[usb] GUI disconnected ({why}); SAFE stream takes over after 300 ms")

    def host_write(self, data: bytes) -> None:
        if self.cli is None:
            return
        try:
            self.cli.sendall(b"".join(self.pm.escape(data)))
        except Exception:
            self._drop_client("write failed")

    def log(self, text: str) -> None:                 # gse_log(): "LOG ...\n"
        self.host_write(f"LOG {text}\n".encode())

    # --------------------------------------------------------------- uplink
    def air_send(self, ftype: int, payload: bytes) -> None:
        frame = P.encode(ftype, self.seq_air, payload)
        self.seq_air = (self.seq_air + 1) & 0xFF
        if ftype == P.T_CMD:
            if self.pending is not None:
                self.st["superseded"] += 1
            self.pending = frame
            self._flush()
        elif self.pad.can_send():
            self.pad.send(frame)
            self.last_air_send = time.monotonic()

    def _flush(self) -> None:
        if self.pending is not None and self.tokens >= 1 and self.pad.can_send():
            self.tokens -= 1
            self.pad.send(self.pending)
            self.pending = None
            self.last_air_send = time.monotonic()

    def air_safe_command(self) -> None:
        c = P.Cmd(veh=1, wdg_ds=self.last_cmd.wdg_ds or P.DEF_WDG_DS,
                  ign_max_ds=self.last_cmd.ign_max_ds or P.DEF_IGN_MAX_DS)
        self.air_send(P.T_CMD, c.pack())
        self.st["safe"] += 1

    def on_host(self, ftype: int, _seq: int, pl: bytes) -> None:
        now = time.monotonic()
        gap = (now - self.last_host) if self.last_host is not None else float("inf")
        self.last_host = now
        if ftype == P.T_CMD:
            if len(pl) != 12:
                self.st["bad_host"] += 1
                return
            if gap > CMD_MAX_AGE_S:
                self.st["stale"] += 1
                return
            self.last_cmd = P.Cmd.unpack(pl)
            self.st["forwarded"] += 1
            self.air_send(P.T_CMD, pl)
        elif ftype == P.T_CFG:
            if len(pl) == 8:
                c = P.Cfg.unpack(pl)
                self.note(f"[usb] CFG travel {c.travel_fill}/{c.travel_veh}/{c.travel_prg} "
                          f"wdg {c.wdg_ds} ign {c.ign_max_ds} purge {c.allow_purge} "
                          f"cfg_seq {c.cfg_seq} -> pad")
                self.air_send(P.T_CFG, pl)
        elif ftype == P.T_HELLO:
            self.air_send(P.T_HELLO, pl)
        else:
            self.st["bad_host"] += 1

    def handle_line(self, line: str) -> None:
        line = line.upper()
        ms = int((time.monotonic() - self.t0) * 1000)
        if line == "PING":
            self.log(f"PONG {ms}")
        elif line == "INFO":
            self.log(f"TX wokwi-bridge (simulated) fwd={self.st['forwarded']} "
                     f"stale={self.st['stale']} safe={self.st['safe']} uptime={ms // 1000}s")
        elif line == "SAFE":
            self.last_host = None
            self.log("SAFE forced")
        elif line == "HELP":
            self.log("cmds: PING INFO SAFE HELP (SET LEVEL/SET KEY/RADIO CONFIG not simulated)")
        elif line.startswith(("SET LEVEL", "SET KEY", "RADIO CONFIG")):
            self.log(f"'{line}' not simulated by wokwi_bridge.py")
        elif line:
            self.log("ERR unknown command")
        self.note(f"[usb] console: {line}")

    def host_bytes(self, data: bytes) -> None:          # gse_tx_tick() host loop
        for b in data:
            if b == P.SOF:
                self.in_binary = True
                self.line.clear()
            if self.in_binary:
                fr = self.host_parser.push_byte(b)
                if fr:
                    self.on_host(*fr)
                if not self.host_parser.buf and b != P.SOF:
                    self.in_binary = False
                continue
            if b in (0x0A, 0x0D):
                if self.line:
                    self.handle_line(self.line.decode("ascii", "replace"))
                    self.line.clear()
                continue
            if 0x20 <= b < 0x7F:
                if len(self.line) < 95:
                    self.line.append(b)
            else:
                self.line.clear()

    # ------------------------------------------------------------- downlink
    def pad_bytes(self, data: bytes) -> None:
        for b in data:
            fr = self.pad_parser.push_byte(b)
            if not fr:
                continue
            ftype, _seq, pl = fr
            self.host_write(P.encode(ftype, self.seq_host, pl))
            self.seq_host = (self.seq_host + 1) & 0xFF
            if ftype == P.T_STATE:
                self.st["rx_state"] += 1
                self.tokens = min(TOKEN_CAP, self.tokens + TOKENS_PER_STATE)
            elif ftype == P.T_EVENT and len(pl) == 3:
                self.st["rx_event"] += 1
                ev = P.Event.unpack(pl)
                self.note(f"[pad] EVENT {ev.name}" + (f" arg={ev.arg}" if ev.arg else ""))

    # ----------------------------------------------------------------- loop
    def step(self) -> None:
        now = time.monotonic()
        # new GUI connection (one at a time, like a USB port)
        for srv in self.srvs:
            try:
                sock, addr = srv.accept()
            except (BlockingIOError, OSError):
                continue
            if self.cli is not None:
                self._drop_client("replaced by a new connection")
            sock.setblocking(False)
            self.cli = sock
            self.pm = W_rfc2217.PortManager(_VirtualPort(), _Conn(sock))
            self.note(f"[usb] GUI connected from {addr[0]}:{addr[1]}")
        # laptop -> bridge
        if self.cli is not None:
            try:
                r, _, _ = select.select([self.cli], [], [], 0)
                if r:
                    raw = self.cli.recv(4096)
                    if not raw:
                        self._drop_client("closed")
                    else:
                        self.host_bytes(b"".join(self.pm.filter(raw)))
            except (ConnectionError, OSError) as exc:
                self._drop_client(str(exc))
        # pad -> bridge -> laptop
        data = self.pad.service(bool(self.pad_parser.buf))
        if data:
            self.pad_bytes(data)
        # laptop watchdog -> synthetic SAFE stream at 20 Hz
        alive = self.last_host is not None and (now - self.last_host) < LINK_TIMEOUT_S
        if alive != self.was_alive:
            self.was_alive = alive
            self.note("[tx] laptop frames arriving: forwarding GUI commands" if alive else
                      "[tx] laptop silent > 300 ms: sending SAFE (FILL closed, VEHICLE open)")
        if not alive and (now - self.last_safe) >= CMD_PERIOD_S:
            self.last_safe = now
            self.air_safe_command()
        # pacing keep-alive while no STATE comes back
        if self.tokens < 1 and now - self.last_air_send > 0.5:
            self.tokens = 1
        self._flush()
        if now - self.last_status >= 10 and not self.quiet:
            self.last_status = now
            s = self.st
            print(f"{stamp()} [tx] fwd={s['forwarded']} stale={s['stale']} safe={s['safe']} "
                  f"superseded={s['superseded']} state={s['rx_state']} event={s['rx_event']} "
                  f"bad_host={s['bad_host']} gui={'yes' if self.cli else 'no'}")

    def run(self, stop=None) -> None:
        while stop is None or not stop.is_set():
            self.step()
            time.sleep(0.002)

    def close(self) -> None:
        if self.cli is not None:
            self.cli.close()
        for s in self.srvs:
            s.close()


import serial.rfc2217 as W_rfc2217  # noqa: E402


def main() -> int:
    ap = argparse.ArgumentParser(description="TX-node + radio stand-in between run_gui.py and Wokwi")
    ap.add_argument("--listen", type=int, default=5000, help="port the GUI connects to")
    ap.add_argument("--wokwi", default="rfc2217://localhost:4000", help="Wokwi's serial port")
    ap.add_argument("-v", "--verbose", action="store_true", help="print the pad's AT dialog")
    args = ap.parse_args()
    br = Bridge(WokwiPad(args.wokwi, args.verbose), args.listen)
    print(f"wokwi_bridge: GUI port  rfc2217://localhost:{args.listen}")
    print(f"              pad side  {args.wokwi}")
    print(f"start the Wokwi simulation, then (from the GSE folder):  python software/run_gui.py "
          f"--port rfc2217://localhost:{args.listen}   and click Connect.  Ctrl-C to stop.\n")
    try:
        br.run()
    except KeyboardInterrupt:
        pass
    finally:
        br.close()
    return 0


if __name__ == "__main__":
    sys.exit(main())
