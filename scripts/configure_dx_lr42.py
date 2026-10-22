#!/usr/bin/env python3
"""Provision and inspect DX-SMART DX-LR42-24T12D (SX1281, 2.4 GHz) radio modules.

Out of the box, two factory-fresh DX-LR42-24T12D modules already match GSE's
UART/RF requirements (`9600 8N1`, `AT+MODE0` transparent, `AT+SLEEP2` continuous RX,
`AT+LEVEL3` 10 kbps LoRa, `AT+CHANNEL0C` = `2424 MHz`, `AT+POWE13` = `+13 dBm`,
`AT+PACKET3` = `230 B`, `AT+DRSSI0`).

Use this helper with a **3.3 V TTL** USB-UART adapter to:
  * inspect a module's stored configuration and ambient 2.4 GHz channel noise (`--inspect`),
  * lock both ground & vehicle modules onto a private team channel / network key,
  * select the `pad` (`LEVEL 3`, `10 kbps`, ~2.2 km) or `long-range` (`LEVEL 1`,
    `SF11 / 406 kHz`, `1 kbps`, up to `3.7 km` LOS) RF preset,
  * optionally enable hardware `M0`/`M1` mode locking (`--lock-pins` -> `AT+SWITCH1`)
    when `PG12` (`M0`) and `PG13` (`M1`) are wired on both NUCLEO-H753ZI boards.

WARNING:
  DX-LR42-24T12D `VCC` accepts `3.3 V - 5.5 V` (`5.0 V` recommended for `+13 dBm`),
  but `RX`, `TX`, `M0`, `M1`, and `AUX` are **strictly 3.3 V TTL**. Set your USB-UART
  jumper to **3.3 V** before connecting `TX`/`RX`.
"""

from __future__ import annotations

import argparse
import os
import sys
import time
from typing import Optional, Protocol

BAUD_CODES = {
    2400: 0,
    4800: 1,
    9600: 2,
    19200: 3,
    38400: 4,
    57600: 5,
    115200: 6,
}

PRESET_LEVELS = {
    "pad": 3,         # SF8,  BW 406 kHz, CR 4/5 -> 10 kbps (~2.2 km LOS, factory default)
    "long-range": 1,  # SF11, BW 406 kHz, CR 4/8 ->  1 kbps (~3.7 km LOS max range)
    "fast": 6,        # SF6,  BW 1625 kHz, CR 4/6 -> 100 kbps
}


class ByteLink(Protocol):
    def read_some(self, timeout_s: float = 0.25) -> bytes: ...
    def write_all(self, data: bytes) -> None: ...
    def close(self) -> None: ...


class NativeSerialLink:
    """Zero-dependency cross-platform 8N1 serial port wrapper."""

    def __init__(self, port: str, baud: int) -> None:
        self.port = port
        self.baud = baud
        self._pyserial = None
        self._fd: Optional[int] = None
        self._win_handle: Optional[int] = None

        try:
            import serial  # type: ignore

            self._pyserial = serial.Serial(port=port, baudrate=baud, timeout=0.05)
            return
        except ImportError:
            pass

        if os.name == "nt":
            self._open_windows(port, baud)
        else:
            self._open_posix(port, baud)

    def _open_posix(self, port: str, baud: int) -> None:
        import termios

        speed = getattr(termios, f"B{baud}", None)
        if speed is None:
            raise ValueError(f"unsupported POSIX baud rate: {baud}")
        fd = os.open(port, os.O_RDWR | os.O_NOCTTY | os.O_NONBLOCK)
        attrs = termios.tcgetattr(fd)
        attrs[0] = 0
        attrs[1] = 0
        attrs[2] = termios.CS8 | termios.CREAD | termios.CLOCAL
        attrs[3] = 0
        attrs[4] = speed
        attrs[5] = speed
        attrs[6][termios.VMIN] = 0
        attrs[6][termios.VTIME] = 0
        termios.tcsetattr(fd, termios.TCSANOW, attrs)
        termios.tcflush(fd, termios.TCIOFLUSH)
        self._fd = fd

    def _open_windows(self, port: str, baud: int) -> None:
        import ctypes
        from ctypes import wintypes

        k32 = ctypes.windll.kernel32
        path = port if port.startswith("\\\\.\\") else f"\\\\.\\{port}"
        handle = k32.CreateFileW(path, 0xC0000000, 0, None, 3, 0, None)
        if handle in (-1, 0xFFFFFFFF, 0xFFFFFFFFFFFFFFFF):
            raise OSError(f"failed to open {port}")

        class DCB(ctypes.Structure):
            _fields_ = [
                ("DCBlength", wintypes.DWORD),
                ("BaudRate", wintypes.DWORD),
                ("fFlags", wintypes.DWORD),
                ("wReserved", wintypes.WORD),
                ("XonLim", wintypes.WORD),
                ("XoffLim", wintypes.WORD),
                ("ByteSize", ctypes.c_ubyte),
                ("Parity", ctypes.c_ubyte),
                ("StopBits", ctypes.c_ubyte),
                ("XonChar", ctypes.c_char),
                ("XoffChar", ctypes.c_char),
                ("ErrorChar", ctypes.c_char),
                ("EofChar", ctypes.c_char),
                ("EvtChar", ctypes.c_char),
                ("wReserved1", wintypes.WORD),
            ]

        class COMMTIMEOUTS(ctypes.Structure):
            _fields_ = [
                ("ReadIntervalTimeout", wintypes.DWORD),
                ("ReadTotalTimeoutMultiplier", wintypes.DWORD),
                ("ReadTotalTimeoutConstant", wintypes.DWORD),
                ("WriteTotalTimeoutMultiplier", wintypes.DWORD),
                ("WriteTotalTimeoutConstant", wintypes.DWORD),
            ]

        dcb = DCB()
        dcb.DCBlength = ctypes.sizeof(DCB)
        if not k32.GetCommState(handle, ctypes.byref(dcb)):
            k32.CloseHandle(handle)
            raise OSError("GetCommState failed")
        dcb.BaudRate = baud
        dcb.fFlags = 0x0001  # fBinary
        dcb.ByteSize = 8
        dcb.Parity = 0
        dcb.StopBits = 0
        if not k32.SetCommState(handle, ctypes.byref(dcb)):
            k32.CloseHandle(handle)
            raise OSError("SetCommState failed")
        timeouts = COMMTIMEOUTS(50, 0, 50, 0, 200)
        k32.SetCommTimeouts(handle, ctypes.byref(timeouts))
        self._win_handle = handle

    def read_some(self, timeout_s: float = 0.25) -> bytes:
        deadline = time.monotonic() + timeout_s
        chunks = bytearray()
        while time.monotonic() < deadline:
            chunk = self._read_once()
            if chunk:
                chunks.extend(chunk)
                # Keep draining trailing bytes in the same burst.
                deadline = min(deadline, time.monotonic() + 0.06)
            else:
                time.sleep(0.01)
        return bytes(chunks)

    def _read_once(self) -> bytes:
        if self._pyserial is not None:
            return self._pyserial.read(256)
        if self._fd is not None:
            import select

            r, _, _ = select.select([self._fd], [], [], 0.02)
            if not r:
                return b""
            try:
                return os.read(self._fd, 256)
            except BlockingIOError:
                return b""
        if self._win_handle is not None:
            import ctypes
            from ctypes import wintypes

            buf = ctypes.create_string_buffer(256)
            got = wintypes.DWORD(0)
            ctypes.windll.kernel32.ReadFile(
                self._win_handle, buf, 256, ctypes.byref(got), None
            )
            return buf.raw[: got.value]
        return b""

    def write_all(self, data: bytes) -> None:
        if self._pyserial is not None:
            self._pyserial.write(data)
            self._pyserial.flush()
            return
        if self._fd is not None:
            view = memoryview(data)
            while view:
                sent = os.write(self._fd, view)
                view = view[sent:]
            return
        if self._win_handle is not None:
            import ctypes
            from ctypes import wintypes

            sent = wintypes.DWORD(0)
            ctypes.windll.kernel32.WriteFile(
                self._win_handle, data, len(data), ctypes.byref(sent), None
            )

    def close(self) -> None:
        if self._pyserial is not None:
            self._pyserial.close()
            self._pyserial = None
        if self._fd is not None:
            os.close(self._fd)
            self._fd = None
        if self._win_handle is not None:
            import ctypes

            ctypes.windll.kernel32.CloseHandle(self._win_handle)
            self._win_handle = None


def channel_to_mhz(channel: int) -> int:
    if not (0 <= channel <= 0x32):
        raise ValueError(f"channel must be 0x00..=0x32 (got 0x{channel:02X})")
    return 2400 + channel * 2


def parse_channel(raw: str) -> int:
    cleaned = raw.strip().lower()
    if cleaned.endswith("mhz"):
        mhz = int(cleaned[:-3].strip(), 10)
        if mhz < 2400 or mhz > 2500 or (mhz - 2400) % 2 != 0:
            raise ValueError(f"frequency {mhz} MHz must be an even integer in 2400..=2500 MHz")
        return (mhz - 2400) // 2
    value = int(cleaned, 16) if cleaned.startswith("0x") or any(c in "abcdef" for c in cleaned) else int(cleaned, 16)
    if not (0 <= value <= 0x32):
        raise ValueError(f"hex channel '{raw}' out of range (expected 00..=32)")
    return value


def enter_at_mode(link: ByteLink, settle_s: float = 0.08) -> str:
    """Ensure the DX-LR42-24T12D is in AT command mode and answers `AT\\r\\n -> OK`."""
    _ = link.read_some(settle_s)
    link.write_all(b"AT\r\n")
    probe = link.read_some(0.20).decode("ascii", errors="replace")
    if "OK" in probe and "ERROR=" not in probe:
        return probe.strip()

    time.sleep(settle_s)
    link.write_all(b"+++")
    resp = link.read_some(0.35).decode("ascii", errors="replace")
    if "Exit AT" in resp:
        # Was in AT mode or resetting; wait for "Power On" and re-enter cleanly.
        time.sleep(settle_s)
        link.write_all(b"+++")
        resp = link.read_some(0.35).decode("ascii", errors="replace")

    link.write_all(b"AT\r\n")
    verify = link.read_some(0.25).decode("ascii", errors="replace")
    if "OK" not in verify:
        raise RuntimeError(
            f"DX-LR42-24T12D did not answer AT command check (got {resp!r} / {verify!r}). "
            "Verify 3.3 V UART TX/RX crossover, common GND, baud rate, and if AT+SWITCH=1 set M0=HIGH, M1=LOW."
        )
    return (resp + verify).strip()


def send_at(link: ByteLink, cmd: str, timeout_s: float = 0.30) -> str:
    link.write_all(f"{cmd}\r\n".encode("ascii"))
    raw = link.read_some(timeout_s).decode("ascii", errors="replace").strip()
    if "ERROR=" in raw or ("OK" not in raw and not cmd.startswith("AT+ERSSI")):
        raise RuntimeError(f"command {cmd!r} failed: {raw!r}")
    return raw


def inspect_module(link: ByteLink) -> dict[str, str]:
    fields: dict[str, str] = {}
    for name in (
        "VERSION",
        "BAUD",
        "PARI",
        "MODE",
        "SLEEP",
        "SWITCH",
        "LEVEL",
        "CHANNEL",
        "POWE",
        "PACKET",
        "DRSSI",
        "OPENKEY",
        "LBT",
        "ERSSI",
    ):
        fields[name] = send_at(link, f"AT+{name}")
    return fields


def provision_module(
    link: ByteLink,
    *,
    level: int = 3,
    channel: int = 0x0C,
    power_dbm: int = 13,
    key: Optional[int] = None,
    lock_pins: bool = False,
    baud: int = 9600,
) -> list[tuple[str, str]]:
    if baud not in BAUD_CODES:
        raise ValueError(f"unsupported DX-LR42 baud {baud}")
    if not (0 <= level <= 8):
        raise ValueError(f"level must be 0..=8 (got {level})")
    if not (0 <= channel <= 0x32):
        raise ValueError(f"channel must be 0x00..=0x32 (got 0x{channel:02X})")
    if not (0 <= power_dbm <= 13):
        raise ValueError(f"power_dbm must be 0..=13 (got {power_dbm})")
    if key is not None and not (0 <= key <= 65535):
        raise ValueError(f"key must be 0..=65535 (got {key})")

    commands = [
        f"AT+BAUD{BAUD_CODES[baud]}",
        "AT+PARI0",
        "AT+MODE0",
        "AT+SLEEP2",
        f"AT+LEVEL{level}",
        f"AT+CHANNEL{channel:02X}",
        f"AT+POWE{power_dbm}",
        "AT+PACKET3",
        "AT+DRSSI0",
        "AT+LBT0",
        f"AT+SWITCH{1 if lock_pins else 0}",
    ]
    if level == 8:
        # Manual section 5.3.8 note 4: OPENKEY is forced off in FLRC mode (LEVEL 8).
        pass
    elif key is not None:
        commands.append("AT+OPENKEY1")
        commands.append(f"AT+KEY{key}")

    transcript: list[tuple[str, str]] = []
    for cmd in commands:
        transcript.append((cmd, send_at(link, cmd)))
    transcript.append(("AT+RESET", send_at(link, "AT+RESET", timeout_s=0.45)))
    return transcript


class _MockDxLr42:
    """In-memory simulator of the official DX-LR42-24T12D AT firmware for `--selftest`."""

    def __init__(self) -> None:
        self.in_at = False
        self.rx = bytearray()
        self.state = {
            "VERSION": "+VERSION=DX-LR42-V2.0",
            "BAUD": "2",
            "PARI": "0",
            "MODE": "0",
            "SLEEP": "2",
            "SWITCH": "0",
            "LEVEL": "3",
            "CHANNEL": "0C",
            "POWE": "13",
            "PACKET": "3",
            "DRSSI": "0",
            "OPENKEY": "1",
            "KEY": "12345",
            "LBT": "0",
            "ERSSI": "-106dBm",
        }

    def read_some(self, timeout_s: float = 0.01) -> bytes:
        out = bytes(self.rx)
        self.rx.clear()
        return out

    def close(self) -> None:
        pass

    def write_all(self, data: bytes) -> None:
        text = data.decode("ascii")
        if text == "+++":
            if not self.in_at:
                self.in_at = True
                self.rx.extend(b"Entry AT\r\n")
            else:
                self.in_at = False
                self.rx.extend(b"Exit AT\r\nPower On\r\n")
            return
        if not self.in_at:
            return
        line = text.strip()
        if line == "AT":
            self.rx.extend(b"OK\r\n")
            return
        if line == "AT+RESET":
            self.in_at = False
            self.rx.extend(b"OK\r\nPower On\r\n")
            return
        if line == "AT+DEFAULT":
            self.rx.extend(b"OK\r\n")
            return
        for key in list(self.state.keys()):
            prefix = f"AT+{key}"
            if line == prefix:
                if key == "VERSION":
                    self.rx.extend(f"{self.state[key]}\r\nOK\r\n".encode("ascii"))
                else:
                    self.rx.extend(f"+{key}={self.state[key]}\r\nOK\r\n".encode("ascii"))
                return
            if line.startswith(prefix):
                val = line[len(prefix) :]
                self.state[key] = val
                self.rx.extend(f"+{key}={val}\r\nOK\r\n".encode("ascii"))
                return
        self.rx.extend(b"ERROR=104\r\n")


def run_selftest() -> None:
    assert channel_to_mhz(0x00) == 2400
    assert channel_to_mhz(0x0C) == 2424
    assert channel_to_mhz(0x32) == 2500
    assert parse_channel("0C") == 0x0C
    assert parse_channel("2440MHz") == 0x14

    mock = _MockDxLr42()
    enter_at_mode(mock, settle_s=0.0)
    info = inspect_module(mock)
    assert "+CHANNEL=0C" in info["CHANNEL"], info

    log = provision_module(
        mock,
        level=PRESET_LEVELS["long-range"],
        channel=0x14,
        power_dbm=13,
        key=27182,
        lock_pins=True,
        baud=9600,
    )
    assert log[-1][0] == "AT+RESET"
    assert not mock.in_at, "AT+RESET must return module to transparent mode"
    assert mock.state["LEVEL"] == "1"
    assert mock.state["CHANNEL"] == "14"
    assert mock.state["SWITCH"] == "1"
    assert mock.state["KEY"] == "27182"
    print("configure_dx_lr42.py selftest: OK")


def main() -> int:
    parser = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    parser.add_argument("--selftest", action="store_true", help="Run offline AT protocol self-test and exit")
    parser.add_argument("--port", help="3.3 V USB-UART serial device (e.g. /dev/ttyUSB0, COM6)")
    parser.add_argument("--current-baud", type=int, default=9600, help="Current module UART baud (default: 9600)")
    parser.add_argument("--inspect", action="store_true", help="Query and print current module settings only")
    parser.add_argument("--factory-reset", action="store_true", help="Send AT+DEFAULT followed by AT+RESET")
    parser.add_argument(
        "--preset",
        choices=sorted(PRESET_LEVELS.keys()),
        default="pad",
        help="Air-rate preset: 'pad' = LEVEL 3 (10 kbps, ~2.2 km), 'long-range' = LEVEL 1 (1 kbps, ~3.7 km)",
    )
    parser.add_argument("--level", type=int, help="Explicit AT+LEVEL override (0..8)")
    parser.add_argument(
        "--channel",
        default="0C",
        help="Hex channel 00..32 or frequency like 2424MHz (default: 0C = 2424 MHz)",
    )
    parser.add_argument("--power", type=int, default=13, help="TX power in dBm, 0..13 (default: 13)")
    parser.add_argument("--key", type=int, help="Optional 16-bit network key 0..65535 (must match on both boards)")
    parser.add_argument(
        "--lock-pins",
        action="store_true",
        help="Set AT+SWITCH1 so NUCLEO PG12/PG13 (M0=0, M1=0) lock transparent mode in hardware",
    )

    args = parser.parse_args()
    if args.selftest:
        run_selftest()
        return 0
    if not args.port:
        parser.error("--port is required unless --selftest is specified")

    channel = parse_channel(args.channel)
    level = args.level if args.level is not None else PRESET_LEVELS[args.preset]

    link = NativeSerialLink(args.port, args.current_baud)
    try:
        banner = enter_at_mode(link)
        print(f"[AT] connected on {args.port} @ {args.current_baud} Bd ({banner})")

        if args.factory_reset:
            print("[AT] restoring factory defaults:", send_at(link, "AT+DEFAULT"))
            print("[AT] rebooting module:", send_at(link, "AT+RESET", timeout_s=0.45))
            return 0

        if args.inspect:
            for k, v in inspect_module(link).items():
                line = " / ".join(s.strip() for s in v.splitlines() if s.strip())
                print(f"  {k:<8} {line}")
            # Exit AT mode cleanly back to transparent mode.
            link.write_all(b"+++")
            _ = link.read_some(0.35)
            return 0

        transcript = provision_module(
            link,
            level=level,
            channel=channel,
            power_dbm=args.power,
            key=args.key,
            lock_pins=args.lock_pins,
            baud=9600,
        )
        for cmd, reply in transcript:
            line = " / ".join(s.strip() for s in reply.splitlines() if s.strip())
            print(f"  {cmd:<14} -> {line}")
        print(
            f"[OK] DX-LR42-24T12D provisioned for GSE transparent mode "
            f"(9600 8N1, {channel_to_mhz(channel)} MHz [0x{channel:02X}], LEVEL {level}, +{args.power} dBm, "
            f"SWITCH={1 if args.lock_pins else 0})."
        )
        return 0
    finally:
        link.close()


if __name__ == "__main__":
    sys.exit(main())
