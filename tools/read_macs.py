#!/usr/bin/env python3
"""
read_macs.py -- read the MAC address of every connected ESP32 board and check
them against the ESP-NOW pairing in firmware/src/config.h.

    python tools/read_macs.py              # every ESP32-looking serial port
    python tools/read_macs.py COM5 COM6    # just these ports

For each port it runs esptool's `read_mac` (read-only: nothing is written to
the board; the board is briefly reset into its bootloader and back), then:

    ESP32 (classic, e.g. ESP32S / WROOM-32) -> ground board  -> GSE_NOW_TX_MAC
    ESP32-S3                                 -> vehicle board -> GSE_NOW_PAD_MAC

and says whether config.h already has those MACs.  If not, it prints the two
lines to paste into config.h (then rebuild and reflash BOTH boards).

Close the GUI / Serial Monitor first: a port another program holds cannot be
read.  Uses esptool from PlatformIO if it is installed, else `python -m esptool`
(pip install esptool).
"""
from __future__ import annotations

import os
import re
import shutil
import subprocess
import sys

HERE = os.path.dirname(os.path.abspath(__file__))
CONFIG_H = os.path.join(os.path.dirname(HERE), "firmware", "src", "config.h")

BOARD_HINTS = ("CP210", "CH340", "CH343", "CH910", "USB JTAG", "ESP32", "USB Serial",
               "CDC", "UART")


def esptool_cmd() -> list[str] | None:
    pio = shutil.which("pio") or shutil.which("platformio")
    if pio:
        return [pio, "pkg", "exec", "-p", "tool-esptoolpy", "--", "esptool.py"]
    try:
        import esptool  # noqa: F401
        return [sys.executable, "-m", "esptool"]
    except ImportError:
        return None


def candidate_ports() -> list[tuple[str, str]]:
    try:
        from serial.tools import list_ports
    except ImportError:
        sys.exit("pyserial is needed to list ports: pip install pyserial "
                 "(or pass the port names, e.g. python tools/read_macs.py COM5 COM6)")
    out = []
    for p in list_ports.comports():
        desc = f"{p.description or ''} {p.product or ''}".strip()
        if any(h.lower() in desc.lower() for h in BOARD_HINTS):
            out.append((p.device, desc))
    return out


def read_mac(cmd: list[str], port: str) -> tuple[str | None, str | None, str]:
    """Return (chip, mac, error)."""
    try:
        r = subprocess.run(cmd + ["--port", port, "read_mac"], capture_output=True,
                           text=True, timeout=60)
    except subprocess.TimeoutExpired:
        return None, None, "esptool timed out"
    text = r.stdout + r.stderr
    chip = (re.search(r"Chip (?:is|type):?\s*(ESP32[^\s(,]*)", text)
            or re.search(r"Detecting chip type\.\.\.\s*(ESP32[^\s(,]*)", text))
    mac = re.search(r"MAC:\s*([0-9A-Fa-f]{2}(?::[0-9A-Fa-f]{2}){5})", text)
    if mac:
        return (chip.group(1) if chip else None), mac.group(1).upper(), ""
    if "Access is denied" in text or "PermissionError" in text or "busy" in text.lower():
        return None, None, "port in use by another program (close the GUI / Serial Monitor)"
    last = [l for l in text.strip().splitlines() if l.strip()]
    return None, None, (last[-1] if last else f"esptool exit code {r.returncode}")


def role_of(chip: str | None) -> str | None:
    if not chip:
        return None
    c = chip.upper()
    if "S3" in c:
        return "vehicle"
    if c.startswith("ESP32") and not any(x in c for x in ("S2", "C3", "C6", "H2", "P4")):
        return "ground"
    return None


def config_macs() -> dict[str, str]:
    """GSE_NOW_PAD_MAC / GSE_NOW_TX_MAC as written in config.h."""
    try:
        text = open(CONFIG_H, encoding="utf-8").read()
    except OSError:
        return {}
    out = {}
    for key in ("GSE_NOW_PAD_MAC", "GSE_NOW_TX_MAC"):
        m = re.search(rf"#\s*define\s+{key}\s*\{{([^}}]*)\}}", text)
        if m:
            out[key] = ":".join(f"{int(x, 0):02X}" for x in m.group(1).split(","))
    return out


def c_bytes(mac: str) -> str:
    return "{ " + ", ".join(f"0x{b}" for b in mac.split(":")) + " }"


def main(argv: list[str]) -> int:
    cmd = esptool_cmd()
    if cmd is None:
        print("esptool not found: install PlatformIO, or `pip install esptool`.")
        return 2
    ports = [(p, "") for p in argv] if argv else candidate_ports()
    if not ports:
        print("No ESP32-looking serial ports found. Plug the boards in (data cables, "
              "not charge-only) or name the ports: python tools/read_macs.py COM5 COM6")
        return 1

    found: dict[str, tuple[str, str, str]] = {}      # role -> (port, chip, mac)
    print(f"{'port':<8} {'chip':<18} {'MAC':<18} role")
    for port, desc in ports:
        chip, mac, err = read_mac(cmd, port)
        if not mac:
            print(f"{port:<8} {'-':<18} {'-':<18} could not read: {err}"
                  + (f"   [{desc}]" if desc else ""))
            continue
        role = role_of(chip)
        print(f"{port:<8} {chip or '?':<18} {mac:<18} "
              f"{role + ' board' if role else 'unknown (not ESP32 / ESP32-S3)'}")
        if role:
            if role in found:
                print(f"         ! a second {role} board ({found[role][0]} and {port}); "
                      "using the first")
            else:
                found[role] = (port, chip or "?", mac)

    have = config_macs()
    print(f"\nconfig.h ({os.path.relpath(CONFIG_H)}):")
    want = {"GSE_NOW_TX_MAC": found.get("ground"), "GSE_NOW_PAD_MAC": found.get("vehicle")}
    mismatch = False
    for key, board in want.items():
        cur = have.get(key, "not found")
        who = "ground (ESP32S)" if key == "GSE_NOW_TX_MAC" else "vehicle (ESP32-S3)"
        if board is None:
            print(f"  {key:<16} = {cur:<18} ({who} not connected - not checked)")
        elif board[2] == cur:
            print(f"  {key:<16} = {cur:<18} matches the {who} on {board[0]}")
        else:
            mismatch = True
            print(f"  {key:<16} = {cur:<18} DOES NOT MATCH the {who} on {board[0]} ({board[2]})")

    if mismatch:
        print("\nPaste into firmware/src/config.h, then rebuild and reflash BOTH boards"
              "\n(and re-run tools/make_arduino_sketches.py if you flash from Arduino):")
        for key, board in want.items():
            if board is not None:
                print(f"#  define {key:<16} {c_bytes(board[2])}")
        return 3
    return 0


if __name__ == "__main__":
    sys.exit(main(sys.argv[1:]))
