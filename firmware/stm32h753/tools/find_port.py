#!/usr/bin/env python3
"""
find_port.py - locate the NUCLEO-H753ZI console port (ST-LINK virtual COM port)

Unlike a generic tty scan this one asks the operating system *which USB device*
the port belongs to, because "a ttyACM port exists" does not mean it is the
NUCLEO:

  * the ESP32-S3 board in this repo also enumerates as /dev/ttyACM0
    (USB-Serial-JTAG) - picking that port is exactly how you end up staring at
    a silent console;
  * on Linux the NUCLEO's ST-LINK shows up as 0483:3748 / 0483:3754 (VID 0483 =
    STMicroelectronics, product string "STLink Virtual COM Port");
  * on macOS the port name itself carries the VID:PID, e.g.
    /dev/cu.usbmodem143303 -> 0483:3748.

Exit status: 0 if a ST-LINK port was found, 1 otherwise (any other ttyACM
ports are still printed first, so the caller can fall back).

Usage:
    find_port.py            # print the best port (one line, no commentary)
    find_port.py --list     # print every candidate with its USB IDs
    find_port.py --all      # print all ports, best first
"""

import glob
import os
import re
import subprocess
import sys

ST_VID = "0483"
STLINK_PIDS = {"3748", "374b", "374e", "374f", "3753", "3754", "3744"}

# Product strings that mean "this is the NUCLEO's ST-LINK", used when the
# sysfs metadata is missing (containers, WSL, older macOS).
STLINK_HINTS = ("stlink", "st-link", "stm32", "nucleo")


def linux_ports():
    """[(port, vid, pid, product)] for every ttyACM/ttyUSB port on Linux."""
    out = []
    for sysdev in sorted(glob.glob("/sys/class/tty/ttyACM*")
                         + glob.glob("/sys/class/tty/ttyUSB*")):
        name = os.path.basename(sysdev)
        port = "/dev/" + name
        vid = pid = product = ""
        # walk up to the USB device directory that holds idVendor
        node = os.path.realpath(sysdev)
        for _ in range(6):
            node = os.path.dirname(node)
            if os.path.exists(os.path.join(node, "idVendor")):
                try:
                    vid = open(os.path.join(node, "idVendor")).read().strip()
                    pid = open(os.path.join(node, "idProduct")).read().strip()
                except OSError:
                    pass
                for cand in ("product", "interface"):
                    p = os.path.join(node, cand)
                    if os.path.exists(p):
                        try:
                            product = open(p).read().strip()
                        except OSError:
                            pass
                        if product:
                            break
                break
        out.append((port, vid, pid, product))

    if not out:
        # No sysfs? fall back to a plain glob.
        for port in sorted(glob.glob("/dev/ttyACM*") + glob.glob("/dev/ttyUSB*")):
            out.append((port, "", "", ""))
    return out


def macos_ports():
    """Ports of the form /dev/cu.usbmodem<loc><vid><pid>."""
    out = []
    for port in sorted(glob.glob("/dev/cu.usbmodem*") + glob.glob("/dev/cu.usbserial*")):
        vid = pid = ""
        m = re.search(r"(?:usbmodem|usbserial)[0-9A-Fa-f]*(0483)([0-9A-Fa-f]{4})", port)
        if m:
            vid, pid = m.group(1).lower(), m.group(2).lower()
        out.append((port, vid, pid, ""))
    return out


def windows_ports():
    """COMx via the registry / wmic is fragile; just list what pyserial sees."""
    try:
        import serial.tools.list_ports as lp
    except ImportError:
        return []
    out = []
    for p in lp.comports():
        out.append((p.device,
                    "%04x" % p.vid if p.vid else "",
                    "%04x" % p.pid if p.pid else "",
                    p.description or ""))
    return out


def collect():
    if sys.platform.startswith("linux"):
        return linux_ports()
    if sys.platform == "darwin":
        return macos_ports()
    if os.name == "nt":
        return windows_ports()
    return []


def score(entry):
    """Higher is better."""
    port, vid, pid, product = entry
    s = 0
    if vid == ST_VID:
        s += 100
        if pid in STLINK_PIDS:
            s += 50
    low = product.lower()
    if any(h in low for h in STLINK_HINTS):
        s += 40
    if "ttyusb" in port.lower():
        s += 5          # ST-LINK never uses a UART bridge; ESP32 clones do
    if "ttyacm" in port.lower() or "usbmodem" in port.lower():
        s += 10
    return s


def main():
    args = sys.argv[1:]
    list_all = "--list" in args
    show_all = "--all" in args or list_all

    ports = collect()
    if not ports:
        if list_all:
            print("no serial ports found")
        return 0

    ranked = sorted(ports, key=score, reverse=True)
    best = ranked[0]
    found_stlink = score(best) >= 100

    if list_all:
        for port, vid, pid, product in ranked:
            usb = ("%s:%s" % (vid, pid)) if vid else "usb id unknown"
            tag = "ST-LINK / NUCLEO" if score((port, vid, pid, product)) >= 100 else "other device"
            print("  %-22s %-10s %-28s %s" % (port, usb, product[:28], tag))
        if not found_stlink:
            print("\nNone of these report an ST-LINK (VID 0483), so this is not the")
            print("NUCLEO's console port. Pass the right device explicitly if you are")
            print("using a USB-UART adapter on the morpho header instead.")
        return 0        # a listing is informational, never an error

    if show_all:
        for port, _, _, _ in ranked:
            print(port)
        return 0 if found_stlink else 1

    if found_stlink:
        print(best[0])
        return 0

    # Nothing that looks like an ST-LINK: refuse to guess by default, because
    # guessing is what opens the wrong board.
    return 1


if __name__ == "__main__":
    sys.exit(main())
