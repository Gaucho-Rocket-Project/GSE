#!/usr/bin/env python3
"""
doctor.py - figure out why the NUCLEO console will not open, and say so plainly

`screen /dev/ttyACM1 115200` followed by "[screen is terminating]" is not an
error message: screen prints the real reason on the line above and then quits.
The usual reasons are

  * the port is held by another program        -> "Cannot open line ... busy"
  * you are not in the dialout/uucp group     -> "... Permission denied"
  * the device node is stale (unplugged)      -> "... No such file or directory"
  * screen was started without a real terminal-> "Must be connected to a terminal"
  * something is stealing the port (brltty, ModemManager, a stale screen)

This script checks all of them on the actual machine and prints a verdict per
port, so you do not have to guess.

Usage
    doctor.py                  # full report
    doctor.py /dev/ttyACM1     # focus on one port, report the exact errno
"""

import errno
import glob
import os
import pwd
import shutil
import subprocess
import sys

ST_VID = "0483"

ERRNO_HELP = {
    errno.EACCES: ("Permission denied",
                   "add yourself to the serial group: sudo usermod -aG dialout $USER "
                   "(uucp on Arch) - then log out and back in"),
    errno.EPERM: ("Operation not permitted",
                  "something is enforcing access (container, snap, SELinux/AppArmor); "
                  "check dmesg and try a plain shell outside the sandbox"),
    errno.EBUSY: ("Device or resource busy",
                  "another process owns the port - see the 'held by' line"),
    errno.ENOENT: ("No such file or directory",
                   "the device node is gone: the board was unplugged, or the USB "
                   "device dropped off the bus (dmesg | tail)"),
    errno.ENODEV: ("No such device",
                   "the USB device disappeared mid-open - cable/power, or a driver "
                   "reset. Replug and check dmesg"),
    errno.EIO: ("Input/output error",
                "the ST-LINK dropped the link mid-open - try another cable/port"),
    errno.ENOTTY: ("Inappropriate ioctl for device",
                   "that path is not a serial port (wrong node?)"),
    errno.ENXIO: ("No such device or address",
                  "the CDC interface is not bound any more - replug the board"),
}

OK = "  OK  "
BAD = " FAIL "
WARN = " WARN "


def say(tag, text):
    """tag: OK/BAD/WARN, or '' for a continuation line."""
    if not tag:
        print("         %s" % text)
    elif tag.strip() == "":
        print("         %s" % text)
    else:
        print("[%s] %s" % (tag, text))


def bar(title):
    print("\n" + title)
    print("-" * len(title))


def run(cmd, timeout=5):
    try:
        r = subprocess.run(cmd, capture_output=True, text=True, timeout=timeout)
        return r.returncode, (r.stdout or "") + (r.stderr or "")
    except (OSError, subprocess.SubprocessError):
        return 127, ""


# --------------------------------------------------------------------------- #
# environment
# --------------------------------------------------------------------------- #
def check_terminal():
    bar("1. Local terminal")
    if sys.stdin.isatty():
        say(OK, "stdin is a terminal (screen/PuTTY-style programs can attach)")
    else:
        say(WARN, "stdin is NOT a terminal - `screen` refuses to start there")
        say("", "      ('Must be connected to a terminal' then it terminates)")
        say("", "      Run the monitor from a normal shell window, or use")
        say("", "      `make sniff SECS=3`, which needs no terminal at all.")
    term = os.environ.get("TERM", "<unset>")
    if term in ("<unset>", "", "dumb"):
        say(WARN, "TERM=%s - screen needs a real terminfo entry" % term)
        say("", "      try: TERM=xterm-256color screen /dev/ttyACM1 115200")
    else:
        say(OK, "TERM=%s" % term)


def check_groups():
    if not sys.platform.startswith("linux"):
        return
    bar("2. Permissions")
    try:
        user = pwd.getpwuid(os.getuid()).pw_name
        groups = {g.gr_name for g in __import__("grp").getgrall()
                  if user in g.gr_mem or g.gr_gid in os.getgroups()}
    except (KeyError, OSError):
        user, groups = "?", set()

    serial_groups = {"dialout", "uucp", "serial", "tty"}
    mine = groups & serial_groups
    if mine:
        say(OK, "user '%s' is in %s" % (user, ", ".join(sorted(mine))))
    else:
        say(BAD, "user '%s' is not in dialout/uucp" % user)
        say("", "      sudo usermod -aG dialout %s   (then log out and back in)" % user)


def check_competitors():
    bar("3. Programs known to steal ttyACM ports")
    problems = 0

    # brltty: treats any CDC-ACM device as a braille display and detaches it.
    if shutil.which("brltty"):
        rc, out = run(["systemctl", "is-active", "brltty"])
        if out.strip() == "active":
            say(BAD, "brltty is running - it grabs CDC-ACM devices as braille displays")
            say("", "      sudo systemctl stop brltty && sudo systemctl disable brltty")
            problems += 1
        else:
            say(OK, "brltty installed but not active")
    else:
        say(OK, "brltty not installed")

    if os.path.exists("/usr/lib/systemd/system/ModemManager.service"):
        rc, out = run(["systemctl", "is-active", "ModemManager"])
        if out.strip() == "active":
            say(WARN, "ModemManager is running - it probes ttyACM ports")
            say("", "      sudo systemctl stop ModemManager")
            problems += 1
        else:
            say(OK, "ModemManager not active")
    else:
        say(OK, "ModemManager not installed")

    rc, out = run(["screen", "-ls"])
    stale = []
    for ln in out.splitlines():
        low = ln.lower()
        if "(" in low and ("detached" in low or "attached" in low or "dead" in low):
            stale.append(ln.strip())
    if stale:
        say(BAD, "screen sessions are still running (they hold serial ports):")
        for ln in stale:
            say("", "      " + ln)
        say("", "      killall screen    # or: screen -S <name> -X quit")
        problems += 1
    else:
        say(OK, "no existing screen sessions")

    return problems


# --------------------------------------------------------------------------- #
# ports
# --------------------------------------------------------------------------- #
def candidates():
    out = []
    for pat in ("/dev/ttyACM*", "/dev/ttyUSB*", "/dev/cu.usbmodem*",
                "/dev/cu.usbserial*", "/dev/serial/by-id/*"):
        out.extend(sorted(glob.glob(pat)))
    seen, uniq = set(), []
    for p in out:
        rp = os.path.realpath(p)
        if rp not in seen:
            seen.add(rp)
            uniq.append(p)
    return uniq


def usb_info(port):
    """(vid, pid, product, manufacturer, serial, driver) from sysfs."""
    info = dict(vid="", pid="", product="", manufacturer="", serial="", driver="")
    node = os.path.realpath(port)
    for _ in range(8):
        if os.path.exists(os.path.join(node, "idVendor")):
            for key, fname in (("vid", "idVendor"), ("pid", "idProduct"),
                               ("product", "product"),
                               ("manufacturer", "manufacturer"),
                               ("serial", "serial")):
                p = os.path.join(node, fname)
                if os.path.exists(p):
                    try:
                        info[key] = open(p).read().strip()
                    except OSError:
                        pass
            break
        parent = os.path.dirname(node)
        if parent == node:
            break
        node = parent
    try:
        info["driver"] = os.path.basename(
            os.readlink("/sys/class/tty/%s/device/driver" % os.path.basename(port)))
    except OSError:
        pass
    return info


def holders(port):
    """Processes with @p port open (Linux /proc first, then lsof/fuser)."""
    names = []
    target = os.path.realpath(port)
    if os.path.isdir("/proc"):
        for pid in os.listdir("/proc"):
            if not pid.isdigit():
                continue
            fddir = "/proc/%s/fd" % pid
            try:
                fds = os.listdir(fddir)
            except OSError:
                continue
            for fd in fds:
                try:
                    link = os.readlink(os.path.join(fddir, fd))
                except OSError:
                    continue
                if link in (port, target):
                    comm = pid
                    try:
                        comm = open("/proc/%s/comm" % pid).read().strip()
                    except OSError:
                        pass
                    names.append("%s (pid %s)" % (comm, pid))
                    break
    if not names:
        rc, out = run(["lsof", "-Fpc", port])
        procs = [ln[1:] for ln in out.splitlines() if ln.startswith("c") and len(ln) > 1]
        pids = [ln[1:] for ln in out.splitlines() if ln.startswith("p") and len(ln) > 1]
        if procs or pids:
            names.append("%s %s" % (", ".join(sorted(set(procs))),
                                    ("pid " + ",".join(pids)).strip()))
    return ", ".join(names) if names else None


def try_open(port):
    """Return (ok, errno, message)."""
    try:
        fd = os.open(port, os.O_RDWR | os.O_NOCTTY | os.O_NONBLOCK)
    except OSError as e:
        return False, e.errno, e.strerror
    try:
        import termios
        try:
            termios.tcgetattr(fd)
        except termios.error:
            return False, errno.ENOTTY, "not a tty"
    finally:
        os.close(fd)
    return True, 0, ""


def check_ports(focus=None):
    bar("4. Serial ports")
    ports = candidates()
    if focus and focus not in ports:
        ports = [focus] + ports

    if not ports:
        say(BAD, "no serial ports at all (/dev/ttyACM*, /dev/ttyUSB*, /dev/cu.usb*)")
        say("", "      the board is not enumerating: check the cable (data, not charge-only),")
        say("", "      the ST-LINK USB connector, and the CN4 jumpers. Then: dmesg | tail -20")
        return []

    stlink_ports = []
    for port in ports:
        info = usb_info(port)
        tag = "ST-LINK/NUCLEO" if info["vid"] == ST_VID else "other device"
        print()
        say("     ", "%s  [%s]" % (port, tag))
        if info["vid"]:
            say("     ", "  USB %s:%s  %s %s" % (info["vid"], info["pid"],
                                                  info["manufacturer"], info["product"]))
        if info["serial"]:
            say("     ", "  serial %s" % info["serial"])
        if info["driver"]:
            say("     ", "  driver %s" % info["driver"])

        held = holders(port)
        if held:
            say(BAD, "  held by: %s" % held)
        else:
            say(OK, "  not held by any process")

        ok, err, msg = try_open(port)
        if ok:
            say(OK, "  opens read/write")
        else:
            friendly, advice = ERRNO_HELP.get(err, (msg, ""))
            say(BAD, "  cannot open: %s (%s)" % (friendly, errno.errorcode.get(err, err)))
            if advice:
                say("     ", "  -> " + advice)

        if info["vid"] == ST_VID:
            stlink_ports.append(port)

    return stlink_ports


def check_kernel_log():
    if not sys.platform.startswith("linux"):
        return
    bar("5. Recent kernel messages about USB/serial")
    rc, out = run(["dmesg", "--level=err,warn,info"], timeout=6)
    if rc != 0:
        rc, out = run(["dmesg"], timeout=6)
    lines = [ln for ln in out.splitlines()
             if any(k in ln.lower() for k in ("ttyacm", "cdc_acm", "stlink", "usb ", "brltty"))]
    if not lines:
        say(WARN, "nothing relevant (or dmesg needs root: sudo dmesg | tail -20)")
        return
    for ln in lines[-8:]:
        print("        " + ln.strip())


def main():
    focus = None
    for a in sys.argv[1:]:
        if a in ("-h", "--help"):
            print(__doc__.strip())
            return 0
        if not a.startswith("-"):
            focus = a

    print("NUCLEO-H753ZI console doctor")
    print("=" * 28)
    print("host: %s %s, python %s" % (os.uname().sysname, os.uname().release,
                                     sys.version.split()[0]))

    check_terminal()
    check_groups()
    stolen = check_competitors()
    stlink = check_ports(focus)
    check_kernel_log()

    bar("Verdict")
    if stlink:
        say(OK, "NUCLEO ST-LINK port: %s" % stlink[0])
        say("", "next:  make monitor%s" % ("" if not focus else " PORT=" + focus))
    elif focus:
        say(BAD, "%s does not look like an ST-LINK (VID %s)" % (focus, ST_VID))
        say("", "It may still be the right port if you use a USB-UART adapter on the")
        say("", "morpho header; otherwise unplug other boards and re-run this script.")
    else:
        say(BAD, "no ST-LINK (VID %s) port found" % ST_VID)
    if stolen:
        say(WARN, "%d program(s) above may be interfering - fix those first" % stolen)
    print()
    return 0


if __name__ == "__main__":
    sys.exit(main())
