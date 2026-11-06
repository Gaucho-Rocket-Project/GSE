#!/usr/bin/env python3
"""
monitor.py - console for the NUCLEO-H753ZI dual-servo firmware

Why not just `screen /dev/ttyACM0 115200`?

  * it sends a CR on connect, so a board already sitting at "servo>" prints a
    fresh banner + prompt instead of looking dead;
  * if nothing answers it says *what to check* (wrong port, wrong image, held by
    another program) instead of leaving you at a blank screen;
  * "Device or resource busy" is detected up front and reported with the name of
    the process holding the port, when the OS lets us find out;
  * raw mode, no local echo, CR/LF handled, Ctrl-C goes to the board,
    Ctrl-] disconnects.

Usage
    monitor.py                 # find the NUCLEO's ST-LINK port, check it, connect
    monitor.py /dev/ttyACM0    # explicit port
    monitor.py --list          # list ports with their USB IDs
    monitor.py --sniff 2       # just print whatever arrives for 2 s
    monitor.py --force         # connect even if the board seems silent
    monitor.py --send 'status\r' --sniff 1     # fire a command and dump the reply

Environment
    BAUD=115200                # console baud rate (must match board.h)
"""

import errno
import os
import select
import subprocess
import sys
import time

HERE = os.path.dirname(os.path.abspath(__file__))
BAUD = int(os.environ.get("BAUD", "115200"))
QUIT_KEY = 0x1D                   # Ctrl-]
_STDIN_ATTRS = None


# --------------------------------------------------------------------------- #
# port discovery
# --------------------------------------------------------------------------- #
def _find_port_script():
    return os.path.join(HERE, "find_port.py")


def find_port():
    """Best guess at the NUCLEO's ST-LINK VCP, or None."""
    try:
        r = subprocess.run([sys.executable, _find_port_script()],
                           capture_output=True, text=True, timeout=15)
        if r.returncode == 0:
            return r.stdout.strip() or None
    except (OSError, subprocess.SubprocessError):
        pass
    return None


def list_ports():
    try:
        subprocess.run([sys.executable, _find_port_script(), "--list"])
    except (OSError, subprocess.SubprocessError) as e:
        print("could not list ports:", e)


def tty_candidates():
    """Every serial-looking port on this OS (used when find_port.py fails)."""
    import glob
    pats = ["/dev/ttyACM*", "/dev/ttyUSB*", "/dev/cu.usbmodem*", "/dev/cu.usbserial*"]
    out = []
    for pat in pats:
        out.extend(sorted(glob.glob(pat)))
    return out


# --------------------------------------------------------------------------- #
# busy detection
# --------------------------------------------------------------------------- #
def exclusive_probe(port):
    """True if the port can be opened with O_EXCL right now.

    A hint, not a verdict: whether a second open trips EBUSY depends on the
    driver (USB CDC does, a plain pty does not). The definitive check is the
    open() itself, which is why main() also handles EBUSY at that point.
    """
    try:
        fd = os.open(port, os.O_RDWR | os.O_NOCTTY | os.O_NONBLOCK | os.O_EXCL)
    except OSError as e:
        if e.errno in (errno.EBUSY, errno.EAGAIN):
            return False
        if e.errno in (errno.EACCES, errno.EPERM):
            raise
        return True
    os.close(fd)
    return True


def linux_holders(port):
    """Linux: find every process that has @p port open, via /proc.

    More reliable than lsof (which may not be installed or permitted) and it
    can name the offender: "screen (pid 5192)". Returns a description or None.
    """
    if not os.path.isdir("/proc"):
        return None

    try:
        target = os.path.realpath(port)
    except OSError:
        return None

    found = []
    for pid in os.listdir("/proc"):
        if not pid.isdigit():
            continue
        fddir = "/proc/%s/fd" % pid
        try:
            entries = os.listdir(fddir)
        except OSError:
            continue                    # not ours / process gone
        for fd in entries:
            try:
                link = os.readlink(os.path.join(fddir, fd))
            except OSError:
                continue
            if link == port or link == target:
                name = pid
                try:
                    with open("/proc/%s/comm" % pid) as f:
                        name = f.read().strip()
                except OSError:
                    pass
                found.append("%s (pid %s)" % (name, pid))
                break
    return ", ".join(found) if found else None


def port_holder(port):
    """Best-effort 'who holds this port' description, or None."""
    holder = linux_holders(port)
    if holder:
        return holder

    try:
        r = subprocess.run(["lsof", "-F", "pc", port],
                           capture_output=True, text=True, timeout=5)
        if r.returncode == 0 and r.stdout.strip():
            procs, pids = [], []
            for line in r.stdout.splitlines():
                if line.startswith("c") and len(line) > 1:
                    procs.append(line[1:])
                elif line.startswith("p") and len(line) > 1:
                    pids.append(line[1:])
            bits = []
            if procs:
                bits.append(", ".join(sorted(set(procs))))
            if pids:
                bits.append("pid " + ", ".join(pids))
            if bits:
                return " / ".join(bits)
    except (OSError, subprocess.SubprocessError):
        pass

    try:
        r = subprocess.run(["fuser", port], capture_output=True, text=True, timeout=5)
        txt = (r.stdout + r.stderr).strip()
        if txt:
            return txt
    except (OSError, subprocess.SubprocessError):
        pass

    try:
        if not exclusive_probe(port):
            return "another process (install lsof to find out which)"
    except OSError:
        pass
    return None


def busy_advice(port):
    txt = ("\nOnly one program can own a serial port at a time.\n"
           "  - close any screen / minicom / PuTTY / Arduino IDE / CubeIDE session\n"
           "  - a stale screen session:   killall screen\n")
    if "acm" in port.lower() or "modem" in port.lower():
        txt += ("  - on Linux, ModemManager probes ttyACM ports and holds them:\n"
                "        sudo systemctl stop ModemManager      (or reboot)\n")
    txt += "  - then retry, or connect anyway with --force\n"
    return txt


# --------------------------------------------------------------------------- #
# tty handling
# --------------------------------------------------------------------------- #
def open_port(port):
    """Open the tty raw at BAUD 8N1. Returns an fd."""
    import termios

    fd = os.open(port, os.O_RDWR | os.O_NOCTTY | os.O_NONBLOCK)
    try:
        attrs = termios.tcgetattr(fd)
    except termios.error:
        # Not a tty (e.g. /dev/pts in a test harness): leave it alone.
        return fd

    iflag, oflag, cflag, lflag, _, _, cc = attrs
    iflag &= ~(termios.IGNBRK | termios.BRKINT | termios.PARMRK | termios.ISTRIP
               | termios.INLCR | termios.IGNCR | termios.ICRNL | termios.IXON
               | termios.IXOFF | termios.IXANY)
    oflag &= ~termios.OPOST
    lflag &= ~(termios.ECHO | termios.ECHONL | termios.ICANON | termios.ISIG
               | termios.IEXTEN)
    cflag &= ~(termios.CSIZE | termios.PARENB | termios.CSTOPB)
    cflag |= termios.CS8 | termios.CREAD | termios.CLOCAL

    speed = getattr(termios, "B%d" % BAUD, termios.B115200)
    cc[termios.VMIN] = 0
    cc[termios.VTIME] = 0
    termios.tcsetattr(fd, termios.TCSANOW,
                      [iflag, oflag, cflag, lflag, speed, speed, cc])
    termios.tcflush(fd, termios.TCIOFLUSH)
    return fd


def stdin_is_tty():
    return sys.stdin.isatty()


def restore_stdin():
    import termios
    try:
        termios.tcsetattr(sys.stdin.fileno(), termios.TCSADRAIN, _STDIN_ATTRS)
    except Exception:
        pass


def stdin_raw():
    """Raw mode for the local terminal; no-op when stdin is not a tty."""
    if not stdin_is_tty():
        return lambda *_: None

    import termios
    import tty
    global _STDIN_ATTRS
    _STDIN_ATTRS = termios.tcgetattr(sys.stdin.fileno())
    tty.setraw(sys.stdin.fileno())
    return restore_stdin


# --------------------------------------------------------------------------- #
# I/O helpers
# --------------------------------------------------------------------------- #
def sniff(port, seconds=1.5, nudge=True, pre_send=None):
    """Listen for a moment. Optionally send a CR (and pre_send) first."""
    fd = open_port(port)
    data = b""
    try:
        if pre_send:
            os.write(fd, pre_send)
        if nudge:
            os.write(fd, b"\r")
            time.sleep(0.05)
        end = time.time() + seconds
        while time.time() < end:
            r, _, _ = select.select([fd], [], [], 0.1)
            if not r:
                continue
            try:
                chunk = os.read(fd, 4096)
            except OSError as e:
                if e.errno in (errno.EAGAIN, errno.EWOULDBLOCK):
                    continue
                break
            data += chunk
    finally:
        os.close(fd)
    return data


def usage():
    print(__doc__.strip())
    return 2


def main():
    args = sys.argv[1:]
    if not args or args[0] in ("-h", "--help"):
        return usage()

    if args[0] in ("--list", "-l"):
        list_ports()
        return 0

    port = None
    force = False
    sniff_secs = None
    send = None
    i = 0
    while i < len(args):
        a = args[i]
        if a in ("--force", "-f"):
            force = True
        elif a in ("--sniff", "-s"):
            i += 1
            sniff_secs = float(args[i]) if i < len(args) else 2.0
            force = True
        elif a in ("--send", "-x"):
            i += 1
            if i < len(args):
                send = args[i].encode().decode("unicode_escape").encode("latin-1")
            force = True
        elif a.startswith("-"):
            print("unknown option:", a)
            return 2
        else:
            port = a
        i += 1

    # ---- pick the port ---------------------------------------------------
    if port is None:
        port = find_port()
    if port is None:
        cands = tty_candidates()
        print("No NUCLEO ST-LINK serial port was identified.\n")
        if cands:
            print("Serial ports that do exist (none of them report an ST-LINK):")
            list_ports()
            print("\nIf one of those IS your NUCLEO, name it explicitly:")
            print("    %s %s" % (os.path.basename(sys.argv[0]), cands[0]))
        else:
            print("No serial ports found at all.")
        print("""
Checklist
  1. USB cable goes to the ST-LINK port (the connector next to the ST-LINK
     logo), with a DATA cable - not a charge-only one.
  2. Both jumpers on CN4 ("ST-LINK") are fitted.
  3. Linux: you must be in the 'dialout' group (log out/in after adding).
  4. The board must be powered (the ST-LINK's green LED on).
""")
        return 1

    if not os.path.exists(port):
        print("Port %s does not exist." % port)
        list_ports()
        return 1

    # ---- one-shot modes ---------------------------------------------------
    if sniff_secs is not None:
        try:
            data = sniff(port, sniff_secs, pre_send=send)
        except OSError as e:
            if e.errno == errno.EBUSY:
                print("Port %s is busy: %s" % (port, port_holder(port) or "unknown holder"))
                print(busy_advice(port))
                return 1
            if e.errno in (errno.EACCES, errno.EPERM):
                print("Permission denied on %s." % port)
                print("Linux: sudo usermod -aG dialout $USER  (then log out and in)")
                return 1
            raise
        sys.stdout.write(data.decode("ascii", "replace"))
        print("\n--- %d bytes ---" % len(data))
        return 0 if data else 1

    if send is not None:
        sniff(port, 0.3, nudge=False, pre_send=send)
        return 0

    # ---- liveness check ---------------------------------------------------
    if not force:
        holder = port_holder(port)
        if holder:
            print("Port %s is already open by: %s" % (port, holder))
            print(busy_advice(port))
            return 1

        print("Checking %s for output ... " % port, end="", flush=True)
        try:
            data = sniff(port, 1.5)
        except OSError as e:
            if e.errno == errno.EBUSY:
                print("busy.")
                print("\nPort %s is held by another program (%s)."
                      % (port, port_holder(port) or "unknown"))
                print(busy_advice(port))
                return 1
            if e.errno in (errno.EACCES, errno.EPERM):
                print("permission denied.")
                print("\nLinux: sudo usermod -aG dialout $USER   (then log out and in)")
                return 1
            print("failed: %s" % e)
            return 1

        if not data:
            print("nothing.\n")
            print("The firmware did not answer a CR on %s. In order of likelihood:\n" % port)
            print("  1. This is not the NUCLEO's port (another board can own ttyACM0).")
            print("     Re-run with the right one, or check:   monitor.py --list")
            print("  2. The firmware is not flashed / was replaced by other code.")
            print("     Flash it:  make flash     (or drop prebuilt/*.bin on the board)")
            print("  3. The console baud was changed in board.h; this terminal is %d." % BAUD)
            print("     Override:  BAUD=9600 %s %s" % (os.path.basename(sys.argv[0]), port))
            print("  4. A debugger has the core halted at a breakpoint, or the ST-LINK")
            print("     jumpers on CN4 are missing.")
            print("\nConnect anyway:   %s %s --force" % (os.path.basename(sys.argv[0]), port))
            return 1
        print("got %d bytes.\n" % len(data))
        sys.stdout.write(data.decode("ascii", "replace"))
        sys.stdout.write("\n")

    # ---- live console -----------------------------------------------------
    try:
        fd = open_port(port)
    except OSError as e:
        if e.errno == errno.EBUSY:
            print("Port %s is busy: %s" % (port, port_holder(port) or "unknown holder"))
            print(busy_advice(port))
        else:
            print("cannot open %s: %s" % (port, e))
        return 1

    interactive = stdin_is_tty()
    cleanup = stdin_raw()
    if interactive:
        print("Connected to %s @ %d 8N1 -- Ctrl-] quits, Ctrl-C goes to the board"
              % (port, BAUD))
    else:
        print("Connected to %s @ %d 8N1 -- stdin is not a terminal, streaming only"
              % (port, BAUD))
    print("-" * 70)
    sys.stdout.flush()
    os.write(fd, b"\r")                     # fresh prompt

    watch = [fd, sys.stdin] if interactive else [fd]
    try:
        while True:
            r, _, _ = select.select(watch, [], [], 0.2)
            if fd in r:
                try:
                    chunk = os.read(fd, 4096)
                except OSError as e:
                    if e.errno in (errno.EAGAIN, errno.EWOULDBLOCK):
                        continue
                    print("\n[serial error: %s]" % e)
                    break
                if chunk:
                    sys.stdout.write(chunk.decode("ascii", "replace"))
                    sys.stdout.flush()
            if interactive and sys.stdin in r:
                try:
                    data = os.read(sys.stdin.fileno(), 1024)
                except OSError:
                    data = b""
                if not data:
                    time.sleep(0.01)
                    continue
                if QUIT_KEY in data:
                    break
                os.write(fd, data)
    except KeyboardInterrupt:
        pass
    finally:
        cleanup()
        try:
            os.close(fd)
        except OSError:
            pass
        print("\n\n[disconnected from %s]" % port)
    return 0


if __name__ == "__main__":
    sys.exit(main())
