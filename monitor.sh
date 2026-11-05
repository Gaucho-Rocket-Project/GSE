#!/usr/bin/env bash
# ---------------------------------------------------------------------------
# monitor.sh - find the ESP32-S3 serial port, sanity-check it, open the console
#
#   ./monitor.sh                 # auto-detect, check for output, then open
#   ./monitor.sh --list          # only list the ports that exist
#   ./monitor.sh /dev/ttyUSB0    # use a specific port
#   ./monitor.sh /dev/ttyUSB0 --force   # skip the "is anything alive" check
#
# Exit from the console:  Ctrl-A then K   (screen)
#                         Ctrl-]          (miniterm / picocom: Ctrl-A X)
# ---------------------------------------------------------------------------
set -uo pipefail

BAUD="${BAUD:-115200}"
LIST_ONLY=0
FORCE=0
WANT=""

for arg in "$@"; do
    case "$arg" in
        --list|-l) LIST_ONLY=1 ;;
        --force|-f) FORCE=1 ;;
        -*) echo "unknown option: $arg"; exit 2 ;;
        *)  WANT="$arg" ;;
    esac
done

OS="$(uname -s)"

# --- collect candidate ports -------------------------------------------------
find_ports() {
    case "$OS" in
        Linux)
            for p in /dev/ttyACM* /dev/ttyUSB*; do [ -e "$p" ] && printf '%s\n' "$p"; done
            ;;
        Darwin)
            for p in /dev/cu.usbmodem* /dev/cu.usbserial* \
                     /dev/cu.wchusbserial* /dev/cu.SLAB_USBtoUART*; do
                [ -e "$p" ] && printf '%s\n' "$p"
            done
            ;;
        *) return 1 ;;
    esac
}

PORTS="$(find_ports 2>/dev/null | sort -u)"

# An explicitly named port wins over auto-detection, even if detection is empty.
if [ -n "$WANT" ]; then
    case "$WANT" in
        /*) PORT="$WANT" ;;
        *)  PORT="/dev/$WANT" ;;
    esac
    if [ -e "$PORT" ]; then
        PORTS="$PORT"
    else
        echo "Port '$PORT' does not exist."
        [ -n "$PORTS" ] && { echo; echo "Ports that do exist:"; printf '%s\n' "$PORTS" | sed 's/^/  /'; }
        exit 1
    fi
fi

if [ -z "$PORTS" ]; then
    echo "No ESP32 serial port found."
    echo
    case "$OS" in
        Linux)
            echo "Checked /dev/ttyACM* and /dev/ttyUSB* -> none exist. Try:"
            echo "  1. the OTHER USB port on the board (native 'USB' vs 'COM/UART')"
            echo "  2. a data cable (charge-only cables never enumerate)"
            echo "  3. dmesg | tail -20   (cdc_acm = native USB, ch341/cp210x = bridge)"
            ;;
        Darwin)
            echo "Checked /dev/cu.usbmodem* and /dev/cu.usbserial* -> none exist. Try:"
            echo "  1. macOS names are /dev/cu.usbmodemXXXX, never /dev/ttyACM0"
            echo "  2. the OTHER USB port on the board"
            echo "  3. a data cable; CH340 boards need the WCH driver"
            ;;
        *)
            echo "Unsupported OS '$OS'. On Windows: Device Manager -> Ports (COM & LPT),"
            echo "then use PuTTY (Serial, 115200)."
            ;;
    esac
    exit 1
fi

COUNT="$(printf '%s\n' "$PORTS" | wc -l | tr -d ' ')"
echo "Serial port(s) found:"
printf '%s\n' "$PORTS" | sed 's/^/  /'

[ "$LIST_ONLY" = "1" ] && exit 0

# --- choose a port ----------------------------------------------------------
if [ -n "$WANT" ]; then
    :   # PORT already resolved above
elif [ "$COUNT" = "1" ]; then
    PORT="$(printf '%s\n' "$PORTS" | head -1)"
else
    echo
    echo "More than one port. Pick one, e.g.:"
    echo "  ./monitor.sh $(printf '%s\n' "$PORTS" | head -1)"
    echo
    echo "Tip: with the native USB cable the port is ttyACM*/cu.usbmodem*;"
    echo "     with the COM/UART cable it is ttyUSB*/cu.usbserial*."
    exit 1
fi

# --- permission check -------------------------------------------------------
if [ ! -r "$PORT" ] || [ ! -w "$PORT" ]; then
    echo
    echo "No read/write permission on $PORT."
    case "$OS" in
        Linux)
            echo "Fix (then log out and back in):"
            echo "  sudo usermod -aG dialout \$USER"
            echo "Or for one session:  sudo chmod 666 $PORT"
            ;;
        *)
            echo "Check the device permissions, or run with appropriate rights."
            ;;
    esac
    exit 1
fi

# --- is the port actually talking? -----------------------------------------
# The firmware prints its banner once at boot and a "servo> " prompt each time a
# line is entered. So: listen briefly, poke it with an empty line, listen again.
# Nothing at all back => either the board isn't running this firmware, or the
# firmware's console is on a different port than the one we opened.
set_baud() {
    case "$OS" in
        Linux)  stty -F "$1" "$BAUD" raw -echo -echoe -echok 2>/dev/null || true ;;
        Darwin) stty -f "$1" "$BAUD" raw -echo -echoe -echok 2>/dev/null || true ;;
    esac
}

sniff() {
    python3 - "$PORT" 1.5 <<'PY' 2>/dev/null
import os, sys, time
port, secs = sys.argv[1], float(sys.argv[2])
data = b""
try:
    fd = os.open(port, os.O_RDWR | os.O_NONBLOCK)
except OSError:
    sys.exit(3)
end = time.time() + secs
try:
    while time.time() < end:
        try:
            chunk = os.read(fd, 4096)
            if chunk:
                data += chunk
        except BlockingIOError:
            time.sleep(0.05)
        except OSError:
            break
finally:
    try:
        os.close(fd)
    except OSError:
        pass
sys.stdout.buffer.write(data)
PY
}

if [ "$FORCE" = "0" ]; then
    echo
    echo -n "Checking ${PORT} for output ... "
    set_baud "$PORT"
    RX="$(sniff)"
    if [ -z "$RX" ]; then
        # nudge the firmware with an empty command line and listen again
        set_baud "$PORT"
        printf '\r' > "$PORT" 2>/dev/null || true
        sleep 0.3
        RX="$(sniff)"
    fi

    if [ -z "$RX" ]; then
        echo "nothing."
        echo
        echo "The port opens, but no data is coming out of it."
        echo
        case "$PORT" in
            *ttyUSB*|*usbserial*|*SLAB*|*wchusbserial*)
                echo "You are on the board's USB-UART bridge, which carries UART0."
                echo "The 'usb' image (the default) puts the console on the board's"
                echo "native USB port instead - so this port stays silent."
                echo
                echo "Easiest fix - flash the UART build to this port:"
                echo "    ./flash.sh $PORT uart"
                echo "    ./monitor.sh $PORT"
                echo
                echo "Or plug a cable into the board's other port (labelled USB/OTG"
                echo "or USB-Serial-JTAG) and run ./monitor.sh again."
                ;;
            *ttyACM*|*usbmodem*)
                echo "You are on the native USB port, but the image on the board"
                echo "has its console on UART0. Flash the USB build:"
                echo "    ./flash.sh $PORT"
                echo "    ./monitor.sh $PORT"
                ;;
            *)
                echo "If you have not flashed this project yet, flash it first:"
                echo "    ./flash.sh $PORT"
                echo
                echo "If you just reflashed, press the board's RESET button and retry."
                ;;
        esac
        echo
        echo "To open the console anyway:  ./monitor.sh $PORT --force"
        exit 1
    fi

    BYTES=$(printf '%s' "$RX" | wc -c | tr -d ' ')
    echo "got $BYTES byte(s) - the board is talking."
fi

# --- open the console -------------------------------------------------------
echo
echo "Opening $PORT at $BAUD baud"
echo "  screen: Ctrl-A then K to quit    miniterm: Ctrl-] to quit"
echo

if command -v screen >/dev/null 2>&1; then
    exec screen "$PORT" "$BAUD"
elif python3 -c "import serial" >/dev/null 2>&1; then
    exec python3 -m serial.tools.miniterm "$PORT" "$BAUD"
elif command -v picocom >/dev/null 2>&1; then
    exec picocom -b "$BAUD" "$PORT"
else
    echo "No console program available."
    echo "  Debian/Ubuntu: sudo apt install screen    (or picocom / python3-serial)"
    echo "  macOS:         screen is preinstalled"
    exit 1
fi
