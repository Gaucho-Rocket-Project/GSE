#!/usr/bin/env bash
# ---------------------------------------------------------------------------
# flash.sh - program the NUCLEO-H753ZI over its on-board ST-LINK
#
#   ./tools/flash.sh                       # newest build (or prebuilt 180 deg)
#   ./tools/flash.sh prebuilt/nucleo_h753zi_servo_270.bin
#   ./tools/flash.sh out/servo.elf
#   ./tools/flash.sh --probe               # just talk to the ST-LINK, no writing
#   ./tools/flash.sh --erase               # mass erase, then flash
#
# Needs ONE of (checked in this order):
#   1. STM32_Programmer_CLI   (STM32CubeProgrammer / CubeCLT)
#   2. openocd                (CubeIDE / CubeCLT / distro package)
#   3. a mounted NODE_H753ZI drive  -> copies the .bin (drag-and-drop equivalent)
# ---------------------------------------------------------------------------
set -uo pipefail

HERE="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
ROOT="$(cd "$HERE/.." && pwd)"
cd "$ROOT" || exit 1

PROBE=0
ERASE=0
FILE=""
for arg in "$@"; do
    case "$arg" in
        --probe|-p) PROBE=1 ;;
        --erase|-e) ERASE=1 ;;
        -h|--help) sed -n '2,20p' "$0"; exit 0 ;;
        -*) echo "unknown option: $arg"; exit 2 ;;
        *) FILE="$arg" ;;
    esac
done

# --- pick the image ---------------------------------------------------------
if [ -z "$FILE" ]; then
    if [ -f out/servo.elf ]; then
        FILE="out/servo.elf"
    elif [ -f prebuilt/nucleo_h753zi_servo_180.bin ]; then
        FILE="prebuilt/nucleo_h753zi_servo_180.bin"
        echo "No build found - using the prebuilt image (make, then re-run to flash your build)."
    else
        echo "Nothing to flash: no out/servo.elf and no prebuilt/*.bin."
        exit 1
    fi
fi
[ -f "$FILE" ] || { echo "File not found: $FILE"; exit 1; }

# ELF is preferred (has symbols/sections); .bin goes in at 0x08000000.
case "$FILE" in
    *.elf) ADDR="" ;;
    *.bin) ADDR="0x08000000" ;;
    *.hex) ADDR="" ;;
    *) echo "Unsupported file type: $FILE (use .elf, .hex or .bin)"; exit 1 ;;
esac

echo "Image : $FILE"
echo

have() { command -v "$1" >/dev/null 2>&1; }

# --- 1. STM32CubeProgrammer ------------------------------------------------
if have STM32_Programmer_CLI; then
    echo "Tool  : STM32_Programmer_CLI"
    if [ "$PROBE" = "1" ]; then
        exec STM32_Programmer_CLI -c port=SWD -l
    fi
    ARGS=(-c port=SWD -w "$FILE" ${ADDR:+$ADDR} -v -rst)
    [ "$ERASE" = "1" ] && ARGS=(-c port=SWD -e all -w "$FILE" ${ADDR:+$ADDR} -v -rst)
    STM32_Programmer_CLI "${ARGS[@]}"
    RC=$?
    if [ "$RC" != "0" ]; then
        cat <<EOF

Programming failed (exit $RC). Common causes:
  - If the error mentions "volume" or "Error: No STM32 target found":
      the firmware currently running disabled the SWD pins, or the board is
      held in "connect under reset" state. Retry with a reset-under-reset
      connection:
         STM32_Programmer_CLI -c port=SWD mode=UR -w "$FILE" ${ADDR:+$ADDR} -v -rst
  - Jumpers CN4 ("ST-LINK") must both be fitted.
  - Use the ST-LINK USB port, and a data-capable cable.
EOF
    fi
    exit $RC
fi

# --- 2. OpenOCD ------------------------------------------------------------
if have openocd; then
    echo "Tool  : openocd (ST-LINK)"
    if [ "$PROBE" = "1" ]; then
        exec openocd -f interface/stlink.cfg -f target/stm32h7x.cfg \
             -c "init; targets; flash probe 0; shutdown"
    fi
    if [ "$ERASE" = "1" ]; then
        openocd -f interface/stlink.cfg -f target/stm32h7x.cfg \
            -c "init; reset halt; flash erase_sector 0 0 last; program $FILE ${ADDR} verify; reset run; shutdown"
    else
        openocd -f interface/stlink.cfg -f target/stm32h7x.cfg \
            -c "program $FILE ${ADDR} verify reset exit"
    fi
    RC=$?
    [ "$RC" != "0" ] && cat <<EOF

Programming failed (exit $RC). Try:
  - a "connect under reset" run:  openocd -f interface/stlink.cfg \\
        -f target/stm32h7x.cfg -c "reset_config srst_only srst_nogate; \\
        init; reset halt; program $FILE ${ADDR} verify; reset run; shutdown"
  - check both CN4 ST-LINK jumpers are fitted
  - hold the black RESET button while the command starts, then release it
EOF
    exit $RC
fi

# --- 3. drag-and-drop (mbed-style MSD drive) -------------------------------
DRIVE=""
for d in /media/"$USER"/NODE_H753ZI /media/"$USER"/NUCLEO \
         /run/media/"$USER"/NODE_H753ZI /run/media/"$USER"/NUCLEO \
         /Volumes/NODE_H753ZI /Volumes/NUCLEO; do
    [ -d "$d" ] && { DRIVE="$d"; break; }
done

case "$FILE" in
    *.bin) ;;
    *)
        echo "No STM32_Programmer_CLI and no openocd found, and the drag-and-drop"
        echo "drive only accepts a raw .bin, not ${FILE##*.}."
        echo "Build the binary first:  make build   (writes out/servo.bin)"
        exit 1
        ;;
esac

if [ -n "$DRIVE" ]; then
    echo "Tool  : drag-and-drop drive ($DRIVE)"
    cp -v "$FILE" "$DRIVE/" || exit 1
    sync
    echo "Copied. The ST-LINK flashes it and resets the MCU (~2 s)."
    exit 0
fi

cat <<EOF
No flashing tool and no NODE_H753ZI drive found.

Install one of:
  STM32CubeProgrammer   https://www.st.com/en/development-tools/stm32cubeprog.html
  openocd               sudo apt install openocd     (or: STM32CubeCLT)

Or copy the .bin onto the board's mass-storage drive by hand:
  cp out/servo.bin /media/\$USER/NODE_H753ZI/
EOF
exit 1
