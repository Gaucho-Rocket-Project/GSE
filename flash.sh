#!/usr/bin/env bash
# ---------------------------------------------------------------------------
# flash.sh - flash the prebuilt ESP32-S3 dual-SG90 firmware with esptool
#
#   ./flash.sh                          # SG90 build, USB console, auto port
#   ./flash.sh /dev/ttyACM0             # pick a port explicitly
#   ./flash.sh /dev/ttyUSB0 uart        # SG90 build, UART0 console
#   ./flash.sh /dev/ttyACM0 ds3225-180  # DSSERVO DS3225MG 180 deg version
#   ./flash.sh --erase-only             # just erase the flash
#
# Needs: python3 + esptool  (pip install esptool)
#        or an ESP-IDF environment loaded (idf.py already provides esptool)
# ---------------------------------------------------------------------------
set -euo pipefail

HERE="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
PORT="${1:-}"
VARIANT="${2:-usb}"

if [[ "${PORT:-}" == "--erase-only" ]]; then
    echo "Erasing flash (port: auto-detect) ..."
    exec python3 -m esptool --chip esp32s3 erase_flash
fi

case "$VARIANT" in
    usb|"")     IMG="${HERE}/prebuilt/esp32s3_servo_usb_full.bin"  ;;
    uart)       IMG="${HERE}/prebuilt/esp32s3_servo_uart_full.bin" ;;
    ds3225|ds3225-180|ds3225_180)
                IMG="${HERE}/prebuilt/esp32s3_servo_ds3225_180_usb_full.bin" ;;
    ds3225-180-uart|ds3225_180_uart)
                IMG="${HERE}/prebuilt/esp32s3_servo_ds3225_180_uart_full.bin" ;;
    ds3225-270|ds3225_270)
                IMG="${HERE}/prebuilt/esp32s3_servo_ds3225_270_usb_full.bin" ;;
    ds3225-270-uart|ds3225_270_uart)
                IMG="${HERE}/prebuilt/esp32s3_servo_ds3225_270_uart_full.bin" ;;
    *)
        echo "unknown variant '$VARIANT'"
        echo
        echo "  usb              SG90 / micro, console on native USB   (default)"
        echo "  uart             SG90 / micro, console on UART0"
        echo "  ds3225-180       DS3225MG 180 deg version, native USB"
        echo "  ds3225-180-uart  DS3225MG 180 deg version, UART0"
        echo "  ds3225-270       DS3225MG 270 deg version, native USB"
        echo "  ds3225-270-uart  DS3225MG 270 deg version, UART0"
        exit 1
        ;;
esac

[[ -f "$IMG" ]] || { echo "image not found: $IMG"; exit 1; }

PORT_ARG=()
[[ -n "$PORT" ]] && PORT_ARG=(-p "$PORT")

echo "Flashing ${IMG##*/} -> ESP32-S3 ${PORT:+on $PORT}"
echo

# --before default_reset --after hard_reset => enters the ROM downloader
# automatically (DTR/RTS auto-reset) and reboots into the new firmware.
python3 -m esptool --chip esp32s3 -b 460800 \
    "${PORT_ARG[@]}" \
    --before default_reset --after hard_reset \
    write_flash --flash_mode dio --flash_freq 80m --flash_size 8MB \
    0x0 "$IMG"

echo
echo "Done. Open the serial console:"
echo "  USB build : it re-appears as a USB-CDC device - open the port at any baud,"
echo "              e.g.  esp-idf-monitor / 'screen /dev/ttyACM0 115200' / PuTTY"
echo "  UART build: 115200 8N1 on the COM port"
echo "Then type 'help' and press Enter."
