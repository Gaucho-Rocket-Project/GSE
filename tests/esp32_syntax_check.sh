#!/bin/sh
# esp32_syntax_check.sh -- compile-check the ESP32 firmware without the toolchain
# ---------------------------------------------------------------------------
# The Espressif toolchain is a ~1 GB download out of dl.espressif.com.  This
# script checks every board variant we ship with the *host* g++ and the small
# Arduino stand-ins in tests/esp32_stub/, so a typo, a pin that does not exist
# on the board, or a board-variant #if that got the wrong way round fails here
# in a second instead of on the bench or in CI.
#
#   sh tests/esp32_syntax_check.sh
#
# It compiles (syntax + full type checking, -fsyntax-only) each of:
#
#   pad  · ESP32-S3   (-D GSE_ROLE_PAD)
#   pad  · ESP-32S    (-D GSE_ROLE_PAD   -D GSE_BOARD_ESP32S)
#   pad  · no flag    (the default that "pio run -e pad-node" uses)
#   pad/tx · ESP32-S3 and ESP-32S with the ESP-NOW transport (-D GSE_LINK_ESPNOW)
#   tx   · ESP32-S3   (-D GSE_ROLE_TX)
#   tx   · ESP-32S    (-D GSE_ROLE_TX    -D GSE_BOARD_ESP32S)
#   tx   · no flag
#   all of the above against the Arduino-ESP32 2.x API as well (ledcSetup /
#   neopixelWrite), because the repo supports both cores
#
# and it proves the two negative cases, which matter as much as the positive
# ones:
#
#   - building the pad for an ESP-32S with GPIO6 must fail (SPI flash pin)
#   - building with GSE_LINK_ESPNOW and GSE_LINK_LORA at once must fail
#   - building ESP-NOW for the STM32 (no Wi-Fi on that board) must fail
#   - building both variants at once must fail
#
# Exits non-zero on the first error or on any warning.
set -e

HERE=$(cd "$(dirname "$0")" && pwd)
FW=$HERE/../firmware
SRC=$FW/src
STUB=$HERE/esp32_stub
CXX=${CXX:-g++}

if ! command -v "$CXX" >/dev/null 2>&1; then
    echo "no $CXX on PATH -- this check needs a host C++ compiler" >&2
    exit 1
fi

TMP=$(mktemp -d)
trap 'rm -rf "$TMP"' EXIT

# -D INCLUDE_DIR: tests/esp32_stub first, so "Arduino.h" resolves to the stub
# and never to a real core that happens to be installed.
INC="-I $STUB -I $SRC -I $SRC/common"
CXXSTD=gnu++17
WARN="-Wall -Wextra -Wno-unused-parameter -Wno-unused-variable -Wno-unused-but-set-variable"

check() {                      # check <label> <expected: ok|fail> <file> <defines...>
    label=$1; want=$2; file=$3; shift 3
    out=$TMP/out.txt
    # shellcheck disable=SC2086  # $INC/$WARN must word-split into flags
    if $CXX -std="$CXXSTD" -fsyntax-only "$@" $INC $WARN "$file" >"$out" 2>&1; then got=ok; else got=fail; fi
    if [ "$got" != "$want" ]; then
        echo "FAIL  $label  (expected $want, got $got)"
        sed 's/^/      /' "$out"
        exit 1
    fi
    # a warning is still a failure: keep the tree warning-clean
    if grep -q "warning:" "$out"; then
        echo "FAIL  $label  (warnings)"
        grep "warning:" "$out" | sed 's/^/      /'
        exit 1
    fi
    printf '  ok   %-46s %s\n' "$label" "$want"
}

PAD=$SRC/pad/pad_node.cpp
TX=$SRC/tx/tx_platform_esp32.cpp

echo "ESP32 firmware, board variants (host g++, -fsyntax-only)"
for core in 3 2; do
    if [ "$core" = 2 ]; then CORE="-D ARDUINO_ESP32_CORE2"; else CORE=""; fi
    v="core $core.x"
    check "pad  ESP32-S3   $v" ok   "$PAD" $CORE -D GSE_ROLE_PAD \
          -D ARDUINO_USB_CDC_ON_BOOT=1 -D ARDUINO_USB_MODE=1
    check "pad  ESP-32S    $v" ok   "$PAD" $CORE -D GSE_ROLE_PAD -D GSE_BOARD_ESP32S
    check "pad  no flag    $v" ok   "$PAD" $CORE -D GSE_ROLE_PAD
    # platformio.ini passes these for the S3 envs; the ESP-32S envs must not,
    # because there the laptop port is the bridge on UART0.
    check "tx   ESP32-S3   $v" ok   "$TX"  $CORE -D GSE_ROLE_TX -D GSE_BOARD_ESP32_S3 \
          -D ARDUINO_USB_CDC_ON_BOOT=1 -D ARDUINO_USB_MODE=1
    check "tx   ESP-32S    $v" ok   "$TX"  $CORE -D GSE_ROLE_TX -D GSE_BOARD_ESP32S
    check "tx   no flag    $v" ok   "$TX"  $CORE -D GSE_ROLE_TX \
          -D ARDUINO_USB_CDC_ON_BOOT=1
done

# a node with AUX not wired must still build and still pace
check "pad  ESP-32S, AUX not wired" ok "$PAD" -D GSE_ROLE_PAD -D GSE_BOARD_ESP32S \
      -D GSE_LORA_AUX_NONE
check "tx   no flag, AUX not wired" ok "$TX" -D GSE_ROLE_TX -D GSE_LORA_AUX_NONE \
      -D ARDUINO_USB_CDC_ON_BOOT=1

# --------------------------------------------- the ESP-NOW transport (optional)
# The second link option: no module, no UART, no AUX, no router, no pairing.
# Both roles and boards, across the IDF 4.4/5.1/5.4/5.5 callback
# signatures. Compile in both C++17 and C++20: the latter diagnoses deprecated
# volatile increments like those reported by newer ESP32 toolchains.
for CXXSTD in gnu++17 gnu++20; do
  for idf in 4.4 5.1 5.4 5.5; do
    case "$idf" in
      4.4) CORE="-D ARDUINO_ESP32_CORE2" ;;
      5.1) CORE="" ;;
      5.4) CORE="-D GSE_TEST_IDF_5_4" ;;
      5.5) CORE="-D GSE_TEST_IDF_5_5" ;;
    esac
    v="IDF $idf $CXXSTD"
    check "pad  ESP32-S3 NOW $v" ok "$PAD" $CORE -D GSE_ROLE_PAD -D GSE_LINK_ESPNOW \
          -D ARDUINO_USB_CDC_ON_BOOT=1 -D ARDUINO_USB_MODE=1
    check "pad  ESP-32S  NOW $v" ok "$PAD" $CORE -D GSE_ROLE_PAD -D GSE_LINK_ESPNOW \
          -D GSE_BOARD_ESP32S
    check "tx   ESP32-S3 NOW $v" ok "$TX"  $CORE -D GSE_ROLE_TX -D GSE_LINK_ESPNOW \
          -D GSE_BOARD_ESP32_S3 -D ARDUINO_USB_CDC_ON_BOOT=1 -D ARDUINO_USB_MODE=1
    check "tx   ESP-32S  NOW $v" ok "$TX"  $CORE -D GSE_ROLE_TX -D GSE_LINK_ESPNOW \
          -D GSE_BOARD_ESP32S
  done
done
CXXSTD=gnu++17
# ... and with the MACs supplied on the command line, the way an operator would
# override them for a second pair of boards.
check "pad  NOW with real MACs" ok "$PAD" -D GSE_ROLE_PAD -D GSE_LINK_ESPNOW \
      -D 'GSE_NOW_TX_MAC={0x24,0x6f,0x28,0xaa,0xbb,0x01}' \
      -D 'GSE_NOW_PAD_MAC={0x24,0x6f,0x28,0xaa,0xbb,0x02}'

echo "negative cases (these must NOT build)"
# The host stub must keep exposing the POSIX names the real core drags in
# through Arduino.h, or a firmware object called `link` (which collides with
# POSIX link()) compiles here and fails on the bench.  This probe is the guard.
check "a global called 'link' (POSIX shadow)" fail "$HERE/tripwire/posix_shadow_probe.cpp"
# The old send callback must be rejected by the IDF 5.5 stub.
check "IDF 5.5 rejects old send callback" fail "$HERE/tripwire/espnow_send_callback_probe.cpp" \
      -D GSE_TEST_IDF_5_5
check "pad  ESP-32S with PURGE on GPIO6 (flash)" fail "$PAD" \
      -D GSE_ROLE_PAD -D GSE_BOARD_ESP32S -D GSE_PIN_SERVO_PRG=6
check "pad  ESP-32S with IGN on strapping GPIO12" fail "$PAD" \
      -D GSE_ROLE_PAD -D GSE_BOARD_ESP32S -D GSE_PIN_RELAY_IGN=12
check "both board flags at once" fail "$PAD" \
      -D GSE_ROLE_PAD -D GSE_BOARD_ESP32S -D GSE_BOARD_ESP32_S3
check "pad  both links at once (NOW+LoRa)" fail "$PAD" \
      -D GSE_ROLE_PAD -D GSE_LINK_ESPNOW -D GSE_LINK_LORA
check "tx   ESP-NOW on the STM32 build" fail "$TX" -D GSE_ROLE_TX -D GSE_LINK_ESPNOW \
      -D GSE_PLATFORM_STM32 -D ARDUINO_USB_CDC_ON_BOOT=1
check "pad  NOW with a 15-character key" fail "$PAD" -D GSE_ROLE_PAD \
      -D GSE_LINK_ESPNOW -D 'GSE_NOW_KEY_TEXT="too-short"'
check "pad firmware built as TX" fail "$PAD" -D GSE_ROLE_TX
check "tx  firmware built as PAD" fail "$TX"  -D GSE_ROLE_PAD \
      -D ARDUINO_USB_CDC_ON_BOOT=1

echo "ESP32 firmware: all board variants compile clean"
