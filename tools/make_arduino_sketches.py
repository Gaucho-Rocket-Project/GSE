#!/usr/bin/env python3
"""
make_arduino_sketches.py -- build Arduino IDE sketch folders from firmware/src.

The PlatformIO project (firmware/) is the source of truth.  The Arduino IDE
cannot use its include paths or build flags, so this copies each node's files
into one flat sketch folder and writes the flags into build_opt.h, which the
ESP32 Arduino core passes to the compiler.  Re-run it after changing anything
in firmware/src; never edit the copies.

    python tools/make_arduino_sketches.py            # -> firmware/arduino/<sketch>/
    python tools/make_arduino_sketches.py --out DIR
"""
from __future__ import annotations

import argparse
import os
import shutil

HERE = os.path.dirname(os.path.abspath(__file__))
GSE = os.path.dirname(HERE)
SRC = os.path.join(GSE, "firmware", "src")

SKETCHES = {
    "gse_tx_esp32s": {
        "what": "ground box / TX node on the ESP32S (ESP-32S, WROOM-32) -- ESP-NOW",
        "files": ["config.h", "link.h", "dxlr41.h",
                  "common/gse_protocol.h", "common/gse_hal.h",
                  "common/gse_tx_core.h", "common/gse_tx_core.c",
                  "tx/tx_platform_esp32.cpp"],
        "flags": ["-DGSE_ROLE_TX", "-DGSE_BOARD_ESP32S",
                  '"-DGSE_PLATFORM_NAME=\\"esp32s\\""'],
        "board": ["Tools > Board: esp32 > ESP32 Dev Module",
                  "Tools > Port: the ESP32S (CP210x USB to UART Bridge)",
                  "Everything else: defaults"],
    },
    "gse_pad_esp32s3": {
        "what": "vehicle / pad node on the ESP32-S3 -- ESP-NOW",
        "files": ["config.h", "link.h", "dxlr41.h",
                  "common/gse_protocol.h",
                  "pad/pad_diag.h", "pad/pad_node.cpp"],
        "flags": ["-DGSE_ROLE_PAD", "-DGSE_BOARD_ESP32_S3"],
        "board": ["Tools > Board: esp32 > ESP32S3 Dev Module",
                  "Tools > Port: the S3's UART socket (USB-SERIAL CH340 / CP210x)",
                  "Tools > USB CDC On Boot: Disabled  -> the pad's log comes out of the",
                  "    same UART socket you flash through (Enabled = native USB socket)",
                  "Everything else: defaults"],
    },
    "gse_servo_sweep_esp32s3": {
        "what": "BENCH TEST, not flight firmware: sweeps the pad's 3 servos 0..180 deg",
        "files": ["config.h", "common/gse_protocol.h", "bench/servo_sweep.cpp"],
        "flags": ["-DGSE_ROLE_PAD", "-DGSE_BOARD_ESP32_S3"],
        "board": ["Library: ESP32Servo (Library Manager)",
                  "Tools > Board: esp32 > ESP32S3 Dev Module",
                  "Tools > Port: the S3's UART socket",
                  "Tools > USB CDC On Boot: Disabled  -> sweep log on the same port",
                  "Afterwards, flash gse_pad_esp32s3 back onto the S3"],
    },
    # The GSE ground / vehicle firmware over the ESP-NOW settings proven by the
    # ping/pong test sketches (firmware/espnow_ping): channel 6, no encryption,
    # fixed MACs, ping/pong built in.  link_simple.h is copied in as link.h.
    "gse_ground_simple_esp32s": {
        "what": "GSE ground board (ESP32S) over the ping/pong ESP-NOW settings",
        "files": ["config.h", ("simple/link_simple.h", "link.h"),
                  "common/gse_protocol.h", "common/gse_hal.h",
                  "common/gse_tx_core.h", "common/gse_tx_core.c",
                  "tx/tx_platform_esp32.cpp"],
        "flags": ["-DGSE_ROLE_TX", "-DGSE_BOARD_ESP32S",
                  '"-DGSE_PLATFORM_NAME=\\"esp32s\\""', "-DGSE_NOW_CHANNEL=6"],
        "board": ["Tools > Board: esp32 > ESP32 Dev Module",
                  "Tools > Port: the ground board",
                  "Pairs with gse_vehicle_simple_esp32s (same channel, no encryption)",
                  "Prints 'LOG PONG seq=.. rtt=.. ms' once a second (GUI serial monitor)"],
    },
    "gse_vehicle_simple_esp32s": {
        "what": "GSE vehicle (ESP32S) over the ping/pong ESP-NOW settings",
        "files": ["config.h", ("simple/link_simple.h", "link.h"),
                  "common/gse_protocol.h", "pad/pad_diag.h", "pad/pad_node.cpp"],
        "flags": ["-DGSE_ROLE_PAD", "-DGSE_BOARD_ESP32S", "-DGSE_NOW_CHANNEL=6"],
        "board": ["Tools > Board: esp32 > ESP32 Dev Module",
                  "Tools > Port: the vehicle board",
                  "Pins: the ESP32S block of config.h (FILL 4, VEH 5, PURGE 15,",
                  "    igniter 13, heartbeat 33, status LED 2)",
                  "Answers the ground board's PINGs; log on the same USB port, 115200"],
    },
}


def _src_dst(entry) -> tuple[str, str]:
    """A files entry is "path" (copied under its own name) or (path, name)."""
    if isinstance(entry, tuple):
        return entry
    return entry, os.path.basename(entry)


def make(out_root: str) -> None:
    for name, s in SKETCHES.items():
        d = os.path.join(out_root, name)
        if os.path.isdir(d):
            shutil.rmtree(d)
        os.makedirs(d)
        for entry in s["files"]:
            src, dst = _src_dst(entry)
            shutil.copy2(os.path.join(SRC, src), os.path.join(d, dst))
        with open(os.path.join(d, "build_opt.h"), "w", newline="\n") as fh:
            fh.write("\n".join(s["flags"]) + "\n")
        board = "\n".join(f" *   {b}" for b in s["board"])
        files = ", ".join(
            dst if src.endswith(dst) else f"{dst} (from {src})"
            for src, dst in map(_src_dst, s["files"]))
        with open(os.path.join(d, name + ".ino"), "w", newline="\n") as fh:
            fh.write(f"""/*
 * {name}.ino -- {s['what']}
 *
 * GENERATED by tools/make_arduino_sketches.py from firmware/src.  Do not edit
 * anything in this folder: change firmware/src and re-run the script.
 *
 * setup() and loop() live in the copied .cpp file; this file is only here
 * because the Arduino IDE needs an .ino named after the folder.
 * build_opt.h holds the build flags (the ESP32 core passes it to the compiler).
 *
 * Arduino IDE settings:
{board}
 *
 * Copied files: {files}
 */
""")
        print(f"{d}  ({len(s['files'])} files)")


def main() -> int:
    ap = argparse.ArgumentParser(description=__doc__.split("\n\n")[0])
    ap.add_argument("--out", default=os.path.join(GSE, "firmware", "arduino"))
    make(os.path.abspath(ap.parse_args().out))
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
