#!/usr/bin/env python3
"""Build both NUCLEO-H753ZI firmware images and emit flashable flat .bin files.

Requires the Rust thumbv7em-none-eabihf target plus cargo-binutils and its
llvm-tools component. The ELF build tree and final images live in repo/artifacts.
"""

from __future__ import annotations

import argparse
import os
from pathlib import Path
import shutil
import subprocess
import sys


ROOT = Path(__file__).resolve().parents[1]
FIRMWARE = ROOT / "firmware" / "stm32h753"
ARTIFACTS = ROOT / "artifacts"
TARGET_DIR = ARTIFACTS / "cargo-target"


def run(command: list[str], *, env: dict[str, str]) -> None:
    print("+", " ".join(command), flush=True)
    subprocess.run(command, cwd=FIRMWARE, env=env, check=True)


def main() -> int:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument(
        "--radio",
        choices=("dx-lr42", "e28-2g4m27s"),
        default="dx-lr42",
        help="compile-time default for a board with blank config flash (default: dx-lr42)",
    )
    args = parser.parse_args()

    if shutil.which("cargo") is None:
        parser.error("cargo was not found; install Rust 1.75+ and add it to PATH")

    ARTIFACTS.mkdir(parents=True, exist_ok=True)
    env = os.environ.copy()
    env["CARGO_TARGET_DIR"] = str(TARGET_DIR)

    feature_args: list[str] = []
    if args.radio == "e28-2g4m27s":
        feature_args = ["--no-default-features", "--features", "radio-e28-2g4m27s"]

    run(["cargo", "build", "--release", "--bins", *feature_args], env=env)

    for binary in ("gse-ground", "gse-vehicle"):
        output = ARTIFACTS / f"{binary}.bin"
        run(
            [
                "cargo",
                "objcopy",
                "--release",
                "--bin",
                binary,
                *feature_args,
                "--",
                "-O",
                "binary",
                str(output),
            ],
            env=env,
        )
        print(f"Created {output}")

    print("Flash either .bin at address 0x08000000 (one image per NUCLEO board).")
    return 0


if __name__ == "__main__":
    try:
        raise SystemExit(main())
    except subprocess.CalledProcessError as error:
        if error.cmd and len(error.cmd) > 1 and error.cmd[1] == "objcopy":
            print(
                "cargo objcopy is required. Install it with:\n"
                "  rustup component add llvm-tools-preview\n"
                "  cargo install cargo-binutils",
                file=sys.stderr,
            )
        raise SystemExit(error.returncode)
