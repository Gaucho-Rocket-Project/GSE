#!/usr/bin/env bash
# Build the native Linux x86_64 controller binary.
set -euo pipefail
cd "$(dirname "$0")"

python3 -m venv .venv
. .venv/bin/activate
python -m pip install --upgrade pip
python -m pip install -r requirements.txt
python -m PyInstaller --clean --noconfirm --onefile --windowed \
  --name ground-controller src/ground_controller.py

echo "Binary: $(pwd)/dist/ground-controller"
