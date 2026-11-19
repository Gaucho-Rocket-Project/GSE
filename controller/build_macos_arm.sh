#!/usr/bin/env bash
# Build the native macOS Apple Silicon (arm64) controller binary/app.
# Run this with an arm64 Python 3 (check: `python3 -c "import platform; print(platform.machine())"`).
set -euo pipefail
cd "$(dirname "$0")"

if [ "$(uname -m)" != "arm64" ]; then
  echo "ERROR: this script must run on Apple Silicon (arm64)." >&2
  exit 1
fi

python3 -m venv .venv
. .venv/bin/activate
python -m pip install --upgrade pip
python -m pip install -r requirements.txt
python -m PyInstaller --clean --noconfirm --onefile --windowed \
  --name ground-controller \
  --osx-bundle-identifier com.gauchorocket.ground-controller \
  src/ground_controller.py

echo "Binary/app: $(pwd)/dist/"
