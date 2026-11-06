#!/usr/bin/env python3
"""Launcher for the HCR-5100 GSE laptop control GUI.

    python software/run_gui.py --sim            # no hardware
    python software/run_gui.py --port /dev/ttyACM0
    python software/run_gui.py                  # pick the port in the window
"""
import os
import sys

sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))
from gse_control.app import main   # noqa: E402

if __name__ == "__main__":
    sys.exit(main())
