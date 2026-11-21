"""Host regression of the vehicle sketch; not a substitute for ESP32 tests."""
import shutil
import subprocess
import tempfile
import unittest
from pathlib import Path


class VehicleFirmwareTests(unittest.TestCase):
    @unittest.skipUnless(shutil.which("g++"), "g++ not available")
    def test_live_ign_pulse_and_srv_hold(self):
        here = Path(__file__).resolve().parent
        with tempfile.TemporaryDirectory() as directory:
            binary = Path(directory) / "vehicle_test"
            subprocess.run(["g++", "-std=c++17", "-Wall", "-Wextra", "-I", str(here / "firmware_stubs"),
                            str(here / "test_vehicle_firmware.cpp"), "-o", str(binary)], check=True)
            subprocess.run([str(binary)], check=True)
