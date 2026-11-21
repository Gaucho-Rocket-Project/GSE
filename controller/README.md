# Ground controller desktop app

The Qt application talks over USB to `../ground/ground.ino`, which bridges to
`../vehicle/vehicle.ino`. Reflash both boards together. See the [root
README](../README.md) for wiring, safety, UART framing, and telemetry details.

## Run / build

```bash
cd controller
python3 -m venv .venv && . .venv/bin/activate
pip install -r requirements.txt
python src/ground_controller.py
python -m unittest discover -s tests -v
```

A desktop environment with Qt and OpenGL libraries is required. Build scripts
`build_linux.sh`, `build_windows.ps1`, and `build_macos_arm.sh` use PyInstaller;
build on the target OS. The executable writes telemetry under the launch
working directory unless `GSE_DATA_DIR` is set.

## Operation

1. Connect ground over USB, refresh ports, select it and connect. `PING` / `PONG`
   checks the USB bridge, not the vehicle radio or output state.
2. Open the **TELEMETRY** panel to see recent records live; closing it does not
   stop recording. CSV is appended in `data/telemetry.csv`; see root README.
3. Set FILL and SRV open targets in degrees of travel from closed (0–270°,
   for the 270° DS3225). The app converts them to a pulse width
   (`µs = 2500 − degrees × 7.41`, shown next to each box; 90° = 1833 µs) and
   sends that with the next actuation, including SRV opening on ignition.
   Degrees assume a linear 270° servo; set and verify endpoints with horns
   detached.
4. TEST retains individual FILL/SRV controls, combined pulse/close, and IGN
   pulse/reset. LIVE hides the SRV and combined-servo control rows; ALL OFF
   remains available. LIVE opens SRV only through IGN ON.
   Switching modes requests ALL OFF.
5. LIVE FILL OPEN starts its countdown. At zero an alert sounds but FILL stays
   open. Close or extend manually. While FILL is commanded open, IGN ARM and
   IGN ON are greyed out, including after the FILL timer expires. With FILL and
   SRV closed, arm IGN and wait for the countdown. IGN ON commands ignition **and SRV open** together.
   IGN OFF resets ignition but does not close SRV; use ALL OFF in LIVE.

The desktop's states and countdowns are optimistic; they are not a hardware
readback. Rejections appear in the log but do not automatically reconcile the
local state, and ACKs may be lost. Never infer safe hardware state from this UI.
Use physical interlocks, test without an igniter, and verify on actual hardware.
