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
3. Set FILL and SRV open targets in microseconds (500–2500). These values are
   sent with the next actuation, including SRV opening on ignition. Set and
   verify endpoints with horns detached.
4. TEST retains individual FILL/SRV controls, combined pulse/close, and IGN
   pulse/reset. LIVE hides the SRV and combined-servo control rows; ALL OFF
   remains available. LIVE opens SRV only through IGN ON.
   Switching modes requests ALL OFF.
5. LIVE FILL OPEN starts its countdown. At zero an alert sounds but FILL stays
   open. Close or extend manually. While FILL is commanded open, IGN ARM and
   IGN ON are greyed out, including after the FILL timer expires. With FILL and
   SRV closed, arm IGN and wait for the countdown. IGN ON commands ignition **and SRV open** together.
   IGN OFF resets ignition but does not close SRV; use ALL OFF in LIVE.

## DAQ panel

The right-hand panel shows the DAQ board (Hotfire-Board) data carried in `TLM`
records:
- **Pressures:** P0–P3 in psi (4–20 mA converted with each channel's range), plus loop mA, peak and status (below 4 mA, above 20 mA, near max, stale).
- **Thermocouples:** TC1–TC4, with MAX31856 faults decoded.
- **Live chart:** the last 30 s.
- **Link line:** TLM rate, radio losses (TLM sequence gaps), and how many DAQ lines the vehicle skipped.

**Channels & ranges…** sets the names, the psi at 4 mA / 20 mA for each channel, and °C or °F. These are saved in the user's Qt settings. Raw µA is always logged.

Each connection writes `data/daq_<date>_<time>.csv`, next to `telemetry.csv`, which is unchanged. It has one row per DAQ frame: raw µA, psi and the range used, temperatures and faults.

The vehicle forwards only the **newest** UART line every `TELEMETRY_INTERVAL_MS` (500 ms = 2 Hz). The **DAQ line format** toggle must match what the DAQ firmware sends:

| Toggle | DAQ sends (UART, ≤ 200 printable chars, `\n`) |
|---|---|
| One line (D,...) (recommended) | `D,<seq>,<t_us>,<p0>,<p1>,<p2>,<p3>,<tc1>,<tc2>,<tc3>,<tc4>,<f1>,<f2>,<f3>,<f4>` |
| Separate P / T lines | `P,<seq>,<t_us>,<p0>..<p3>` and `T,<seq>,<t_us>,<tc1>..<tc4>,<f1>..<f4>` |

**Field rules:**
- **`p*`:** µA, integer.
- **`tc*`:** °C.
- **`f*`:** the MAX31856 fault register in hex (`00` = no fault).
- **Empty field:** no reading.
- **`HB,<n>` or `heartbeat`:** accepted in either mode.

With separate lines at 2 Hz, thermocouple lines are rarely the newest line, so TC readings go stale. The panel warns about this, and also warns when the toggle doesn't match the lines arriving.

**The DAQ firmware must send these lines for the panel to show data.** Today's DAQ firmware only sends `heartbeat`. With that, the panel says "DAQ alive … no DAQ data lines yet", which confirms the DAQ → vehicle → ground → laptop chain without showing any values.

**Robustness:**
- **Errors are contained:** a DAQ panel error never interrupts command or ACK handling.
- **CSV failure:** if the DAQ CSV can't be written, this is logged once and shown on the panel. The display keeps working, and the next connection retries.
- **Duplicates and reboots:** duplicate packets are ignored, and DAQ or vehicle reboots are detected.
- **Small screens:** the DAQ panel scrolls, and its divider can be dragged.

## Simulation

`python src/ground_controller.py --sim` runs without hardware. It simulates the ground board's USB text, the vehicle's interlocks and ACKs, the 2 Hz newest-line telemetry, and a DAQ on a filled Mojave Sphinx (HCR-5100 Table 2.6). IGN (test pulse or LIVE IGN ON) starts a simulated burn, and the simulated DAQ follows the format toggle.

**Options:**
- `--sim-ambient <°F>`: the ambient temperature, which sets the simulated tank pressure.
- `--sim-interval-ms <ms>`: the vehicle's `TELEMETRY_INTERVAL_MS`.

The desktop's states and countdowns are optimistic; they are not a hardware
readback. Rejections appear in the log but do not automatically reconcile the
local state, and ACKs may be lost. Never infer safe hardware state from this UI.
Use physical interlocks, test without an igniter, and verify on actual hardware.
