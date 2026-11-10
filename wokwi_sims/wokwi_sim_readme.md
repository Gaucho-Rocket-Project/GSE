# Wokwi simulation with run_gui.py

Runs the real GUI against the real pad-node firmware (ESP32-S3 only!) in Wokwi:

`run_gui.py` → `wokwi_bridge.py` (stands in for the TX node and both radios) → Wokwi pad

**Needs:** VS Code with the Wokwi extension, Python 3 with tkinter and pyserial (see [Setup](#setup-by-operating-system)), and PlatformIO if you rebuild. Works on Windows, macOS and Linux.

## Run (from the `GSE` folder)

On macOS and Linux, type `python3` wherever this says `python` (or activate the venv from [Setup](#setup-by-operating-system), which makes `python` work).

1. **Build the firmware** (only if `firmware/.pio/build/pad-node/firmware.bin` is missing or out of date): `cd firmware && pio run -e pad-node`
2. **Start the bridge** and leave it running: `python wokwi_sims/wokwi_bridge.py`
   - `-v` prints the pad's radio set-up (AT command) dialog.
   - `--pad-first` sends nothing to the pad until its radio set-up finishes, as if the pad were powered before the ground box. Without it, the SAFE stream runs from the start, which can make the pad's radio set-up fail (D26).
3. **Start the simulation:** in VS Code, run **Wokwi: Start Simulator** using `firmware/wokwi.toml`, and keep the simulator tab visible.
4. **Start the GUI** in another terminal, then click **Connect**: `python software/run_gui.py --port rfc2217://localhost:4001`
5. **Check the pins:** stop the simulation, then run `python wokwi_sims/wokwi_pad_test.py inspect` to see what each servo, igniter and heartbeat pin actually did.

Pad events, configuration sent, and the radio set-up result print in the bridge terminal.

**Automated check (no GUI):** start `python wokwi_sims/wokwi_pad_test.py run -v` first, then the simulation.

## Setup by operating system

| | Windows | macOS | Linux (Debian/Ubuntu) |
|---|---|---|---|
| Python command | `python` | `python3` | `python3` |
| tkinter (for the GUI) | included with the python.org installer | included with the python.org installer; Homebrew Python needs `brew install python-tk` | `sudo apt install python3-tk` |
| pyserial | `pip install pyserial` | in a venv (below) | in a venv (below) |

Homebrew Python and recent Ubuntu/Debian refuse a plain `pip install` ("externally-managed-environment"), so on macOS and Linux use a virtual environment, once, from the `GSE` folder:

```bash
python3 -m venv .venv
source .venv/bin/activate      # again in every new terminal
pip install pyserial
```

## Notes

- `firmware/diagram.json` wires Wokwi's serial port to the pad's radio UART (GPIO16/17), ties GPIO18 (radio busy) to GND, and puts a logic analyzer on GPIO4–8.
- The GUI can only open `rfc2217://` because of two changes in `software/gse_control/link.py`:
  - `serial_for_url()` in `SerialLink.open()`;
  - `SerialLink._read()` reads until nothing is waiting.
- Only the pad firmware is simulated. The ground-box behaviour (stale-command drop, SAFE stream after 300 ms of laptop silence, console) is a Python copy of `firmware/src/common/gse_tx_core.c`.
- Wokwi usually runs slower than real time, so the 1 s watchdog takes several seconds on the wall clock.
- Ports: Wokwi serves the pad's radio UART on 4000 (`firmware/wokwi.toml`) and the bridge serves the GUI on 4001. If 4001 is taken, start the bridge with `--listen <port>` and give the GUI the same port. (5000 is avoided because macOS uses it for AirPlay Receiver.)
- On macOS, Tk ignores button background colours, so the ARM, HOLD TO FIRE and valve buttons don't change colour. Read the armed state from the "arm interlock" row in PAD READ-BACK.
