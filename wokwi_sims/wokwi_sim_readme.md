# Wokwi simulation with run_gui.py

Runs the real GUI against the real pad-node firmware (ESP32-S3 only!) in Wokwi:

`run_gui.py` → `wokwi_bridge.py` (stands in for the TX node and both radios) → Wokwi pad

**Needs:** VS Code with the Wokwi extension, Python 3 with `pip install pyserial`, and PlatformIO if you rebuild.

## Run (from the `GSE` folder)

1. **Build the firmware** (only if `firmware/.pio/build/pad-node/firmware.bin` is missing or out of date): `cd firmware && pio run -e pad-node`
2. **Start the bridge** and leave it running: `python wokwi_sims/wokwi_bridge.py`
3. **Start the simulation:** in VS Code, run **Wokwi: Start Simulator** using `firmware/wokwi.toml`, and keep the simulator tab visible.
4. **Start the GUI** in another terminal, then click **Connect**: `python software/run_gui.py --port rfc2217://localhost:5000`
5. **Check the pins:** stop the simulation, then run `python wokwi_sims/wokwi_pad_test.py inspect` to see what each servo, igniter and heartbeat pin actually did.

Pad events, configuration sent, and the radio set-up result print in the bridge terminal.

**Automated check (no GUI):** start `python wokwi_sims/wokwi_pad_test.py run -v` first, then the simulation.

## Notes

- `firmware/diagram.json` wires Wokwi's serial port to the pad's radio UART (GPIO16/17), ties GPIO18 (radio busy) to GND, and puts a logic analyzer on GPIO4–8.
- The GUI can only open `rfc2217://` because of two changes in `software/gse_control/link.py`:
  - `serial_for_url()` in `SerialLink.open()`;
  - `SerialLink._read()` reads until nothing is waiting.
- Only the pad firmware is simulated. The ground-box behaviour (stale-command drop, SAFE stream after 300 ms of laptop silence, console) is a Python copy of `firmware/src/common/gse_tx_core.c`.
- Wokwi usually runs slower than real time, so the 1 s watchdog takes several seconds on the wall clock.
