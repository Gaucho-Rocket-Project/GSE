# Arduino IDE sketches

The same firmware as the PlatformIO project, for flashing from the Arduino IDE. Setup:
`laptop → ESP32S ground box → (ESP-NOW) → ESP32-S3 vehicle`.

| Sketch | Board | Arduino IDE settings |
|---|---|---|
| `gse_tx_esp32s/` | ESP32S ground box | Board **ESP32 Dev Module**; Port: the ESP32S (CP210x) |
| `gse_pad_esp32s3/` | ESP32-S3 vehicle | Board **ESP32S3 Dev Module**; **USB CDC On Boot: Disabled**; Port: the S3's UART socket |
| `gse_servo_sweep_esp32s3/` | ESP32-S3 vehicle, **bench test only** | As above; needs the **ESP32Servo** library. Sweeps the 3 servos 0–180° (500–2500 µs) on the pad's pins, igniter held off. Flash `gse_pad_esp32s3` back afterwards. |

- Needs the esp32 board package (Boards Manager → "esp32" by Espressif). Tested with 3.3.12.
- With USB CDC On Boot disabled, the vehicle's log (Serial Monitor, 115200) comes out of the same UART socket you flash through.
- If an upload stalls at "Connecting…", hold BOOT. If it fails partway, set Upload Speed to 115200.

## These folders are generated: don't edit them

`firmware/src` is the source of truth. The Arduino IDE can't use PlatformIO's include paths or build flags, so `tools/make_arduino_sketches.py`:
- copies each node's files unchanged into one flat folder;
- writes the build flags (which node, which board) into `build_opt.h`, which the ESP32 board package passes to the compiler;
- adds an `.ino`, which only holds instructions; `setup()` and `loop()` are in the copied `.cpp`.

**After any change in `firmware/src`** (MAC addresses, servo travel, pins…), regenerate before flashing from Arduino. From the `GSE` folder:

```
python tools/make_arduino_sketches.py
```

The script deletes and recreates the two sketch folders. It leaves this README alone.
