# tof-grid — VL53L5CX 8×8 ToF live view

Bench tool for trying out the **VL53L5CX** 8×8 multizone time-of-flight sensor
(e.g. for tennis-ball detection). An ESP32-S3 reads the sensor and streams
frames over USB; a small Python script shows them as a live colour grid in the
browser.

This is separate from the robot firmware in `software/firmware/`.

## Hardware

| Sensor pin | ESP32-S3 (N16R8) |
|------------|------------------|
| SDA        | GPIO 8           |
| SCL        | GPIO 9           |
| LPn        | GPIO 6           |
| INT        | GPIO 5           |
| VIN / GND  | 3V3 / GND        |

Plug the S3 in through its native USB port (it shows up as `/dev/ttyACM*` on
Linux).

## Flash the firmware

```bash
cd software/tools/tof-grid/firmware
pio run -t upload
```

The board is configured for the **N16R8** module (16 MB flash, 8 MB octal
PSRAM). For other S3 modules change `board_build.arduino.memory_type` /
`board_upload.flash_size` in `platformio.ini`.

At boot the firmware uploads the sensor's internal firmware (~1.5 s), then
streams 8×8 frames at 15 Hz, one line each:

```
F <64 distances in mm> | <64 target status codes>
```

Status 5 and 9 are valid ranges; 255 means no target. Lines starting with `#`
are log messages.

## Run the viewer

```bash
pip install -r software/tools/requirements.txt
python3 software/tools/tof-grid/viewer.py
```

It auto-detects the ESP32's serial port (or pass `--port /dev/ttyACM0` /
`--port COM5`) and opens <http://localhost:8766>. The page shows distances,
frame rate, nearest/mean distance, and has controls for colour range and
mirror / flip / rotate so the grid matches how the sensor is mounted.

Stop the viewer before re-flashing — it holds the serial port open.
