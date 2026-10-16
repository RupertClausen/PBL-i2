# Software

| Folder | Contents |
|--------|----------|
| [`firmware/`](firmware/) | ESP32 firmware, built with [PlatformIO](https://platformio.org/) (Arduino framework) |
| [`tools/`](tools/) | PC-side scripts: calibration helpers, gait plotting, ball-detection experiments, ... |

## Setup

1. Install [VS Code](https://code.visualstudio.com/) and the **PlatformIO IDE**
   extension (VS Code will suggest it when you open this repo), **or** install
   the CLI: `pip install platformio`.
2. Open the `software/firmware` folder in VS Code / PlatformIO.
3. Connect the ESP32 over USB and click **Upload** (→), or from a terminal:

```bash
cd software/firmware
pio run                 # build only
pio run -t upload       # build and flash
pio device monitor      # serial console (115200 baud)
```

> On Linux, if the board isn't detected, add yourself to the `dialout` group:
> `sudo usermod -aG dialout $USER` and log in again.

## Firmware structure

```
firmware/
├── platformio.ini          Board, framework and library dependencies
├── include/
│   └── robot_config.h      Pins, leg dimensions, servo channels, trims, directions
├── lib/
│   ├── Kinematics/         3-DOF leg inverse kinematics
│   └── LegServos/          PCA9685 servo output with trim / direction
├── src/
│   └── main.cpp            Setup, serial command interface, mode state machine
└── test/                   Unit tests (`pio test`)
```

### Coordinate frame (per leg)

Foot positions are given relative to that leg's hip axis, in millimetres:
**x** forward, **y** outward (away from the body), **z** down.

### Serial commands

| Command | Action |
|---------|--------|
| `center` | Move every servo to 90° + trim — do this **before** fitting servo horns |
| `stand` | Stand at `STAND_HEIGHT` using inverse kinematics |
| `relax` | Stop driving the servos |
| `help` | List commands |

## Roadmap

- [x] Servo driver + calibration (`center`)
- [x] Inverse kinematics + `stand`
- [ ] Walking gait (creep / trot)
- [ ] Stair-climbing gait for 2 cm steps (+ IMU body levelling)
- [ ] Remote control (Wi-Fi / Bluetooth / web UI)
- [ ] Tennis-ball detection and goalkeeping behaviour
