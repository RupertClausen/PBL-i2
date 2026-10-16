# Electronics

Put the wiring diagram (e.g. `wiring.png` / `wiring.pdf` exported from
Fritzing, KiCad or draw.io — keep the source file too) in this folder.

## Pinout (draft — keep in sync with `software/firmware/include/robot_config.h`)

| ESP32 pin | Connected to |
|-----------|--------------|
| GPIO21 (SDA) | PCA9685 SDA, IMU SDA |
| GPIO22 (SCL) | PCA9685 SCL, IMU SCL |
| 3V3 | PCA9685 VCC (logic) |
| GND | Common ground |

## PCA9685 channels

| Channel | Servo | | Channel | Servo |
|---------|-------|-|---------|-------|
| 0 | FL hip   | | 6  | RL hip   |
| 1 | FL upper | | 7  | RL upper |
| 2 | FL lower | | 8  | RL lower |
| 3 | FR hip   | | 9  | RR hip   |
| 4 | FR upper | | 10 | RR upper |
| 5 | FR lower | | 11 | RR lower |

## Power

- Servos are powered from a dedicated high-current buck converter connected to
  the PCA9685 **V+** terminal — never from the ESP32's 5 V pin.
- All grounds (battery, buck converters, ESP32, PCA9685) must be connected.
