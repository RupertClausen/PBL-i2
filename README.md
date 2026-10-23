# PBL-i2 — ESP32 Servo Quadruped

A small four-legged robot built from hobby servos and an ESP32, developed as a
Project-Based Learning (PBL) collaboration between
**King Mongkut's University of Technology North Bangkok (KMUTNB), Thailand** and
**Kyushu Institute of Technology (Kyutech), Japan**.

<!-- Add a photo or render of the robot here once one exists:
![PBL-i2](docs/images/robot.jpg) -->

## Objectives

The robot must be able to:

1. **Climb stairs** — walk up a stairway with **2 cm/3 cm** high steps.
2. **Defend a goal** — block a tennis ball from entering a goal.

See [docs/requirements.md](docs/requirements.md) for the full task description
and success criteria.

## Repository layout

```
PBL-i2/
├── software/               Everything that runs on (or talks to) the robot
│   ├── firmware/           ESP32 firmware (PlatformIO + Arduino)
│   └── tools/              PC-side helper scripts (calibration, gait plotting, ...)
├── hardware/               Everything you need to build the robot
│   ├── CAD/
│   │   ├── SolidWorks/     Native SolidWorks parts and assemblies (source of truth)
│   │   └── STEP/           STEP exports for people without SolidWorks
│   ├── STLs/               Print-ready STL files
│   ├── BOM/                Bill of materials
│   └── electronics/        Wiring diagram, pinout, power distribution
├── docs/                   Requirements, assembly guide, meeting notes, images
├── CONTRIBUTING.md         How we work together (branches, naming, reviews)
└── LICENSE                 GPLv3
```

## Quick start

**Build the robot** — start at [hardware/README.md](hardware/README.md): print
the parts in `hardware/STLs/`, order the items in
[hardware/BOM/BOM.csv](hardware/BOM/BOM.csv), and wire it up following
[hardware/electronics/](hardware/electronics/).

**Flash the firmware** — see [software/README.md](software/README.md). In short:

```bash
pip install platformio          # or install the PlatformIO extension for VS Code
cd software/firmware
pio run -t upload               # build and flash
pio device monitor              # serial console at 115200 baud
```

## Team

| Name | University | Role |
|------|------------|------|
| Takahashi Yuuki     | Kyutech    | Software     |
| Dambara Riki     | Kyutech    | Software     |
| Mizutani Mutsumi     | Kyutech    | Hardware     |
| Yamaguchi Asuka     | Kyutech    | Hardware     |
| Jitmiyom Supicha     | KMUTNB     | Main Designer     |
| Raungrit Punroon     | KMUTNB     | Sup Designer     |
| Thoetseritham Matsakon     | KMUTNB     | Software     |
| Sugihara Kotaro     | Kyutech    | Hardware     |

## Contributing

Please read [CONTRIBUTING.md](CONTRIBUTING.md) before opening a pull request.
Issues and discussions can be written in English; Thai or Japanese notes are
welcome as long as an English summary is included.

## License

This project — software, CAD and documentation — is released under the
[GNU General Public License v3.0](LICENSE).
