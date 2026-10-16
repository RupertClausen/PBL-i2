# Contributing

This repository is shared between the KMUTNB and Kyutech team members. A few
simple rules keep it usable for everyone across two countries and time zones.

## Workflow

1. **Never commit directly to `main`.** `main` should always contain firmware
   that builds and CAD that matches the latest physical robot.
2. Create a branch for your work:
   - `sw/<short-description>` — software, e.g. `sw/stair-gait`
   - `hw/<short-description>` — hardware, e.g. `hw/leg-v2`
   - `docs/<short-description>` — documentation
3. Open a Pull Request and ask at least **one teammate from the other
   university** to review it when possible.
4. Link the PR to an issue (`Closes #12`) so we can track progress.

## Commit messages

Short, imperative, and prefixed with the area:

```
firmware: add inverse kinematics for 3-DOF leg
cad: thicken femur bracket to 3 mm
bom: switch to MG996R for hip servos
docs: add stair-climbing test results
```

## Software

- Firmware lives in `software/firmware/` and is built with PlatformIO.
- Make sure `pio run` succeeds before opening a PR.
- Put pin numbers, servo offsets and robot dimensions in
  `software/firmware/include/robot_config.h` — do not hard-code them elsewhere.
- Reusable code goes in `software/firmware/lib/<Name>/`.

## Hardware

- **SolidWorks files are the source of truth.** Whenever you change a part,
  also export an updated **STEP** (to `hardware/CAD/STEP/`) and, for printed
  parts, an **STL** (to `hardware/STLs/`) in the same PR.
- Save SolidWorks files with the **oldest SolidWorks version used by the team**
  so everyone can open them. Write the version in the PR description.
- SolidWorks files are binary and cannot be merged. **Before editing a part,
  say so in the issue or team chat** so two people don't edit the same file.
- Keep part numbers consistent between file names and the BOM (see
  [hardware/README.md](hardware/README.md#file-naming)).
- Update `hardware/BOM/BOM.csv` whenever you add, remove or change a component.

## Large files

Keep individual files under ~50 MB. Do not commit renders, videos or slicer
project files; put them on the shared drive and link them instead.
