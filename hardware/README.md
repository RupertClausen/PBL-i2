# Hardware

Everything needed to physically build the robot.

| Folder | Contents |
|--------|----------|
| [`CAD/SolidWorks/`](CAD/SolidWorks/) | Native SolidWorks parts (`.SLDPRT`), assemblies (`.SLDASM`) and drawings — **source of truth** |
| [`CAD/STEP/`](CAD/STEP/) | STEP exports of every part and the top-level assembly, for any CAD tool |
| [`STLs/`](STLs/) | Print-ready STL files, already oriented for printing |
| [`BOM/`](BOM/) | Bill of materials (`BOM.csv`) |
| [`electronics/`](electronics/) | Wiring diagram, pinout and power distribution |

## Robot layout

- 4 legs × 2 DOF (hip abduction, hip/shoulder) = **8 servos**
- Leg naming used everywhere (CAD, firmware, BOM):

```
        FRONT
   FL ┌───────┐ FR
      │       │
      │ body  │
      │       │
   RL └───────┘ RR
        REAR
```

  `FL` front-left, `FR` front-right, `RL` rear-left, `RR` rear-right.
  Joints: `hip` (abduction), `upper` (shoulder/thigh), `lower` (knee/shin).

## File naming

Part files use a short, consistent name shared by SolidWorks, STEP and STL:

```
<assembly>_<part>[_<variant>]          e.g.  leg_femur, leg_tibia, body_top_plate
```

- Parts that are mirrored for left/right sides get `_L` / `_R`.
- If a part must be printed more than once, add `_x<qty>` to the **STL name
  only**, e.g. `leg_femur_x4.stl` (same idea as Voron's `_x2` suffix).
- Parts that can be printed in an accent colour get an `[a]_` prefix, e.g.
  `[a]_body_cover.stl` (Voron convention).

## Exporting from SolidWorks

When you change a part, export in the same commit:

1. **STEP**: *File → Save As → STEP AP214 (\*.step)* → `CAD/STEP/<name>.step`
2. **STL** (printed parts only): *File → Save As → STL*, Options → Units:
   **millimetres**, Resolution: **Fine**, "Do not translate STL output data to
   positive space" **unchecked** → `STLs/<name>.stl`

Also export the top-level assembly to `CAD/STEP/PBL-i2_full_assembly.step`
whenever the design changes significantly.

## Printing

Recommended defaults (adjust per part and note exceptions here):

| Setting | Value |
|---------|-------|
| Material | ABS |
| Layer height | 0.2 mm |
| Walls / perimeters | 3 |
| Top / bottom layers | 3 |
| Infill | 30–40 % gyroid |
| Supports | Only where noted |

Load-bearing leg parts (femur, tibia, servo brackets) should be printed with
higher infill and oriented so layer lines are not loaded in tension.
