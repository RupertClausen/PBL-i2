# Requirements

## Platform constraints

- Quadruped (four legs) actuated by **hobby RC servos**.
- Controlled by an **ESP32**.

## Task 1 — Stair climbing

| Item | Requirement |
|------|-------------|
| Step height | 2 cm |
| Number of steps | _TBD_ |
| Step depth (tread) | _TBD_ |
| Surface | _TBD_ |
| Time limit | _TBD_ |

**Success criteria:** _TBD — e.g. robot reaches the top step without falling or
being touched._

## Task 2 — Goalkeeping

| Item | Requirement |
|------|-------------|
| Ball | Tennis ball (≈ 6.7 cm diameter, ≈ 58 g) |
| Goal width × height | _TBD_ |
| Shooting distance | _TBD_ |
| Ball launched by | _TBD (hand / launcher / rolled)_ |
| Sensing allowed | _TBD (camera, distance sensors, remote control?)_ |

**Success criteria:** _TBD — e.g. number of blocked shots out of N._

## Design implications (to be confirmed)

- A 2 cm step requires a foot lift comfortably above 2 cm (target ≥ 3 cm) while
  keeping the body stable — this drives leg length and servo torque.
- Blocking a tennis ball needs either a wide stance / body that covers the goal,
  or fast lateral motion. Decide whether the robot reacts autonomously (sensor)
  or is teleoperated over Wi-Fi/Bluetooth.

## Open questions

- [ ] Full competition rules and dimensions from the organisers
- [ ] Is remote control allowed, or must the robot be autonomous?
- [ ] Weight / size limits?
- [ ] Battery / power restrictions?
