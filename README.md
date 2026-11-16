# PDE4446 Motion Control – Coursework 1

Differential-drive robot (TI-RSLK MAX chassis + Arduino Uno). Goal: drive a straight route, stop 3 times for 1 s,
detect a cone, slow down, bypass it, return to the line, show the mode on LEDs.

## Team
| Name | Student ID | Role |
|---|---|---|
| Sanjit | M01142490 | firmware / bring-up |
| Havinesh Kumar | M01152797 | role to be confirmed |

## Repository layout
| Path | What it is |
|---|---|
| `firmware/teleop_test/` | Stage 0: open-loop keyboard test (motors + encoders sanity check) |
| `firmware/stage1_straight_line/` | Stage 1: closed-loop PI wheel speed, straight line (tested) |
| `firmware/stage2_controlled_stops/` | Stage 2: three 3 s legs with 1 s stops (untested) |
| `tools/teleop.py` | PC keyboard client for `teleop_test` |
| `docs/pinmap.md` | Which Uno pin goes to which chassis-board signal |
| `docs/test_log.md` | Every test: date, setup, result (needed for marking) |
| `docs/project_log.md` | Requirements, plan, status and change log |

## Stages
- [x] Hardware bring-up – motors spin, encoders count
- [x] Stage 1 – closed-loop straight line (functionally passed; calibration pending)
- [ ] Calibration – counts/rev, wheel diameter
- [ ] Stage 2 – three controlled stops (code written, untested)
- [ ] Stage 3 – ultrasonic detection and speed reduction
- [ ] Stage 4 – obstacle avoidance and return to path
- [ ] Stage 5 – state machine and LEDs
- [ ] Stage 6 – evaluation, documentation, demo

## Rules for the team
- One small change per commit. Message says *what* and *why*.
- Everyone commits under their own name (`git config user.name`).
- Never commit a firmware change that was not tested; say in the message if it is untested.
