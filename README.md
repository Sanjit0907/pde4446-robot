# PDE4446 Motion Control – Coursework 1

Differential-drive robot (TI-RSLK MAX chassis + Arduino Uno). Goal: drive a straight route, stop 3 times for 1 s,
detect a cone, slow down, bypass it, return to the line, show the mode on LEDs.

## Team
| Name | Student ID | Role |
|---|---|---|
| Sanjit | M01142490 | firmware / bring-up |
| (add members) | | |

## Repository layout
| Path | What it is |
|---|---|
| `firmware/teleop_test/` | Stage 0: open-loop keyboard test (motors + encoders sanity check) |
| `firmware/stage1_straight_pid/` | Stage 1: closed-loop PI wheel speed, straight line |
| `tools/teleop.py` | PC keyboard client for `teleop_test` |
| `docs/pinmap.md` | Which Uno pin goes to which chassis-board signal |
| `docs/test_log.md` | Every test: date, setup, result (needed for marking) |

## Stages
- [x] Stage 0 – motors spin, encoders count
- [ ] Stage 1 – closed-loop straight line (in progress)
- [ ] Stage 2 – calibration (wheel diameter, counts/rev, track width)
- [ ] Stage 3 – 3 stops, 1 s each
- [ ] Stage 4 – ultrasonic cone detection and bypass
- [ ] Stage 5 – status LEDs, final tuning, demo

## Rules for the team
- One small change per commit. Message says *what* and *why*.
- Everyone commits under their own name (`git config user.name`).
- Never commit a firmware change that was not tested; say in the message if it is untested.
