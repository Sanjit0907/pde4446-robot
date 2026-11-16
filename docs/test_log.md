# Test log
One entry per test. Numbers are copied from serial telemetry. Raw logs stay local (`logs/`, git-ignored).
Source column: **user-run** = run by the team and reported in `project_log.md`.
Exact dates of T1-T3 are still to be confirmed.

| # | Date | Who | Test | Setup | Result | Pass/Fail | Next action |
|---|---|---|---|---|---|---|---|
| T0 | TBC | Sanjit | Motors spin, encoders count, directions | wheels lifted, `teleop_test` | Both motors forward; `LEFT_ENC_INVERTED = true` needed | Pass | none |
| T1 | TBC | Sanjit | PI speed tracking, no load | wheels lifted, target 120 mm/s, KP 0.20 KI 0.50 BASE_PWM 65 | VL/VR 118-122 mm/s; PWM 28-30; ticks L/R differ by 3-4 over ~80 ticks | Pass | none |
| T2 | TBC | Sanjit | PI speed tracking on floor, USB attached | floor, target 120 mm/s | Start transient up to 195 mm/s (+63 %), settled to 119 mm/s in about 8 telemetry lines; ticks L/R within ~1.4 % | Pass (steady state) | Look at start-up overshoot before Stage 2 |
| T3 | TBC | Sanjit | Straight-line drive, 3 runs with USB cable on right | floor | Drifted right in all 3 runs | Fail | Cable drag suspected |
| T4 | TBC | Sanjit | Straight-line drive, cable not pulling | floor | Reported smooth and straight (qualitative) | Pass (qualitative) | Measure deviation over a known distance |

## Still needed for marking
- Quantified straight-line result: distance driven and sideways deviation, repeated at least 3 times.
- Calibration: counts per revolution and wheel circumference (see project log, section 5).
