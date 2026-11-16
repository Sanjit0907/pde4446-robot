# Test log
One entry per test. Numbers are copied from serial telemetry. Raw logs stay local (`logs/`, git-ignored).
Source column: **user-run** = run by the team and reported in `project_log.md`.
Work started 8 Nov 2026. Exact test dates were not recorded; entries show the period 8-16 Nov 2026 and will be
corrected if the real dates are found (file timestamps, saved serial logs).

| # | Date | Who | Test | Setup | Result | Pass/Fail | Next action |
|---|---|---|---|---|---|---|---|
| T0 | 8-16 Nov 2026 | Sanjit | Motors spin, encoders count, directions | wheels lifted, `teleop_test` | Both motors forward; `LEFT_ENC_INVERTED = true` needed | Pass | none |
| T1 | 8-16 Nov 2026 | Sanjit | PI speed tracking, no load | wheels lifted, target 120 mm/s, KP 0.20 KI 0.50 BASE_PWM 65 | VL/VR 118-122 mm/s; PWM 28-30; ticks L/R differ by 3-4 over ~80 ticks | Pass | none |
| T2 | 8-16 Nov 2026 | Sanjit | PI speed tracking on floor, USB attached | floor, target 120 mm/s | Start transient up to 195 mm/s (+63 %), settled to 119 mm/s in about 8 telemetry lines; ticks L/R within ~1.4 % | Pass (steady state) | Look at start-up overshoot before Stage 2 |
| T3 | 8-16 Nov 2026 | Sanjit | Straight-line drive, 3 runs with USB cable on right | floor | Drifted right in all 3 runs | Fail | Cable drag suspected |
| T4 | 8-16 Nov 2026 | Sanjit | Straight-line drive, cable not pulling | floor | Run of about 1 m, reported smooth and straight; sideways deviation not measured | Pass (qualitative) | Repeat over the 4.4 m route and measure deviation |

## Still needed for marking
- Quantified straight-line result: sideways deviation over a known distance (route is about 4.4 m), repeated at least 3 times.
- Calibration: counts per revolution and wheel circumference (see project log, section 5).

## Calibration procedure (not yet done)
`MM_PER_TICK` is assumed, so every distance and speed in mm is unverified.

**C1 - distance scale.** Mark 1000 mm on the floor with tape. Press `r`, drive the robot from the
start mark to the end mark, press `x`. Record `TICKS_L` and `TICKS_R`.
Then `MM_PER_TICK = 1000 / average(TICKS_L, TICKS_R)`, and
`COUNTS_PER_REV = (PI * 70) / MM_PER_TICK`.

**C2 - feedforward fit.** With the wheels lifted, hold a fixed PWM and record the steady speed at
PWM = 20, 30, 40, 60, 80. Straight-line fit gives `FF_OFFSET` and `FF_SLOPE`.
Current values (8 and 0.18) are fitted to a single point and are a guess.

## Stage 2 test checklist (not yet run)
| Check | Result |
|---|---|
| Three stops happen, in order | |
| Each driving leg is 3 s (from `# STATE ->` timestamps) | |
| Each dwell is ~1000 ms (printed as `dwell NNN ms`) | |
| Robot resumes after each stop | |
| Robot keeps driving after the third stop | |
| Start-up overshoot gone (peak VL/VR vs 120 target) | |
| Serial stays responsive throughout | |
