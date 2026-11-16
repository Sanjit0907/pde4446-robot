# Pin map (as wired and used by `stage1_straight_pid`)

| Uno pin | Chassis board signal | Function |
|---|---|---|
| D2 (INT0) | ELA (P10.5) | left encoder A |
| D4 | ELB (P5.2) | left encoder B |
| D3 (INT1) | ERA (P10.4) | right encoder A |
| D12 | ERB (P5.0) | right encoder B |
| D5 (PWM) | PWML (P2.7) | left motor speed |
| D13 | DIRL (P5.4) | left motor direction |
| D6 (PWM) | PWMR (P2.6) | right motor speed |
| D8 | DIRR (P5.5) | right motor direction |
| D10 | SLPL (P3.7) | left driver nSLEEP (HIGH = awake) |
| D11 | SLPR (P3.6) | right driver nSLEEP (HIGH = awake) |
| GND | GND | common ground |

Notes
- ELA/ERA are only on the 6-pin motor connectors, not on a header.
- `teleop_test` uses different pins for DIRL (D7) and one shared nSLEEP (D11). This table is the source of truth;
  change `teleop_test` to match if it is used again.
- D10 is taken by nSLEEP, so the HC-SR04 ECHO cannot use D10 (planned: TRIG D9, ECHO must move, e.g. A4/A5 or D7).
- Direction convention: DIR low = forward. `LEFT_ENC_INVERTED = true` was needed for the left encoder.
