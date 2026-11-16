# Pin map (as wired and used by `straight_line_pid`)

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
- Planned HC-SR04 (not yet wired): TRIG D9, ECHO D7 (D10 is used by nSLEEP). HC-SR04 is powered from 5 V, so its 5 V ECHO is safe for the Uno.
- Direction convention: DIR low = forward. `LEFT_ENC_INVERTED = true` was needed for the left encoder.
