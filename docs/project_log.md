# TI-RSLK MAX Motion Control Coursework — Project Log

> **Project:** PDE4446 Motion Control Coursework 1  
> **Platform:** TI-RSLK MAX mobile robot  
> **Controller:** Arduino Uno  
> **Status:** Stage 1 straight-line control tested successfully; Stage 2 controlled-stop sequence is next.  
> **Last updated:** 16 November 2026

---

## 1. Project objective

Develop a motion-control system for the TI-RSLK MAX mobile robot that can complete the required route and behaviours.

### Coursework task requirements

1. Drive straight along a designated route.
2. Stop three times, with each stop occurring after three seconds of driving.
3. Remain stopped for one second at each stop, then resume.
4. Detect a randomly placed obstacle.
5. Reduce speed when an obstacle is detected.
6. Navigate around the obstacle.
7. Return to the original path and normal speed after avoiding it.
8. Use LED indications for relevant operating modes:
   - Normal driving
   - Stopping
   - Obstacle avoidance

### Obstacle and platform notes

- Route length: about **4.4 m**.
- The cone is placed **on the robot's straight path** (centred on the line). The robot must avoid it and then continue along the same line.
- The robot must continue past the third stop, detect the cone, slow down, avoid it and return to the line (a Stage 2 `FINISHED` state is only a temporary test ending).
- Coursework obstacle: plastic cone, approximately **130 × 130 × 165 mm**.
- Platform reference: two-wheel mobile robot.
- The obstacle-avoidance design currently planned uses a **fixed, forward-facing ultrasonic sensor**.
- A servo for rotating/scanning the ultrasonic sensor was explicitly ruled out.
- With a fixed sensor, the robot cannot scan both sides to determine which side is clearer. The initial avoidance direction will therefore be predetermined and tested. This is a design limitation to document and account for in testing.

### Submission expectations

- GitHub repository containing source code and configuration.
- README with system-design images, run instructions, results and limitations.
- Meaningful, frequent commits showing project progress and group contributions.
- 15-minute demonstration.

### Coursework marking areas

| Area | Weight |
|---|---:|
| System design and component selection | 15% |
| Straight-line control | 20% |
| Stop control | 20% |
| Obstacle detection and speed control | 15% |
| Operating modes and LEDs | 10% |
| Testing and evaluation | 10% |
| Git, code quality and documentation | 10% |

---

## 2. System architecture

| Subsystem | Selected component / approach | Purpose |
|---|---|---|
| Main controller | Arduino Uno | Runs the motion-control logic and state machine |
| Mobile platform | TI-RSLK MAX chassis | Two-wheel differential-drive robot |
| Motor drivers | TI-RSLK MAX onboard motor-driver circuitry | Drives the left and right DC motors |
| Motor feedback | Left and right quadrature encoders | Measures wheel motion and provides feedback for speed control |
| Obstacle sensor | Fixed forward-facing ultrasonic sensor (planned) | Measures distance to an obstacle ahead |
| Mode indication | LEDs (planned) | Indicates normal driving, stopping and obstacle avoidance |
| Motor control | Independent PI speed controllers | Regulates left and right wheel speeds |
| Behaviour control | Non-blocking state machine (planned) | Coordinates driving, stopping and obstacle manoeuvres |

### Development strategy

The project is being developed in stages. Each stage should be tested before the next is integrated.

1. Encoder-based straight-line wheel-speed control.
2. Three controlled stops.
3. Ultrasonic obstacle detection and speed reduction.
4. Predetermined obstacle avoidance and return to the original path.
5. State-machine integration and LED indications.
6. Testing, evaluation, GitHub documentation and demonstration preparation.

The current priority is to complete and validate each stage without destabilising the working lower-level motor and encoder code.

---

## 3. Hardware wiring reference

The following is the current working Arduino Uno pin mapping. Preserve these assignments when extending the code unless a deliberate wiring change is made and tested.

### Motor driver connections

| Signal | Arduino Uno pin |
|---|---:|
| Left motor PWM | D5 |
| Left motor direction (DIR) | D13 |
| Right motor PWM | D6 |
| Right motor direction (DIR) | D8 |
| Left motor sleep (SLPL) | D10 |
| Right motor sleep (SLPR) | D11 |

Both sleep pins must be driven HIGH to enable the motor drivers.

### Encoder connections

| Encoder signal | Arduino Uno pin |
|---|---:|
| Left encoder channel A (ELA) | D2 |
| Left encoder channel B (ELB) | D4 |
| Right encoder channel A (ERA) | D3 |
| Right encoder channel B (ERB) | D12 |

- D2 is used for the left encoder interrupt.
- D3 is used for the right encoder interrupt.
- Both encoder channels are read by the quadrature decoding logic.
- The encoder outputs are pulled up to 3.3 V by the chassis board. The current working setup uses `INPUT`, not the Arduino's internal pull-ups.
- Arduino and chassis board must share a common ground.

### Ultrasonic sensor connections (planned)

| Signal | Arduino Uno pin |
|---|---:|
| TRIG | D9 |
| ECHO | D7 |

The ultrasonic sensor is currently disabled in the working motion-control sketch. Confirm the sensor's electrical output voltage is safe for the Arduino input and verify the actual wiring before enabling it.

### Motor and encoder direction configuration

Current tested values:

```cpp
const bool LEFT_MOTOR_INVERTED = false;
const bool RIGHT_MOTOR_INVERTED = false;

const bool LEFT_ENC_INVERTED = true;
const bool RIGHT_ENC_INVERTED = false;
```

These settings produced correct motor direction and encoder direction during testing. Do not change them without a specific diagnostic reason.

---

## 4. Current software baseline

### Working sketches

- `teleop_test.ino` — earlier motor/encoder bring-up and teleoperation sketch.
- `straight_line_pid.ino` — forward-only closed-loop PI speed-control sketch used for the straight-line tests.

The latest working `straight_line_pid.ino` should be treated as the baseline for Stage 2. Keep an untouched copy or Git commit before modifying it.

### PI controller configuration

The latest tested controller settings are:

```cpp
const float KP = 0.20f;
const float KI = 0.50f;

const float BASE_PWM = 65.0f;
const float INTEGRAL_LIMIT = 60.0f;

const int PWM_MIN = 0;
const int PWM_MAX = 150;
```

Other current settings and assumptions:

| Parameter | Current value |
|---|---:|
| Target wheel speed | 120 mm/s |
| Controller update interval | 100 ms |
| Telemetry interval | 200 ms |
| Wheel diameter used in calculation | 70 mm |
| Assumed encoder counts per wheel revolution | 720 |
| Provisional distance per tick | `PI * 70 / 720` mm |
| Ultrasonic support | Disabled (`USE_ULTRASONIC 0`) |

**Calibration warning:** The 720 counts/revolution value and resulting `MM_PER_TICK` are still assumptions, not confirmed physical calibration values. Therefore, the calculated absolute speed in mm/s may be inaccurate. The current tests are useful for comparing left and right wheel speeds, but the distance and speed scale should be calibrated before relying on absolute measurements.

### Existing serial controls

The forward PI sketch has used the following commands:

| Command | Function |
|---|---|
| `w` | Start forward PI control |
| `x` | Stop |
| `+` / `-` | Increase / decrease target speed |
| `r` | Reset encoder counts |
| `p` | Toggle telemetry |

Use **115200 baud** in the Serial Monitor.

The earlier teleoperation sketch also supported additional manual movement commands. Check the actual sketch before relying on commands not listed above.

### Encoder measurement approach

The encoder interrupt routines use both quadrature channels to determine direction. The left and right tick counts are signed, and wheel-speed estimates are calculated from the change in counts over the controller sampling interval.

The current encoder approach is therefore not the earlier one-channel-only arrangement. Both A and B signals are connected and used.

---

## 5. Stage 1 — Encoder-based straight-line PI control

<details>
<summary>Objective and implementation</summary>

The objective was to make both wheels follow the same target speed using independent PI controllers.

- Left wheel has its own speed measurement and PI controller.
- Right wheel has its own speed measurement and PI controller.
- Both controllers use the same target speed.
- The motor PWM is adjusted based on each wheel's speed error.
- The controller runs periodically rather than relying on open-loop PWM alone.

Equal wheel speeds are necessary for straight travel, but they do not guarantee a perfectly straight path. Wheel diameter differences, friction, alignment, floor conditions and cable forces can still affect the physical trajectory.

</details>

### Test 1 — Wheels lifted

The robot was tested with its wheels lifted off the floor.

Representative telemetry:

```text
TARGET,120.00,VL,122.17,VR,122.17,PWM_L,29,PWM_R,28,TICKS_L,6126,TICKS_R,6129
TARGET,120.00,VL,119.12,VR,119.12,PWM_L,29,PWM_R,29,TICKS_L,6204,TICKS_R,6208
TARGET,120.00,VL,117.94,VR,117.94,PWM_L,30,PWM_R,29,TICKS_L,6282,TICKS_R,6285
TARGET,120.00,VL,122.17,VR,119.12,PWM_L,29,PWM_R,29,TICKS_L,6361,TICKS_R,6364
```

**Observation:**

- Both calculated wheel speeds stayed close to the 120 mm/s target.
- PWM values settled around 28–30.
- Left and right encoder counts were very close.
- This showed that the two PI loops were operating and synchronising the wheels under unloaded conditions.

**Limitation:** Wheels-off-the-floor testing does not reproduce the load and friction experienced during normal driving.

### Test 2 — Floor test with USB cable connected

The robot was tested on the floor at a target of 120 mm/s. It initially appeared to drift right.

Representative telemetry:

```text
TARGET,120.00,VL,128.28,VR,109.96,PWM_L,28,PWM_R,38,TICKS_L,8608,TICKS_R,8476
TARGET,120.00,VL,183.26,VR,183.26,PWM_L,55,PWM_R,55,TICKS_L,8744,TICKS_R,8604
TARGET,120.00,VL,195.48,VR,192.42,PWM_L,42,PWM_R,43,TICKS_L,8887,TICKS_R,8744
TARGET,120.00,VL,177.15,VR,167.99,PWM_L,40,PWM_R,43,TICKS_L,9002,TICKS_R,8853
TARGET,120.00,VL,155.77,VR,149.66,PWM_L,36,PWM_R,40,TICKS_L,9217,TICKS_R,9059
TARGET,120.00,VL,137.44,VR,137.44,PWM_L,34,PWM_R,38,TICKS_L,9408,TICKS_R,9246
TARGET,120.00,VL,119.12,VR,119.12,PWM_L,31,PWM_R,32,TICKS_L,10241,TICKS_R,10086
TARGET,120.00,VL,119.12,VR,119.12,PWM_L,31,PWM_R,32,TICKS_L,11184,TICKS_R,11033
```

**Interpretation:**

- The initial readings include startup/transient behaviour and should not be treated as steady-state performance.
- After settling, wheel speeds were generally close to the 120 mm/s target.
- The cumulative tick changes over the later portion of the test were very similar.
- The telemetry did not establish a persistent wheel-speed imbalance large enough to explain a consistent rightward drift.
- The physical drift was therefore investigated separately rather than immediately changing PI gains.

### Test 3 — Straight-line test with USB cable removed from the robot

Three physical runs were performed:

| Run | Observation |
|---|---|
| 1 | Drifted right |
| 2 | Drifted right |
| 3 | Drifted right |

The USB cable was suspected of pulling the robot sideways because it was positioned towards the right side.

The cable was then removed from the robot for the physical straight-line check. The user reported that the robot travelled **smoothly and straight**.

**Conclusion:**

- USB cable drag was the likely cause of the earlier rightward drift.
- The physical straight-line test passed once the cable force was removed.
- No PI gain or motor-direction change is currently justified.
- Future floor tests should avoid cable drag. If serial communication is needed, arrange the cable with slack and support it so it cannot pull the chassis.

<details>
<summary>Stage 1 acceptance status</summary>

- [x] Both motors operate.
- [x] Both encoders provide feedback.
- [x] Motor direction verified.
- [x] Encoder direction verified.
- [x] Independent PI wheel-speed control implemented.
- [x] Wheel-speed tracking tested with wheels lifted.
- [x] Wheel-speed tracking tested on the floor.
- [x] Physical straight-line movement checked.
- [x] Cable drag identified as the likely cause of the earlier drift.
- [ ] Calibrate encoder counts per revolution and effective wheel circumference for accurate distance/speed measurements.
- [ ] Capture final repeatable straight-line test data without cable drag for the project report.

**Stage 1 status: functionally passed for the current development baseline.**

</details>

---

## 6. Power and serial connection notes

- The Arduino Uno is currently powered through its USB connection to the laptop.
- The USB connection is also being used to send commands through the Serial Monitor.
- The robot's straight-line test was affected by the USB cable physically pulling the chassis when the cable was positioned to the right.
- For physical testing, the cable must not exert force on the robot. The successful straight-line check was performed with the cable removed from the robot.
- Plan for the final stage only: power the Uno from the chassis board's 5 V rail and use the Uno **reset button as the start button** (the sketch must then start by itself a few seconds after boot, with a visible countdown). Not yet adopted or verified. Watch for Uno resets caused by supply dips when the motors start.
- Do not connect external 5 V and USB power in a way that could cause supply contention or backfeeding. Verify the exact power arrangement before attempting simultaneous external power and USB serial communication.

---

## 7. Stage 2 — Three controlled stops (next task)

### Required sequence

The robot should:

1. Drive forward for 3 seconds.
2. Stop for 1 second.
3. Resume forward driving for another 3 seconds.
4. Repeat until it has completed three stops.
5. Stop permanently after the third stop.

The intended interpretation is **three stop events**, each following a three-second active-driving interval. This results in three one-second pauses, followed by a final stop after the third pause.

### Planned implementation

Use a non-blocking state machine driven by `millis()`.

Proposed states:

- `IDLE`
- `DRIVING`
- `STOPPED`
- `FINISHED`

Proposed behaviour:

- On start, enter `DRIVING`.
- While driving, keep the existing PI speed controller active.
- When three seconds of active driving have elapsed, command both motors to stop and enter `STOPPED`.
- Remain stopped for one second.
- Increment the stop counter and resume driving if fewer than three stops have been completed.
- After the third one-second stop, enter `FINISHED` and keep both motors stopped.
- The driving timer must not count time spent in the one-second stop state.
- Avoid long blocking `delay()` calls so the control loop and serial interface remain responsive.

### Integration constraints

- Start from the latest working `straight_line_pid.ino`.
- Preserve the current motor pin mapping.
- Preserve the current encoder pin mapping and direction inversion values.
- Preserve the current PI gains and limits initially.
- Preserve the 120 mm/s target initially.
- Do not introduce ultrasonic behaviour yet.
- Test the state sequence first with the wheels lifted, then perform a controlled floor test in a clear area.
- Keep a known-good Stage 1 version in Git before modifying it.

### Stage 2 test checklist

- [ ] Robot starts driving when commanded.
- [ ] First driving interval lasts 3 seconds.
- [ ] First stop lasts 1 second.
- [ ] Robot resumes driving.
- [ ] Second driving interval lasts 3 seconds.
- [ ] Second stop lasts 1 second.
- [ ] Robot resumes driving.
- [ ] Third driving interval lasts 3 seconds.
- [ ] Third stop lasts 1 second.
- [ ] Robot remains stopped after completing the sequence.
- [ ] Serial telemetry and control remain responsive.
- [ ] Timing is repeatable and documented.

---

## 8. Later development stages

### Stage 3 — Obstacle detection and speed reduction

- Connect and test the fixed forward-facing ultrasonic sensor.
- Use D9 for TRIG and D7 for ECHO, subject to final wiring verification.
- Establish a reliable distance measurement.
- Define a detection threshold based on the robot's speed, braking distance and sensor behaviour.
- Reduce the target speed when an obstacle is detected.
- Test detection and slowdown with the coursework cone.

### Stage 4 — Obstacle avoidance and path recovery

- Use a predetermined avoidance direction because the sensor is fixed and does not scan.
- Design a controlled manoeuvre to move around the cone.
- Return the robot to the original route.
- Restore normal target speed after successful avoidance.
- Document the limitation that the robot cannot inspect both sides before choosing a direction.
- Test repeatability and ensure the manoeuvre does not collide with the obstacle.

### Stage 5 — Full state machine and LEDs

Integrate the required behaviours into a coherent state machine.

Expected operating modes:

- Normal driving
- Stopping
- Obstacle detected / reduced speed
- Obstacle avoidance
- Return to path
- Finished / idle

Add LED indications for the modes required by the coursework. Verify that each LED state corresponds to the actual controller state.

### Stage 6 — Testing, evaluation and documentation

- Record repeatable test procedures and results.
- Measure straight-line deviation and stop timing.
- Record obstacle-detection distance and slowdown behaviour.
- Evaluate obstacle-avoidance success and path recovery.
- Document limitations, calibration assumptions and failure cases.
- Prepare system diagrams, wiring diagrams and demonstration instructions.
- Ensure Git history shows meaningful progress and contributions.

---

## 9. Git workflow

Suggested repository layout:

```text
ti-rslk-motion-control/
├── README.md
├── docs/
│   ├── system_design.md
│   ├── wiring.md
│   ├── test_results.md
│   └── images/
├── firmware/
│   ├── stage1_straight_line/
│   │   └── straight_line_pid.ino
│   ├── stage2_controlled_stops/
│   │   └── controlled_stops.ino
│   └── final_robot/
│       └── final_robot.ino
└── .gitignore
```

Keep the exact tested Stage 1 sketch unchanged in its own folder. Develop Stage 2 in a separate sketch until it has passed testing.

### Suggested commit sequence

Use small commits that describe one meaningful change. Examples:

```text
docs: add coursework requirements and project plan
docs: record TI-RSLK MAX pin mapping
feat: add encoder-based left and right wheel speed measurement
feat: implement independent PI wheel speed controllers
test: record unloaded wheel speed tracking results
test: record floor straight-line test results
docs: document USB cable drag during straight-line testing
feat: add non-blocking three-stop state machine
test: verify three drive-and-stop cycles
feat: add ultrasonic obstacle detection
feat: add obstacle speed reduction
feat: implement obstacle avoidance and path recovery
feat: add operating-mode LED indications
docs: add final test results and known limitations
```

Do not make a commit claiming a test passed until the test has actually been run.

---

## 10. Current project status

| Stage | Status | Notes |
|---|---|---|
| Git repository | Created 16 Nov 2026 | Stage 1 baseline, pin map, test log committed |
| Hardware bring-up | Complete | Motors and both encoders working |
| Stage 1: straight-line PI control | Tested / passed | Smooth straight movement after removing USB cable drag |
| Encoder and wheel calibration | Pending | 720 counts/rev and 70 mm wheel diameter are provisional |
| Stage 2: three controlled stops | Next | Non-blocking `millis()` state machine |
| Stage 3: obstacle detection and slowdown | Not started | Ultrasonic sensor not yet connected or tested |
| Stage 4: obstacle avoidance and recovery | Not started | Predetermined avoidance direction |
| Stage 5: LED and full state-machine integration | Not started | Integrate after individual behaviours work |
| Stage 6: final evaluation and documentation | Not started | Results, limitations, diagrams and demo |

### Immediate next action

Upload or add the latest working `straight_line_pid.ino` to the repository. Commit it as the Stage 1 baseline. Then implement Stage 2 in a separate sketch, preserving the known-good Stage 1 version.

---

## 11. Engineering notes and known limitations

1. **Encoder calibration:** The assumed 720 counts/revolution and 70 mm wheel diameter have not yet been independently calibrated. Absolute distance and speed estimates need validation.
2. **Straight-line accuracy:** Matching wheel speeds does not alone guarantee a straight trajectory. Physical testing remains necessary.
3. **USB cable force:** A cable pulling on the chassis can create apparent steering drift. Keep it disconnected from the robot during physical tests, or arrange it so it exerts no force.
4. **Fixed ultrasonic sensor:** A forward-facing sensor cannot scan for a safe side to pass around an obstacle. The avoidance direction is predetermined and must be tested under the coursework's placement conditions.
5. **Power arrangement:** The Arduino is currently USB-powered. Chassis 5 V powering has not been verified as part of the working setup.
6. **Controller tuning:** Current PI gains are retained because the latest tests did not justify changing them. Revisit only if repeatable measurements show a control problem.

---

## 12. Change log

### 16 November 2026 (update 3)

- Cone confirmed to be placed on the robot's path. Avoid it and rejoin the same line.
- Raising the cruise speed to about 365 mm/s is NOT a requirement. Open decision: how to place the three stops (time-based at 120 mm/s vs evenly spaced along 4.4 m).

### 16 November 2026 (update 2)

- Team: Havinesh Kumar (M01152797) added.
- Route length recorded as about 4.4 m. Cone must be detected and avoided after the stops.
- T4 run length recorded as about 1 m, deviation not yet measured.
- Test dates recorded as the 8-16 Nov period (exact dates not logged).
- Ultrasonic sensor confirmed as not started.
- Final start method: reset button, Uno powered from chassis 5 V (final stage only).

### 16 November 2026 (repository)

- Git repository created. Stage 1 sketch committed as `firmware/stage1_straight_line/straight_line_pid.ino`, unchanged.
- Pin map committed in `docs/pinmap.md`; planned ultrasonic pins TRIG D9 / ECHO D7.
- Stage 1 results copied into `docs/test_log.md` (T0-T4). Exact test dates still to be confirmed.
- Open items: team member list, quantified straight-line data, calibration, start-up overshoot check, definition of behaviour after the third stop.

### 16 November 2026

- Consolidated coursework requirements and staged development plan.
- Recorded current TI-RSLK MAX to Arduino Uno motor and encoder pin mapping.
- Recorded tested motor and encoder inversion settings.
- Recorded current PI gains, target speed and provisional encoder calibration assumptions.
- Documented unloaded and floor PI controller tests.
- Documented the initial rightward drift and the successful straight-line test after removing USB cable drag.
- Marked Stage 1 as functionally passed for the current baseline.
- Defined Stage 2 as the next task: three 3-second driving intervals, each followed by a 1-second stop, using a non-blocking state machine.
