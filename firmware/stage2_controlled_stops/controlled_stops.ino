#include <Arduino.h>

// ============================================================
// TI-RSLK MAX - STAGE 2: THREE CONTROLLED STOPS
// Arduino Uno
//
// Built on the tested Stage 1 sketch (firmware/stage1_straight_line).
// Unchanged from Stage 1: pin map, encoder decoding, motor inversion
// flags, PI gains, 120 mm/s target.
//
// New in Stage 2:
//   1. Non-blocking state machine: IDLE -> DRIVING -> STOPPED -> ... -> FINISHED
//      Drive 3 s, stop 1 s, three times, then drive on to the end of the route.
//   2. Target-speed ramp, to remove the start-up overshoot seen in test T2
//      (speed reached ~195 mm/s against a 120 mm/s target).
//   3. Speed-dependent feedforward instead of a fixed BASE_PWM (see note below).
//   4. Stop dwell is timed from measured standstill, so each stop is a true 1 s.
//   5. Mode LEDs (harmless if nothing is wired to A0/A1/A2).
//
// Serial commands:
//   w = start run     x = stop/abort     r = reset counts
//   + / - = target speed     p = toggle telemetry
// ============================================================

// -------------------- MOTOR PINS ----------------------------

const uint8_t PIN_PWM_L   = 5;
const uint8_t PIN_DIR_L   = 13;
const uint8_t PIN_PWM_R   = 6;
const uint8_t PIN_DIR_R   = 8;

const uint8_t PIN_SLEEP_L = 10;
const uint8_t PIN_SLEEP_R = 11;

// -------------------- ENCODER PINS --------------------------

const uint8_t PIN_ENC_L_A = 2;
const uint8_t PIN_ENC_L_B = 4;

const uint8_t PIN_ENC_R_A = 3;
const uint8_t PIN_ENC_R_B = 12;

// -------------------- MODE LEDS -----------------------------
// Wire: pin -> 220 ohm -> LED -> GND. Safe to leave unwired.

#define USE_LEDS 1

const uint8_t PIN_LED_DRIVE = A0;   // green  - normal driving
const uint8_t PIN_LED_STOP  = A1;   // red    - stopped
const uint8_t PIN_LED_AVOID = A2;   // yellow - obstacle mode (Stage 3/4, unused here)

// -------------------- DIRECTION FLAGS -----------------------
// Tested values from Stage 1. Do not change without a diagnostic reason.

const bool LEFT_MOTOR_INVERTED  = false;
const bool RIGHT_MOTOR_INVERTED = false;

const bool LEFT_ENC_INVERTED  = true;
const bool RIGHT_ENC_INVERTED = false;

// -------------------- WHEEL PARAMETERS ----------------------
// NOT YET CALIBRATED. Distances and speeds in mm are only as good as these two
// numbers. Calibrate before trusting ROUTE_LENGTH_MM (see docs/project_log.md).

const float WHEEL_DIAMETER_MM = 70.0f;
const float COUNTS_PER_REV    = 720.0f;

const float MM_PER_TICK = (PI * WHEEL_DIAMETER_MM) / COUNTS_PER_REV;

// -------------------- MISSION PARAMETERS --------------------

const uint8_t  NUM_STOPS        = 3;
const uint32_t DRIVE_LEG_MS     = 3000;   // "stop three times every three seconds"
const uint32_t STOP_DWELL_MS    = 1000;   // "wait for 1 second on each stop"
const uint32_t STANDSTILL_MS    = 500;    // max wait for the wheels to actually stop
const float    STANDSTILL_MM_S  = 15.0f;  // below this counts as stopped

// End of route. Depends on the uncalibrated constants above.
// Set to 0 to drive until 'x' instead of stopping at a distance.
const float ROUTE_LENGTH_MM = 4400.0f;

// Auto-start after boot, for running from the reset button with no PC.
// 0 = disabled (wait for 'w'). Final stage: set to e.g. 3000.
const uint32_t AUTOSTART_MS = 0;

// -------------------- PI CONTROLLER -------------------------

float KP = 0.20f;
float KI = 0.50f;

float targetSpeed = 120.0f;   // mm/s, commanded
float rampedTarget = 0.0f;    // mm/s, what the PI loop actually chases

// Acceleration limit for the ramp. 400 mm/s^2 reaches 120 mm/s in 0.3 s.
const float ACCEL_MM_S2 = 400.0f;

// Feedforward: approximate PWM needed to hold a given speed.
//   pwm_ff = FF_OFFSET + FF_SLOPE * speed
// Fitted to ONE data point from test T1/T2 (PWM ~29 held 120 mm/s):
//   8 + 0.18 * 120 = 29.6
// The old fixed BASE_PWM = 65 was more than twice what the motors needed, which
// is what caused the start-up overshoot. These two numbers still need a proper
// measurement: see the PWM sweep in docs/test_log.md.
const float FF_OFFSET = 8.0f;
const float FF_SLOPE  = 0.18f;

const int PWM_MIN = 0;
const int PWM_MAX = 150;

const float INTEGRAL_LIMIT = 60.0f;

// -------------------- TIMING --------------------------------

const uint32_t CONTROL_INTERVAL_MS   = 100;
const uint32_t TELEMETRY_INTERVAL_MS = 200;

// -------------------- ENCODER COUNTS ------------------------

volatile long ticksL = 0;
volatile long ticksR = 0;

void isrLeft() {
  int8_t d =
    (digitalRead(PIN_ENC_L_A) != digitalRead(PIN_ENC_L_B))
    ? 1 : -1;

  ticksL += LEFT_ENC_INVERTED ? -d : d;
}

void isrRight() {
  int8_t d =
    (digitalRead(PIN_ENC_R_A) != digitalRead(PIN_ENC_R_B))
    ? 1 : -1;

  ticksR += RIGHT_ENC_INVERTED ? -d : d;
}

long readTicks(volatile long &counter) {
  noInterrupts();
  long value = counter;
  interrupts();
  return value;
}

// -------------------- MOTOR CONTROL -------------------------

void driveMotor(
  uint8_t pwmPin,
  uint8_t dirPin,
  bool inverted,
  int pwm
) {
  pwm = constrain(pwm, -255, 255);

  if (pwm == 0) {
    analogWrite(pwmPin, 0);   // EN low with nSLEEP high = brake
    return;
  }

  bool forward = (pwm > 0) != inverted;

  digitalWrite(dirPin, forward ? LOW : HIGH);
  analogWrite(pwmPin, abs(pwm));
}

void stopMotors() {
  driveMotor(PIN_PWM_L, PIN_DIR_L, LEFT_MOTOR_INVERTED, 0);
  driveMotor(PIN_PWM_R, PIN_DIR_R, RIGHT_MOTOR_INVERTED, 0);
}

// -------------------- PI CONTROLLER -------------------------

float integralL = 0.0f;
float integralR = 0.0f;

int pwmL = 0;
int pwmR = 0;

int calculatePI(
  float target,
  float measured,
  float &integral,
  float dt
) {
  float error = target - measured;

  float feedforward = (target > 1.0f)
    ? (FF_OFFSET + FF_SLOPE * target)
    : 0.0f;

  float candidateIntegral =
    integral + KI * error * dt;

  candidateIntegral = constrain(
    candidateIntegral,
    -INTEGRAL_LIMIT,
    INTEGRAL_LIMIT
  );

  float output =
    feedforward + KP * error + candidateIntegral;

  float limitedOutput = constrain(
    output,
    (float)PWM_MIN,
    (float)PWM_MAX
  );

  // Anti-windup: only integrate when not saturated, or when the error
  // would bring the output back out of saturation.
  if (output == limitedOutput ||
      (output > PWM_MAX && error < 0) ||
      (output < PWM_MIN && error > 0)) {
    integral = candidateIntegral;
  }

  return (int)limitedOutput;
}

// -------------------- STATE MACHINE -------------------------
// Plain constants, not an enum: the Arduino IDE generates function
// prototypes above the type definitions, so an enum in a function
// signature fails to compile.

const uint8_t ST_IDLE     = 0;
const uint8_t ST_DRIVING  = 1;
const uint8_t ST_STOPPED  = 2;
const uint8_t ST_FINISHED = 3;

const char STATE_NAMES[4][9] = { "IDLE", "DRIVING", "STOPPED", "FINISHED" };

uint8_t  state         = ST_IDLE;
uint32_t stateEnterMs  = 0;
uint8_t  stopsDone     = 0;

// Inside ST_STOPPED: false = still rolling to a halt, true = 1 s dwell running.
bool     dwellStarted  = false;
uint32_t dwellStartMs  = 0;

bool telemetryOn = true;

uint32_t lastControlMs   = 0;
uint32_t lastTelemetryMs = 0;

long previousTicksL = 0;
long previousTicksR = 0;

float velocityL = 0.0f;
float velocityR = 0.0f;

float distanceMm = 0.0f;

void setLeds(uint8_t s) {
#if USE_LEDS
  digitalWrite(PIN_LED_DRIVE, s == ST_DRIVING ? HIGH : LOW);
  digitalWrite(PIN_LED_STOP,  (s == ST_STOPPED || s == ST_FINISHED) ? HIGH : LOW);
  digitalWrite(PIN_LED_AVOID, LOW);   // Stage 3/4 will use this
#else
  (void)s;
#endif
}

void enterState(uint8_t s) {
  state = s;
  stateEnterMs = millis();
  setLeds(s);

  if (s == ST_DRIVING) {
    // Clear windup so each leg starts clean, and ramp up from rest.
    integralL = 0;
    integralR = 0;
    rampedTarget = 0;
  } else {
    stopMotors();
    pwmL = 0;
    pwmR = 0;
    rampedTarget = 0;
  }

  if (s == ST_STOPPED) {
    dwellStarted = false;
  }

  Serial.print(F("# STATE -> "));
  Serial.print(STATE_NAMES[s]);
  Serial.print(F("  stops_done="));
  Serial.print(stopsDone);
  Serial.print(F("  dist_mm="));
  Serial.println(distanceMm, 0);
}

void startRun() {
  noInterrupts();
  ticksL = 0;
  ticksR = 0;
  interrupts();

  previousTicksL = 0;
  previousTicksR = 0;
  distanceMm = 0;
  stopsDone = 0;

  lastControlMs = millis();
  enterState(ST_DRIVING);
}

void abortRun() {
  stopsDone = 0;
  enterState(ST_IDLE);
}

// -------------------- SERIAL COMMANDS -----------------------

void handleSerial() {
  while (Serial.available()) {
    char c = Serial.read();

    switch (c) {

      case 'w':
      case 'W':
        Serial.println(F("# Run started"));
        startRun();
        break;

      case 'x':
      case 'X':
      case ' ':
        Serial.println(F("# Aborted"));
        abortRun();
        break;

      case '+':
      case '=':
        targetSpeed = min(250.0f, targetSpeed + 10.0f);
        Serial.print(F("# Target speed: "));
        Serial.println(targetSpeed);
        break;

      case '-':
      case '_':
        targetSpeed = max(40.0f, targetSpeed - 10.0f);
        Serial.print(F("# Target speed: "));
        Serial.println(targetSpeed);
        break;

      case 'r':
      case 'R':
        noInterrupts();
        ticksL = 0;
        ticksR = 0;
        interrupts();

        previousTicksL = 0;
        previousTicksR = 0;
        distanceMm = 0;

        Serial.println(F("# Encoder counts reset"));
        break;

      case 'p':
      case 'P':
        telemetryOn = !telemetryOn;
        break;
    }
  }
}

// -------------------- SETUP ---------------------------------

void setup() {
  Serial.begin(115200);

  pinMode(PIN_PWM_L, OUTPUT);
  pinMode(PIN_DIR_L, OUTPUT);

  pinMode(PIN_PWM_R, OUTPUT);
  pinMode(PIN_DIR_R, OUTPUT);

  pinMode(PIN_SLEEP_L, OUTPUT);
  pinMode(PIN_SLEEP_R, OUTPUT);

  analogWrite(PIN_PWM_L, 0);
  analogWrite(PIN_PWM_R, 0);

  digitalWrite(PIN_SLEEP_L, HIGH);
  digitalWrite(PIN_SLEEP_R, HIGH);

#if USE_LEDS
  pinMode(PIN_LED_DRIVE, OUTPUT);
  pinMode(PIN_LED_STOP, OUTPUT);
  pinMode(PIN_LED_AVOID, OUTPUT);
#endif

  pinMode(PIN_ENC_L_A, INPUT);
  pinMode(PIN_ENC_L_B, INPUT);

  pinMode(PIN_ENC_R_A, INPUT);
  pinMode(PIN_ENC_R_B, INPUT);

  attachInterrupt(
    digitalPinToInterrupt(PIN_ENC_L_A),
    isrLeft,
    CHANGE
  );

  attachInterrupt(
    digitalPinToInterrupt(PIN_ENC_R_A),
    isrRight,
    CHANGE
  );

  lastControlMs = millis();
  lastTelemetryMs = millis();

  setLeds(ST_IDLE);

  Serial.println(F("\n# TI-RSLK MAX - Stage 2: three controlled stops"));
  Serial.println(F("# Drive 3 s, stop 1 s, x3, then run to the end of the route."));
  Serial.println(F("# Commands: w=start, x=abort, +/-=speed, r=reset, p=telemetry"));

  if (AUTOSTART_MS > 0) {
    Serial.print(F("# Auto-start in "));
    Serial.print(AUTOSTART_MS);
    Serial.println(F(" ms"));
  } else {
    Serial.println(F("# Send w to start."));
  }
}

// -------------------- MAIN LOOP -----------------------------

void loop() {
  handleSerial();

  uint32_t now = millis();

  if (AUTOSTART_MS > 0 && state == ST_IDLE && now >= AUTOSTART_MS) {
    Serial.println(F("# Auto-start"));
    startRun();
  }

  if (now - lastControlMs >= CONTROL_INTERVAL_MS) {

    float dt = (now - lastControlMs) / 1000.0f;
    lastControlMs = now;

    // ---------- measure ----------

    long currentL = readTicks(ticksL);
    long currentR = readTicks(ticksR);

    long deltaL = currentL - previousTicksL;
    long deltaR = currentR - previousTicksR;

    previousTicksL = currentL;
    previousTicksR = currentR;

    velocityL = (deltaL * MM_PER_TICK) / dt;
    velocityR = (deltaR * MM_PER_TICK) / dt;

    distanceMm = ((currentL + currentR) * 0.5f) * MM_PER_TICK;

    // ---------- state transitions ----------

    if (state == ST_DRIVING) {

      bool legDone = (stopsDone < NUM_STOPS) &&
                     (now - stateEnterMs >= DRIVE_LEG_MS);

      bool routeDone = (ROUTE_LENGTH_MM > 0.0f) &&
                       (distanceMm >= ROUTE_LENGTH_MM);

      if (legDone) {
        enterState(ST_STOPPED);
      } else if (routeDone && stopsDone >= NUM_STOPS) {
        enterState(ST_FINISHED);
      }

    } else if (state == ST_STOPPED) {

      bool halted = (fabs(velocityL) < STANDSTILL_MM_S) &&
                    (fabs(velocityR) < STANDSTILL_MM_S);

      if (!dwellStarted) {
        // Start the 1 s only once the wheels have actually stopped, so the
        // dwell is a true 1 s and not 1 s minus the braking time.
        if (halted || (now - stateEnterMs >= STANDSTILL_MS)) {
          dwellStarted = true;
          dwellStartMs = now;
        }
      } else if (now - dwellStartMs >= STOP_DWELL_MS) {
        stopsDone++;
        Serial.print(F("# Stop "));
        Serial.print(stopsDone);
        Serial.print(F(" complete, dwell "));
        Serial.print(now - dwellStartMs);
        Serial.println(F(" ms"));
        enterState(ST_DRIVING);
      }
    }

    // ---------- control ----------

    if (state == ST_DRIVING) {

      // Ramp the target so the wheels are never asked for a step change.
      float step = ACCEL_MM_S2 * dt;

      if (rampedTarget < targetSpeed) {
        rampedTarget = min(targetSpeed, rampedTarget + step);
      } else if (rampedTarget > targetSpeed) {
        rampedTarget = max(targetSpeed, rampedTarget - step);
      }

      pwmL = calculatePI(rampedTarget, velocityL, integralL, dt);
      pwmR = calculatePI(rampedTarget, velocityR, integralR, dt);

      driveMotor(PIN_PWM_L, PIN_DIR_L, LEFT_MOTOR_INVERTED, pwmL);
      driveMotor(PIN_PWM_R, PIN_DIR_R, RIGHT_MOTOR_INVERTED, pwmR);

    } else {
      stopMotors();
      pwmL = 0;
      pwmR = 0;
    }
  }

  // ---------------- TELEMETRY ----------------

  if (telemetryOn &&
      now - lastTelemetryMs >= TELEMETRY_INTERVAL_MS) {

    lastTelemetryMs = now;

    Serial.print(F("STATE,"));
    Serial.print(STATE_NAMES[state]);

    Serial.print(F(",STOPS,"));
    Serial.print(stopsDone);

    Serial.print(F(",DIST,"));
    Serial.print(distanceMm, 0);

    Serial.print(F(",TARGET,"));
    Serial.print(rampedTarget);

    Serial.print(F(",VL,"));
    Serial.print(velocityL);

    Serial.print(F(",VR,"));
    Serial.print(velocityR);

    Serial.print(F(",PWM_L,"));
    Serial.print(pwmL);

    Serial.print(F(",PWM_R,"));
    Serial.print(pwmR);

    Serial.print(F(",TICKS_L,"));
    Serial.print(readTicks(ticksL));

    Serial.print(F(",TICKS_R,"));
    Serial.println(readTicks(ticksR));
  }
}
