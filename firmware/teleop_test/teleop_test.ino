// teleop_test.ino - FIRST bring-up sketch for the TI-RSLK MAX chassis driven by an Arduino Uno.
//
// Purpose: prove, one step at a time, that the wiring works BEFORE the closed-loop firmware is used:
//   1. each motor spins, in the right direction            (keys j / k / w / s / a / d)
//   2. each encoder counts, with the right sign             (telemetry line "E,...")
//   3. the ultrasonic sensor reads                          (set USE_ULTRASONIC 1)
//
// Control is OPEN LOOP (a raw PWM value, not a speed). Keys come from the PC over USB serial,
// either typed in the Arduino Serial Monitor (115200 baud) or via tools/teleop.py.
//
// SAFETY
//  * Dead-man: if no movement command arrives for DEADMAN_MS the motors stop. So a crashed script,
//    a pulled cable or a lifted finger all stop the robot.
//  * Power is limited (PWM_MAX) and ramped (SLEW) - the motors are rated 4.5 V but run from ~7-9 V,
//    and a stalled wheel can exceed the DRV8838 current limit. Never hold a wheel still while driving.
//
// Keys:  w forward  s backward  a pivot left  d pivot right  x / space stop
//        j left wheel only (forward)   k right wheel only (forward)    <- diagnostics
//        + / -  speed up / down        r reset encoder counts          p toggle telemetry   ? help

#include <Arduino.h>

#define USE_ULTRASONIC 0   // set to 1 once the HC-SR04 is wired (TRIG D9, ECHO D10)

// ------------------------------------------------------------------ pins (module map)
const uint8_t PIN_ENC_L_A = 2;    // ELA  (external interrupt INT0)
const uint8_t PIN_ENC_L_B = 4;    // ELB
const uint8_t PIN_ENC_R_A = 3;    // ERA  (external interrupt INT1)
const uint8_t PIN_ENC_R_B = 12;   // ERB
const uint8_t PIN_PWM_L   = 5;    // PWML -> DRV8838 EN (speed)
const uint8_t PIN_DIR_L   = 7;    // DIRL -> DRV8838 PH (direction)
const uint8_t PIN_PWM_R   = 6;    // PWMR
const uint8_t PIN_DIR_R   = 8;    // DIRR
const uint8_t PIN_SLEEP_N = 11;   // nSLPL + nSLPR (both), HIGH = drivers awake
const uint8_t PIN_TRIG    = 9;
const uint8_t PIN_ECHO    = 10;

// ------------------------------------------------------- flip these if something runs backwards
const bool LEFT_MOTOR_INVERTED  = false;   // wheel turns backwards when told "forward"
const bool RIGHT_MOTOR_INVERTED = false;
const bool LEFT_ENC_INVERTED    = false;   // counts go DOWN when the wheel rolls forward
const bool RIGHT_ENC_INVERTED   = false;

// --------------------------------------------------------------------------- tunables
const float    MM_PER_TICK    = 3.14159265f * 70.0f / 720.0f;  // 70 mm wheel, 720 counts/rev (derived)
const uint8_t  PWM_START      = 110;    // initial drive PWM (0-255). Increase with '+' if wheels do not move
const uint8_t  PWM_STEP       = 10;
const uint8_t  PWM_LIMIT      = 150;    // hard ceiling for this test sketch
const int      SLEW_PER_LOOP  = 6;      // PWM change per 10 ms loop (full 150 in ~0.25 s)
const uint32_t DEADMAN_MS     = 1500;   // stop if no command for this long
const uint32_t LOOP_MS        = 10;
const uint32_t TELEMETRY_MS   = 100;

// ------------------------------------------------------------------------- encoders
volatile long ticksL = 0, ticksR = 0;

void isrLeft() {
  int8_t d = (digitalRead(PIN_ENC_L_A) != digitalRead(PIN_ENC_L_B)) ? 1 : -1;
  ticksL += LEFT_ENC_INVERTED ? -d : d;
}
void isrRight() {
  int8_t d = (digitalRead(PIN_ENC_R_A) != digitalRead(PIN_ENC_R_B)) ? 1 : -1;
  ticksR += RIGHT_ENC_INVERTED ? -d : d;
}

long readTicks(volatile long& t) {
  noInterrupts();
  long v = t;
  interrupts();
  return v;
}

// ----------------------------------------------------------------------------- motors
enum Mode : uint8_t { M_STOP, M_FWD, M_BACK, M_LEFT, M_RIGHT, M_LEFT_ONLY, M_RIGHT_ONLY };
const char MODE_NAMES[] = {'X', 'W', 'S', 'A', 'D', 'J', 'K'};

Mode mode = M_STOP;
uint8_t drivePwm = PWM_START;
int curL = 0, curR = 0;         // current (slewed) signed PWM
uint32_t lastCmdMs = 0;
bool telemetryOn = true;

void driveMotor(uint8_t pwmPin, uint8_t dirPin, bool inverted, int value) {
  value = constrain(value, -255, 255);
  if (value == 0) {
    analogWrite(pwmPin, 0);                       // EN low = DRV8838 brake
    return;
  }
  const bool forward = (value > 0) != inverted;   // forward = DIR low (TI reference convention)
  digitalWrite(dirPin, forward ? LOW : HIGH);
  analogWrite(pwmPin, abs(value));
}

// NOTE: parameter is uint8_t, not Mode. The Arduino IDE auto-generates function prototypes ABOVE the
// enum definition, so a function that names the enum type in its signature fails to compile.
void targetsFor(uint8_t m, int& tL, int& tR) {
  const int p = drivePwm;
  switch (m) {
    case M_FWD:        tL = p;  tR = p;  break;
    case M_BACK:       tL = -p; tR = -p; break;
    case M_LEFT:       tL = -p; tR = p;  break;   // pivot left (counter-clockwise from above)
    case M_RIGHT:      tL = p;  tR = -p; break;
    case M_LEFT_ONLY:  tL = p;  tR = 0;  break;
    case M_RIGHT_ONLY: tL = 0;  tR = p;  break;
    default:           tL = 0;  tR = 0;  break;
  }
}

int slewToward(int cur, int target) {
  if (target > cur) return min(cur + SLEW_PER_LOOP, target);
  if (target < cur) return max(cur - SLEW_PER_LOOP, target);
  return cur;
}

// --------------------------------------------------------------------------- ultrasonic
#if USE_ULTRASONIC
float readDistanceCm() {
  digitalWrite(PIN_TRIG, LOW);
  delayMicroseconds(2);
  digitalWrite(PIN_TRIG, HIGH);
  delayMicroseconds(10);
  digitalWrite(PIN_TRIG, LOW);
  const unsigned long us = pulseIn(PIN_ECHO, HIGH, 6000UL);   // ~1 m max
  if (us == 0) return 999.0f;
  return us * 0.346f * 0.5f / 10.0f;                         // c = 0.346 mm/us, round trip, mm -> cm
}
#endif

// ------------------------------------------------------------------------------- UI
void printHelp() {
  Serial.println(F("# keys: w s a d = drive, x/space = stop, j/k = left/right wheel only,"));
  Serial.println(F("#       +/- = speed, r = reset counts, p = telemetry on/off, ? = help"));
  Serial.println(F("# E,t_ms,ticksL,ticksR,mmL,mmR,vL_mm_s,vR_mm_s,pwmL,pwmR,mode,drivePwm[,dist_cm]"));
}

void handleKey(char c) {
  switch (c) {
    case 'w': case 'W': mode = M_FWD;        lastCmdMs = millis(); break;
    case 's': case 'S': mode = M_BACK;       lastCmdMs = millis(); break;
    case 'a': case 'A': mode = M_LEFT;       lastCmdMs = millis(); break;
    case 'd': case 'D': mode = M_RIGHT;      lastCmdMs = millis(); break;
    case 'j': case 'J': mode = M_LEFT_ONLY;  lastCmdMs = millis(); break;
    case 'k': case 'K': mode = M_RIGHT_ONLY; lastCmdMs = millis(); break;
    case 'x': case 'X': case ' ': mode = M_STOP; break;
    case '+': case '=':
      drivePwm = (uint8_t)min((int)PWM_LIMIT, (int)drivePwm + PWM_STEP);
      break;
    case '-': case '_':
      drivePwm = (uint8_t)max(0, (int)drivePwm - PWM_STEP);
      break;
    case 'r': case 'R':
      noInterrupts(); ticksL = 0; ticksR = 0; interrupts();
      break;
    case 'p': case 'P': telemetryOn = !telemetryOn; break;
    case '?': printHelp(); break;
    default: break;   // ignore newlines and anything else
  }
}

// ---------------------------------------------------------------------------- Arduino
void setup() {
  Serial.begin(115200);

  pinMode(PIN_PWM_L, OUTPUT);  pinMode(PIN_DIR_L, OUTPUT);
  pinMode(PIN_PWM_R, OUTPUT);  pinMode(PIN_DIR_R, OUTPUT);
  analogWrite(PIN_PWM_L, 0);   analogWrite(PIN_PWM_R, 0);

  // Wake the DRV8838 drivers (nSLEEP is active-low). Both lines tied to this one pin.
  pinMode(PIN_SLEEP_N, OUTPUT);
  digitalWrite(PIN_SLEEP_N, HIGH);

  // Plain INPUT (no Uno pull-up): the chassis board already pulls the open-drain Hall outputs up to 3.3 V.
  pinMode(PIN_ENC_L_A, INPUT);  pinMode(PIN_ENC_L_B, INPUT);
  pinMode(PIN_ENC_R_A, INPUT);  pinMode(PIN_ENC_R_B, INPUT);
  attachInterrupt(digitalPinToInterrupt(PIN_ENC_L_A), isrLeft, CHANGE);
  attachInterrupt(digitalPinToInterrupt(PIN_ENC_R_A), isrRight, CHANGE);

#if USE_ULTRASONIC
  pinMode(PIN_TRIG, OUTPUT);
  pinMode(PIN_ECHO, INPUT);
#endif

  Serial.println(F("# teleop_test ready"));
  printHelp();
}

void loop() {
  static uint32_t lastLoopMs = 0, lastTelemetryMs = 0;
  static long prevL = 0, prevR = 0;
  static uint32_t prevSpeedMs = 0;
  static float vL = 0, vR = 0;

  while (Serial.available() > 0) handleKey((char)Serial.read());

  const uint32_t now = millis();
  if (now - lastLoopMs < LOOP_MS) return;
  lastLoopMs = now;

  // Dead-man: no fresh movement command -> stop.
  if (mode != M_STOP && now - lastCmdMs > DEADMAN_MS) mode = M_STOP;

  int tL, tR;
  targetsFor(mode, tL, tR);
  curL = slewToward(curL, tL);
  curR = slewToward(curR, tR);
  driveMotor(PIN_PWM_L, PIN_DIR_L, LEFT_MOTOR_INVERTED, curL);
  driveMotor(PIN_PWM_R, PIN_DIR_R, RIGHT_MOTOR_INVERTED, curR);

  if (telemetryOn && now - lastTelemetryMs >= TELEMETRY_MS) {
    const long l = readTicks(ticksL), r = readTicks(ticksR);
    const float dt = (now - prevSpeedMs) * 1.0e-3f;
    if (dt > 0) {
      vL = (l - prevL) * MM_PER_TICK / dt;
      vR = (r - prevR) * MM_PER_TICK / dt;
    }
    prevL = l; prevR = r; prevSpeedMs = now;
    lastTelemetryMs = now;

    Serial.print(F("E,"));  Serial.print(now);
    Serial.print(',');      Serial.print(l);
    Serial.print(',');      Serial.print(r);
    Serial.print(',');      Serial.print(l * MM_PER_TICK, 0);
    Serial.print(',');      Serial.print(r * MM_PER_TICK, 0);
    Serial.print(',');      Serial.print(vL, 0);
    Serial.print(',');      Serial.print(vR, 0);
    Serial.print(',');      Serial.print(curL);
    Serial.print(',');      Serial.print(curR);
    Serial.print(',');      Serial.print(MODE_NAMES[mode]);
    Serial.print(',');      Serial.print((int)drivePwm);
#if USE_ULTRASONIC
    Serial.print(',');      Serial.print(readDistanceCm(), 1);
#endif
    Serial.println();
  }
}
