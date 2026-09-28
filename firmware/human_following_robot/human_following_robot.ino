/*
 * Human Following Robot
 * ---------------------
 * A four-wheel robot that detects a person in front of it and follows them.
 *
 *   - HC-SR04 ultrasonic sensor (on a servo "head") measures distance.
 *   - Two IR sensors on the left and right tell which way the person moved.
 *   - An Arduino Uno decides what to do and drives four BO motors through an
 *     L293D motor shield.
 *
 * Behaviour (see README for the full state diagram):
 *   FOLLOW  - target inside the follow band            -> drive forward
 *   TURN_L  - only the left IR sensor sees the target  -> pivot left
 *   TURN_R  - only the right IR sensor sees the target -> pivot right
 *   HOLD    - target too close, or lost only briefly   -> stop and wait
 *   SEARCH  - target lost for LOST_TIMEOUT_MS          -> sweep the servo,
 *             then turn the body toward whatever it finds
 *
 * Libraries: Adafruit Motor Shield library (AFMotor, v1), Servo (built in).
 *
 * Indian Patent Application No. 202341080347 A, "Human Following Robot Using
 * Arduino Uno", filed 27 Nov 2023 by R.M.K. Engineering College.
 */

#include <AFMotor.h>
#include <Servo.h>
#include "config.h"

// ---------------------------------------------------------------------------
// Hardware objects
// ---------------------------------------------------------------------------
AF_DCMotor motorFL(MOTOR_FRONT_LEFT,  MOTOR12_1KHZ);
AF_DCMotor motorFR(MOTOR_FRONT_RIGHT, MOTOR12_1KHZ);
AF_DCMotor motorRR(MOTOR_REAR_RIGHT,  MOTOR34_1KHZ);
AF_DCMotor motorRL(MOTOR_REAR_LEFT,   MOTOR34_1KHZ);
Servo head;

// ---------------------------------------------------------------------------
// State
// ---------------------------------------------------------------------------
enum class State : uint8_t { HOLD, FOLLOW, TURN_LEFT, TURN_RIGHT, SEARCH };

struct Sensors {
  uint16_t distanceCm;   // 0 = no echo / out of range
  bool irLeft;
  bool irRight;
};

State    state          = State::HOLD;
uint32_t lastSeenMs     = 0;
uint32_t lastTickMs     = 0;
int16_t  leftSpeed      = 0;   // signed: + forward, - backward
int16_t  rightSpeed     = 0;

// search sweep
uint8_t  searchAngle    = SERVO_CENTER;
int8_t   searchDir      = 1;
uint32_t lastSearchStep = 0;
uint16_t bestSearchDist = 0;
uint8_t  bestSearchAng  = SERVO_CENTER;

// ---------------------------------------------------------------------------
// Sensors
// ---------------------------------------------------------------------------
uint16_t pingOnceCm() {
  digitalWrite(PIN_US_TRIG, LOW);
  delayMicroseconds(2);
  digitalWrite(PIN_US_TRIG, HIGH);
  delayMicroseconds(10);
  digitalWrite(PIN_US_TRIG, LOW);

  // Time out after the round trip for DIST_MAX_RANGE (~58 us per cm).
  const unsigned long echoUs = pulseIn(PIN_US_ECHO, HIGH, (unsigned long)DIST_MAX_RANGE * 58UL);
  if (echoUs == 0) return 0;
  return (uint16_t)(echoUs / 58UL);
}

// Median of three pings rejects the occasional spurious echo.
uint16_t readDistanceCm() {
  uint16_t a = pingOnceCm(); delayMicroseconds(300);
  uint16_t b = pingOnceCm(); delayMicroseconds(300);
  uint16_t c = pingOnceCm();
  if (a > b) { uint16_t t = a; a = b; b = t; }
  if (b > c) { uint16_t t = b; b = c; c = t; }
  if (a > b) { uint16_t t = a; a = b; b = t; }
  return b;
}

bool readIr(uint8_t pin) {
  const bool high = digitalRead(pin) == HIGH;
  return IR_ACTIVE_LOW ? !high : high;
}

Sensors readSensors() {
  Sensors s;
  s.distanceCm = readDistanceCm();
  s.irLeft     = readIr(PIN_IR_LEFT);
  s.irRight    = readIr(PIN_IR_RIGHT);
  return s;
}

// ---------------------------------------------------------------------------
// Motors
// ---------------------------------------------------------------------------
void applyMotor(AF_DCMotor &m, int16_t speed) {
  if (speed == 0) { m.run(RELEASE); return; }
  const int16_t mag = speed < 0 ? -speed : speed;
  m.setSpeed((uint8_t)(mag > 255 ? 255 : mag));
  m.run(speed > 0 ? FORWARD : BACKWARD);
}

int16_t rampToward(int16_t current, int16_t target) {
  if (current < target) {
    const int16_t next = current + SPEED_RAMP;
    return next > target ? target : next;
  }
  if (current > target) {
    const int16_t next = current - SPEED_RAMP;
    return next < target ? target : next;
  }
  return current;
}

// Differential drive: set the target speed of each side, ramped for smooth motion.
void drive(int16_t leftTarget, int16_t rightTarget) {
  leftSpeed  = rampToward(leftSpeed,  leftTarget);
  rightSpeed = rampToward(rightSpeed, rightTarget);
  applyMotor(motorFL, leftSpeed);
  applyMotor(motorRL, leftSpeed);
  applyMotor(motorFR, rightSpeed);
  applyMotor(motorRR, rightSpeed);
}

void stopNow() {
  leftSpeed = rightSpeed = 0;
  drive(0, 0);
}

// ---------------------------------------------------------------------------
// Decision logic
// ---------------------------------------------------------------------------
bool inFollowBand(uint16_t d) {
  return d >= DIST_FOLLOW_MIN && d <= DIST_FOLLOW_MAX;
}

State decide(const Sensors &s, uint32_t now) {
  const bool tooClose  = s.distanceCm > 0 && s.distanceCm < DIST_TOO_CLOSE;
  const bool targetFwd = inFollowBand(s.distanceCm);
  const bool onlyLeft  = s.irLeft && !s.irRight;
  const bool onlyRight = s.irRight && !s.irLeft;

  if (targetFwd || s.irLeft || s.irRight || tooClose) lastSeenMs = now;

  if (tooClose)   return State::HOLD;        // safety first
  if (onlyLeft)   return State::TURN_LEFT;
  if (onlyRight)  return State::TURN_RIGHT;
  if (targetFwd)  return State::FOLLOW;
  if (now - lastSeenMs > LOST_TIMEOUT_MS) return State::SEARCH;
  return State::HOLD;
}

// Sweep the ultrasonic head; remember the closest in-band reading, then
// pivot the body toward it.
void runSearch(uint32_t now) {
  drive(0, 0);
  if (now - lastSearchStep < SEARCH_STEP_MS) return;
  lastSearchStep = now;

  const uint16_t d = readDistanceCm();
  if (inFollowBand(d) && (bestSearchDist == 0 || d < bestSearchDist)) {
    bestSearchDist = d;
    bestSearchAng  = searchAngle;
  }

  searchAngle += searchDir * 10;
  if (searchAngle >= SERVO_LEFT || searchAngle <= SERVO_RIGHT) {
    searchDir = -searchDir;
    if (bestSearchDist > 0) {
      // Found someone: turn the body toward them, then resume following.
      const bool toLeft = bestSearchAng > SERVO_CENTER;
      const uint16_t turnMs = (uint16_t)abs((int)bestSearchAng - SERVO_CENTER) * 6;
      head.write(SERVO_CENTER);
      applyMotor(motorFL, toLeft ? -SPEED_TURN :  SPEED_TURN);
      applyMotor(motorRL, toLeft ? -SPEED_TURN :  SPEED_TURN);
      applyMotor(motorFR, toLeft ?  SPEED_TURN : -SPEED_TURN);
      applyMotor(motorRR, toLeft ?  SPEED_TURN : -SPEED_TURN);
      delay(turnMs);
      stopNow();
      bestSearchDist = 0;
      searchAngle    = SERVO_CENTER;
      lastSeenMs     = now;
      state          = State::HOLD;
      return;
    }
  }
  head.write(searchAngle);
}

const char *stateName(State s) {
  switch (s) {
    case State::FOLLOW:     return "FOLLOW";
    case State::TURN_LEFT:  return "TURN_L";
    case State::TURN_RIGHT: return "TURN_R";
    case State::SEARCH:     return "SEARCH";
    default:                return "HOLD";
  }
}

// ---------------------------------------------------------------------------
// Arduino entry points
// ---------------------------------------------------------------------------
void setup() {
#if DEBUG_SERIAL
  Serial.begin(115200);
  Serial.println(F("Human Following Robot - ready"));
#endif
  pinMode(PIN_US_TRIG, OUTPUT);
  pinMode(PIN_US_ECHO, INPUT);
  pinMode(PIN_IR_LEFT, INPUT);
  pinMode(PIN_IR_RIGHT, INPUT);

  head.attach(PIN_SERVO);
  // Short "wake up" look left and right, then face forward.
  head.write(SERVO_LEFT);   delay(350);
  head.write(SERVO_RIGHT);  delay(350);
  head.write(SERVO_CENTER); delay(300);

  stopNow();
  lastSeenMs = millis();
}

void loop() {
  const uint32_t now = millis();
  if (now - lastTickMs < CONTROL_PERIOD_MS) return;
  lastTickMs = now;

  const Sensors s = readSensors();
  const State next = decide(s, now);

  if (next != State::SEARCH && state == State::SEARCH) {
    head.write(SERVO_CENTER);
    bestSearchDist = 0;
  }
  state = next;

  switch (state) {
    case State::FOLLOW:     drive( SPEED_FORWARD,  SPEED_FORWARD); break;
    case State::TURN_LEFT:  drive(-SPEED_TURN,     SPEED_TURN);    break;
    case State::TURN_RIGHT: drive( SPEED_TURN,    -SPEED_TURN);    break;
    case State::SEARCH:     runSearch(now);                        break;
    case State::HOLD:
    default:                drive(0, 0);                           break;
  }

#if DEBUG_SERIAL
  static uint8_t printDiv = 0;
  if (++printDiv >= 5) {          // ~5 lines per second
    printDiv = 0;
    Serial.print(F("dist="));  Serial.print(s.distanceCm);
    Serial.print(F("cm  L=")); Serial.print(s.irLeft);
    Serial.print(F("  R="));   Serial.print(s.irRight);
    Serial.print(F("  state=")); Serial.println(stateName(state));
  }
#endif
}
