/*
 * config.h — Human Following Robot
 * All pins and tuning values live here so the behaviour can be adjusted
 * without touching the control logic.
 */
#pragma once

// ---------------------------------------------------------------------------
// Pin map (Arduino Uno + Adafruit-style L293D Motor Shield v1)
// The shield uses D3–D8, D11 and D12 internally; the sensors use the analog
// header, which is broken out on the shield.
// ---------------------------------------------------------------------------
constexpr uint8_t PIN_US_TRIG   = A1;   // HC-SR04 trigger
constexpr uint8_t PIN_US_ECHO   = A0;   // HC-SR04 echo
constexpr uint8_t PIN_IR_RIGHT  = A2;   // right IR obstacle sensor (digital out)
constexpr uint8_t PIN_IR_LEFT   = A3;   // left  IR obstacle sensor (digital out)
constexpr uint8_t PIN_SERVO     = 10;   // "SERVO_1" header on the motor shield

// Motor channels on the shield (M1..M4)
constexpr uint8_t MOTOR_FRONT_LEFT  = 1;
constexpr uint8_t MOTOR_FRONT_RIGHT = 2;
constexpr uint8_t MOTOR_REAR_RIGHT  = 3;
constexpr uint8_t MOTOR_REAR_LEFT   = 4;

// Most FC-51 / TCRT5000 IR modules pull their output LOW when they see an object.
constexpr bool IR_ACTIVE_LOW = true;

// ---------------------------------------------------------------------------
// Following behaviour (distances in centimetres)
// ---------------------------------------------------------------------------
constexpr uint16_t DIST_TOO_CLOSE  = 8;    // closer than this: stop, never ram the person
constexpr uint16_t DIST_FOLLOW_MIN = 10;   // start of the follow band
constexpr uint16_t DIST_FOLLOW_MAX = 35;   // end of the follow band
constexpr uint16_t DIST_MAX_RANGE  = 200;  // readings beyond this are treated as "nothing"

// ---------------------------------------------------------------------------
// Motion tuning (0–255 PWM)
// ---------------------------------------------------------------------------
constexpr uint8_t SPEED_FORWARD = 170;
constexpr uint8_t SPEED_TURN    = 190;
constexpr uint8_t SPEED_RAMP    = 12;      // max PWM change per control tick (smooth starts)

// ---------------------------------------------------------------------------
// Timing
// ---------------------------------------------------------------------------
constexpr uint16_t CONTROL_PERIOD_MS = 40;    // 25 Hz control loop
constexpr uint16_t LOST_TIMEOUT_MS   = 2500;  // no target for this long -> search mode
constexpr uint16_t SEARCH_STEP_MS    = 120;   // servo step interval while scanning

// Servo angles for the ultrasonic "head"
constexpr uint8_t SERVO_CENTER = 90;
constexpr uint8_t SERVO_LEFT   = 150;
constexpr uint8_t SERVO_RIGHT  = 30;

// Set to 1 to stream sensor readings and state over Serial (115200 baud).
#define DEBUG_SERIAL 1
