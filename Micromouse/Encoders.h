#ifndef ENCODERS_H
#define ENCODERS_H

#include "Config.h"

// ─── Raw tick counts ───────────────────────────────────────────
volatile long leftTicks  = 0;
volatile long rightTicks = 0;

// ─── For speed calculation ─────────────────────────────────────
volatile long lastLeftTicks  = 0;
volatile long lastRightTicks = 0;
unsigned long lastSpeedTime  = 0;

// ─── N20 encoder specs ─────────────────────────────────────────
// Change this to match your encoder's PPR (pulses per revolution)
// Common N20 with encoder = 7 PPR on motor shaft
// With gear ratio (e.g. 30:1) → 7 * 30 = 210 ticks per wheel revolution
// const int PPR         = 7;    // pulses per motor shaft revolution
// const int GEAR_RATIO  = 30;   // your N20 gear ratio — change if different
const int TICKS_PER_REV = 300; // ticks per wheel revolution

// Wheel diameter in mm — measure yours and update this
const float WHEEL_DIAMETER_MM = 34.0;
const float WHEEL_CIRCUMFERENCE_MM = 3.14159 * WHEEL_DIAMETER_MM;

// ─── Interrupt handlers ────────────────────────────────────────
void IRAM_ATTR countLeft() {
  if (digitalRead(ENC_L_B) == HIGH) leftTicks++;
  else leftTicks--;
}

void IRAM_ATTR countRight() {
  if (digitalRead(ENC_R_B) == HIGH) rightTicks++;
  else rightTicks--;
}

// ─── Init ──────────────────────────────────────────────────────
void initEncoders() {
  pinMode(ENC_L_A, INPUT_PULLUP);
  pinMode(ENC_L_B, INPUT_PULLUP);
  pinMode(ENC_R_A, INPUT_PULLUP);
  pinMode(ENC_R_B, INPUT_PULLUP);

  attachInterrupt(digitalPinToInterrupt(ENC_L_A), countLeft,  RISING);
  attachInterrupt(digitalPinToInterrupt(ENC_R_A), countRight, RISING);

  lastSpeedTime = millis();
}

// ─── Reset ticks (call before each move) ──────────────────────
void resetEncoders() {
  leftTicks  = 0;
  rightTicks = 0;
  lastLeftTicks  = 0;
  lastRightTicks = 0;
  lastSpeedTime  = millis();
}

// ─── Encoder data struct ───────────────────────────────────────
struct EncoderData {
  long  leftTicks;         // raw tick count
  long  rightTicks;        // raw tick count
  float leftSpeedRPM;      // motor speed in RPM
  float rightSpeedRPM;     // motor speed in RPM
  float leftSpeedMMPS;     // wheel speed in mm/s
  float rightSpeedMMPS;    // wheel speed in mm/s
  float leftDistanceMM;    // distance travelled by left wheel
  float rightDistanceMM;   // distance travelled by right wheel
};

// ─── Main read function ────────────────────────────────────────
EncoderData readEncoders() {
  EncoderData data;

  // snapshot ticks (disable interrupts briefly for safe read)
  noInterrupts();
  long currentLeft  = leftTicks;
  long currentRight = rightTicks;
  interrupts();

  // time since last call
  unsigned long now     = millis();
  float deltaTime       = (now - lastSpeedTime) / 1000.0; // seconds

  // tick difference since last call
  long deltaLeft  = currentLeft  - lastLeftTicks;
  long deltaRight = currentRight - lastRightTicks;

  // ── RPM ──────────────────────────────────────────────────────
  // RPM = (ticks / TICKS_PER_REV) / time_in_minutes
  if (deltaTime > 0) {
    data.leftSpeedRPM  = (deltaLeft  / (float)TICKS_PER_REV) / (deltaTime / 60.0);
    data.rightSpeedRPM = (deltaRight / (float)TICKS_PER_REV) / (deltaTime / 60.0);
  } else {
    data.leftSpeedRPM  = 0;
    data.rightSpeedRPM = 0;
  }

  // ── Speed in mm/s ─────────────────────────────────────────────
  // mm/s = (ticks / TICKS_PER_REV) * circumference / time_in_seconds
  if (deltaTime > 0) {
    data.leftSpeedMMPS  = (deltaLeft  / (float)TICKS_PER_REV) * WHEEL_CIRCUMFERENCE_MM / deltaTime;
    data.rightSpeedMMPS = (deltaRight / (float)TICKS_PER_REV) * WHEEL_CIRCUMFERENCE_MM / deltaTime;
  } else {
    data.leftSpeedMMPS  = 0;
    data.rightSpeedMMPS = 0;
  }

  // ── Distance in mm ────────────────────────────────────────────
  data.leftDistanceMM  = (currentLeft  / (float)TICKS_PER_REV) * WHEEL_CIRCUMFERENCE_MM;
  data.rightDistanceMM = (currentRight / (float)TICKS_PER_REV) * WHEEL_CIRCUMFERENCE_MM;

  // ── Raw ticks ─────────────────────────────────────────────────
  data.leftTicks  = currentLeft;
  data.rightTicks = currentRight;

  // ── Update last values for next call ──────────────────────────
  lastLeftTicks  = currentLeft;
  lastRightTicks = currentRight;
  lastSpeedTime  = now;

  return data;
}

#endif