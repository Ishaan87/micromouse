#ifndef ENCODERS_H
#define ENCODERS_H

#include "Config.h"

// ==========================================
// RAW TICK COUNTS
// volatile — changed inside interrupts
// ==========================================
volatile long leftTicks  = 0;
volatile long rightTicks = 0;

// ==========================================
// FOR SPEED CALCULATION
// ==========================================
volatile long lastLeftTicks  = 0;
volatile long lastRightTicks = 0;
unsigned long lastSpeedTime  = 0;

// ==========================================
// INTERRUPT HANDLERS
// IRAM_ATTR — runs from RAM, not Flash
// ==========================================
void IRAM_ATTR countLeft() {
  if (digitalRead(ENC_L_B) == HIGH) leftTicks += LEFT_ENCODER_SIGN;
  else                               leftTicks -= LEFT_ENCODER_SIGN;
}

void IRAM_ATTR countRight() {
  if (digitalRead(ENC_R_B) == HIGH) rightTicks += RIGHT_ENCODER_SIGN;
  else                               rightTicks -= RIGHT_ENCODER_SIGN;
}

// ==========================================
// INIT
// ==========================================
void initEncoders() {
  pinMode(ENC_L_A, INPUT_PULLUP);
  pinMode(ENC_L_B, INPUT_PULLUP);
  pinMode(ENC_R_A, INPUT_PULLUP);
  pinMode(ENC_R_B, INPUT_PULLUP);

  attachInterrupt(digitalPinToInterrupt(ENC_L_A), countLeft,  RISING);
  attachInterrupt(digitalPinToInterrupt(ENC_R_A), countRight, RISING);

  lastSpeedTime = millis();
}

// ==========================================
// RESET — call before each movement
// ==========================================
void resetEncoders() {
  noInterrupts();
  leftTicks      = 0;
  rightTicks     = 0;
  lastLeftTicks  = 0;
  lastRightTicks = 0;
  interrupts();
  lastSpeedTime  = millis();
}

// ==========================================
// SAFE TICK READ
// — disables interrupts briefly to prevent
//   reading mid-update
// ==========================================
void readTicks(long &left, long &right) {
  noInterrupts();
  left  = leftTicks;
  right = rightTicks;
  interrupts();
}

// ==========================================
// ENCODER DATA STRUCT
// ==========================================
struct EncoderData {
  long  leftTicks;
  long  rightTicks;
  long  avgTicks;         // average of both wheels
  float leftSpeedRPM;
  float rightSpeedRPM;
  float leftSpeedMMPS;    // mm per second
  float rightSpeedMMPS;
  float leftDistanceMM;
  float rightDistanceMM;
};

// ==========================================
// FULL ENCODER READ
// ==========================================
EncoderData readEncoders() {
  EncoderData data;

  noInterrupts();
  long currentLeft  = leftTicks;
  long currentRight = rightTicks;
  interrupts();

  unsigned long now      = millis();
  float deltaTime        = (now - lastSpeedTime) / 1000.0;

  long deltaLeft         = currentLeft  - lastLeftTicks;
  long deltaRight        = currentRight - lastRightTicks;

  // RPM
  if (deltaTime > 0) {
    data.leftSpeedRPM  = (deltaLeft  / (float)TICKS_PER_REV) / (deltaTime / 60.0);
    data.rightSpeedRPM = (deltaRight / (float)TICKS_PER_REV) / (deltaTime / 60.0);
  } else {
    data.leftSpeedRPM  = 0;
    data.rightSpeedRPM = 0;
  }

  // mm/s
  if (deltaTime > 0) {
    data.leftSpeedMMPS  = (deltaLeft  / (float)TICKS_PER_REV) * WHEEL_CIRCUMFERENCE_MM / deltaTime;
    data.rightSpeedMMPS = (deltaRight / (float)TICKS_PER_REV) * WHEEL_CIRCUMFERENCE_MM / deltaTime;
  } else {
    data.leftSpeedMMPS  = 0;
    data.rightSpeedMMPS = 0;
  }

  // distance
  data.leftDistanceMM  = (currentLeft  / (float)TICKS_PER_REV) * WHEEL_CIRCUMFERENCE_MM;
  data.rightDistanceMM = (currentRight / (float)TICKS_PER_REV) * WHEEL_CIRCUMFERENCE_MM;

  // raw ticks
  data.leftTicks  = currentLeft;
  data.rightTicks = currentRight;
  data.avgTicks   = (currentLeft + currentRight) / 2;

  // update last values
  lastLeftTicks  = currentLeft;
  lastRightTicks = currentRight;
  lastSpeedTime  = now;

  return data;
}

#endif
