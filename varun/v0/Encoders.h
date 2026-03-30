#ifndef ENCODERS_H
#define ENCODERS_H

#include "Config.h"

// Volatile because these change inside hardware interrupts
volatile long leftTicks = 0;
volatile long rightTicks = 0;

// Add these missing variables:
long lastLeftTicks = 0;
long lastRightTicks = 0;
unsigned long lastSpeedTime = 0;

// IRAM_ATTR loads the interrupt into the ESP32's fast RAM for instant execution
void IRAM_ATTR leftEncoderISR() {
  if (digitalRead(ENC_L_B) == HIGH) leftTicks++;
  else                               leftTicks--; 
}

void IRAM_ATTR rightEncoderISR() {
  if (digitalRead(ENC_R_B) == HIGH) rightTicks++;
  else                               rightTicks--;
}

void initEncoders() {
  pinMode(ENC_L_A, INPUT_PULLUP);
  pinMode(ENC_L_B, INPUT_PULLUP);
  pinMode(ENC_R_A, INPUT_PULLUP);
  pinMode(ENC_R_B, INPUT_PULLUP);

  // Attach interrupts to trigger on the RISING edge of the encoder signal
  attachInterrupt(digitalPinToInterrupt(ENC_L_A), leftEncoderISR, RISING);
  attachInterrupt(digitalPinToInterrupt(ENC_R_A), rightEncoderISR, RISING);
}

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

  // // RPM
  // if (deltaTime > 0) {
  //   data.leftSpeedRPM  = (deltaLeft  / (float)TICKS_PER_REV) / (deltaTime / 60.0);
  //   data.rightSpeedRPM = (deltaRight / (float)TICKS_PER_REV) / (deltaTime / 60.0);
  // } else {
  //   data.leftSpeedRPM  = 0;
  //   data.rightSpeedRPM = 0;
  // }

  // // mm/s
  // if (deltaTime > 0) {
  //   data.leftSpeedMMPS  = (deltaLeft  / (float)TICKS_PER_REV) * WHEEL_CIRCUMFERENCE_MM / deltaTime;
  //   data.rightSpeedMMPS = (deltaRight / (float)TICKS_PER_REV) * WHEEL_CIRCUMFERENCE_MM / deltaTime;
  // } else {
  //   data.leftSpeedMMPS  = 0;
  //   data.rightSpeedMMPS = 0;
  // }

  // // distance
  // data.leftDistanceMM  = (currentLeft  / (float)TICKS_PER_REV) * WHEEL_CIRCUMFERENCE_MM;
  // data.rightDistanceMM = (currentRight / (float)TICKS_PER_REV) * WHEEL_CIRCUMFERENCE_MM;

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