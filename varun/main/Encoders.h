#ifndef ENCODERS_H
#define ENCODERS_H

#include "Config.h"

// Volatile because these change inside hardware interrupts
volatile long leftTicks = 0;
volatile long rightTicks = 0;
volatile uint8_t leftEncoderState = 0;
volatile uint8_t rightEncoderState = 0;
volatile unsigned long lastLeftEncoderEdgeUs = 0;
volatile unsigned long lastRightEncoderEdgeUs = 0;

inline int8_t encoderTransitionDelta(uint8_t previousState, uint8_t currentState) {
  static const int8_t transitionTable[16] = {
    0, -1,  1,  0,
    1,  0,  0, -1,
   -1,  0,  0,  1,
    0,  1, -1,  0
  };

  return transitionTable[(previousState << 2) | currentState];
}

inline uint8_t readLeftEncoderState() {
  return (digitalRead(ENC_L_A) << 1) | digitalRead(ENC_L_B);
}

inline uint8_t readRightEncoderState() {
  return (digitalRead(ENC_R_A) << 1) | digitalRead(ENC_R_B);
}

// IRAM_ATTR loads the interrupt into the ESP32's fast RAM for instant execution
void IRAM_ATTR leftEncoderISR() {
  unsigned long now = micros();
  if (now - lastLeftEncoderEdgeUs < ENCODER_GLITCH_FILTER_US) {
    return;
  }

  uint8_t currentState = readLeftEncoderState();
  int8_t delta = encoderTransitionDelta(leftEncoderState, currentState);
  if (LEFT_ENCODER_INVERTED) {
    delta = -delta;
  }

  leftTicks += delta;
  leftEncoderState = currentState;
  lastLeftEncoderEdgeUs = now;
}

void IRAM_ATTR rightEncoderISR() {
  unsigned long now = micros();
  if (now - lastRightEncoderEdgeUs < ENCODER_GLITCH_FILTER_US) {
    return;
  }

  uint8_t currentState = readRightEncoderState();
  int8_t delta = encoderTransitionDelta(rightEncoderState, currentState);
  if (RIGHT_ENCODER_INVERTED) {
    delta = -delta;
  }

  rightTicks += delta;
  rightEncoderState = currentState;
  lastRightEncoderEdgeUs = now;
}

void initEncoders() {
  pinMode(ENC_L_A, INPUT_PULLUP);
  pinMode(ENC_L_B, INPUT_PULLUP);
  pinMode(ENC_R_A, INPUT_PULLUP);
  pinMode(ENC_R_B, INPUT_PULLUP);

  leftEncoderState = readLeftEncoderState();
  rightEncoderState = readRightEncoderState();

  // Decode both channels to avoid missing edges and to reject invalid transitions.
  attachInterrupt(digitalPinToInterrupt(ENC_L_A), leftEncoderISR, CHANGE);
  attachInterrupt(digitalPinToInterrupt(ENC_L_B), leftEncoderISR, CHANGE);
  attachInterrupt(digitalPinToInterrupt(ENC_R_A), rightEncoderISR, CHANGE);
  attachInterrupt(digitalPinToInterrupt(ENC_R_B), rightEncoderISR, CHANGE);
}

void resetEncoders() {
  leftTicks = 0;
  rightTicks = 0;
  leftEncoderState = readLeftEncoderState();
  rightEncoderState = readRightEncoderState();
  lastLeftEncoderEdgeUs = micros();
  lastRightEncoderEdgeUs = micros();
}

#endif
