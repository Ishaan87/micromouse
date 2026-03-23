#ifndef ENCODERS_H
#define ENCODERS_H

#include "Config.h"

// Volatile because these change inside hardware interrupts
volatile long leftTicks = 0;
volatile long rightTicks = 0;

// IRAM_ATTR loads the interrupt into the ESP32's fast RAM for instant execution
void IRAM_ATTR leftEncoderISR() {
  // Simple single-channel counting (assumes always moving forward during maze cell transitions)
  leftTicks++; 
}

void IRAM_ATTR rightEncoderISR() {
  rightTicks++;
}

void initEncoders() {
  pinMode(ENC_L_A, INPUT_PULLUP);
  pinMode(ENC_R_A, INPUT_PULLUP);

  // Attach interrupts to trigger on the RISING edge of the encoder signal
  attachInterrupt(digitalPinToInterrupt(ENC_L_A), leftEncoderISR, RISING);
  attachInterrupt(digitalPinToInterrupt(ENC_R_A), rightEncoderISR, RISING);
}

void resetEncoders() {
  leftTicks = 0;
  rightTicks = 0;
}

#endif