#include "Config.h"

// Volatile variables are used because they change inside interrupts
volatile long leftTicks = 0;
volatile long rightTicks = 0;

void IRAM_ATTR countLeft() {
  if (digitalRead(ENC_L_B) == HIGH) leftTicks++;
  else leftTicks--;
}

void IRAM_ATTR countRight() {
  if (digitalRead(ENC_R_B) == HIGH) rightTicks++;
  else rightTicks--;
}

void initEncoders() {
  pinMode(ENC_L_A, INPUT_PULLUP);
  pinMode(ENC_L_B, INPUT_PULLUP);
  pinMode(ENC_R_A, INPUT_PULLUP);
  pinMode(ENC_R_B, INPUT_PULLUP);

  attachInterrupt(digitalPinToInterrupt(ENC_L_A), countLeft, RISING);
  attachInterrupt(digitalPinToInterrupt(ENC_R_A), countRight, RISING);
}