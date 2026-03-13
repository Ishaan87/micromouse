#include "Config.h"

void initMotors() {
  pinMode(M1_PWM, OUTPUT);
  pinMode(M1_DIR, OUTPUT);
  pinMode(M2_PWM, OUTPUT);
  pinMode(M2_DIR, OUTPUT);
  
  // Start with motors off
  analogWrite(M1_PWM, 0);
  analogWrite(M2_PWM, 0);
}

// speed: 0 to 255
void moveForward(int speed) {
  digitalWrite(M1_DIR, HIGH);
  digitalWrite(M2_DIR, HIGH);
  analogWrite(M1_PWM, speed);
  analogWrite(M2_PWM, speed);
}

void moveBackward(int speed) {
  digitalWrite(M1_DIR, LOW);
  digitalWrite(M2_DIR, LOW);
  analogWrite(M1_PWM, speed);
  analogWrite(M2_PWM, speed);
}

void stopMotors() {
  analogWrite(M1_PWM, 0);
  analogWrite(M2_PWM, 0);
}