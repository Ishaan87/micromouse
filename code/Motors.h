#ifndef MOTORS_H
#define MOTORS_H

#include "Config.h"

void initMotors() {
  pinMode(M1_PWM, OUTPUT);
  pinMode(M1_DIR, OUTPUT);
  pinMode(M2_PWM, OUTPUT);
  pinMode(M2_DIR, OUTPUT);
  
  // Ensure motors start totally stopped
  analogWrite(M1_PWM, 0);
  analogWrite(M2_PWM, 0);
}

// Accepts speeds from -255 (full reverse) to 255 (full forward)
void setMotorSpeeds(int leftSpeed, int rightSpeed) {
  // Constrain speeds to safe 8-bit PWM limits
  leftSpeed = constrain(leftSpeed, -255, 255);
  rightSpeed = constrain(rightSpeed, -255, 255);

  // Left Motor Logic
  if (leftSpeed >= 0) {
    digitalWrite(M1_DIR, HIGH); // Forward
    analogWrite(M1_PWM, leftSpeed);
  } else {
    digitalWrite(M1_DIR, LOW);  // Reverse
    analogWrite(M1_PWM, -leftSpeed); // Make speed positive for PWM
  }

  // Right Motor Logic
  if (rightSpeed >= 0) {
    digitalWrite(M2_DIR, HIGH); // Forward
    analogWrite(M2_PWM, rightSpeed);
  } else {
    digitalWrite(M2_DIR, LOW);  // Reverse
    analogWrite(M2_PWM, -rightSpeed); 
  }
}

void stopMotors() {
  analogWrite(M1_PWM, 0);
  analogWrite(M2_PWM, 0);
}

#endif