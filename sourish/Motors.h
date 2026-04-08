#ifndef MOTORS_H
#define MOTORS_H

#include "Config.h"
#include "Encoders.h"

float targetLeftSpeedMMPS = 0.0;
float targetRightSpeedMMPS = 0.0;
float measuredLeftSpeedMMPS = 0.0;
float measuredRightSpeedMMPS = 0.0;
int commandedLeftPWM = 0;
int commandedRightPWM = 0;
float leftMotorIntegral = 0.0;
float rightMotorIntegral = 0.0;
float leftMotorLastError = 0.0;
float rightMotorLastError = 0.0;
float leftMotorFilteredD = 0.0;
float rightMotorFilteredD = 0.0;
unsigned long lastMotorPidMicros = 0;

void applyMotorPWM(int leftSpeed, int rightSpeed) {
  leftSpeed = constrain(leftSpeed, -255, 255);
  rightSpeed = constrain(rightSpeed, -255, 255);

  // Left Motor Logic (Unchanged)
  if (leftSpeed >= 0) {
    digitalWrite(M1_DIR, LOW); // Forward
    analogWrite(M1_PWM, leftSpeed);
  } else {
    digitalWrite(M1_DIR, HIGH);  // Reverse
    analogWrite(M1_PWM, -leftSpeed); // Make speed positive for PWM
  }

  // Right Motor Logic (FLIPPED)
  if (rightSpeed >= 0) {
    digitalWrite(M2_DIR, LOW);  // Forward is now LOW
    analogWrite(M2_PWM, rightSpeed);
  } else {
    digitalWrite(M2_DIR, HIGH); // Reverse is now HIGH
    analogWrite(M2_PWM, -rightSpeed); 
  }
}

void initMotors() {
  pinMode(M1_PWM, OUTPUT);
  pinMode(M1_DIR, OUTPUT);
  pinMode(M2_PWM, OUTPUT);
  pinMode(M2_DIR, OUTPUT);

  applyMotorPWM(0, 0);
  lastMotorPidMicros = micros();
}

void resetMotorSpeedPID() {
  targetLeftSpeedMMPS = 0.0;
  targetRightSpeedMMPS = 0.0;
  measuredLeftSpeedMMPS = 0.0;
  measuredRightSpeedMMPS = 0.0;
  commandedLeftPWM = 0;
  commandedRightPWM = 0;
  leftMotorIntegral = 0.0;
  rightMotorIntegral = 0.0;
  leftMotorLastError = 0.0;
  rightMotorLastError = 0.0;
  leftMotorFilteredD = 0.0;
  rightMotorFilteredD = 0.0;
  lastMotorPidMicros = micros();
  applyMotorPWM(0, 0);
}

// void setWheelSpeedTargetsMMPS(float leftTarget, float rightTarget) {
//   targetLeftSpeedMMPS = constrain(leftTarget, -MAX_WHEEL_SPEED_MMPS, MAX_WHEEL_SPEED_MMPS);
//   targetRightSpeedMMPS = constrain(rightTarget, -MAX_WHEEL_SPEED_MMPS, MAX_WHEEL_SPEED_MMPS);
// }

// void moveForward(float speedMMPS) {
//   setWheelSpeedTargetsMMPS(speedMMPS, speedMMPS);
// }

// void moveBackward(float speedMMPS) {
//   setWheelSpeedTargetsMMPS(-speedMMPS, -speedMMPS);
// }

// void stopMotors() {
//   resetMotorSpeedPID();
// }

// void spinInPlace(float speedMMPS) {
//   setWheelSpeedTargetsMMPS(speedMMPS, -speedMMPS);
//}

#endif
