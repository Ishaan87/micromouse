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

void applyMotorPWM(int leftPwm, int rightPwm) {
  if (leftPwm >= 0) {
    digitalWrite(M1_DIR, HIGH);
    analogWrite(M1_PWM, constrain(leftPwm, 0, 255));
  } else {
    digitalWrite(M1_DIR, LOW);
    analogWrite(M1_PWM, constrain(-leftPwm, 0, 255));
  }

  if (rightPwm >= 0) {
    digitalWrite(M2_DIR, HIGH);
    analogWrite(M2_PWM, constrain(rightPwm, 0, 255));
  } else {
    digitalWrite(M2_DIR, LOW);
    analogWrite(M2_PWM, constrain(-rightPwm, 0, 255));
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

void setWheelSpeedTargetsMMPS(float leftTarget, float rightTarget) {
  targetLeftSpeedMMPS = constrain(leftTarget, -MAX_WHEEL_SPEED_MMPS, MAX_WHEEL_SPEED_MMPS);
  targetRightSpeedMMPS = constrain(rightTarget, -MAX_WHEEL_SPEED_MMPS, MAX_WHEEL_SPEED_MMPS);
}

int runSingleMotorPID(float targetSpeed,
                      float measuredSpeed,
                      int currentPwm,
                      float &integral,
                      float &lastError,
                      float &filteredD,
                      float dt) {
  if (abs(targetSpeed) < 1.0f) {
    integral = 0.0;
    lastError = 0.0;
    filteredD = 0.0;
    return 0;
  }

  float error = targetSpeed - measuredSpeed;
  integral += error * dt;
  integral = constrain(integral, -MOTOR_INTEGRAL_LIMIT, MOTOR_INTEGRAL_LIMIT);

  float rawD = (error - lastError) / dt;
  filteredD = (MOTOR_PID_ALPHA * rawD) + ((1.0f - MOTOR_PID_ALPHA) * filteredD);
  lastError = error;

  float output = (Kp_motor_speed * error) +
                 (Ki_motor_speed * integral) +
                 (Kd_motor_speed * filteredD);

  int nextPwm = currentPwm + (int)output;
  int minPwm = max(35, (int)((MIN_WHEEL_SPEED_MMPS / MAX_WHEEL_SPEED_MMPS) * 255.0f));

  if (targetSpeed > 0) {
    nextPwm = constrain(nextPwm, minPwm, 255);
  } else {
    nextPwm = constrain(nextPwm, -255, -minPwm);
  }

  return nextPwm;
}

void updateMotorSpeedPID() {
  unsigned long now = micros();
  float dt = max((now - lastMotorPidMicros) / 1000000.0f, 0.0001f);
  lastMotorPidMicros = now;

  EncoderData encoderData = readEncoders();
  measuredLeftSpeedMMPS = encoderData.leftSpeedMMPS;
  measuredRightSpeedMMPS = encoderData.rightSpeedMMPS;

  commandedLeftPWM = runSingleMotorPID(targetLeftSpeedMMPS,
                                       measuredLeftSpeedMMPS,
                                       commandedLeftPWM,
                                       leftMotorIntegral,
                                       leftMotorLastError,
                                       leftMotorFilteredD,
                                       dt);

  commandedRightPWM = runSingleMotorPID(targetRightSpeedMMPS,
                                        measuredRightSpeedMMPS,
                                        commandedRightPWM,
                                        rightMotorIntegral,
                                        rightMotorLastError,
                                        rightMotorFilteredD,
                                        dt);

  applyMotorPWM(commandedLeftPWM, commandedRightPWM);
}

void moveForward(float speedMMPS) {
  setWheelSpeedTargetsMMPS(speedMMPS, speedMMPS);
}

void moveBackward(float speedMMPS) {
  setWheelSpeedTargetsMMPS(-speedMMPS, -speedMMPS);
}

void stopMotors() {
  resetMotorSpeedPID();
}

void spinInPlace(float speedMMPS) {
  setWheelSpeedTargetsMMPS(speedMMPS, -speedMMPS);
}

#endif
