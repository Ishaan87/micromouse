#ifndef PID_H
#define PID_H

#include "Config.h"
#include "Motors.h"

void startWheelSpeedControl() {
  resetMotorSpeedPID();
}

void setWheelSpeedControl(float leftSpeedMMPS, float rightSpeedMMPS) {
  setWheelSpeedTargetsMMPS(leftSpeedMMPS, rightSpeedMMPS);
  updateMotorSpeedPID();
}

void driveForwardSpeed(float speedMMPS) {
  setWheelSpeedControl(speedMMPS, speedMMPS);
}

void driveBackwardSpeed(float speedMMPS) {
  setWheelSpeedControl(-speedMMPS, -speedMMPS);
}

void turnInPlaceSpeed(float speedMMPS) {
  setWheelSpeedControl(speedMMPS, -speedMMPS);
}

void stopWheelSpeedControl() {
  stopMotors();
}

#endif
