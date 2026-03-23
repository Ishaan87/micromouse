#ifndef PID_H
#define PID_H

#include "Config.h"
#include "Motors.h"
#include "Encoders.h"
#include "Sensors.h"

enum RobotState {
  MOVING_FORWARD,
  MOVING_BACKWARD,
  TURNING_LEFT,
  TURNING_RIGHT,
  TURNING_180,
  STOPPED
};

typedef void (*MotionServiceCallback)();

RobotState robotState = STOPPED;
float currentHeading = 0.0;
unsigned long lastTimeMicros = 0;

float last_positionError = 0.0;
float filtered_D_position = 0.0;
float last_distanceError = 0.0;
float filtered_D_distance = 0.0;
float last_headingError = 0.0;
float filtered_D_heading = 0.0;
float last_turnError = 0.0;
float filtered_D_turn = 0.0;
float turnTarget = 0.0;

MotionServiceCallback motionServiceCallback = nullptr;
bool motionAbortRequested = false;

const int LOG_CAPACITY = 40;
String controlLogs[LOG_CAPACITY];
int controlLogStart = 0;
int controlLogCount = 0;

void appendControlLog(const String &message) {
  Serial.println(message);

  int index = (controlLogStart + controlLogCount) % LOG_CAPACITY;
  controlLogs[index] = message;

  if (controlLogCount < LOG_CAPACITY) {
    controlLogCount++;
  } else {
    controlLogStart = (controlLogStart + 1) % LOG_CAPACITY;
  }
}

String getControlLogsJson() {
  String body = "[";

  for (int i = 0; i < controlLogCount; i++) {
    if (i > 0) {
      body += ",";
    }

    int index = (controlLogStart + i) % LOG_CAPACITY;
    String entry = controlLogs[index];
    entry.replace("\\", "\\\\");
    entry.replace("\"", "\\\"");
    body += "\"";
    body += entry;
    body += "\"";
  }

  body += "]";
  return body;
}

void setMotionServiceCallback(MotionServiceCallback callback) {
  motionServiceCallback = callback;
}

void requestMotionAbort() {
  motionAbortRequested = true;
}

void clearMotionAbort() {
  motionAbortRequested = false;
}

float calcPID(float error,
              float kp, float kd,
              float alpha,
              float &lastError,
              float &filteredD,
              float dt) {
  float rawD = (error - lastError) / dt;
  filteredD = (alpha * rawD) + ((1.0f - alpha) * filteredD);
  lastError = error;
  return (kp * error) + (kd * filteredD);
}

void resetPID() {
  last_positionError = 0.0;
  filtered_D_position = 0.0;
  last_distanceError = 0.0;
  filtered_D_distance = 0.0;
  last_headingError = 0.0;
  filtered_D_heading = 0.0;
  last_turnError = 0.0;
  filtered_D_turn = 0.0;
}

void updateHeading(float dt) {
  float gyroVelocity = readGyroHeading();
  currentHeading += gyroVelocity * dt * (180.0f / 3.14159f);
}

void serviceMotionLoop() {
  if (motionServiceCallback != nullptr) {
    motionServiceCallback();
  }

  if (motionAbortRequested) {
    robotState = STOPPED;
  }
}

float userSpeedToWheelMMPS(int userSpeed) {
  return map(constrain(userSpeed, 60, 150),
             60, 150,
             (int)MIN_WHEEL_SPEED_MMPS,
             (int)MAX_WHEEL_SPEED_MMPS);
}

float headingHoldPID(float targetHeading, float dt) {
  float headingError = targetHeading - currentHeading;

  return calcPID(headingError,
                 Kp_heading_hold, Kd_heading_hold,
                 alpha_D,
                 last_headingError,
                 filtered_D_heading,
                 dt);
}

float sideWallCorrectionPID(float dt) {
  DistanceData lidars = readLidars();
  float positionError = 0.0;

  bool hasLeft = lidars.left < WALL_THRESHOLD;
  bool hasRight = lidars.right < WALL_THRESHOLD;

  if (hasLeft && hasRight) {
    positionError = lidars.left - lidars.right;
  } else if (hasLeft) {
    positionError = lidars.left - IDEAL_SIDE_DIST;
  } else if (hasRight) {
    positionError = IDEAL_SIDE_DIST - lidars.right;
  }

  return calcPID(positionError,
                 Kp_straight_sensor, Kd_straight_sensor,
                 alpha_D,
                 last_positionError,
                 filtered_D_position,
                 dt);
}

float distanceToBaseSpeedMMPS(long progressTicks, long targetTicks, int maxUserSpeed, int direction, float dt) {
  float distanceError = targetTicks - progressTicks;
  float control = calcPID(distanceError,
                          Kp_dist_encoder, Kd_dist_encoder,
                          alpha_D,
                          last_distanceError,
                          filtered_D_distance,
                          dt);

  float maxWheelSpeed = userSpeedToWheelMMPS(maxUserSpeed);
  float minWheelSpeed = min(MIN_WHEEL_SPEED_MMPS, maxWheelSpeed);
  float commanded = constrain(control, -maxWheelSpeed, maxWheelSpeed);

  if (distanceError > 5 && abs(commanded) < minWheelSpeed) {
    commanded = minWheelSpeed;
  }

  return direction > 0 ? abs(commanded) : -abs(commanded);
}

void executeStraightMove(long targetTicks, int maxSpeed, int direction, const char* label) {
  resetEncoders();
  resetPID();
  resetMotorSpeedPID();
  clearMotionAbort();
  currentHeading = 0.0;
  lastTimeMicros = micros();
  robotState = (direction > 0) ? MOVING_FORWARD : MOVING_BACKWARD;

  appendControlLog(String(label) + " started");
  unsigned long lastLogMs = 0;

  while (robotState == MOVING_FORWARD || robotState == MOVING_BACKWARD) {
    serviceMotionLoop();
    if (robotState == STOPPED) {
      break;
    }

    unsigned long now = micros();
    float dt = max((now - lastTimeMicros) / 1000000.0f, 0.0001f);
    lastTimeMicros = now;

    updateHeading(dt);

    long leftTicks = 0;
    long rightTicks = 0;
    readTicks(leftTicks, rightTicks);
    long progressTicks = (abs(leftTicks) + abs(rightTicks)) / 2;

    float distanceError = targetTicks - progressTicks;
    if (distanceError <= 5) {
      robotState = STOPPED;
      break;
    }

    float targetHeading = constrain(sideWallCorrectionPID(dt), -10.0f, 10.0f);
    float headingCorrection = headingHoldPID(targetHeading, dt);
    float baseSpeedMMPS = distanceToBaseSpeedMMPS(progressTicks, targetTicks, maxSpeed, direction, dt);
    float leftTargetMMPS = baseSpeedMMPS - headingCorrection;
    float rightTargetMMPS = baseSpeedMMPS + headingCorrection;

    setWheelSpeedTargetsMMPS(leftTargetMMPS, rightTargetMMPS);
    updateMotorSpeedPID();

    if (millis() - lastLogMs >= 120) {
      lastLogMs = millis();
      appendControlLog(String(label) +
                       " ticks=" + progressTicks +
                       "/" + targetTicks +
                       " heading=" + String(currentHeading, 1) +
                       " wheelL=" + String(measuredLeftSpeedMMPS, 1) +
                       " wheelR=" + String(measuredRightSpeedMMPS, 1));
    }

    delay(10);
  }

  stopMotors();

  if (motionAbortRequested) {
    appendControlLog(String(label) + " aborted");
  } else {
    appendControlLog(String(label) + " complete");
  }

  clearMotionAbort();
  robotState = STOPPED;
}

void moveForwardOneCellPID(int maxSpeed = BASE_SPEED) {
  executeStraightMove(TARGET_TICKS, constrain(maxSpeed, 60, 150), 1, "Forward cell");
}

void moveBackwardOneCellPID(int maxSpeed = BASE_SPEED) {
  executeStraightMove(TARGET_TICKS, constrain(maxSpeed, 60, 150), -1, "Backward cell");
}

void executeTurn(float targetDegrees,
                 RobotState turnState,
                 const char* label,
                 int maxTurnSpeed = TURN_SPEED_MAX) {
  resetPID();
  resetMotorSpeedPID();
  clearMotionAbort();
  currentHeading = 0.0;
  turnTarget = targetDegrees;
  robotState = turnState;
  lastTimeMicros = micros();

  int clampedTurnSpeed = constrain(maxTurnSpeed, TURN_SPEED_MIN, TURN_SPEED_MAX);
  appendControlLog(String(label) + " started");
  unsigned long lastLogMs = 0;

  while (robotState == TURNING_LEFT ||
         robotState == TURNING_RIGHT ||
         robotState == TURNING_180) {
    serviceMotionLoop();
    if (robotState == STOPPED) {
      break;
    }

    unsigned long now = micros();
    float dt = max((now - lastTimeMicros) / 1000000.0f, 0.0001f);
    lastTimeMicros = now;

    updateHeading(dt);

    float turnError = turnTarget - currentHeading;
    float turnCorrection = calcPID(turnError,
                                   Kp_turn, Kd_turn,
                                   alpha_D,
                                   last_turnError,
                                   filtered_D_turn,
                                   dt);

    float maxTurnSpeedMMPS = userSpeedToWheelMMPS(clampedTurnSpeed);
    float minTurnSpeedMMPS = min(MIN_WHEEL_SPEED_MMPS, maxTurnSpeedMMPS);
    float turnSpeedMMPS = constrain(abs(turnCorrection), minTurnSpeedMMPS, maxTurnSpeedMMPS);

    if (turnError > 0) {
      setWheelSpeedTargetsMMPS(turnSpeedMMPS, -turnSpeedMMPS);
    } else {
      setWheelSpeedTargetsMMPS(-turnSpeedMMPS, turnSpeedMMPS);
    }

    updateMotorSpeedPID();

    if (millis() - lastLogMs >= 120) {
      lastLogMs = millis();
      appendControlLog(String(label) +
                       " heading=" + String(currentHeading, 1) +
                       " target=" + String(turnTarget, 1) +
                       " wheelL=" + String(measuredLeftSpeedMMPS, 1) +
                       " wheelR=" + String(measuredRightSpeedMMPS, 1));
    }

    if (abs(turnError) < TURN_THRESHOLD_DEG) {
      robotState = STOPPED;
    }

    delay(5);
  }

  stopMotors();
  delay(80);

  if (motionAbortRequested) {
    appendControlLog(String(label) + " aborted");
  } else {
    appendControlLog(String(label) + " complete");
  }

  clearMotionAbort();
  robotState = STOPPED;
}

void turnLeft90PID(int maxTurnSpeed = TURN_SPEED_MAX) {
  executeTurn(-90.0f, TURNING_LEFT, "Turn left 90", maxTurnSpeed);
}

void turnRight90PID(int maxTurnSpeed = TURN_SPEED_MAX) {
  executeTurn(90.0f, TURNING_RIGHT, "Turn right 90", maxTurnSpeed);
}

void turnAroundPID(int maxTurnSpeed = TURN_SPEED_MAX) {
  executeTurn(180.0f, TURNING_180, "Turn 180", maxTurnSpeed);
}

#endif
