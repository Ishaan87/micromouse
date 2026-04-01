#include <Arduino.h>
#include "Config.h"
#include "Motors.h"
#include "Encoders.h"
#include "Sensors.h"
#include "EKF.h"
#include "CellTracker.h"
#include "WallMap.h"

// ==========================================
// TUNING PARAMETERS
// ==========================================
// 1. Inner Loop (Velocity PID)
float Kp_vel_L = 1.0, Ki_vel_L = 0.0, Kd_vel_L = 0.1;
float Kp_vel_R = 1.0, Ki_vel_R = 0.0, Kd_vel_R = 0.1;

// 2. Middle Loop (Yaw PID)
float Kp_yaw = 1.0, Ki_yaw = 0.0, Kd_yaw = 0.05;

// 3. Outer Loop (Wall Centering PID)
float Kp_wall = 0.25;
float Ki_wall = 0.0;
float Kd_wall = 0.05;

float baseTargetVelocity = 15.0;
int basePWM = 50;

// Heading Management
float baseTargetYaw = 0.0;
float correction_angle = 0.0;
float targetYaw = 0.0;

float vel_tolerance = 0.5;
float yaw_tolerance = 0.5;
float wall_tolerance = 5.0;
const int WALL_THRESHOLD = 150;

const float SINGLE_WALL_TARGET_MM = 63.0;
const int FRONT_STOP_MM = 150;
const int FRONT_HALT_MM = 75;

// ==========================================
// EKF / ODOMETRY CONSTANTS
// ==========================================
// Replace with your measured values.
const float TICKS_PER_REV = 306.0f;
const float WHEEL_CIRCUMFERENCE_MM = 144.5f;
const float TRACK_WIDTH_MM = 72.0f;

// Encoder totals must remain unchanged for 1 second
// before we print the final EKF distance.
const unsigned long STOP_DETECT_MS = 1000;

// ==========================================
// SYSTEM VARIABLES
// ==========================================
const int LOOP_INTERVAL_MS = 20;
const int LIDAR_INTERVAL_MS = 50;
const int PRINT_INTERVAL_MS = 100;

unsigned long lastLoopTime = 0;
unsigned long lastLidarTime = 0;
unsigned long lastPrintTime = 0;

float integral_vel_L = 0, integral_vel_R = 0;
float prev_error_vel_L = 0, prev_error_vel_R = 0;

float integral_yaw = 0, prev_error_yaw = 0;
float current_yaw_angle = 0.0;

float integral_wall = 0, prev_error_wall = 0;

long prevLeftTicks = 0;
long prevRightTicks = 0;

int final_pwm_L = 0;
int final_pwm_R = 0;

DistanceData current_lidars;
EKFTelemetry ekfTelemetry;

// ==========================================
// EKF DISTANCE REPORT STATE
// ==========================================
bool movementStarted = false;
bool finalDistancePrinted = false;
long lastObservedLeftTicks = 0;
long lastObservedRightTicks = 0;
unsigned long encoderStillStartMs = 0;

void runControlLoop(float dt);
void runWallPIDLoop(float dt);
void printTelemetry();

void resetDistanceReportState() {
  movementStarted = false;
  finalDistancePrinted = false;
  lastObservedLeftTicks = 0;
  lastObservedRightTicks = 0;
  encoderStillStartMs = 0;
}

void maybeFinalizeDistance(long currentLeftTicks, long currentRightTicks) {
  if (finalDistancePrinted) return;

  long deltaLeftTicks = currentLeftTicks - prevLeftTicks;
  long deltaRightTicks = currentRightTicks - prevRightTicks;

  if (!movementStarted) {
    if (deltaLeftTicks != 0 || deltaRightTicks != 0) {
      movementStarted = true;
      lastObservedLeftTicks = currentLeftTicks;
      lastObservedRightTicks = currentRightTicks;
      encoderStillStartMs = 0;
      Serial.println("Motion detected. EKF distance tracking active.");
    }
    return;
  }

  if (currentLeftTicks == lastObservedLeftTicks && currentRightTicks == lastObservedRightTicks) {
    if (encoderStillStartMs == 0) {
      encoderStillStartMs = millis();
    } else if (millis() - encoderStillStartMs >= STOP_DETECT_MS) {
      EKFState s = ekfGetState();
      float axisDistanceMM = fabs(s.x_mm);
      float displacementMM = sqrt(s.x_mm * s.x_mm + s.y_mm * s.y_mm);

      finalDistancePrinted = true;

      Serial.println();
      Serial.println("===== EKF LOCALIZATION RESULT =====");
      Serial.print("EKF x (travel axis): ");
      Serial.print(s.x_mm, 2);
      Serial.println(" mm");

      Serial.print("EKF y (side drift): ");
      Serial.print(s.y_mm, 2);
      Serial.println(" mm");

      Serial.print("EKF heading: ");
      Serial.print(ekfRadToDeg(s.theta_rad), 2);
      Serial.println(" deg");

      Serial.print("Estimated travel-axis distance: ");
      Serial.print(axisDistanceMM, 2);
      Serial.println(" mm");

      Serial.print("Estimated travel-axis distance: ");
      Serial.print(axisDistanceMM / 10.0f, 2);
      Serial.println(" cm");

      Serial.print("Estimated total displacement magnitude: ");
      Serial.print(displacementMM, 2);
      Serial.println(" mm");
      Serial.println("===================================");
      Serial.println();
    }
  } else {
    encoderStillStartMs = 0;
    lastObservedLeftTicks = currentLeftTicks;
    lastObservedRightTicks = currentRightTicks;
  }
}

void setup() {
  Serial.begin(115200);
  delay(1000);

  Serial.println("\n=================================");
  Serial.println("  MICROMOUSE SYSTEM STARTUP");
  Serial.println("=================================");

  initMotors();
  initEncoders();
  initSensors();
  delay(5000);

  Serial.println("\n[!] PREPARING BNO055 CALIBRATION [!]");
  Serial.println("Please ensure the bot is perfectly still.");
  delay(2000);
  calibrateGyro();
  Serial.println("Gyro Calibration Successful.");

  resetYaw();
  current_yaw_angle = 0.0;

  ekfConfigure(TICKS_PER_REV, WHEEL_CIRCUMFERENCE_MM, TRACK_WIDTH_MM);
  ekfInit(0.0f, 0.0f, 0.0f);
  initCellTracker();
  initWallMap();
  ekfTelemetry = ekfGetTelemetry();

  resetEncoders();
  resetDistanceReportState();

  prevLeftTicks = 0;
  prevRightTicks = 0;
  integral_vel_L = 0;
  integral_vel_R = 0;
  integral_yaw = 0;
  integral_wall = 0;

  unsigned long now = millis();
  lastLoopTime = now;
  lastLidarTime = now;
  lastPrintTime = now;
}

void loop() {
  unsigned long currentTime = millis();

  if (currentTime - lastLidarTime >= LIDAR_INTERVAL_MS) {
    float dt_lidar = (currentTime - lastLidarTime) / 1000.0f;
    lastLidarTime = currentTime;
    runWallPIDLoop(dt_lidar);
  }

  if (currentTime - lastLoopTime >= LOOP_INTERVAL_MS) {
    float dt_motor = (currentTime - lastLoopTime) / 1000.0f;
    lastLoopTime = currentTime;
    runControlLoop(dt_motor);
  }

  if (currentTime - lastPrintTime >= PRINT_INTERVAL_MS) {
    lastPrintTime = currentTime;
    printTelemetry();
  }
}

// ==========================================
// WALL CENTERING PID
// ==========================================
void runWallPIDLoop(float dt) {
  current_lidars = readLidars();

  bool hasLeft = current_lidars.left < WALL_THRESHOLD;
  bool hasRight = current_lidars.right < WALL_THRESHOLD;

  float error_wall = 0.0f;

  if (hasLeft && hasRight) {
    error_wall = (float)(current_lidars.left - current_lidars.right);

  } else if (hasLeft && !hasRight) {
    error_wall = -(current_lidars.left - SINGLE_WALL_TARGET_MM);

  } else if (!hasLeft && hasRight) {
    error_wall = (current_lidars.right - SINGLE_WALL_TARGET_MM);

  } else {
    correction_angle = 0.0f;
    integral_wall = 0.0f;
    prev_error_wall = 0.0f;
    targetYaw = baseTargetYaw;
    return;
  }

  if (abs(error_wall) <= wall_tolerance) {
    error_wall = 0.0f;
    integral_wall = 0.0f;
  }

  integral_wall += error_wall * dt;
  float deriv_wall = (error_wall - prev_error_wall) / dt;

  correction_angle = (Kp_wall * error_wall) + (Ki_wall * integral_wall) + (Kd_wall * deriv_wall);
  correction_angle = constrain(correction_angle, -10.0f, 10.0f);

  prev_error_wall = error_wall;
  targetYaw = baseTargetYaw + correction_angle;
}

// ==========================================
// FRONT WALL DECELERATION
// ==========================================
float frontBrakeScale() {
  int d = current_lidars.front;

  if (d >= FRONT_STOP_MM) return 1.0f;
  if (d <= FRONT_HALT_MM) return 0.0f;

  return (float)(d - FRONT_HALT_MM) / (float)(FRONT_STOP_MM - FRONT_HALT_MM);
}

// ==========================================
// MOTOR + YAW CONTROL LOOP
// ==========================================
void runControlLoop(float dt) {
  noInterrupts();
  long currentLeftTicks = leftTicks;
  long currentRightTicks = -rightTicks;
  interrupts();

  long deltaLeftTicks = currentLeftTicks - prevLeftTicks;
  long deltaRightTicks = currentRightTicks - prevRightTicks;

  float vel_L = (float)deltaLeftTicks;
  float vel_R = (float)deltaRightTicks;

  current_yaw_angle = readYawDegrees();

  // EKF localization
  ekfPredict(deltaLeftTicks, deltaRightTicks);
  ekfUpdateYawDeg(current_yaw_angle);
  ekfTelemetry = ekfGetTelemetry();

  bool newCell = updateCellTracker();
  if (newCell) {
    // Robot just entered a new cell — snapshot walls immediately
    recordWalls(
      ctGetRow(),
      ctGetCol(),
      ctGetHeading(),
      current_lidars        // already populated by runWallPIDLoop
    );
    printCellWalls(ctGetRow(), ctGetCol());   // serial debug
    printCellState();                         // serial debug
  }

  // Stop/final-report logic
  maybeFinalizeDistance(currentLeftTicks, currentRightTicks);

  // Yaw PID
  float error_yaw = targetYaw - current_yaw_angle;

  if (abs(error_yaw) <= yaw_tolerance) {
    error_yaw = 0;
    prev_error_yaw = 0;
  }

  integral_yaw += error_yaw * dt;
  float deriv_yaw = (error_yaw - prev_error_yaw) / dt;

  float heading_correction_vel =
    (Kp_yaw * error_yaw) + (Ki_yaw * integral_yaw) + (Kd_yaw * deriv_yaw);

  prev_error_yaw = error_yaw;

  float brakeFactor = frontBrakeScale();

  if (brakeFactor <= 0.0f) {
    applyMotorPWM(0, 0);
    final_pwm_L = 0;
    final_pwm_R = 0;
    integral_vel_L = 0;
    integral_vel_R = 0;
    prev_error_vel_L = 0;
    prev_error_vel_R = 0;

    prevLeftTicks = currentLeftTicks;
    prevRightTicks = currentRightTicks;
    return;
  }

  float effectiveVelocity = baseTargetVelocity * brakeFactor;
  int effectiveBasePWM = (int)(basePWM * brakeFactor);

  float target_vel_L = effectiveVelocity - heading_correction_vel;
  float target_vel_R = effectiveVelocity + heading_correction_vel;

  float error_vel_L = target_vel_L - vel_L;
  float error_vel_R = target_vel_R - vel_R;

  if (abs(error_vel_L) <= vel_tolerance) {
    error_vel_L = 0;
    prev_error_vel_L = 0;
  }
  if (abs(error_vel_R) <= vel_tolerance) {
    error_vel_R = 0;
    prev_error_vel_R = 0;
  }

  integral_vel_L += error_vel_L * dt;
  integral_vel_R += error_vel_R * dt;

  float deriv_vel_L = (error_vel_L - prev_error_vel_L) / dt;
  float deriv_vel_R = (error_vel_R - prev_error_vel_R) / dt;

  final_pwm_L = effectiveBasePWM + (int)((Kp_vel_L * error_vel_L) + (Ki_vel_L * integral_vel_L) + (Kd_vel_L * deriv_vel_L));

  final_pwm_R = effectiveBasePWM + (int)((Kp_vel_R * error_vel_R) + (Ki_vel_R * integral_vel_R) + (Kd_vel_R * deriv_vel_R));

  prev_error_vel_L = error_vel_L;
  prev_error_vel_R = error_vel_R;

  applyMotorPWM(final_pwm_L, final_pwm_R);

  prevLeftTicks = currentLeftTicks;
  prevRightTicks = currentRightTicks;
}

// ==========================================
// TELEMETRY
// ==========================================
void printTelemetry() {
  noInterrupts();
  long currLeft = leftTicks;
  long currRight = rightTicks;
  interrupts();

  char telemetryString[280];
  snprintf(
    telemetryString,
    sizeof(telemetryString),
    "Target: %.1f | Enc: %ld / %ld | PWM: %d / %d | Yaw: %.2f | EKF: %.1f, %.1f, %.2f | Cell: (%d,%d) %s | Lidar: %d / %d / %d",
    baseTargetVelocity,
    currLeft,
    currRight,
    final_pwm_L,
    final_pwm_R,
    current_yaw_angle,
    ekfTelemetry.x_mm,
    ekfTelemetry.y_mm,
    ekfTelemetry.theta_deg,
    ctGetRow(), ctGetCol(), headingName(ctGetHeading()),   // ← new
    current_lidars.left,
    current_lidars.front,
    current_lidars.right
  );

  Serial.println(telemetryString);
}
