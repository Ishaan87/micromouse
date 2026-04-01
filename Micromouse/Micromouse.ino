#include <Arduino.h>
#include "Config.h"
#include "Motors.h"
#include "Encoders.h"
#include "Sensors.h"
#include "EKF.h"
#include "CellTracker.h"
#include "WallMap.h"
#include "Turns.h"           
#include "WallFollower.h"    

// ==========================================
// TUNING PARAMETERS
// ==========================================
// 1. Inner Loop (Velocity PID)
float Kp_vel_L = 1.0, Ki_vel_L = 0.0, Kd_vel_L = 0.1;
float Kp_vel_R = 1.0, Ki_vel_R = 0.0, Kd_vel_R = 0.1;

// 2. Middle Loop (Yaw PID)
float Kp_yaw = 0.6, Ki_yaw = 0.0, Kd_yaw = 0.06; 

// 3. Outer Loop (Wall Centering PID) - GAIN SCHEDULING
float Kp_tunnel = 0.12; 
float Kd_tunnel = 0.08;
float Kp_single = 0.06; 
float Kd_single = 0.12; 

// Active constants
float Kp_wall = Kp_tunnel;
float Kd_wall = Kd_tunnel;
float Ki_wall = 0.0;

float baseTargetVelocity = 15.0; // Ticks per loop
int basePWM = 50;

// Heading Management
float baseTargetYaw = 0.0;
float correction_angle = 0.0;
float targetYaw = 0.0;

float vel_tolerance = 0.5;
float yaw_tolerance = 0.5;
float wall_tolerance = 10.0; 

// --- 155MM CELL SIZE CORRECTIONS ---
const int WALL_THRESHOLD = 110;  
const float SINGLE_WALL_TARGET_MM = 63.0; 
const int FRONT_STOP_MM = 110;   
const int FRONT_HALT_MM = 65;    

// ==========================================
// EKF / ODOMETRY CONSTANTS
// ==========================================
const float TICKS_PER_REV = 306.0f;
const float WHEEL_CIRCUMFERENCE_MM = 144.5f;
const float TRACK_WIDTH_MM = 72.0f;

const unsigned long STOP_DETECT_MS = 1000;

// ==========================================
// SYSTEM VARIABLES
// ==========================================
const int LOOP_INTERVAL_MS  = 20;
const int LIDAR_INTERVAL_MS = 50;
const int PRINT_INTERVAL_MS = 100;

unsigned long lastLoopTime  = 0;
unsigned long lastLidarTime = 0;
unsigned long lastPrintTime = 0;

float integral_vel_L = 0, integral_vel_R = 0;
float prev_error_vel_L = 0, prev_error_vel_R = 0;

float integral_yaw = 0, prev_error_yaw = 0;
float current_yaw_angle = 0.0;

float integral_wall = 0, prev_error_wall = 0;

long prevLeftTicks  = 0;
long prevRightTicks = 0;

int final_pwm_L = 0;
int final_pwm_R = 0;

DistanceData current_lidars;
EKFTelemetry ekfTelemetry;

// ==========================================
// EKF DISTANCE REPORT STATE
// ==========================================
bool movementStarted       = false;
bool finalDistancePrinted  = false;
long lastObservedLeftTicks  = 0;
long lastObservedRightTicks = 0;
unsigned long encoderStillStartMs = 0;

// ==========================================
// FORWARD DECLARATIONS
// ==========================================
void runControlLoop(float dt);
void runWallPIDLoop(float dt);
void printTelemetry();

void resetDistanceReportState() {
  movementStarted       = false;
  finalDistancePrinted  = false;
  lastObservedLeftTicks  = 0;
  lastObservedRightTicks = 0;
  encoderStillStartMs   = 0;
}

void maybeFinalizeDistance(long currentLeftTicks, long currentRightTicks) {
  if (finalDistancePrinted) return;

  long deltaLeftTicks  = currentLeftTicks  - prevLeftTicks;
  long deltaRightTicks = currentRightTicks - prevRightTicks;

  if (!movementStarted) {
    if (deltaLeftTicks != 0 || deltaRightTicks != 0) {
      movementStarted        = true;
      lastObservedLeftTicks  = currentLeftTicks;
      lastObservedRightTicks = currentRightTicks;
      encoderStillStartMs    = 0;
      Serial.println("Motion detected. EKF distance tracking active.");
    }
    return;
  }

  if (currentLeftTicks == lastObservedLeftTicks &&
      currentRightTicks == lastObservedRightTicks) {
    if (encoderStillStartMs == 0) {
      encoderStillStartMs = millis();
    } else if (millis() - encoderStillStartMs >= STOP_DETECT_MS) {
      EKFState s = ekfGetState();
      float axisDistanceMM  = fabs(s.x_mm);
      float displacementMM  = sqrt(s.x_mm * s.x_mm + s.y_mm * s.y_mm);

      finalDistancePrinted = true;

      Serial.println();
      Serial.println("===== EKF LOCALIZATION RESULT =====");
      Serial.print("EKF x (travel axis): ");  Serial.print(s.x_mm, 2);       Serial.println(" mm");
      Serial.print("EKF y (side drift):  ");  Serial.print(s.y_mm, 2);       Serial.println(" mm");
      Serial.print("EKF heading:         ");  Serial.print(ekfRadToDeg(s.theta_rad), 2); Serial.println(" deg");
      Serial.print("Axis distance:       ");  Serial.print(axisDistanceMM, 2); Serial.println(" mm");
      Serial.print("Axis distance:       ");  Serial.print(axisDistanceMM / 10.0f, 2); Serial.println(" cm");
      Serial.print("Total displacement:  ");  Serial.print(displacementMM, 2); Serial.println(" mm");
      Serial.println("===================================");
      Serial.println();
    }
  } else {
    encoderStillStartMs    = 0;
    lastObservedLeftTicks  = currentLeftTicks;
    lastObservedRightTicks = currentRightTicks;
  }
}

// ==========================================
// SETUP
// ==========================================
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

  prevLeftTicks  = 0;
  prevRightTicks = 0;
  integral_vel_L = 0;
  integral_vel_R = 0;
  integral_yaw   = 0;
  integral_wall  = 0;

  initWallFollower();

  unsigned long now = millis();
  lastLoopTime  = now;
  lastLidarTime = now;
  lastPrintTime = now;
}

// ==========================================
// LOOP
// ==========================================
void loop() {
  unsigned long currentTime = millis();

  bool doMotor = (currentTime - lastLoopTime  >= LOOP_INTERVAL_MS);
  bool doLidar = (currentTime - lastLidarTime >= LIDAR_INTERVAL_MS);
  bool doPrint = (currentTime - lastPrintTime >= PRINT_INTERVAL_MS);

  float dt_motor = doMotor ? (currentTime - lastLoopTime)  / 1000.0f : 0.0f;
  float dt_lidar = doLidar ? (currentTime - lastLidarTime) / 1000.0f : 0.0f;

  if (doMotor)  lastLoopTime  = currentTime;
  if (doLidar)  lastLidarTime = currentTime;

  wallFollowerUpdate(dt_motor, dt_lidar, doMotor, doLidar);

  if (doPrint) {
    lastPrintTime = currentTime;
    printTelemetry();
  }
}

// ==========================================
// WALL CENTERING PID
// ==========================================
void runWallPIDLoop(float dt) {
  current_lidars = readLidars();
  bool hasLeft  = current_lidars.left  < WALL_THRESHOLD;
  bool hasRight = current_lidars.right < WALL_THRESHOLD;

  float error_wall = 0.0f;

  if (hasLeft && hasRight) {
    Kp_wall = Kp_tunnel;
    Kd_wall = Kd_tunnel;
    error_wall = (float)(current_lidars.left - current_lidars.right);
    
  } else if (hasLeft && !hasRight) {
    Kp_wall = Kp_single;
    Kd_wall = Kd_single;
    error_wall = -(current_lidars.left - SINGLE_WALL_TARGET_MM);
    
  } else if (!hasLeft && hasRight) {
    Kp_wall = Kp_single;
    Kd_wall = Kd_single;
    error_wall = (current_lidars.right - SINGLE_WALL_TARGET_MM);
    
  } else {
    correction_angle = 0.0f;
    integral_wall    = 0.0f;
    prev_error_wall  = 0.0f;
    targetYaw        = baseTargetYaw;
    return;
  }

  if (abs(error_wall) <= wall_tolerance) {
    error_wall    = 0.0f;
    integral_wall = 0.0f;
  }

  integral_wall += error_wall * dt;
  float deriv_wall = (error_wall - prev_error_wall) / dt;
  
  float target_correction = (Kp_wall * error_wall) + (Ki_wall * integral_wall) + (Kd_wall * deriv_wall);
  
  correction_angle = (correction_angle * 0.7f) + (target_correction * 0.3f);
  correction_angle = constrain(correction_angle, -8.0f, 8.0f);

  prev_error_wall = error_wall;
  targetYaw       = baseTargetYaw + correction_angle;
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
// ORIGINAL CONTROL LOOP
// ==========================================
void runControlLoop(float dt) {
  // Logic lives in wallFollowerUpdate() inside WallFollower.h
}

// ==========================================
// TELEMETRY
// ==========================================
void printTelemetry() {
  noInterrupts();
  long currLeft  = leftTicks;
  long currRight = rightTicks;
  interrupts();

  char telemetryString[300];
  snprintf(
    telemetryString, sizeof(telemetryString),
    "Target: %.1f | Enc: %ld / %ld | PWM: %d / %d | Yaw: %.2f | "
    "EKF: %.1f, %.1f, %.2f | Cell: (%d,%d) %s | "
    "Lidar: %d / %d / %d | WF: %s",
    baseTargetVelocity,
    currLeft, currRight,
    final_pwm_L, final_pwm_R,
    current_yaw_angle,
    ekfTelemetry.x_mm, ekfTelemetry.y_mm, ekfTelemetry.theta_deg,
    ctGetRow(), ctGetCol(), headingName(ctGetHeading()),
    current_lidars.left, current_lidars.front, current_lidars.right,
    (wf_state == WF_IDLE)         ? "IDLE" :
    (wf_state == WF_MOVING)       ? "MOVING" :
                                    "GOAL!"
  );
  Serial.println(telemetryString);
}