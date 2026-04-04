#include <Arduino.h>
#include "Config.h"
#include "Motors.h"
#include "Encoders.h"
#include "Sensors.h"
#include "ekf.h"
#include "CellTracker.h"
#include "WallMap.h"
#include "Turns.h"

// ==========================================
// TUNING PARAMETERS
// ==========================================
// Velocity PID
float Kp_vel_L = 1.0,  Ki_vel_L = 0.0, Kd_vel_L = 0.1;
float Kp_vel_R = 1.0,  Ki_vel_R = 0.0, Kd_vel_R = 0.1;

// Yaw PID
float Kp_yaw = 0.6,  Ki_yaw = 0.0,  Kd_yaw = 0.06;

// Wall-centering PID
float Kp_tunnel = 0.12, Kd_tunnel = 0.08;
float Kp_single = 0.06, Kd_single = 0.12;
float Kp_wall   = Kp_tunnel;
float Kd_wall   = Kd_tunnel;
float Ki_wall   = 0.0;

float baseTargetVelocity = 75.0;
int   basePWM            = 35;

// Heading management
float baseTargetYaw    = 0.0;
float correction_angle = 0.0;
float targetYaw        = 0.0;

// Tolerances
float vel_tolerance  = 0.5;
float yaw_tolerance  = 0.5;
float wall_tolerance = 10.0;

// ==========================================
// GEOMETRY  (155 mm cells)
// ==========================================
const int   WALL_THRESHOLD         = 110;   // lidar < this → wall present for PID
const float SINGLE_WALL_TARGET_MM  = 63.0f;
const int   FRONT_STOP_MM          = 100;   // start braking
const int   FRONT_HALT_MM          = 75;    // full stop

// ==========================================
// EKF / ODOMETRY CONSTANTS
// ==========================================
const float TICKS_PER_REV          = 306.0f;
const float WHEEL_CIRCUMFERENCE_MM = 144.5f;
const float TRACK_WIDTH_MM         = 72.0f;

// ==========================================
// REACTIVE SOLVER CONSTANTS
// ==========================================
const int WALL_MISSING_THRESHOLD  = 180;  
const int FRONT_BLOCKED_THRESHOLD = 70;   
const float PIVOT_OFFSET_MM       = 77.5f;

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
int  final_pwm_L    = 0;
int  final_pwm_R    = 0;

DistanceData current_lidars;

// ==========================================
// REACTIVE SOLVER STATE
// ==========================================
enum BotState {
  DRIVING,
  TURN_COOLDOWN
};

BotState      currentState      = DRIVING;
unsigned long cooldownStartMs   = 0;
const unsigned long TURN_COOLDOWN_MS = 600; 

bool rightWallWasPresent = true;
bool leftWallWasPresent  = true;

// ==========================================
// FORWARD DECLARATIONS
// ==========================================
void runWallPIDLoop(float dt);
void runDrivingPID(float dt_motor, bool frontBlocked);
void printTelemetry();
void driveForwardDistanceMM(float distanceMM);
void resetPIDIntegrals();
float frontBrakeScale();

// ==========================================
// RESET HELPERS
// ==========================================
void resetPIDIntegrals() {
  integral_vel_L = 0; prev_error_vel_L = 0;
  integral_vel_R = 0; prev_error_vel_R = 0;
  integral_yaw   = 0; prev_error_yaw   = 0;
  integral_wall  = 0; prev_error_wall  = 0;
}

// ==========================================
// BLOCKING HELPER: DRIVE DISTANCE
// ==========================================
void driveForwardDistanceMM(float distanceMM) {
  noInterrupts();
  long startLeft  = leftTicks;
  long startRight = rightTicks;
  interrupts();

  float targetTicks  = (distanceMM / WHEEL_CIRCUMFERENCE_MM) * TICKS_PER_REV;
  float yawRef       = readYawDegrees();

  while (true) {
    noInterrupts();
    long curLeft  = leftTicks;
    long curRight = rightTicks;
    interrupts();

    float avgMoved = ((curLeft - startLeft) + (curRight - startRight)) / 2.0f;
    if (avgMoved >= targetTicks) {
      applyMotorPWM(0, 0);
      break;
    }

    float yaw_err = yawRef - readYawDegrees();
    float corr    = Kp_yaw * yaw_err * 20.0f;
    applyMotorPWM(60 - (int)corr, 60 + (int)corr);
    delay(10);
  }
  delay(100); 
}

// ==========================================
// FRONT WALL BRAKE SCALE
// ==========================================
float frontBrakeScale() {
  int d = current_lidars.front;
  if (d >= FRONT_STOP_MM) return 1.0f;
  if (d <= FRONT_HALT_MM) return 0.0f;
  return (float)(d - FRONT_HALT_MM) / (float)(FRONT_STOP_MM - FRONT_HALT_MM);
}

// ==========================================
// SETUP
// ==========================================
void setup() {
  Serial.begin(115200);
  delay(1000);

  Serial.println("\n=================================");
  Serial.println("  SIMPLE WALL FOLLOWER STARTUP");
  Serial.println("=================================");

  initMotors();
  initEncoders();
  initSensors();
  delay(5000);

  Serial.println("\n[!] BNO055 CALIBRATION [!]");
  Serial.println("Keep bot still...");
  delay(2000);
  calibrateGyro();
  Serial.println("Gyro Calibration OK.");

  resetYaw();
  current_yaw_angle = 0.0;
  baseTargetYaw     = 0.0;
  targetYaw         = 0.0;

  // Keep EKF initialized for accurate yaw/encoder tracking if turns rely on it
  ekfConfigure(TICKS_PER_REV, WHEEL_CIRCUMFERENCE_MM, TRACK_WIDTH_MM);
  ekfInit(0.0f, 0.0f, 0.0f);

  resetEncoders();
  prevLeftTicks  = 0;
  prevRightTicks = 0;
  resetPIDIntegrals();

  current_lidars = readLidars();
  rightWallWasPresent = (current_lidars.right < WALL_MISSING_THRESHOLD);
  leftWallWasPresent  = (current_lidars.left  < WALL_MISSING_THRESHOLD);

  unsigned long now = millis();
  lastLoopTime  = now;
  lastLidarTime = now;
  lastPrintTime = now;
  currentState  = DRIVING;

  Serial.println("Setup complete. Running Right-Hand Rule.");
}

// ==========================================
// MAIN LOOP
// ==========================================
void loop() {
  unsigned long now = millis();
  
  // ── 1. TELEMETRY ──
  if (now - lastPrintTime >= PRINT_INTERVAL_MS) {
    lastPrintTime = now;
    printTelemetry();
  }

  bool doLidar = (now - lastLidarTime >= LIDAR_INTERVAL_MS);
  bool doMotor = (now - lastLoopTime  >= LOOP_INTERVAL_MS);

  // ── 2. LIDAR UPDATE ──
  if (doLidar) {
    float dt_lidar = (now - lastLidarTime) / 1000.0f;
    lastLidarTime  = now;
    current_lidars = readLidars();

    if (currentState == DRIVING) {
      runWallPIDLoop(dt_lidar);
    }
  }

  // ── 3. MOTOR / SOLVER UPDATE ──
  if (doMotor) {
    float dt_motor = (now - lastLoopTime) / 1000.0f;
    lastLoopTime   = now;

    noInterrupts();
    long currentLeftTicks  =  leftTicks;
    long currentRightTicks = rightTicks;
    interrupts();

    current_yaw_angle = readYawDegrees();

    prevLeftTicks  = currentLeftTicks;
    prevRightTicks = currentRightTicks;

    // ── COOLDOWN STATE ──
    if (currentState == TURN_COOLDOWN) {
      if (now - cooldownStartMs >= TURN_COOLDOWN_MS) {
        current_lidars      = readLidars();
        rightWallWasPresent = (current_lidars.right < WALL_MISSING_THRESHOLD);
        leftWallWasPresent  = (current_lidars.left  < WALL_MISSING_THRESHOLD);
        currentState        = DRIVING;
      } else {
        runDrivingPID(dt_motor, false);
      }
    } 
    // ── DRIVING STATE (SIMPLE RIGHT-HAND RULE) ──
    else {
      bool rightOpen  = (current_lidars.right > WALL_MISSING_THRESHOLD);
      bool leftOpen   = (current_lidars.left  > WALL_MISSING_THRESHOLD);
      bool frontOpen  = (current_lidars.front > FRONT_BLOCKED_THRESHOLD);

      bool rightGapSustained = (rightOpen && !rightWallWasPresent);
      bool shouldDecide = (!frontOpen) || rightGapSustained;

      if (shouldDecide) {
        Serial.println("\n[RX] INTERSECTION DETECTED");

        // Move to the center of the intersection before turning
        if (frontOpen && PIVOT_OFFSET_MM > 0.0f) {
          driveForwardDistanceMM(PIVOT_OFFSET_MM);
        }

        // Read sensors one more time from the center of the intersection
        current_lidars = readLidars();
        rightOpen  = (current_lidars.right > WALL_MISSING_THRESHOLD);
        leftOpen   = (current_lidars.left  > WALL_MISSING_THRESHOLD);
        frontOpen  = (current_lidars.front > FRONT_BLOCKED_THRESHOLD);

        // Simple Right-Hand Wall Follower Priority: Right -> Straight -> Left -> U-Turn
        if (rightOpen) {
          Serial.println("[RX] Following Wall: TURN RIGHT");
          turnCW90();
        } else if (frontOpen) {
          Serial.println("[RX] Following Wall: GO STRAIGHT");
        } else if (leftOpen) {
          Serial.println("[RX] Following Wall: TURN LEFT");
          turnACW90();
        } else {
          Serial.println("[RX] Following Wall: DEAD END — U-TURN");
          turn180();
        }

        baseTargetYaw = readYawDegrees();
        targetYaw     = baseTargetYaw;
        resetPIDIntegrals();

        currentState    = TURN_COOLDOWN;
        cooldownStartMs = millis();

      } else {
        // Keep driving straight and maintaining center
        runDrivingPID(dt_motor, !frontOpen);
      }

      rightWallWasPresent = !rightOpen;
      leftWallWasPresent  = !leftOpen;
    }
  }
}

// ==========================================
// DRIVING PID
// ==========================================
void runDrivingPID(float dt_motor, bool frontBlocked) {
  noInterrupts();
  long curLeft  =  leftTicks;
  long curRight = rightTicks;
  interrupts();

  float vel_L = (float)(curLeft  - prevLeftTicks);
  float vel_R = (float)(curRight - prevRightTicks);

  float error_yaw = targetYaw - current_yaw_angle;
  if (fabsf(error_yaw) <= yaw_tolerance) { error_yaw = 0; prev_error_yaw = 0; }
  integral_yaw += error_yaw * dt_motor;
  float deriv_yaw = (error_yaw - prev_error_yaw) / dt_motor;
  float heading_corr = (Kp_yaw * error_yaw) + (Ki_yaw * integral_yaw) + (Kd_yaw * deriv_yaw);
  prev_error_yaw = error_yaw;

  float brakeFactor = frontBrakeScale();

  if (brakeFactor <= 0.0f) {
    applyMotorPWM(0, 0);
    final_pwm_L = 0; final_pwm_R = 0;
    integral_vel_L = 0; integral_vel_R = 0;
    prev_error_vel_L = 0; prev_error_vel_R = 0;
    return;
  }

  float effVel = baseTargetVelocity * brakeFactor;
  int   effPWM = (int)(basePWM * brakeFactor);

  float tgt_L = effVel - heading_corr;
  float tgt_R = effVel + heading_corr;

  float err_L = tgt_L - vel_L;
  float err_R = tgt_R - vel_R;

  if (fabsf(err_L) <= vel_tolerance) { err_L = 0; prev_error_vel_L = 0; }
  if (fabsf(err_R) <= vel_tolerance) { err_R = 0; prev_error_vel_R = 0; }

  integral_vel_L += err_L * dt_motor;
  integral_vel_R += err_R * dt_motor;

  float d_L = (err_L - prev_error_vel_L) / dt_motor;
  float d_R = (err_R - prev_error_vel_R) / dt_motor;

  final_pwm_L = effPWM + (int)((Kp_vel_L * err_L) + (Ki_vel_L * integral_vel_L) + (Kd_vel_L * d_L));
  final_pwm_R = effPWM + (int)((Kp_vel_R * err_R) + (Ki_vel_R * integral_vel_R) + (Kd_vel_R * d_R));

  prev_error_vel_L = err_L;
  prev_error_vel_R = err_R;

  applyMotorPWM(final_pwm_L, final_pwm_R);
}

// ==========================================
// WALL CENTERING PID
// ==========================================
void runWallPIDLoop(float dt) {
  bool hasLeft  = (current_lidars.left  < WALL_THRESHOLD);
  bool hasRight = (current_lidars.right < WALL_THRESHOLD);
  float error_wall = 0.0f;

  if (hasLeft && hasRight) {
    Kp_wall    = Kp_tunnel;
    Kd_wall    = Kd_tunnel;
    error_wall = (float)(current_lidars.left - current_lidars.right);

  } else if (hasLeft && !hasRight) {
    Kp_wall    = Kp_single;
    Kd_wall    = Kd_single;
    error_wall = -(current_lidars.left - SINGLE_WALL_TARGET_MM);

  } else if (!hasLeft && hasRight) {
    Kp_wall    = Kp_single;
    Kd_wall    = Kd_single;
    error_wall = (current_lidars.right - SINGLE_WALL_TARGET_MM);

  } else {
    correction_angle = 0.0f;
    integral_wall    = 0.0f;
    prev_error_wall  = 0.0f;
    targetYaw        = baseTargetYaw;
    return;
  }

  if (fabsf(error_wall) <= wall_tolerance) {
    error_wall    = 0.0f;
    integral_wall = 0.0f;
  }

  integral_wall += error_wall * dt;
  float deriv_wall  = (error_wall - prev_error_wall) / dt;
  float target_corr = (Kp_wall * error_wall) + (Ki_wall * integral_wall) + (Kd_wall * deriv_wall);

  correction_angle = (correction_angle * 0.7f) + (target_corr * 0.3f);
  correction_angle = constrain(correction_angle, -8.0f, 8.0f);

  prev_error_wall = error_wall;
  targetYaw       = baseTargetYaw + correction_angle;
}

// ==========================================
// TELEMETRY
// ==========================================
void printTelemetry() {
  noInterrupts();
  long currLeft  = leftTicks;
  long currRight = rightTicks;
  interrupts();

  char buf[200];
  snprintf(buf, sizeof(buf),
    "Target: %.1f | Enc: %ld / %ld | PWM: %d / %d | Yaw: %.2f | Lidar: %d / %d / %d",
    baseTargetVelocity,
    currLeft, currRight,
    final_pwm_L, final_pwm_R,
    current_yaw_angle,
    current_lidars.left, current_lidars.front, current_lidars.right
  );

  Serial.println(buf);
}