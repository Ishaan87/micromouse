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
float Kp_yaw = 0.75,  Ki_yaw = 0.0,  Kd_yaw = 0.06;

// Wall-centering PID
float Kp_tunnel = 0.15, Kd_tunnel = 0.08;
float Kp_single = 0.06, Kd_single = 0.12;
float Kp_wall   = Kp_tunnel;
float Kd_wall   = Kd_tunnel;
float Ki_wall   = 0.0;

// Front Wall Braking PD
float Kp_front = 0.009;   
float Kd_front = 0.001;  
float prev_error_front = -1.0f; // Initialized to -1 to prevent math spikes

float baseTargetVelocity = 25.0;
int   basePWM            = 125;

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
const int   WALL_THRESHOLD         = 110;
const float SINGLE_WALL_TARGET_MM  = 63.0f;
const int   FRONT_STOP_MM          = 150; // start braking
const int   FRONT_HALT_MM          = 60;  // full stop

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
bool isStartup           = true;

// ==========================================
// FORWARD DECLARATIONS
// ==========================================
void runWallPIDLoop(float dt);
void runDrivingPID(float dt_motor, bool frontBlocked);
void printTelemetry();
void driveForwardDistanceMM(float distanceMM);
void resetPIDIntegrals();
float calculateFrontPDBrake(float dt);

// ==========================================
// RESET HELPERS
// ==========================================
void resetPIDIntegrals() {
  integral_vel_L = 0; prev_error_vel_L = 0;
  integral_vel_R = 0; prev_error_vel_R = 0;
  integral_yaw   = 0; prev_error_yaw   = 0;
  integral_wall  = 0; prev_error_wall  = 0;
  prev_error_front = -1.0f; // Reset to prevent derivative spike
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

    // SAFETY ABORT
    int current_front_dist = readLidars().front;
    if (current_front_dist <= FRONT_HALT_MM) {
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
// FRONT WALL PD BRAKE SCALE
// ==========================================
float calculateFrontPDBrake(float dt) {
  int d = current_lidars.front;

  if (d >= FRONT_STOP_MM) {
    prev_error_front = -1.0f; 
    return 1.0f;
  }

  float error_front = (float)(d - FRONT_HALT_MM);

  if (error_front <= 0.0f) {
    prev_error_front = error_front;
    return 0.0f;
  }

  float d_front = 0.0f;
  if (dt > 0.0f && prev_error_front >= 0.0f) {
    d_front = (error_front - prev_error_front) / dt;
  }

  float pd_output = (Kp_front * error_front) + (Kd_front * d_front);
  prev_error_front = error_front;

  return constrain(pd_output, 0.0f, 1.0f);
}

// ==========================================
// SETUP
// ==========================================
void setup() {
  Serial.begin(115200);
  delay(1000);

  Serial.println("\n=================================");
  Serial.println("  STATIONARY PRIORITY FOLLOWER");
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
  isStartup     = true; 

  Serial.println("Setup complete.");
}

// ==========================================
// MAIN LOOP
// ==========================================
void loop() {
  unsigned long now = millis();

  if (now - lastPrintTime >= PRINT_INTERVAL_MS) {
    lastPrintTime = now;
    printTelemetry();
  }

  bool doLidar = (now - lastLidarTime >= LIDAR_INTERVAL_MS);
  bool doMotor = (now - lastLoopTime  >= LOOP_INTERVAL_MS);

  if (doLidar) {
    float dt_lidar = (now - lastLidarTime) / 1000.0f;
    lastLidarTime  = now;
    current_lidars = readLidars();

    if (currentState == DRIVING) {
      runWallPIDLoop(dt_lidar);
    }
  }

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
    else {
      bool rightOpen  = (current_lidars.right > WALL_MISSING_THRESHOLD);
      bool leftOpen   = (current_lidars.left  > WALL_MISSING_THRESHOLD);
      bool frontOpen  = (current_lidars.front > FRONT_BLOCKED_THRESHOLD);

      // Trigger stops based on falling edges or reaching a front wall
      bool rightGapJustDetected = (rightOpen && !rightWallWasPresent);
      bool stoppedAtFrontWall   = (current_lidars.front <= FRONT_HALT_MM + 5);
      bool frontIsClear         = (current_lidars.front >= FRONT_STOP_MM);

      // IMMEDIATE TRIGGER: Only stop for an open intersection if the front is clear
      // (If a front wall is there, let the PD brake naturally stop us)
      bool clearIntersectionDetected = rightGapJustDetected && frontIsClear;

      bool shouldDecide = stoppedAtFrontWall || clearIntersectionDetected || isStartup;

      if (shouldDecide) {
        Serial.println("\n[RX] DECISION MATRIX TRIGGERED");

        // Center the bot in the intersection ONLY if we found an open right gap
        if (clearIntersectionDetected && !isStartup && PIVOT_OFFSET_MM > 0.0f) {
          driveForwardDistanceMM(PIVOT_OFFSET_MM);
        }

        // KILL MOMENTUM & SETTLE
        applyMotorPWM(0, 0);
        delay(300); 

        // ── READ FRESH STATIONARY DATA ──
        // This is exactly what you asked for: Decision based purely on the lidars AFTER stopping.
        current_lidars = readLidars();
        
        bool priorityRight = (current_lidars.right > WALL_MISSING_THRESHOLD);
        bool priorityLeft  = (current_lidars.left  > WALL_MISSING_THRESHOLD);
        
        // Front priority is only valid if we didn't just stop because of a front wall
        bool priorityFront = (current_lidars.front > FRONT_BLOCKED_THRESHOLD) && !stoppedAtFrontWall;

        if (priorityRight) {
          Serial.println("[RX] Priority: TURN RIGHT");
          turnCW90();
        } else if (priorityFront) {
          Serial.println("[RX] Priority: GO STRAIGHT");
        } else if (priorityLeft) {
          Serial.println("[RX] Priority: TURN LEFT");
          turnACW90();
        } else {
          Serial.println("[RX] Priority: DEAD END — U-TURN");
          turn180();
        }

        // ── WIPE MEMORY FOR NEXT CORRIDOR ──
        isStartup = false;
        
        baseTargetYaw = readYawDegrees();
        targetYaw     = baseTargetYaw;
        resetPIDIntegrals();

        noInterrupts();
        prevLeftTicks  = leftTicks;
        prevRightTicks = rightTicks;
        interrupts();
        lastLoopTime   = millis();

        currentState    = TURN_COOLDOWN;
        cooldownStartMs = millis();

      } else {
        // Safe to keep driving; PD Brake is fully in charge of stopping
        runDrivingPID(dt_motor, !frontOpen);
      }

      // Record wall state for falling edge detection on the next loop
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

  float brakeFactor = calculateFrontPDBrake(dt_motor);

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