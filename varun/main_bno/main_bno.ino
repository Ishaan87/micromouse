#include <Arduino.h>
#include "Config.h"
#include "Motors.h"
#include "Encoders.h"
#include "Sensors.h"
#include "Maze.h"

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
float baseTargetYaw = 0.0;     // The absolute heading we want (0 = forward, set by resetYaw())
float correction_angle = 0.0;  // The adjustment given by LiDARs
float targetYaw = 0.0;         // baseTargetYaw + correction_angle

float vel_tolerance = 0.5; 
float yaw_tolerance = 0.5; 
float wall_tolerance = 5.0;    // mm

const float SINGLE_WALL_TARGET_MM = 63.0; // desired distance when only one wall visible
const int   FRONT_STOP_MM         = 150;  // begin decelerating below this distance
const int   FRONT_HALT_MM         = 75;   // full stop below this distance

// ==========================================
// SYSTEM VARIABLES
// ==========================================
const int LOOP_INTERVAL_MS  = 20;   // 50Hz  — fast motor/gyro control
const int LIDAR_INTERVAL_MS = 50;   // 20Hz  — lidar + wall PID
const int PRINT_INTERVAL_MS = 100;  // 10Hz  — telemetry

unsigned long lastLoopTime  = 0;
unsigned long lastLidarTime = 0;
unsigned long lastPrintTime = 0;

float integral_vel_L = 0, integral_vel_R = 0;
float prev_error_vel_L = 0, prev_error_vel_R = 0;

float integral_yaw = 0, prev_error_yaw = 0;

// current_yaw_angle is now read directly from the BNO055 quaternion
// via readYawDegrees() — no more integration drift.
float current_yaw_angle = 0.0;

float integral_wall = 0, prev_error_wall = 0;

long prevLeftTicks  = 0;
long prevRightTicks = 0;

int final_pwm_L = 0;
int final_pwm_R = 0;

DistanceData current_lidars;

// ==========================================
// STATE MACHINE & NAVIGATION
// ==========================================
enum RobotState {
  DECIDING,         // Stopped, running Flood Fill, updating map
  DRIVING_STRAIGHT, // Moving 180mm forward
  TURNING           // Executing a 90 or 180 degree pivot
};

RobotState currentState = DECIDING;

// Variables to track distance inside the current cell
long cellStartLeftTicks = 0;
long cellStartRightTicks = 0;

void runControlLoop(float dt);
void runWallPIDLoop(float dt);
void printTelemetry();
void printCellData(int x, int y);

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
  calibrateGyro();   // Waits for BNO055 gyro to reach cal level 3
  Serial.println("Gyro Calibration Successful.");

  // Zero the heading: robot's current orientation becomes 0 degrees
  resetYaw();
  current_yaw_angle = 0.0;

  resetEncoders();
  prevLeftTicks  = 0;
  prevRightTicks = 0;
  integral_vel_L = 0;
  integral_vel_R = 0;
  integral_yaw   = 0;
  integral_wall  = 0;
  
  unsigned long now = millis();
  lastLoopTime  = now;
  lastLidarTime = now;
  lastPrintTime = now;
}

void loop() {
  unsigned long currentTime = millis();
  
  // 1. Medium Loop: LiDAR & Wall PID
  if (currentTime - lastLidarTime >= LIDAR_INTERVAL_MS) {
    float dt_lidar = (currentTime - lastLidarTime) / 1000.0;
    lastLidarTime = currentTime;
    runWallPIDLoop(dt_lidar);
  }

  // 2. Fast Loop: Yaw, Encoders, Motors
  if (currentTime - lastLoopTime >= LOOP_INTERVAL_MS) {
    float dt_motor = (currentTime - lastLoopTime) / 1000.0;
    lastLoopTime = currentTime;
    runControlLoop(dt_motor);
  }

  // 3. Slow Loop: Telemetry
  // if (currentTime - lastPrintTime >= PRINT_INTERVAL_MS) {
  //   lastPrintTime = currentTime;
  //   printTelemetry();
  // }
}

// ==========================================
// WALL CENTERING PID (Updates Target Yaw)
// Three cases:
//   1. Both walls visible  → centre between them
//   2. One wall visible    → maintain SINGLE_WALL_TARGET_MM from it
//   3. No walls            → hold current heading, reset integral
// ==========================================
void runWallPIDLoop(float dt) {
  current_lidars = readLidars();

  bool hasLeft  = current_lidars.left  < WALL_THRESHOLD;
  bool hasRight = current_lidars.right < WALL_THRESHOLD;

  float error_wall = 0.0;

  if (hasLeft && hasRight) {
    // Case 1: tunnel — centre between both walls
    error_wall = (float)(current_lidars.left - current_lidars.right);

  } else if (hasLeft && !hasRight) {
    // Case 2: only left wall visible — keep 63 mm from it
    // Positive error → too close to left → steer right (correction goes negative)
    error_wall = -(current_lidars.left - SINGLE_WALL_TARGET_MM);

  } else if (!hasLeft && hasRight) {
    // Case 2: only right wall visible — keep 63 mm from it
    // Negative error → too close to right → steer left (correction goes positive)
    error_wall = (current_lidars.right - SINGLE_WALL_TARGET_MM);

  } else {
    // Case 3: no walls — coast straight, reset
    correction_angle = 0.0;
    integral_wall    = 0.0;
    prev_error_wall  = 0.0;
    targetYaw = baseTargetYaw;
    return;
  }

  if (abs(error_wall) <= wall_tolerance) {
    error_wall    = 0.0;
    integral_wall = 0.0;
  }

  integral_wall += error_wall * dt;
  float deriv_wall = (error_wall - prev_error_wall) / dt;

  correction_angle = (Kp_wall * error_wall) + (Ki_wall * integral_wall) + (Kd_wall * deriv_wall);
  correction_angle = constrain(correction_angle, -10.0, 10.0);

  prev_error_wall = error_wall;
  targetYaw = baseTargetYaw + correction_angle;
}

// ==========================================
// FRONT WALL DECELERATION
// - Returns a velocity scale 0.0..1.0
//   based on front lidar distance.
//   1.0 = full speed, 0.0 = full stop.
// ==========================================
float frontBrakeScale() {
  int d = current_lidars.front;

  if (d >= FRONT_STOP_MM)  return 1.0;                                         // nothing ahead — full speed
  if (d <= FRONT_HALT_MM)  return 0.0;                                         // too close — stop

  // Linear ramp between FRONT_HALT_MM and FRONT_STOP_MM
  return (float)(d - FRONT_HALT_MM) / (float)(FRONT_STOP_MM - FRONT_HALT_MM);
}

// ==========================================
// MOTOR & YAW CONTROL LOOP
// ==========================================
void runControlLoop(float dt) {
  // Encoder velocity (ticks per loop)
  noInterrupts();
  long currentLeftTicks  = leftTicks;
  long currentRightTicks = -rightTicks;
  interrupts();

  float vel_L = (float)(currentLeftTicks  - prevLeftTicks);
  float vel_R = (float)(currentRightTicks - prevRightTicks);
  
  prevLeftTicks  = currentLeftTicks;
  prevRightTicks = currentRightTicks;

  // Now reads absolute, drift-free yaw directly from the BNO055 quaternion.
  current_yaw_angle = readYawDegrees();

  // =====================================================
  // STATE MACHINE LOGIC
  // =====================================================
  if (currentState == DRIVING_STRAIGHT) {
    // 1. Calculate how far we've moved since entering this cell
    long ticksMovedL = abs(currentLeftTicks - cellStartLeftTicks);
    long ticksMovedR = abs(currentRightTicks - cellStartRightTicks);
    long avgTicksMoved = (ticksMovedL + ticksMovedR) / 2;

    // 2. Check if we have reached the center of the next cell (180mm)
    if (avgTicksMoved >= TICKS_PER_CELL) {
      baseTargetVelocity = 0.0;   // Slam the brakes
      currentState = DECIDING;    // Change state to trigger the algorithm
      
      // Update global position (from Maze.h)
      moveToNextCell(); 
      updateMap(current_lidars, current_yaw_angle);
      printCellData(currentX, currentY);

    } else {
      baseTargetVelocity = 5.0;   // Keep driving
    }
  } 
  else if (currentState == DECIDING) {
    baseTargetVelocity = 0.0; // Ensure we stay completely still
    
    // NOTE: This is where we will eventually call the Flood Fill algorithm
    // For now, let's just wait 1 second and then simulate deciding to go straight again
    static unsigned long decideTimer = 0;
    if (decideTimer == 0) decideTimer = millis();
    
    if (millis() - decideTimer > 1000) {
      // "Decided" to go straight! Reset the counters and go.
      cellStartLeftTicks = currentLeftTicks;
      cellStartRightTicks = currentRightTicks;
      currentState = DRIVING_STRAIGHT;
      decideTimer = 0;
    }
  }
  else if (currentState == TURNING) {
    baseTargetVelocity = 0.0; // No forward movement while pivoting
  }

  // =====================================================
  // YAW & VELOCITY PIDs (Middle & Inner Loops)
  // =====================================================
  float error_yaw = targetYaw - current_yaw_angle;
  if (abs(error_yaw) <= yaw_tolerance) { error_yaw = 0; prev_error_yaw = 0; }

  integral_yaw += error_yaw * dt;
  float deriv_yaw = (error_yaw - prev_error_yaw) / dt;
  
  float heading_correction_vel = (Kp_yaw * error_yaw) + (Ki_yaw * integral_yaw) + (Kd_yaw * deriv_yaw);
  prev_error_yaw = error_yaw;

<<<<<<< HEAD
  // Apply heading correction to our velocity targets
  float target_vel_L = baseTargetVelocity - heading_correction_vel;
  float target_vel_R = baseTargetVelocity + heading_correction_vel;
=======
  // Scale target velocity down if front wall is close
  float brakeFactor = frontBrakeScale();

  // Hard stop — bypass PID entirely and kill motors
  if (brakeFactor <= 0.0) {
    applyMotorPWM(0, 0);
    final_pwm_L = 0;
    final_pwm_R = 0;
    // Reset velocity integrals so there's no windup while stopped
    integral_vel_L = 0;
    integral_vel_R = 0;
    prev_error_vel_L = 0;
    prev_error_vel_R = 0;
    return;
  }

  float effectiveVelocity = baseTargetVelocity * brakeFactor;
  int   effectiveBasePWM  = (int)(basePWM * brakeFactor);

  // Positive correction = turning ACW/Left → left slows, right speeds up
  float target_vel_L = effectiveVelocity - heading_correction_vel;
  float target_vel_R = effectiveVelocity + heading_correction_vel;
>>>>>>> 8aafd8b085eed09396755417831300e93e6ea5b0

  // Inner Loop (Velocity PID)
  float error_vel_L = target_vel_L - vel_L;
  float error_vel_R = target_vel_R - vel_R;
  
  if (abs(error_vel_L) <= vel_tolerance) { error_vel_L = 0; prev_error_vel_L = 0; }
  if (abs(error_vel_R) <= vel_tolerance) { error_vel_R = 0; prev_error_vel_R = 0; }

  integral_vel_L += error_vel_L * dt;
  integral_vel_R += error_vel_R * dt;

  float deriv_vel_L = (error_vel_L - prev_error_vel_L) / dt;
  float deriv_vel_R = (error_vel_R - prev_error_vel_R) / dt;

  final_pwm_L = effectiveBasePWM + (int)((Kp_vel_L * error_vel_L) + (Ki_vel_L * integral_vel_L) + (Kd_vel_L * deriv_vel_L));
  final_pwm_R = effectiveBasePWM + (int)((Kp_vel_R * error_vel_R) + (Ki_vel_R * integral_vel_R) + (Kd_vel_R * deriv_vel_R));

  prev_error_vel_L = error_vel_L;
  prev_error_vel_R = error_vel_R;

  applyMotorPWM(final_pwm_L, final_pwm_R);
}

// ==========================================
// DEBUG: PRINT CELL DATA
// ==========================================
void printCellData(int x, int y) {
  byte cell = maze[x][y];
  
  Serial.print("Map Cell ("); 
  Serial.print(x); 
  Serial.print(", "); 
  Serial.print(y); 
  Serial.print(") | Status: ");

  // Use Bitwise AND (&) to check if specific bits are turned on
  if (cell & VISITED) {
    Serial.print("VISITED | Walls: ");
  } else {
    Serial.print("UNKNOWN | Walls: ");
  }

  // Check each wall bit and print its letter
  if (cell & WALL_NORTH) Serial.print("N ");
  if (cell & WALL_EAST)  Serial.print("E ");
  if (cell & WALL_SOUTH) Serial.print("S ");
  if (cell & WALL_WEST)  Serial.print("W ");
  
  // If no walls are detected yet
  if ((cell & 15) == 0) Serial.print("None"); // 15 is the sum of 1+2+4+8

  Serial.println(); // Print a new line
}

// ==========================================
// TELEMETRY
// ==========================================
void printTelemetry() {
  noInterrupts();
  long currLeft  = leftTicks;
  long currRight = rightTicks;
  interrupts();

  char telemetryString[220];
  snprintf(telemetryString, sizeof(telemetryString), 
           "Target: %.1f | Enc: %ld / %ld | PWM: %d / %d | Yaw: %.2f | TargetYaw: %.2f | Lidar: %d / %d / %d", 
           baseTargetVelocity, currLeft, currRight, final_pwm_L, final_pwm_R, current_yaw_angle,
           targetYaw, current_lidars.left, current_lidars.front, current_lidars.right);
           
  Serial.println(telemetryString);
}