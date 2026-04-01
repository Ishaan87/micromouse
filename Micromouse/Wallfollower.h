#ifndef WALL_FOLLOWER_H
#define WALL_FOLLOWER_H

#include "WallMap.h"
#include "CellTracker.h"
#include "Motors.h"
#include "Turns.h"

// ==========================================
// WALL FOLLOWER
// Implements the right-hand rule maze
// traversal algorithm on top of the existing
// cell tracker and wall map infrastructure.
//
// RIGHT-HAND RULE PRIORITY ORDER:
//   1. Turn RIGHT if right is open
//   2. Go STRAIGHT if front is open
//   3. Turn LEFT  if left is open
//   4. Turn AROUND (dead end)
//
// HOW TO USE:
//   1. Call initWallFollower() in setup(),
//      after initWallMap() and initCellTracker().
//   2. In your loop() use wallFollowerUpdate()
//      instead of the old runControlLoop calls.
//
// GOAL:
//   Set GOAL_ROW / GOAL_COL below.
//   When reached the robot stops and prints
//   a full traversal summary.
// ==========================================

// ==========================================
// GOAL CELL  —  adjust to your maze target
// ==========================================
const int GOAL_ROW = 4;
const int GOAL_COL = 4;

// ==========================================
// TURN HELPERS
// Each wrapper calls the blocking Turns.h
// function then re-syncs baseTargetYaw to the
// yaw the robot actually settled at, so the
// yaw PID doesn't fight the new heading.
// ==========================================

void wf_turnRight() {
  turnCW90();
  // Snap baseTargetYaw to the settled heading so
  // the yaw PID has a clean reference going forward.
  baseTargetYaw = readYawDegrees();
  targetYaw     = baseTargetYaw;
  // Reset yaw PID integrator to avoid windup from
  // any error that built up during the turn.
  integral_yaw   = 0.0f;
  prev_error_yaw = 0.0f;
}

void wf_turnLeft() {
  turnACW90();
  baseTargetYaw  = readYawDegrees();
  targetYaw      = baseTargetYaw;
  integral_yaw   = 0.0f;
  prev_error_yaw = 0.0f;
}

void wf_turnAround() {
  turn180();
  baseTargetYaw  = readYawDegrees();
  targetYaw      = baseTargetYaw;
  integral_yaw   = 0.0f;
  prev_error_yaw = 0.0f;
}

// ==========================================
// WALL FOLLOWER STATE
// ==========================================
enum WFState {
  WF_IDLE,        // not yet started
  WF_MOVING,      // driving straight through a cell
  WF_GOAL_REACHED // finished — robot has stopped
};

static WFState  wf_state        = WF_IDLE;
static int      wf_cellsVisited = 0;
static int      wf_turnsMade    = 0;
static uint32_t wf_startTimeMs  = 0;

// ==========================================
// GOAL CHECK
// ==========================================
inline bool wf_atGoal() {
  return (ctGetRow() == GOAL_ROW && ctGetCol() == GOAL_COL);
}

// ==========================================
// PRINT STATS
// Called once when goal is reached.
// ==========================================
void wf_printStats() {
  EKFState s = ekfGetState();
  uint32_t elapsedMs = millis() - wf_startTimeMs;

  Serial.println();
  Serial.println("╔══════════════════════════════════╗");
  Serial.println("║      WALL FOLLOWER — GOAL!       ║");
  Serial.println("╚══════════════════════════════════╝");
  Serial.printf("  Goal cell     : (%d, %d)\n", GOAL_ROW, GOAL_COL);
  Serial.printf("  Cells visited : %d\n",  wf_cellsVisited);
  Serial.printf("  Turns made    : %d\n",  wf_turnsMade);
  Serial.printf("  Elapsed time  : %.2f s\n", elapsedMs / 1000.0f);
  Serial.printf("  EKF position  : x=%.1f mm  y=%.1f mm\n", s.x_mm, s.y_mm);
  Serial.printf("  EKF heading   : %.1f deg\n", ekfRadToDeg(s.theta_rad));
  Serial.println();
  printWallMapASCII();
}

// ==========================================
// INIT
// Call after initWallMap() + initCellTracker()
// in setup().
// ==========================================
void initWallFollower() {
  wf_state        = WF_MOVING;   // start driving immediately
  wf_cellsVisited = 0;
  wf_turnsMade    = 0;
  wf_startTimeMs  = millis();
  Serial.println("[WF] Wall follower initialised — right-hand rule active.");
  Serial.printf("[WF] Goal: (%d, %d)\n", GOAL_ROW, GOAL_COL);
}

// ==========================================
// DECIDE NEXT ACTION
// Uses the walls already recorded in wallMap
// for the current cell and applies right-hand
// rule priority:
//   RIGHT → STRAIGHT → LEFT → TURN AROUND
//
// Call this right after recordWalls() has
// populated the current cell's wall data
// (i.e. inside the newCell branch of your
// control loop).
// ==========================================
void wf_decideAndTurn() {
  int         row     = ctGetRow();
  int         col     = ctGetCol();
  MazeHeading heading = ctGetHeading();

  // Query absolute walls for the current heading
  bool rightOpen  = canMove(row, col, (MazeHeading)((heading + 1) % 4));
  bool frontOpen  = canMove(row, col, heading);
  bool leftOpen   = canMove(row, col, (MazeHeading)((heading + 3) % 4));

  Serial.printf(
    "[WF] Cell (%d,%d) facing %s | R=%s F=%s L=%s\n",
    row, col, headingName(heading),
    rightOpen ? "open" : "wall",
    frontOpen ? "open" : "wall",
    leftOpen  ? "open" : "wall"
  );

  // Right-hand rule decision tree
  if (rightOpen) {
    Serial.println("[WF] Decision: TURN RIGHT");
    wf_turnRight();
    wf_turnsMade++;

  } else if (frontOpen) {
    Serial.println("[WF] Decision: STRAIGHT");
    // No turn needed — keep driving

  } else if (leftOpen) {
    Serial.println("[WF] Decision: TURN LEFT");
    wf_turnLeft();
    wf_turnsMade++;

  } else {
    Serial.println("[WF] Decision: DEAD END — turn around");
    wf_turnAround();
    wf_turnsMade += 2;   // counts as two 90° turns
  }
}

// ==========================================
// WALL FOLLOWER UPDATE
// Call this INSTEAD OF your raw
//   runControlLoop() / runWallPIDLoop()
// calls inside loop().
//
// It does NOT replace those functions — it
// calls them and then adds the maze-logic
// layer on top.
// ==========================================
void wallFollowerUpdate(float dt_motor, float dt_lidar, bool runMotor, bool runLidar) {

  // Already done — do nothing
  if (wf_state == WF_GOAL_REACHED) {
    applyMotorPWM(0, 0);
    return;
  }

  // Run the existing lidar PID at its own cadence
  if (runLidar) {
    runWallPIDLoop(dt_lidar);   // keeps wall-centering active while moving
  }

  // Run the existing motor / EKF / cell-tracker loop
  if (runMotor) {
    // ── replicate runControlLoop, adding cell-event hook ──

    noInterrupts();
    long currentLeftTicks  = leftTicks;
    long currentRightTicks = rightTicks;
    interrupts();

    long deltaLeftTicks  = currentLeftTicks  - prevLeftTicks;
    long deltaRightTicks = currentRightTicks - prevRightTicks;

    current_yaw_angle = readYawDegrees();

    ekfPredict(deltaLeftTicks, deltaRightTicks);
    ekfUpdateYawDeg(current_yaw_angle);
    ekfTelemetry = ekfGetTelemetry();

    bool newCell = updateCellTracker();

    if (newCell) {
      wf_cellsVisited++;

      // Record walls for the cell we just entered
      recordWalls(ctGetRow(), ctGetCol(), ctGetHeading(), current_lidars);
      printCellWalls(ctGetRow(), ctGetCol());
      printCellState();

      // Check for goal BEFORE deciding next turn
      if (wf_atGoal()) {
        applyMotorPWM(0, 0);
        wf_state = WF_GOAL_REACHED;
        wf_printStats();
        prevLeftTicks  = currentLeftTicks;
        prevRightTicks = currentRightTicks;
        return;
      }

      // Apply right-hand rule — may call a blocking turn function
      wf_decideAndTurn();
    }

    // ── standard yaw + velocity PID (unchanged from your original) ──
    maybeFinalizeDistance(currentLeftTicks, currentRightTicks);

    float error_yaw = targetYaw - current_yaw_angle;
    if (abs(error_yaw) <= yaw_tolerance) { error_yaw = 0; prev_error_yaw = 0; }

    integral_yaw += error_yaw * dt_motor;
    float deriv_yaw = (error_yaw - prev_error_yaw) / dt_motor;
    float heading_correction_vel =
      (Kp_yaw * error_yaw) + (Ki_yaw * integral_yaw) + (Kd_yaw * deriv_yaw);
    prev_error_yaw = error_yaw;

    float brakeFactor = frontBrakeScale();

    if (brakeFactor <= 0.0f) {
      // Front wall imminent: stop, then the next newCell event will
      // trigger a turn decision (turn left or around — can't go right
      // or straight with a wall in front)
      applyMotorPWM(0, 0);
      final_pwm_L = 0;
      final_pwm_R = 0;
      integral_vel_L = 0; integral_vel_R = 0;
      prev_error_vel_L = 0; prev_error_vel_R = 0;
      prevLeftTicks  = currentLeftTicks;
      prevRightTicks = currentRightTicks;
      return;
    }

    float effectiveVelocity  = baseTargetVelocity * brakeFactor;
    int   effectiveBasePWM   = (int)(basePWM * brakeFactor);

    float target_vel_L = effectiveVelocity - heading_correction_vel;
    float target_vel_R = effectiveVelocity + heading_correction_vel;

    float vel_L = (float)deltaLeftTicks;
    float vel_R = (float)deltaRightTicks;

    float error_vel_L = target_vel_L - vel_L;
    float error_vel_R = target_vel_R - vel_R;

    if (abs(error_vel_L) <= vel_tolerance) { error_vel_L = 0; prev_error_vel_L = 0; }
    if (abs(error_vel_R) <= vel_tolerance) { error_vel_R = 0; prev_error_vel_R = 0; }

    integral_vel_L += error_vel_L * dt_motor;
    integral_vel_R += error_vel_R * dt_motor;

    float deriv_vel_L = (error_vel_L - prev_error_vel_L) / dt_motor;
    float deriv_vel_R = (error_vel_R - prev_error_vel_R) / dt_motor;

    final_pwm_L = effectiveBasePWM +
      (int)((Kp_vel_L * error_vel_L) + (Ki_vel_L * integral_vel_L) + (Kd_vel_L * deriv_vel_L));
    final_pwm_R = effectiveBasePWM +
      (int)((Kp_vel_R * error_vel_R) + (Ki_vel_R * integral_vel_R) + (Kd_vel_R * deriv_vel_R));

    prev_error_vel_L = error_vel_L;
    prev_error_vel_R = error_vel_R;

    applyMotorPWM(final_pwm_L, final_pwm_R);

    prevLeftTicks  = currentLeftTicks;
    prevRightTicks = currentRightTicks;
  }
}

#endif // WALL_FOLLOWER_H