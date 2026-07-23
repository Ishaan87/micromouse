#ifndef SOLVER_H
#define SOLVER_H

#include "Floodfill.h"
#include "CellTracker.h"
#include "WallMap.h"
#include "Turns.h"
#include "Motors.h"
#include "Sensors.h"
#include "ekf.h"

// ============================================================
<<<<<<< Updated upstream
// SOLVER — Speed-Run Phase Constants
// ============================================================
const float SOLVE_BASE_VELOCITY  = 90.0f;   
const int   SOLVE_BASE_PWM       = 65;
const float SOLVE_TURN_APPROACH_VELOCITY = 45.0f;
const int   SOLVE_TURN_APPROACH_PWM      = 32;

const int   SOLVE_FRONT_STOP_MM  = 90;
const int   SOLVE_FRONT_HALT_MM  = 55;
const unsigned long SOLVE_COOLDOWN_MS = 500;

=======
// SOLVER — Speed-Run Phase
//
// Executes the pre-planned path from ffComputePath() as fast
// as reliably possible. Key differences from the survey phase:
//
//  1. PATH IS KNOWN — no lidar-based decisions. The robot
//     follows the ff_path[] sequence step by step, turning
//     only when the plan says to, not when a wall gap appears.
//
//  2. STRAIGHT-RUN MERGING — consecutive moves in the same
//     heading direction are merged into one continuous straight
//     run. The robot never stops between cells going the same
//     direction; it only stops/slows at actual turns.
//
//  3. SPEED PROFILE — separate (faster) PID gains and velocity
//     target for the solve phase. These are tuned aggressively
//     but can be dialled back if the bot becomes unstable.
//
//  4. SMOOTH TURNS — turn approach: decelerate into the turn
//     cell using a tighter brake window, centre precisely, turn,
//     then accelerate out. All done with the existing Turns.h
//     blocking turn functions (they're already PD-controlled).
//
//  5. WALL CENTERING stays active throughout straight runs,
//     providing lateral correction even at higher speed.
//
// HOW TO USE:
//   After ffComputePath() returns true:
//     solverInit(startHeading);
//   Then in loop():
//     solverUpdate(dt_motor, dt_lidar, doMotor, doLidar);
//   Check solverDone() to know when the goal is reached.
// ============================================================

// ============================================================
// SPEED-RUN TUNING  — adjust these for your hardware
// These are intentionally separate from the survey-phase
// values so you can tune each phase independently.
// ============================================================

// Straight-line speed (ticks/loop-tick, same units as survey)
const float SOLVE_BASE_VELOCITY  = 90.0f;   // ~1.8× survey speed
const int   SOLVE_BASE_PWM       = 65;

// Approach speed when a turn is coming up within 1 cell
const float SOLVE_TURN_APPROACH_VELOCITY = 45.0f;
const int   SOLVE_TURN_APPROACH_PWM      = 32;

// Brake window for speed-run (tighter than survey — we know the map)
const int   SOLVE_FRONT_STOP_MM  = 90;
const int   SOLVE_FRONT_HALT_MM  = 55;

// After a turn, wait this long before re-engaging wall PID
// (shorter than survey because we're more confident in pose)
const unsigned long SOLVE_COOLDOWN_MS = 500;

// Velocity PID gains for solve phase (can be more aggressive)
>>>>>>> Stashed changes
const float SOLVE_KP_VEL  = 1.2f;
const float SOLVE_KI_VEL  = 0.05f;
const float SOLVE_KD_VEL  = 0.08f;

<<<<<<< Updated upstream
=======
// Yaw PID gains for solve phase
>>>>>>> Stashed changes
const float SOLVE_KP_YAW  = 0.8f;
const float SOLVE_KI_YAW  = 0.01f;
const float SOLVE_KD_YAW  = 0.06f;

<<<<<<< Updated upstream
enum SolverState {
  SV_IDLE,
  SV_STRAIGHT,           
  SV_APPROACHING,        
  SV_BRAKING_TO_CENTER,  
  SV_COOLDOWN,           
=======
// ============================================================
// SOLVER STATE
// ============================================================
enum SolverState {
  SV_IDLE,
  SV_STRAIGHT,      // driving between cells, no turn coming
  SV_APPROACHING,   // turn is next — slow down, aim for centre
  SV_TURNING,       // blocked: executing blocking turn
  SV_COOLDOWN,      // post-turn: straight drive, no wall PID
>>>>>>> Stashed changes
  SV_GOAL_REACHED
};

static SolverState sv_state       = SV_IDLE;
<<<<<<< Updated upstream
static int         sv_moveIndex   = 0;    
static int         sv_pathLen     = 0;

static MazeHeading sv_currentHeading = HEADING_NORTH;
static unsigned long sv_cooldownStart = 0;
static float sv_pendingTurnDeg = 0.0f;

// External references linked to Micromouse.ino
extern float baseTargetYaw;
extern float targetYaw;
extern float correction_angle;
extern float integral_yaw;
extern float prev_error_yaw;
extern float integral_vel_L;
extern float integral_vel_R;
extern float prev_error_vel_L;
extern float prev_error_vel_R;
extern float integral_wall;
extern float prev_error_wall;
extern float current_yaw_angle;
extern int   final_pwm_L;
extern int   final_pwm_R;
extern long  prevLeftTicks;
extern long  prevRightTicks;
extern DistanceData current_lidars;
extern EKFTelemetry ekfTelemetry;

extern float Kp_tunnel;
extern float Kd_tunnel;
extern float Kp_single;
extern float Kd_single;
extern float Kp_wall;
extern float Kd_wall;
extern float Ki_wall;
extern float wall_tolerance;

// Helper from Micromouse.ino
float getEKFDistanceToCenter();
=======
static int         sv_moveIndex   = 0;    // current position in ff_path[]
static int         sv_pathLen     = 0;

// Heading the robot is currently facing (absolute)
static MazeHeading sv_currentHeading = HEADING_NORTH;

// Used to detect "turn coming up" one cell ahead
static MazeHeading sv_nextMoveHeading = HEADING_NORTH;
static bool        sv_turnComingUp    = false;

// Cooldown timer
static unsigned long sv_cooldownStart = 0;

// External references — these live in Micromouse.ino.
// Declared extern so Solver.h can read/write them.
extern float baseTargetYaw;
extern float targetYaw;
extern float correction_angle;
extern float integral_yaw, prev_error_yaw;
extern float integral_vel_L, integral_vel_R;
extern float prev_error_vel_L, prev_error_vel_R;
extern float integral_wall, prev_error_wall;
extern float current_yaw_angle;
extern int   final_pwm_L, final_pwm_R;
extern long  prevLeftTicks, prevRightTicks;
extern DistanceData current_lidars;
extern EKFTelemetry ekfTelemetry;

// NEW GLOBAL 2D DISTANCE TRACKERS
extern float total_distance_mm;
extern float cell_entry_distance_mm;

// Wall PID parameters (shared with survey phase via Micromouse.ino)
extern float Kp_tunnel, Kd_tunnel;
extern float Kp_single, Kd_single;
extern float Kp_wall, Kd_wall, Ki_wall;
extern float wall_tolerance;
>>>>>>> Stashed changes

// ============================================================
// INTERNAL HELPERS
// ============================================================
<<<<<<< Updated upstream
static void sv_resetAllPID() {
  integral_vel_L = 0; 
  prev_error_vel_L = 0;
  integral_vel_R = 0; 
  prev_error_vel_R = 0;
  integral_yaw   = 0; 
  prev_error_yaw   = 0;
  integral_wall  = 0; 
  prev_error_wall  = 0;
  correction_angle = 0.0f; 
  targetYaw = baseTargetYaw;
}

static float sv_wrapAngleDegrees(float angle) {
  while (angle > 180.0f) angle -= 360.0f;
  while (angle < -180.0f) angle += 360.0f;
  return angle;
}

static float sv_relativeTurnDeg(MazeHeading from, MazeHeading to) {
  int delta = ((int)to - (int)from + 4) % 4;  
  if (delta == 0) return   0.0f;
  if (delta == 1) return -90.0f;   
  if (delta == 2) return -180.0f;  
  return  90.0f;                   
}

=======

static void sv_resetAllPID() {
  integral_vel_L   = 0; prev_error_vel_L = 0;
  integral_vel_R   = 0; prev_error_vel_R = 0;
  integral_yaw     = 0; prev_error_yaw   = 0;
  integral_wall    = 0; prev_error_wall  = 0;
  correction_angle = 0.0f;
  targetYaw        = baseTargetYaw;
}

// How many consecutive moves from sv_moveIndex share the same heading?
// Used to decide whether to slow for a turn.
static int sv_straightRunLength() {
  MazeHeading h = ffGetMove(sv_moveIndex);
  int count = 0;
  for (int i = sv_moveIndex; i < sv_pathLen; i++) {
    if (ffGetMove(i) == h) count++;
    else break;
  }
  return count;
}

// True if the next move (after sv_moveIndex) changes heading
static bool sv_nextMoveIsTurn() {
  if (sv_moveIndex + 1 >= sv_pathLen) return false;  // last move — no next
  return (ffGetMove(sv_moveIndex + 1) != ffGetMove(sv_moveIndex));
}

// Compute relative turn needed to go from currentHeading to targetHeading.
// Returns angle in degrees: +90 = left (ACW), -90 = right (CW), ±180 = U-turn.
static float sv_relativeTurnDeg(MazeHeading from, MazeHeading to) {
  int delta = ((int)to - (int)from + 4) % 4;  // 0,1,2,3
  // 0 = straight, 1 = turn right (CW, -90), 2 = U-turn, 3 = turn left (ACW, +90)
  if (delta == 0) return   0.0f;
  if (delta == 1) return -90.0f;   // CW
  if (delta == 2) return 180.0f;   // U-turn (use -180 for CW spin)
  return  90.0f;                   // CCW
}

// Brake scale for solve phase (tighter window than survey)
>>>>>>> Stashed changes
static float sv_frontBrakeScale() {
  int d = current_lidars.front;
  if (d >= SOLVE_FRONT_STOP_MM) return 1.0f;
  if (d <= SOLVE_FRONT_HALT_MM) return 0.0f;
  return (float)(d - SOLVE_FRONT_HALT_MM) / (float)(SOLVE_FRONT_STOP_MM - SOLVE_FRONT_HALT_MM);
}

<<<<<<< Updated upstream
static void executeSolveTurn(float turnDeltaDeg) {
  float nextBaseTargetYaw = sv_wrapAngleDegrees(baseTargetYaw + turnDeltaDeg);
  
  if (turnDeltaDeg == -90.0f) {
      turnCW90(nextBaseTargetYaw);
  } else if (turnDeltaDeg == 90.0f) {
      turnACW90(nextBaseTargetYaw);
  } else if (fabsf(turnDeltaDeg) == 180.0f) {
      turn180(nextBaseTargetYaw);
  }

  delay(100);
  baseTargetYaw = nextBaseTargetYaw;
  targetYaw     = baseTargetYaw;
  sv_resetAllPID();
}

// ============================================================
// SOLVER PID LOOPS
// ============================================================
static void sv_runDrivingPID(float dt, float algorithmScale, bool isApproaching, long cL, long cR) {
  float vel_L = (float)(cL - prevLeftTicks);
  float vel_R = (float)(cR - prevRightTicks);
  
  float error_yaw = sv_wrapAngleDegrees(targetYaw - current_yaw_angle);
  
  if (fabsf(error_yaw) <= 0.5f) { 
      error_yaw = 0; 
      prev_error_yaw = 0; 
  }
  
  integral_yaw += error_yaw * dt;
  float deriv_yaw = (error_yaw - prev_error_yaw) / dt;
  float heading_corr = (SOLVE_KP_YAW * error_yaw) + (SOLVE_KI_YAW * integral_yaw) + (SOLVE_KD_YAW * deriv_yaw);
  prev_error_yaw = error_yaw;

  float effVel = isApproaching ? SOLVE_TURN_APPROACH_VELOCITY : SOLVE_BASE_VELOCITY;
  int   effPWM = isApproaching ? SOLVE_TURN_APPROACH_PWM      : SOLVE_BASE_PWM;

  float lidarBrake = sv_frontBrakeScale();
  float finalScale;

  // The Clash Fix: Algorithm overrides Lidar during controlled braking
  if (algorithmScale > 0.9f) {
      finalScale = min(lidarBrake, algorithmScale);
  } else {
      finalScale = algorithmScale;
  }

  if (finalScale <= 0.0f) {
    applyMotorPWM(0, 0);
    final_pwm_L = 0; 
    final_pwm_R = 0;
    integral_vel_L = 0; 
    integral_vel_R = 0; 
    prev_error_vel_L = 0; 
    prev_error_vel_R = 0;
    return;
  }

  effVel *= finalScale; 
  effPWM = (int)(effPWM * finalScale);
  
  float tgt_L = effVel - heading_corr;
  float tgt_R = effVel + heading_corr;
=======
// Align to cell centre using robust 2D encoder odometry
static void sv_alignToCentre() {
  // Use global trackers instead of 1D EKF limits
  float distanceMovedInCell = total_distance_mm - cell_entry_distance_mm;
  float remaining = CELL_HALF_MM - distanceMovedInCell;

  if (remaining <= CENTRE_TOLERANCE_MM) {
    Serial.printf("[SV] Already centred (moved=%.1f).\n", distanceMovedInCell);
    return;
  }

  Serial.printf("[SV] Aligning %.1fmm to centre.\n", remaining);
  float targetTicks = (remaining / WHEEL_CIRCUMFERENCE_MM) * TICKS_PER_REV;

  noInterrupts();
  long startL = leftTicks, startR = rightTicks;
  interrupts();

  float yawRef = readYawDegrees();

  while (true) {
    noInterrupts();
    long cL = leftTicks, cR = rightTicks;
    interrupts();
    float moved = ((cL - startL) + (cR - startR)) / 2.0f;
    if (moved >= targetTicks) { applyMotorPWM(0,0); delay(50); break; }

    // Gentle straight drive during alignment
    float yaw_err = yawRef - readYawDegrees();
    int corr = (int)(SOLVE_KP_YAW * yaw_err * 20.0f);
    applyMotorPWM(50 - corr, 50 + corr);
    delay(10);
  }
}

// Execute the physical turn for the current move index
static void sv_executeTurn() {
  MazeHeading from = sv_currentHeading;
  MazeHeading to   = ffGetMove(sv_moveIndex);
  float deg        = sv_relativeTurnDeg(from, to);

  Serial.printf("[SV] Turn: %s → %s (%.0f°)\n",
                headingName(from), headingName(to), deg);

  if      (deg ==  -90.0f) turnCW90(baseTargetYaw - 90.0f);
  else if (deg ==   90.0f) turnACW90(baseTargetYaw + 90.0f);
  else if (fabsf(fabsf(deg) - 180.0f) < 1.0f) turn180(baseTargetYaw + 180.0f);
  // deg == 0 → straight, no turn needed

  delay(100);
  baseTargetYaw = wrapTurnAngleDegrees(baseTargetYaw + deg);
  targetYaw     = baseTargetYaw;
  sv_currentHeading = to;
  sv_resetAllPID();

  // Re-read walls now we're at cell centre facing the new direction
  current_lidars = readLidars();
}

// ============================================================
// DRIVING PID (solve phase — faster gains)
// ============================================================
static void sv_runDrivingPID(float dt, bool approaching) {
  noInterrupts();
  long cL = leftTicks, cR = rightTicks;
  interrupts();

  float vel_L = (float)(cL - prevLeftTicks);
  float vel_R = (float)(cR - prevRightTicks);

  float error_yaw   = wrapTurnAngleDegrees(targetYaw - current_yaw_angle);
  if (fabsf(error_yaw) <= 0.5f) { error_yaw = 0; prev_error_yaw = 0; }
  integral_yaw     += error_yaw * dt;
  float deriv_yaw   = (error_yaw - prev_error_yaw) / dt;
  float heading_corr = (SOLVE_KP_YAW * error_yaw) +
                       (SOLVE_KI_YAW * integral_yaw) +
                       (SOLVE_KD_YAW * deriv_yaw);
  prev_error_yaw    = error_yaw;

  // Use the approaching (slower) profile when a turn is one cell away
  float effVel = approaching ? SOLVE_TURN_APPROACH_VELOCITY : SOLVE_BASE_VELOCITY;
  int   effPWM = approaching ? SOLVE_TURN_APPROACH_PWM      : SOLVE_BASE_PWM;

  // Apply front brake (only relevant if approaching a wall)
  float brakeFactor = sv_frontBrakeScale();
  if (brakeFactor <= 0.0f) {
    applyMotorPWM(0, 0);
    final_pwm_L = 0; final_pwm_R = 0;
    integral_vel_L = 0; integral_vel_R = 0;
    return;
  }
  effVel *= brakeFactor;
  effPWM  = (int)(effPWM * brakeFactor);

  float tgt_L = effVel - heading_corr;
  float tgt_R = effVel + heading_corr;

>>>>>>> Stashed changes
  float err_L = tgt_L - vel_L;
  float err_R = tgt_R - vel_R;

  if (fabsf(err_L) <= 0.5f) { err_L = 0; prev_error_vel_L = 0; }
  if (fabsf(err_R) <= 0.5f) { err_R = 0; prev_error_vel_R = 0; }

<<<<<<< Updated upstream
  integral_vel_L += err_L * dt; 
  integral_vel_R += err_R * dt;
  
=======
  integral_vel_L += err_L * dt;
  integral_vel_R += err_R * dt;

>>>>>>> Stashed changes
  float d_L = (err_L - prev_error_vel_L) / dt;
  float d_R = (err_R - prev_error_vel_R) / dt;

  final_pwm_L = effPWM + (int)((SOLVE_KP_VEL * err_L) + (SOLVE_KI_VEL * integral_vel_L) + (SOLVE_KD_VEL * d_L));
  final_pwm_R = effPWM + (int)((SOLVE_KP_VEL * err_R) + (SOLVE_KI_VEL * integral_vel_R) + (SOLVE_KD_VEL * d_R));

<<<<<<< Updated upstream
  prev_error_vel_L = err_L; 
  prev_error_vel_R = err_R;
  
  applyMotorPWM(final_pwm_L, final_pwm_R);
}

static void sv_runWallPID(float dt) {
  bool hasLeft = (current_lidars.left < WALL_THRESHOLD);
  bool hasRight = (current_lidars.right < WALL_THRESHOLD);
  float error_wall = 0.0f;

  if (hasLeft && hasRight) { 
      Kp_wall = Kp_tunnel; 
      Kd_wall = Kd_tunnel; 
      error_wall = (float)(current_lidars.left - current_lidars.right); 
  } else if (hasLeft) { 
      Kp_wall = Kp_single; 
      Kd_wall = Kd_single; 
      error_wall = -(current_lidars.left - SINGLE_WALL_TARGET_MM); 
  } else if (hasRight) { 
      Kp_wall = Kp_single; 
      Kd_wall = Kd_single; 
      error_wall = (current_lidars.right - SINGLE_WALL_TARGET_MM); 
  } else { 
      correction_angle = 0.0f; 
      integral_wall = 0.0f; 
      prev_error_wall = 0.0f; 
      targetYaw = baseTargetYaw; 
      return; 
  }

  if (fabsf(error_wall) <= wall_tolerance) { 
      error_wall = 0.0f; 
      integral_wall = 0.0f; 
  }
  
  integral_wall += error_wall * dt;
  float deriv_wall = (error_wall - prev_error_wall) / dt;
  float target_corr = (Kp_wall * error_wall) + (Ki_wall * integral_wall) + (Kd_wall * deriv_wall);
  
  correction_angle = (correction_angle * 0.7f) + (target_corr * 0.3f);
  correction_angle = constrain(correction_angle, -8.0f, 8.0f);
  prev_error_wall = error_wall;
  targetYaw = baseTargetYaw + correction_angle;
}

// ============================================================
// MAIN SOLVER API
// ============================================================
=======
  prev_error_vel_L = err_L;
  prev_error_vel_R = err_R;

  applyMotorPWM(final_pwm_L, final_pwm_R);
}

// ============================================================
// WALL CENTERING PID (reused from survey, but called here too)
// ============================================================
static void sv_runWallPID(float dt) {
  bool hasLeft  = (current_lidars.left  < WALL_THRESHOLD);
  bool hasRight = (current_lidars.right < WALL_THRESHOLD);
  float error_wall = 0.0f;

  if (hasLeft && hasRight) {
    Kp_wall    = Kp_tunnel;
    Kd_wall    = Kd_tunnel;
    error_wall = (float)(current_lidars.left - current_lidars.right);
  } else if (hasLeft) {
    Kp_wall    = Kp_single;
    Kd_wall    = Kd_single;
    error_wall = -(current_lidars.left - SINGLE_WALL_TARGET_MM);
  } else if (hasRight) {
    Kp_wall    = Kp_single;
    Kd_wall    = Kd_single;
    error_wall = (current_lidars.right - SINGLE_WALL_TARGET_MM);
  } else {
    correction_angle = 0.0f; integral_wall = 0.0f; prev_error_wall = 0.0f;
    targetYaw        = baseTargetYaw;
    return;
  }

  if (fabsf(error_wall) <= wall_tolerance) { error_wall = 0.0f; integral_wall = 0.0f; }

  integral_wall      += error_wall * dt;
  float deriv_wall    = (error_wall - prev_error_wall) / dt;
  float target_corr   = (Kp_wall * error_wall) + (Ki_wall * integral_wall) + (Kd_wall * deriv_wall);

  correction_angle    = (correction_angle * 0.7f) + (target_corr * 0.3f);
  correction_angle    = constrain(correction_angle, -8.0f, 8.0f);
  prev_error_wall     = error_wall;
  targetYaw           = baseTargetYaw + correction_angle;
}

// ============================================================
// PUBLIC API
// ============================================================

// Call after ffComputePath() returns true.
// startHeading: the heading the robot is facing at the start
// of the solve run (typically HEADING_NORTH unless it returned
// to start and re-oriented).
>>>>>>> Stashed changes
void solverInit(MazeHeading startHeading) {
  sv_moveIndex      = 0;
  sv_pathLen        = ffGetPathLength();
  sv_currentHeading = startHeading;
<<<<<<< Updated upstream
  sv_cooldownStart  = 0;
  sv_resetAllPID();

  if (sv_pathLen > 1 && ffGetMove(1) != startHeading) {
      sv_state = SV_APPROACHING;
  } else {
      sv_state = SV_STRAIGHT;
  }
  Serial.printf("[SV] Solver initialised. Path length=%d. Heading=%s\n", sv_pathLen, headingName(startHeading));
}

inline bool solverDone() { 
    return sv_state == SV_GOAL_REACHED; 
}

void solverUpdate(float dt_motor, float dt_lidar, bool doMotor, bool doLidar) {
  if (sv_state == SV_GOAL_REACHED) { 
      applyMotorPWM(0, 0); 
      return; 
  }

  if (doLidar) {
    current_lidars = readLidars();
    // Keep Wall PID running during braking
    if (sv_state == SV_STRAIGHT || sv_state == SV_APPROACHING || sv_state == SV_BRAKING_TO_CENTER) {
        sv_runWallPID(dt_lidar);
    }
  }

  if (!doMotor) return;

  noInterrupts();
  long curL = leftTicks;
  long curR = rightTicks;
  interrupts();
  
  long dL = curL - prevLeftTicks;
  long dR = curR - prevRightTicks;
  
=======
  sv_state          = SV_STRAIGHT;
  sv_cooldownStart  = 0;

  sv_resetAllPID();

  Serial.printf("[SV] Solver initialised. Path length=%d. Heading=%s\n",
                sv_pathLen, headingName(startHeading));
}

// Returns true when the goal cell has been reached.
inline bool solverDone() { return sv_state == SV_GOAL_REACHED; }

// ============================================================
// MAIN SOLVER UPDATE — call every loop tick
//
// doMotor: true when the motor PID interval has elapsed
// doLidar: true when the lidar interval has elapsed
// ============================================================
void solverUpdate(float dt_motor, float dt_lidar, bool doMotor, bool doLidar) {

  if (sv_state == SV_GOAL_REACHED) {
    applyMotorPWM(0, 0);
    return;
  }

  // ── LIDAR ──
  if (doLidar) {
    current_lidars = readLidars();
    // Wall PID runs in STRAIGHT and APPROACHING states only
    if (sv_state == SV_STRAIGHT || sv_state == SV_APPROACHING) {
      sv_runWallPID(dt_lidar);
    }
  }

  // ── MOTOR ──
  if (!doMotor) return;

  // EKF update every motor tick
  noInterrupts();
  long curL = leftTicks, curR = rightTicks;
  interrupts();
  long dL = curL - prevLeftTicks, dR = curR - prevRightTicks;
  
  // Track continuous global 2D distance for robust alignment
  float move_ds = ((ekfTicksToMM(dL) + ekfTicksToMM(dR)) / 2.0f);
  total_distance_mm += move_ds;

>>>>>>> Stashed changes
  current_yaw_angle = readYawDegrees();
  ekfPredict(dL, dR);
  ekfUpdateYawDeg(current_yaw_angle);
  ekfTelemetry = ekfGetTelemetry();

  bool newCell = updateCellTracker();

<<<<<<< Updated upstream
  switch (sv_state) {
    case SV_STRAIGHT:
    case SV_APPROACHING: {
      if (newCell) {
        sv_moveIndex++;
        if (sv_moveIndex >= sv_pathLen) {
          applyMotorPWM(0, 0); 
          sv_state = SV_GOAL_REACHED;
          Serial.println("[SV] *** GOAL REACHED ***"); 
          break;
        }

        MazeHeading requiredHeading = ffGetMove(sv_moveIndex);
        if (requiredHeading != sv_currentHeading) {
          sv_state = SV_BRAKING_TO_CENTER;
          sv_pendingTurnDeg = sv_relativeTurnDeg(sv_currentHeading, requiredHeading);
          Serial.printf("[SV] Cell %d: Turn needed. BRAKING.\n", sv_moveIndex);
        } else {
          if (sv_moveIndex + 1 < sv_pathLen && ffGetMove(sv_moveIndex + 1) != sv_currentHeading) {
            sv_state = SV_APPROACHING; 
          } else {
            sv_state = SV_STRAIGHT;
          }
        }
      }
      
      bool isApproaching = (sv_state == SV_APPROACHING);
      sv_runDrivingPID(dt_motor, 1.0f, isApproaching, curL, curR);
      break;
    }

    case SV_BRAKING_TO_CENTER: {
      float distanceLeft = getEKFDistanceToCenter();

      // Tolerances Applied: 5.0f for math, pure emergency 25mm for Lidar
      if (distanceLeft <= 5.0f || current_lidars.front <= 25) {
        applyMotorPWM(0, 0);
        executeSolveTurn(sv_pendingTurnDeg);
        
        sv_currentHeading = ffGetMove(sv_moveIndex); 
        sv_cooldownStart = millis();
        sv_state = SV_COOLDOWN;
      } else {
        float totalBrakeDist = 80.0f - CELL_ENTRY_MARGIN; // 60.0f
        float slowDownFactor = distanceLeft / totalBrakeDist;
        
        // Anti-Stall Crawl Fix
        slowDownFactor = constrain(slowDownFactor, 0.35f, 1.0f);
        
        sv_runDrivingPID(dt_motor, slowDownFactor, true, curL, curR); 
      }
      break;
    }

    case SV_COOLDOWN: {
      if (millis() - sv_cooldownStart >= SOLVE_COOLDOWN_MS) {
        bool nextIsTurn = (sv_moveIndex + 1 < sv_pathLen) && (ffGetMove(sv_moveIndex + 1) != sv_currentHeading);
        sv_state = nextIsTurn ? SV_APPROACHING : SV_STRAIGHT;
      }
      sv_runDrivingPID(dt_motor, 1.0f, false, curL, curR);
      break;
    }
    
    default: 
      applyMotorPWM(0, 0); 
=======
  // ── STATE MACHINE ──
  switch (sv_state) {

    // ─────────────────────────────────────────
    case SV_STRAIGHT: {
      // Check if the NEXT move (after the one we're currently heading for)
      // requires a turn. If so, switch to APPROACHING one cell before it.
      bool turnNext = sv_nextMoveIsTurn();

      if (newCell) {
        // Record exact odometer reading at the boundary for centering
        cell_entry_distance_mm = total_distance_mm;
        
        // We just entered the next cell — advance move pointer
        sv_moveIndex++;
        Serial.printf("[SV] Cell entered. Move %d/%d.\n", sv_moveIndex, sv_pathLen);

        // Goal check
        if (sv_moveIndex >= sv_pathLen) {
          applyMotorPWM(0, 0);
          sv_state = SV_GOAL_REACHED;
          Serial.println("[SV] *** GOAL REACHED — SOLVE COMPLETE! ***");
          prevLeftTicks = curL; prevRightTicks = curR;
          return;
        }

        // Check if the move we just started requires a turn
        if (ffGetMove(sv_moveIndex) != sv_currentHeading) {
          // Turn needed — slow down and head to cell centre
          sv_state = SV_APPROACHING;
          Serial.printf("[SV] → Turn needed at this cell. Entering APPROACHING.\n");
        } else if (sv_nextMoveIsTurn()) {
          // No turn now, but next cell needs one — pre-emptively slow down
          sv_state = SV_APPROACHING;
          Serial.printf("[SV] → Turn coming next cell. Entering APPROACHING.\n");
        }
        // else stay in SV_STRAIGHT
      }

      sv_runDrivingPID(dt_motor, false);
      break;
    }

    // ─────────────────────────────────────────
    case SV_APPROACHING: {
      // Drive slowly and wait for cell centre, then turn if needed
      if (newCell) {
        cell_entry_distance_mm = total_distance_mm;
        sv_moveIndex++;

        if (sv_moveIndex >= sv_pathLen) {
          applyMotorPWM(0, 0);
          sv_state = SV_GOAL_REACHED;
          Serial.println("[SV] *** GOAL REACHED — SOLVE COMPLETE! ***");
          prevLeftTicks = curL; prevRightTicks = curR;
          return;
        }
      }

      // Check if we now need to turn (move index may have advanced)
      bool needsTurn = (ffGetMove(sv_moveIndex) != sv_currentHeading);

      if (needsTurn) {
        // Evaluate distance to the exact center using the 2D global tracker
        float distanceMovedInCell = total_distance_mm - cell_entry_distance_mm;
        float remaining = CELL_HALF_MM - distanceMovedInCell;

        if (remaining <= CENTRE_TOLERANCE_MM || current_lidars.front < SOLVE_FRONT_HALT_MM) {
            // Stop driving, align to centre, execute turn
            applyMotorPWM(0, 0);
            delay(80);

            sv_state = SV_TURNING;
            sv_alignToCentre();
            sv_executeTurn();

            sv_cooldownStart = millis();
            sv_state         = SV_COOLDOWN;
            prevLeftTicks    = curL;
            prevRightTicks   = curR;
            return;
        }
      }

      // No turn yet — keep approaching slowly
      sv_runDrivingPID(dt_motor, true);
      break;
    }

    // ─────────────────────────────────────────
    case SV_COOLDOWN: {
      if (millis() - sv_cooldownStart >= SOLVE_COOLDOWN_MS) {
        // Check if next move is also a turn (back to APPROACHING) or straight
        bool nextIsTurn = (sv_moveIndex + 1 < sv_pathLen) &&
                          (ffGetMove(sv_moveIndex + 1) != sv_currentHeading);
        sv_state = nextIsTurn ? SV_APPROACHING : SV_STRAIGHT;
        Serial.printf("[SV] Cooldown done → %s\n",
                      sv_state == SV_APPROACHING ? "APPROACHING" : "STRAIGHT");
      }
      // Drive straight during cooldown — no wall PID
      sv_runDrivingPID(dt_motor, false);
      break;
    }

    // ─────────────────────────────────────────
    case SV_GOAL_REACHED:
    case SV_IDLE:
    default:
      applyMotorPWM(0, 0);
>>>>>>> Stashed changes
      break;
  }

  prevLeftTicks  = curL;
  prevRightTicks = curR;
}

<<<<<<< Updated upstream
#endif
=======
#endif // SOLVER_H
>>>>>>> Stashed changes
