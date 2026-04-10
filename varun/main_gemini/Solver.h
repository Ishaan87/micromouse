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
// SOLVER — Speed-Run Phase
// ============================================================

// Straight-line speed (Full Speed)
const float SOLVE_BASE_VELOCITY  = 90.0f;   
const int   SOLVE_BASE_PWM       = 65;

// Approach speed (Medium Speed for the cell right before a turn)
const float SOLVE_TURN_APPROACH_VELOCITY = 45.0f;
const int   SOLVE_TURN_APPROACH_PWM      = 32;

// Brake window for speed-run (Physical failsafe)
const int   SOLVE_FRONT_STOP_MM  = 90;
const int   SOLVE_FRONT_HALT_MM  = 55;

// Cooldown after a turn before re-engaging wall PID
const unsigned long SOLVE_COOLDOWN_MS = 500;

// Velocity PID gains for solve phase (Aggressive)
const float SOLVE_KP_VEL  = 1.2f;
const float SOLVE_KI_VEL  = 0.05f;
const float SOLVE_KD_VEL  = 0.08f;

// Yaw PID gains for solve phase
const float SOLVE_KP_YAW  = 0.8f;
const float SOLVE_KI_YAW  = 0.01f;
const float SOLVE_KD_YAW  = 0.06f;

// ============================================================
// SOLVER STATE
// ============================================================
enum SolverState {
  SV_IDLE,
  SV_STRAIGHT,           // Driving full speed, no turns nearby
  SV_APPROACHING,        // Turn is in the NEXT cell — slow to medium speed
  SV_BRAKING_TO_CENTER,  // Turn is in THIS cell — smoothly brake to 0 at the 80mm mark
  SV_COOLDOWN,           // Post-turn straight drive (no wall PID)
  SV_GOAL_REACHED
};

static SolverState sv_state       = SV_IDLE;
static int         sv_moveIndex   = 0;    
static int         sv_pathLen     = 0;

static MazeHeading sv_currentHeading = HEADING_NORTH;
static unsigned long sv_cooldownStart = 0;
static float sv_pendingTurnDeg = 0.0f;

// External references from Micromouse.ino
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

extern float Kp_tunnel, Kd_tunnel;
extern float Kp_single, Kd_single;
extern float Kp_wall, Kd_wall, Ki_wall;
extern float wall_tolerance;

// ============================================================
// INTERNAL HELPERS
// ============================================================

static void sv_resetAllPID() {
  integral_vel_L   = 0; prev_error_vel_L = 0;
  integral_vel_R   = 0; prev_error_vel_R = 0;
  integral_yaw     = 0; prev_error_yaw   = 0;
  integral_wall    = 0; prev_error_wall  = 0;
  correction_angle = 0.0f;
  targetYaw        = baseTargetYaw;
}

static float sv_wrapAngleDegrees(float angle) {
  while (angle > 180.0f) angle -= 360.0f;
  while (angle < -180.0f) angle += 360.0f;
  return angle;
}

static float sv_relativeTurnDeg(MazeHeading from, MazeHeading to) {
  int delta = ((int)to - (int)from + 4) % 4;  
  if (delta == 0) return   0.0f;
  if (delta == 1) return -90.0f;   // CW
  if (delta == 2) return -180.0f;  // U-turn
  return  90.0f;                   // CCW
}

static float sv_frontBrakeScale() {
  int d = current_lidars.front;
  if (d >= SOLVE_FRONT_STOP_MM) return 1.0f;
  if (d <= SOLVE_FRONT_HALT_MM) return 0.0f;
  return (float)(d - SOLVE_FRONT_HALT_MM) / (float)(SOLVE_FRONT_STOP_MM - SOLVE_FRONT_HALT_MM);
}

// Executes the physical turn using your global yaw target system
static void executeSolveTurn(float turnDeltaDeg) {
  float nextBaseTargetYaw = sv_wrapAngleDegrees(baseTargetYaw + turnDeltaDeg);

  if (turnDeltaDeg == -90.0f) turnCW90(nextBaseTargetYaw);
  else if (turnDeltaDeg == 90.0f) turnACW90(nextBaseTargetYaw);
  else if (fabsf(turnDeltaDeg) == 180.0f) turn180(nextBaseTargetYaw);

  delay(100);
  baseTargetYaw = nextBaseTargetYaw;
  targetYaw     = baseTargetYaw;
  sv_resetAllPID();
}

// ============================================================
// DRIVING PID (Synchronized Ticks & EKF Braking)
// ============================================================
static void sv_runDrivingPID(float dt, float algorithmScale, bool isApproaching, long cL, long cR) {
  float vel_L = (float)(cL - prevLeftTicks);
  float vel_R = (float)(cR - prevRightTicks);

  float error_yaw   = targetYaw - current_yaw_angle;
  error_yaw = sv_wrapAngleDegrees(error_yaw);
  if (fabsf(error_yaw) <= 0.5f) { error_yaw = 0; prev_error_yaw = 0; }
  
  integral_yaw     += error_yaw * dt;
  float deriv_yaw   = (error_yaw - prev_error_yaw) / dt;
  float heading_corr = (SOLVE_KP_YAW * error_yaw) + (SOLVE_KI_YAW * integral_yaw) + (SOLVE_KD_YAW * deriv_yaw);
  prev_error_yaw    = error_yaw;

  // Base speeds depend on whether we are approaching a turn cell
  float effVel = isApproaching ? SOLVE_TURN_APPROACH_VELOCITY : SOLVE_BASE_VELOCITY;
  int   effPWM = isApproaching ? SOLVE_TURN_APPROACH_PWM      : SOLVE_BASE_PWM;

  float lidarBrake = sv_frontBrakeScale();
  float finalScale = min(lidarBrake, algorithmScale);

  if (finalScale <= 0.0f) {
    applyMotorPWM(0, 0);
    final_pwm_L = 0; final_pwm_R = 0;
    integral_vel_L = 0; integral_vel_R = 0;
    prev_error_vel_L = 0; prev_error_vel_R = 0;
    return;
  }

  effVel *= finalScale;
  effPWM  = (int)(effPWM * finalScale);

  float tgt_L = effVel - heading_corr;
  float tgt_R = effVel + heading_corr;

  float err_L = tgt_L - vel_L;
  float err_R = tgt_R - vel_R;

  if (fabsf(err_L) <= 0.5f) { err_L = 0; prev_error_vel_L = 0; }
  if (fabsf(err_R) <= 0.5f) { err_R = 0; prev_error_vel_R = 0; }

  integral_vel_L += err_L * dt;
  integral_vel_R += err_R * dt;

  float d_L = (err_L - prev_error_vel_L) / dt;
  float d_R = (err_R - prev_error_vel_R) / dt;

  final_pwm_L = effPWM + (int)((SOLVE_KP_VEL * err_L) + (SOLVE_KI_VEL * integral_vel_L) + (SOLVE_KD_VEL * d_L));
  final_pwm_R = effPWM + (int)((SOLVE_KP_VEL * err_R) + (SOLVE_KI_VEL * integral_vel_R) + (SOLVE_KD_VEL * d_R));

  prev_error_vel_L = err_L;
  prev_error_vel_R = err_R;

  applyMotorPWM(final_pwm_L, final_pwm_R);
}

// ============================================================
// WALL CENTERING PID
// ============================================================
static void sv_runWallPID(float dt) {
  bool hasLeft  = (current_lidars.left  < WALL_THRESHOLD);
  bool hasRight = (current_lidars.right < WALL_THRESHOLD);
  float error_wall = 0.0f;

  if (hasLeft && hasRight) {
    Kp_wall = Kp_tunnel; Kd_wall = Kd_tunnel;
    error_wall = (float)(current_lidars.left - current_lidars.right);
  } else if (hasLeft) {
    Kp_wall = Kp_single; Kd_wall = Kd_single;
    error_wall = -(current_lidars.left - SINGLE_WALL_TARGET_MM);
  } else if (hasRight) {
    Kp_wall = Kp_single; Kd_wall = Kd_single;
    error_wall = (current_lidars.right - SINGLE_WALL_TARGET_MM);
  } else {
    correction_angle = 0.0f; integral_wall = 0.0f; prev_error_wall = 0.0f;
    targetYaw = baseTargetYaw;
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

void solverInit(MazeHeading startHeading) {
  sv_moveIndex      = 0;
  sv_pathLen        = ffGetPathLength();
  sv_currentHeading = startHeading;
  sv_cooldownStart  = 0;
  sv_resetAllPID();

  // If the very first cell after start requires a turn, start in APPROACHING
  if (sv_pathLen > 1 && ffGetMove(1) != startHeading) {
    sv_state = SV_APPROACHING;
  } else {
    sv_state = SV_STRAIGHT;
  }

  Serial.printf("[SV] Solver initialised. Path length=%d. Heading=%s\n", sv_pathLen, headingName(startHeading));
}

inline bool solverDone() { return sv_state == SV_GOAL_REACHED; }

// ============================================================
// MAIN SOLVER UPDATE
// ============================================================
void solverUpdate(float dt_motor, float dt_lidar, bool doMotor, bool doLidar) {

  if (sv_state == SV_GOAL_REACHED) {
    applyMotorPWM(0, 0);
    return;
  }

  // ── LIDAR ──
  if (doLidar) {
    current_lidars = readLidars();
    if (sv_state == SV_STRAIGHT || sv_state == SV_APPROACHING) {
      sv_runWallPID(dt_lidar);
    }
  }

  // ── MOTOR ──
  if (!doMotor) return;

  // Stale Ticks FIX: Read exactly once here
  noInterrupts();
  long curL = leftTicks, curR = rightTicks;
  interrupts();
  
  long dL = curL - prevLeftTicks, dR = curR - prevRightTicks;
  current_yaw_angle = readYawDegrees();
  ekfPredict(dL, dR);
  ekfUpdateYawDeg(current_yaw_angle);
  ekfTelemetry = ekfGetTelemetry();

  bool newCell = updateCellTracker();

  // ── STATE MACHINE ──
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
          // We must turn at the center of THIS cell.
          sv_state = SV_BRAKING_TO_CENTER;
          sv_pendingTurnDeg = sv_relativeTurnDeg(sv_currentHeading, requiredHeading);
          Serial.printf("[SV] Cell %d: Turn needed. BRAKING.\n", sv_moveIndex);
        } else {
          // We are going straight through this cell. 
          // Do we need to turn in the NEXT cell?
          if (sv_moveIndex + 1 < sv_pathLen && ffGetMove(sv_moveIndex + 1) != sv_currentHeading) {
            sv_state = SV_APPROACHING; 
            Serial.printf("[SV] Cell %d: Turn coming up. APPROACHING.\n", sv_moveIndex);
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
      EKFState s = ekfGetState();
      float halfCell = 160.0f / 2.0f; // 80.0f
      float posInCellX = fmodf(fabsf(s.x_mm), 160.0f); 
      float distanceLeft = halfCell - posInCellX;

      if (distanceLeft <= 0.0f || current_lidars.front < SOLVE_FRONT_HALT_MM) {
        // Hit dead center! Stop and Turn.
        applyMotorPWM(0, 0);
        executeSolveTurn(sv_pendingTurnDeg);
        
        // Update logical tracker
        sv_currentHeading = ffGetMove(sv_moveIndex); 

        sv_cooldownStart = millis();
        sv_state = SV_COOLDOWN;
      } else {
        // Smoothly decelerate over the remaining distance
        float totalBrakeDist = halfCell - CELL_ENTRY_MARGIN;
        float slowDownFactor = max(0.0f, distanceLeft / totalBrakeDist);
        
        // Pass `true` for approaching so we brake smoothly from 45, not 90
        sv_runDrivingPID(dt_motor, slowDownFactor, true, curL, curR); 
      }
      break;
    }

    case SV_COOLDOWN: {
      if (millis() - sv_cooldownStart >= SOLVE_COOLDOWN_MS) {
        // Cooldown over. Check if next move is a turn.
        bool nextIsTurn = (sv_moveIndex + 1 < sv_pathLen) && (ffGetMove(sv_moveIndex + 1) != sv_currentHeading);
        sv_state = nextIsTurn ? SV_APPROACHING : SV_STRAIGHT;
      }
      // Drive straight, no wall PID
      sv_runDrivingPID(dt_motor, 1.0f, false, curL, curR);
      break;
    }

    case SV_GOAL_REACHED:
    case SV_IDLE:
    default:
      applyMotorPWM(0, 0);
      break;
  }

  // Stale Ticks FIX: Update strictly at the end
  prevLeftTicks  = curL;
  prevRightTicks = curR;
}

#endif // SOLVER_H