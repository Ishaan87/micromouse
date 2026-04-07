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
//     solverInit(startRow, startCol, startHeading);
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
  SV_STRAIGHT,      // driving between cells, no turn coming
  SV_APPROACHING,   // turn is next — slow down, aim for centre
  SV_TURNING,       // blocked: executing blocking turn
  SV_COOLDOWN,      // post-turn: straight drive, no wall PID
  SV_GOAL_REACHED
};

static SolverState sv_state       = SV_IDLE;
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
// (The ino must declare them before including Solver.h.)
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

// Wall PID parameters (shared with survey phase via Micromouse.ino)
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
static float sv_frontBrakeScale() {
  int d = current_lidars.front;
  if (d >= SOLVE_FRONT_STOP_MM) return 1.0f;
  if (d <= SOLVE_FRONT_HALT_MM) return 0.0f;
  return (float)(d - SOLVE_FRONT_HALT_MM) / (float)(SOLVE_FRONT_STOP_MM - SOLVE_FRONT_HALT_MM);
}

// Align to cell centre using encoder odometry (identical logic to survey phase)
static void sv_alignToCentre() {
  EKFState s = ekfGetState();
  float posInCell  = fmodf(fabsf(s.x_mm), CELL_SIZE_NAV_MM);
  float remaining  = CELL_HALF_MM - posInCell;

  if (remaining <= CENTRE_TOLERANCE_MM) {
    Serial.printf("[SV] Already centred (posInCell=%.1f).\n", posInCell);
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

  if      (deg ==  -90.0f) turnCW90();
  else if (deg ==   90.0f) turnACW90();
  else if (fabsf(fabsf(deg) - 180.0f) < 1.0f) turn180();
  // deg == 0 → straight, no turn needed

  delay(100);
  baseTargetYaw = readYawDegrees();
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

  float error_yaw   = targetYaw - current_yaw_angle;
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
void solverInit(MazeHeading startHeading) {
  sv_moveIndex      = 0;
  sv_pathLen        = ffGetPathLength();
  sv_currentHeading = startHeading;
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
  current_yaw_angle = readYawDegrees();
  ekfPredict(dL, dR);
  ekfUpdateYawDeg(current_yaw_angle);
  ekfTelemetry = ekfGetTelemetry();

  bool newCell = updateCellTracker();

  // ── STATE MACHINE ──
  switch (sv_state) {

    // ─────────────────────────────────────────
    case SV_STRAIGHT: {
      // Check if the NEXT move (after the one we're currently heading for)
      // requires a turn. If so, switch to APPROACHING one cell before it.
      bool turnNext = sv_nextMoveIsTurn();

      if (newCell) {
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
      break;
  }

  prevLeftTicks  = curL;
  prevRightTicks = curR;
}

#endif // SOLVER_H
