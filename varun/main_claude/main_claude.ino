// ============================================================
// Micromouse.ino
//
// STRATEGY  (mirrors iteration-1_algorithm.py)
// ─────────────────────────────────────────────
//  Run 1  — Floodfill-driven exploration
//           Phase A: start → goal  (green cells)
//           Phase B: goal  → start (orange cells)
//           Records walls at every cell via CellTracker + WallMap.
//
//  Post-Run1 — Build the full flood-fill distance table
//              (BFS from goal, wall-aware, stored in flood_dist[][]).
//
//  Run 2+  — A* speed run using flood_dist as a perfect heuristic.
//             Live wall sensing + replanning if a new wall blocks
//             the planned path.
//
// TURN ABSTRACTION
// ─────────────────
//  The algorithm plans in *absolute* cardinal directions (N/E/S/W).
//  moveToAbsDir() works out how many quarter-turns are needed, calls
//  turnCW90() / turnACW90() / turn180() from Turns.h, then drives
//  forward one cell via the PID loop.
//
// STATE MACHINE
// ──────────────
//  BOT_IDLE          – waiting to start (motors off)
//  BOT_MOVING        – driving forward one cell
//  BOT_TURN_CW       – executing a clockwise  90° turn
//  BOT_TURN_ACW      – executing a CCW        90° turn
//  BOT_TURN_180      – executing a 180° turn
//  BOT_TURN_COOLDOWN – brief pause after a turn before next move
//  BOT_DONE          – goal reached / run complete
// ============================================================

#include <Arduino.h>
#include <WiFi.h>
#include <WebServer.h>
#include <WebSocketsServer.h>
#include <stdarg.h>
#include "Config.h"
#include "Motors.h"
#include "Encoders.h"
#include "Sensors.h"
#include "ekf.h"
#include "CellTracker.h"
#include "WallMap.h"
#include "Turns.h"

// ─── Wi-Fi / Web dashboard ────────────────────────────────────
const char *ssid     = "Jerry";
const char *password = "mouse1234";

WebServer        server(80);
WebSocketsServer webSocket(81);

const char index_html[] PROGMEM = R"rawliteral(
<!DOCTYPE html><html lang="en"><head>
<meta charset="UTF-8">
<meta name="viewport" content="width=device-width, initial-scale=1.0">
<title>Micromouse</title>
<style>body{margin:0;padding:12px;background:#000;color:#fff;font-family:Consolas,monospace}
pre{margin:0;white-space:pre-wrap;word-break:break-word;font-size:14px;line-height:1.35}</style>
</head><body><pre id="log"></pre>
<script>
var ws;
const MAX=300;
window.addEventListener('load',()=>{connect()});
function connect(){ws=new WebSocket(`ws://${location.hostname}:81/`);
ws.onmessage=e=>{const log=document.getElementById('log');
const lines=log.textContent?log.textContent.split('\n'):[];
lines.push(e.data.startsWith('MAZE:')?e.data.substring(5).trim():e.data);
while(lines.length>MAX)lines.shift();
log.textContent=lines.join('\n');log.parentElement.scrollTop=1e9;};
ws.onclose=()=>setTimeout(connect,2000);}
</script></body></html>
)rawliteral";

// ─── Goal cell (6×6 maze: centre = row 3, col 3 for 0-based) ─
// For a 6×6 maze, the "centre" is a 2×2 block at (2,2),(2,3),(3,2),(3,3)
// Adjust GOAL_CELLS_COUNT and the array below to match your maze.
const int GOAL_CELLS_COUNT = 1;        // single goal for a 6×6 test maze
struct Cell { int row; int col; };
const Cell GOAL_CELLS[GOAL_CELLS_COUNT] = { {5, 5} };  // ← change as needed

// ─── Timing ───────────────────────────────────────────────────
const int LOOP_INTERVAL_MS   = 20;
const int LIDAR_INTERVAL_MS  = 50;
const int PRINT_INTERVAL_MS  = 200;
const unsigned long TURN_COOLDOWN_MS = 900;

unsigned long lastLoopTime  = 0;
unsigned long lastLidarTime = 0;
unsigned long lastPrintTime = 0;

// ─── Shared sensor state ──────────────────────────────────────
float integral_vel_L = 0, integral_vel_R = 0;
float prev_error_vel_L = 0, prev_error_vel_R = 0;
float integral_yaw = 0,    prev_error_yaw  = 0;
float current_yaw_angle = 0.0f;
float integral_wall = 0,   prev_error_wall = 0;

long prevLeftTicks = 0, prevRightTicks = 0;
int  final_pwm_L = 0,   final_pwm_R   = 0;

DistanceData current_lidars;
EKFTelemetry ekfTelemetry = {0.0f, 0.0f, 0.0f};

// ─── Log ring buffer ──────────────────────────────────────────
const int LOG_HISTORY_SIZE = 300;
String logHistory[LOG_HISTORY_SIZE];
int    logHistoryStart = 0;
int    logHistoryCount = 0;

// ─── Algorithm state ─────────────────────────────────────────
//  Absolute heading tracked by the algorithm (separate from EKF,
//  updated by every quarter-turn command so the planner always
//  knows which way the robot is physically facing).
MazeHeading robotFacing = HEADING_NORTH;

int runNumber = 0;  // 1 = explore, 2+ = A* speed run

// ─── Flood-fill distance table ────────────────────────────────
//  flood_dist[row][col] = wall-aware BFS distance to nearest goal.
//  Initialised to INF; populated once after Run 1.
const int INF_DIST = 32767;
int flood_dist[MAZE_SIZE][MAZE_SIZE];

// ─── A* path storage ──────────────────────────────────────────
//  We plan up to MAZE_SIZE*MAZE_SIZE steps; store as a sequence
//  of (row,col) pairs from next-cell to goal (inclusive).
const int MAX_PATH = MAZE_SIZE * MAZE_SIZE;
struct PathCell { int row; int col; };
PathCell plannedPath[MAX_PATH];
int      pathLen     = 0;
int      pathIdx     = 0;   // next step to execute

// ─── High-level state machine ────────────────────────────────
enum RunPhase {
  PHASE_EXPLORE_TO_GOAL,  // Run 1, leg A
  PHASE_EXPLORE_TO_START, // Run 1, leg B
  PHASE_ASTAR,            // Run 2+
};

enum BotState {
  BOT_IDLE,
  BOT_MOVING,
  BOT_TURN_COOLDOWN,
  BOT_DONE,
};

RunPhase currentPhase = PHASE_EXPLORE_TO_GOAL;
BotState botState     = BOT_IDLE;

unsigned long cooldownStartMs = 0;

// Pending turn direction (set before entering BOT_TURN_COOLDOWN)
// 0=straight, +1=CW90, -1=ACW90, 2=180
int pendingTurnType = 0;

// Next absolute direction the algorithm wants to move after the turn
MazeHeading pendingMoveDir = HEADING_NORTH;
bool hasPendingMove = false;

// Whether we've sensed+recorded walls for the current cell yet
bool wallsSensedThisCell = false;

// ─── Live flood table (rebuilt during exploration) ────────────
int live_flood[MAZE_SIZE][MAZE_SIZE];

// ─── Forward declarations ─────────────────────────────────────
void logLine(const String &s);
void logPrintf(const char *fmt, ...);
void appendLogHistory(const String &s);
void replayLogHistory(uint8_t num);
void resetPIDIntegrals();
void resetWallPID();
void runDrivingPID(float dt);
void runWallPIDLoop(float dt);
float frontBrakeScale();
float wrapAngleDegrees(float a);
void buildLiveFlood(const Cell *targets, int nTargets, int dist[MAZE_SIZE][MAZE_SIZE]);
void buildFloodFill();
bool planAstar(int startRow, int startCol);
void senseAndRecordWalls(int row, int col, MazeHeading facing);
bool isGoalCell(int row, int col);
bool isStartCell(int row, int col);
MazeHeading chooseBestMove(int row, int col, int dist[MAZE_SIZE][MAZE_SIZE]);
int  turnTypeNeeded(MazeHeading from, MazeHeading to);
void executeTurn(int turnType);
void issueMove(MazeHeading dir);
void printTelemetry();
String buildMazeWebReport();
void broadcastMazeWebReport();
void stepExplore();
void stepAstar();
void tickStateMachine(float dt);

// ============================================================
// LOGGING
// ============================================================
void appendLogHistory(const String &line) {
  int idx = (logHistoryStart + logHistoryCount) % LOG_HISTORY_SIZE;
  logHistory[idx] = line;
  if (logHistoryCount < LOG_HISTORY_SIZE) logHistoryCount++;
  else logHistoryStart = (logHistoryStart + 1) % LOG_HISTORY_SIZE;
}

void replayLogHistory(uint8_t num) {
  for (int i = 0; i < logHistoryCount; i++)
    webSocket.sendTXT(num, logHistory[(logHistoryStart + i) % LOG_HISTORY_SIZE]);
}

void logLine(const String &line) {
  appendLogHistory(line);
  Serial.println(line);
  webSocket.broadcastTXT(line);
}

void logPrintf(const char *fmt, ...) {
  char buf[384];
  va_list args;
  va_start(args, fmt);
  vsnprintf(buf, sizeof(buf), fmt, args);
  va_end(args);
  logLine(String(buf));
}

// ============================================================
// PID HELPERS  (identical to original)
// ============================================================
void resetPIDIntegrals() {
  integral_vel_L = 0; prev_error_vel_L = 0;
  integral_vel_R = 0; prev_error_vel_R = 0;
  integral_yaw   = 0; prev_error_yaw   = 0;
  integral_wall  = 0; prev_error_wall  = 0;
}

void resetWallPID() {
  integral_wall    = 0;
  prev_error_wall  = 0;
  correction_angle = 0.0f;
  targetYaw        = baseTargetYaw;
}

float wrapAngleDegrees(float a) {
  while (a >  180.0f) a -= 360.0f;
  while (a < -180.0f) a += 360.0f;
  return a;
}

float frontBrakeScale() {
  int d = current_lidars.front;
  if (d >= FRONT_STOP_MM) return 1.0f;
  if (d <= FRONT_HALT_MM) return 0.0f;
  return (float)(d - FRONT_HALT_MM) / (float)(FRONT_STOP_MM - FRONT_HALT_MM);
}

void runDrivingPID(float dt) {
  noInterrupts();
  long cL = leftTicks, cR = rightTicks;
  interrupts();

  float vel_L = (float)(cL - prevLeftTicks);
  float vel_R = (float)(cR - prevRightTicks);

  float error_yaw = targetYaw - current_yaw_angle;
  if (fabsf(error_yaw) <= yaw_tolerance) { error_yaw = 0; prev_error_yaw = 0; }
  integral_yaw += error_yaw * dt;
  float deriv_yaw   = (error_yaw - prev_error_yaw) / dt;
  float heading_corr = Kp_yaw * error_yaw + Ki_yaw * integral_yaw + Kd_yaw * deriv_yaw;
  prev_error_yaw = error_yaw;

  float brakeFactor = frontBrakeScale();
  if (brakeFactor <= 0.0f) {
    applyMotorPWM(0, 0);
    final_pwm_L = final_pwm_R = 0;
    integral_vel_L = integral_vel_R = 0;
    prev_error_vel_L = prev_error_vel_R = 0;
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

  integral_vel_L += err_L * dt;
  integral_vel_R += err_R * dt;
  float d_L = (err_L - prev_error_vel_L) / dt;
  float d_R = (err_R - prev_error_vel_R) / dt;

  final_pwm_L = effPWM + (int)(Kp_vel_L * err_L + Ki_vel_L * integral_vel_L + Kd_vel_L * d_L);
  final_pwm_R = effPWM + (int)(Kp_vel_R * err_R + Ki_vel_R * integral_vel_R + Kd_vel_R * d_R);

  prev_error_vel_L = err_L;
  prev_error_vel_R = err_R;

  applyMotorPWM(final_pwm_L, final_pwm_R);
}

void runWallPIDLoop(float dt) {
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
  integral_wall   += error_wall * dt;
  float deriv_wall = (error_wall - prev_error_wall) / dt;
  float corr = Kp_wall * error_wall + Ki_wall * integral_wall + Kd_wall * deriv_wall;
  correction_angle = correction_angle * 0.7f + corr * 0.3f;
  correction_angle = constrain(correction_angle, -8.0f, 8.0f);
  prev_error_wall  = error_wall;
  targetYaw = baseTargetYaw + correction_angle;
}

// ============================================================
// WALL SENSING HELPER
// ============================================================
// Returns true if any new wall was discovered.
bool senseAndRecordWallsInternal(int row, int col, MazeHeading facing) {
  bool anyNew = false;
  // Map relative directions to absolute using WallMap's getWallMapping.
  // We re-read lidar here for freshness.
  current_lidars = readLidars();
  delay(5);

  WallMapping m = getWallMapping(row, col, facing);
  bool frontWall = (current_lidars.front < WALL_DETECT_MM);
  bool leftWall  = (current_lidars.left  < WALL_DETECT_MM);
  bool rightWall = (current_lidars.right < WALL_DETECT_MM);

  auto markWall = [&](bool* wallPtr, bool present) {
    if (present && !(*wallPtr)) { *wallPtr = true; anyNew = true; }
  };
  markWall(m.frontWall, frontWall);
  markWall(m.leftWall,  leftWall);
  markWall(m.rightWall, rightWall);

  wallMap[row][col].visited = true;
  syncNeighbourWalls(row, col);
  return anyNew;
}

// ============================================================
// GOAL / START PREDICATES
// ============================================================
bool isGoalCell(int row, int col) {
  for (int i = 0; i < GOAL_CELLS_COUNT; i++)
    if (GOAL_CELLS[i].row == row && GOAL_CELLS[i].col == col) return true;
  return false;
}

bool isStartCell(int row, int col) {
  return (row == 0 && col == 0);
}

// ============================================================
// FLOOD-FILL  (BFS from targets, wall-aware)
// ============================================================
void buildFloodFromTargets(const Cell *targets, int nTargets,
                           int dist[MAZE_SIZE][MAZE_SIZE]) {
  // Initialise
  for (int r = 0; r < MAZE_SIZE; r++)
    for (int c = 0; c < MAZE_SIZE; c++)
      dist[r][c] = INF_DIST;

  // Simple queue using a static ring buffer
  static Cell q[MAZE_SIZE * MAZE_SIZE];
  int head = 0, tail = 0;

  for (int i = 0; i < nTargets; i++) {
    int r = targets[i].row, c = targets[i].col;
    dist[r][c] = 0;
    q[tail++]  = {r, c};
  }

  while (head != tail) {
    Cell cur = q[head++];
    int  r = cur.row, c = cur.col;
    int  nd = dist[r][c] + 1;

    // Neighbours: N(r+1,c), S(r-1,c), E(r,c+1), W(r,c-1)
    // North
    if (!wallMap[r][c].north && r + 1 < MAZE_SIZE && dist[r+1][c] == INF_DIST) {
      dist[r+1][c] = nd; q[tail++] = {r+1, c};
    }
    // South
    if (!wallMap[r][c].south && r - 1 >= 0 && dist[r-1][c] == INF_DIST) {
      dist[r-1][c] = nd; q[tail++] = {r-1, c};
    }
    // East
    if (!wallMap[r][c].east && c + 1 < MAZE_SIZE && dist[r][c+1] == INF_DIST) {
      dist[r][c+1] = nd; q[tail++] = {r, c+1};
    }
    // West
    if (!wallMap[r][c].west && c - 1 >= 0 && dist[r][c-1] == INF_DIST) {
      dist[r][c-1] = nd; q[tail++] = {r, c-1};
    }
  }
}

// Rebuild the live flood table toward the current exploration targets.
void rebuildLiveFlood(bool toGoal) {
  if (toGoal) {
    buildFloodFromTargets(GOAL_CELLS, GOAL_CELLS_COUNT, live_flood);
  } else {
    static const Cell startCell = {0, 0};
    buildFloodFromTargets(&startCell, 1, live_flood);
  }
}

// Called once after Run 1 to build the permanent heuristic table.
void buildFloodFill() {
  logLine("[FF] Building wall-aware flood-fill distance table...");
  buildFloodFromTargets(GOAL_CELLS, GOAL_CELLS_COUNT, flood_dist);
  logPrintf("[FF] Done. Start→goal distance = %d", flood_dist[0][0]);
}

// ============================================================
// CHOOSE BEST MOVE  (lowest flood value among open neighbours)
// ============================================================
// Returns the heading to the best open neighbour, or HEADING_NORTH
// with *found=false if stuck.
MazeHeading chooseBestMove(int row, int col,
                           int dist[MAZE_SIZE][MAZE_SIZE],
                           bool *found) {
  int best = INF_DIST;
  MazeHeading bestDir = HEADING_NORTH;
  *found = false;

  // North
  if (!wallMap[row][col].north && row + 1 < MAZE_SIZE) {
    int v = dist[row+1][col];
    if (v < best) { best = v; bestDir = HEADING_NORTH; *found = true; }
  }
  // South
  if (!wallMap[row][col].south && row - 1 >= 0) {
    int v = dist[row-1][col];
    if (v < best) { best = v; bestDir = HEADING_SOUTH; *found = true; }
  }
  // East
  if (!wallMap[row][col].east && col + 1 < MAZE_SIZE) {
    int v = dist[row][col+1];
    if (v < best) { best = v; bestDir = HEADING_EAST; *found = true; }
  }
  // West
  if (!wallMap[row][col].west && col - 1 >= 0) {
    int v = dist[row][col-1];
    if (v < best) { best = v; bestDir = HEADING_WEST; *found = true; }
  }
  return bestDir;
}

// ============================================================
// A*  (flood_dist as heuristic)
// ============================================================
// Returns true on success; fills plannedPath[0..pathLen-1].
bool planAstar(int startRow, int startCol) {
  // g[r][c], f for heap, came_from
  static int   g_score[MAZE_SIZE][MAZE_SIZE];
  static int   came_fromR[MAZE_SIZE][MAZE_SIZE];
  static int   came_fromC[MAZE_SIZE][MAZE_SIZE];
  static bool  closed[MAZE_SIZE][MAZE_SIZE];

  for (int r = 0; r < MAZE_SIZE; r++)
    for (int c = 0; c < MAZE_SIZE; c++) {
      g_score[r][c]   = INF_DIST;
      came_fromR[r][c] = -1;
      came_fromC[r][c] = -1;
      closed[r][c]    = false;
    }

  // Simple min-heap via linear scan (maze is small, <= 36 cells)
  // We store entries as (f, r, c)
  struct HeapEntry { int f, g, r, c; };
  static HeapEntry heap[MAZE_SIZE * MAZE_SIZE * 4];
  int heapSize = 0;

  auto push = [&](int f, int g, int r, int c) {
    heap[heapSize++] = {f, g, r, c};
  };
  auto pop = [&]() -> HeapEntry {
    int best = 0;
    for (int i = 1; i < heapSize; i++)
      if (heap[i].f < heap[best].f ||
          (heap[i].f == heap[best].f && heap[i].g > heap[best].g))
        best = i;
    HeapEntry e = heap[best];
    heap[best] = heap[--heapSize];
    return e;
  };

  g_score[startRow][startCol] = 0;
  push(flood_dist[startRow][startCol], 0, startRow, startCol);

  while (heapSize > 0) {
    HeapEntry cur = pop();
    int r = cur.r, c = cur.c, g = cur.g;

    if (closed[r][c]) continue;
    closed[r][c] = true;

    if (isGoalCell(r, c)) {
      // Reconstruct path (excludes start, includes goal)
      pathLen = 0;
      int pr = r, pc = c;
      while (came_fromR[pr][pc] != -1 || pr != startRow || pc != startCol) {
        if (came_fromR[pr][pc] == -1) break;  // reached start
        plannedPath[pathLen++] = {pr, pc};
        int nr = came_fromR[pr][pc];
        int nc = came_fromC[pr][pc];
        pr = nr; pc = nc;
      }
      // Reverse
      for (int i = 0, j = pathLen - 1; i < j; i++, j--) {
        PathCell tmp = plannedPath[i];
        plannedPath[i] = plannedPath[j];
        plannedPath[j] = tmp;
      }
      pathIdx = 0;
      logPrintf("[A*] Path found: %d steps", pathLen);
      return true;
    }

    // Expand neighbours
    int dr[] = {1, -1, 0, 0};
    int dc[] = {0, 0, 1, -1};
    bool walls_arr[] = {wallMap[r][c].north, wallMap[r][c].south,
                        wallMap[r][c].east,  wallMap[r][c].west};
    for (int d = 0; d < 4; d++) {
      if (walls_arr[d]) continue;
      int nr = r + dr[d], nc = c + dc[d];
      if (nr < 0 || nr >= MAZE_SIZE || nc < 0 || nc >= MAZE_SIZE) continue;
      if (closed[nr][nc]) continue;
      int ng = g + 1;
      if (ng < g_score[nr][nc]) {
        g_score[nr][nc]   = ng;
        came_fromR[nr][nc] = r;
        came_fromC[nr][nc] = c;
        push(ng + flood_dist[nr][nc], ng, nr, nc);
      }
    }
  }

  logLine("[A*] No path found!");
  return false;
}

// ============================================================
// TURN HELPERS
// ============================================================
// Returns: 0=none, 1=CW90, -1=ACW90, 2=180
int turnTypeNeeded(MazeHeading from, MazeHeading to) {
  int diff = ((int)to - (int)from + 4) % 4;
  if (diff == 0) return 0;
  if (diff == 1) return 1;   // CW
  if (diff == 3) return -1;  // ACW
  return 2;                   // 180
}

// Update robotFacing and baseTargetYaw, then call Turns.h function.
void executeTurn(int turnType) {
  if (turnType == 0) return;
  if (turnType == 1) {
    turnCW90();
    baseTargetYaw = wrapAngleDegrees(baseTargetYaw - 90.0f);
    robotFacing   = (MazeHeading)(((int)robotFacing + 1) % 4);
  } else if (turnType == -1) {
    turnACW90();
    baseTargetYaw = wrapAngleDegrees(baseTargetYaw + 90.0f);
    robotFacing   = (MazeHeading)(((int)robotFacing + 3) % 4);
  } else {
    turn180();
    baseTargetYaw = wrapAngleDegrees(baseTargetYaw + 180.0f);
    robotFacing   = (MazeHeading)(((int)robotFacing + 2) % 4);
  }
  targetYaw = baseTargetYaw;
  resetPIDIntegrals();
  resetWallPID();
}

// ============================================================
// ISSUE A MOVE IN AN ABSOLUTE DIRECTION
// Turns the robot to face `dir` then begins the BOT_MOVING phase.
// ============================================================
void issueMove(MazeHeading dir) {
  int tt = turnTypeNeeded(robotFacing, dir);
  if (tt != 0) {
    executeTurn(tt);
    delay(100);  // brief settle after turn
    current_lidars = readLidars();
    delay(20);
  }
  // Now facing `dir` — start driving forward
  botState         = BOT_MOVING;
  wallsSensedThisCell = false;
}

// ============================================================
// CHECK IF A NEWLY DISCOVERED WALL BLOCKS THE PLANNED PATH
// ============================================================
bool newWallBlocksPath(int row, int col) {
  // Look at every wall direction; if a blocked neighbour is on the path
  bool walls_arr[] = {wallMap[row][col].north, wallMap[row][col].south,
                      wallMap[row][col].east,  wallMap[row][col].west};
  int dr[] = {1, -1, 0, 0};
  int dc[] = {0, 0, 1, -1};
  for (int d = 0; d < 4; d++) {
    if (!walls_arr[d]) continue;
    int nr = row + dr[d], nc = col + dc[d];
    for (int i = pathIdx; i < pathLen; i++) {
      if (plannedPath[i].row == nr && plannedPath[i].col == nc) return true;
    }
  }
  return false;
}

// ============================================================
// EXPLORATION STEP  (Run 1)
// Called once per cell entry.
// ============================================================
void stepExplore() {
  int row     = ctGetRow();
  int col     = ctGetCol();
  bool toGoal = (currentPhase == PHASE_EXPLORE_TO_GOAL);

  // Check termination
  if (toGoal && isGoalCell(row, col)) {
    logLine("[EXPLORE] Goal reached. Turning back...");
    currentPhase = PHASE_EXPLORE_TO_START;
    rebuildLiveFlood(false);  // re-flood toward start
    return;
  }
  if (!toGoal && isStartCell(row, col)) {
    logLine("[EXPLORE] Returned to start. Exploration complete.");
    applyMotorPWM(0, 0);
    botState = BOT_DONE;
    logLine("[EXPLORE] Building permanent flood-fill table...");
    buildFloodFill();
    logLine("[EXPLORE] Ready. Starting A* speed run.");
    broadcastMazeWebReport();
    // Immediately queue Run 2
    runNumber = 2;
    currentPhase = PHASE_ASTAR;
    bool ok = planAstar(ctGetRow(), ctGetCol());
    if (!ok) { logLine("[A*] No path — halting."); return; }
    // Issue first move
    PathCell next = plannedPath[pathIdx];
    MazeHeading dir = HEADING_NORTH;
    int dr = next.row - row, dc = next.col - col;
    if      (dr ==  1) dir = HEADING_NORTH;
    else if (dr == -1) dir = HEADING_SOUTH;
    else if (dc ==  1) dir = HEADING_EAST;
    else               dir = HEADING_WEST;
    issueMove(dir);
    botState = BOT_MOVING;
    return;
  }

  // Sense walls; if new walls found, rebuild live flood
  bool anyNew = senseAndRecordWallsInternal(row, col, ctGetHeading());
  if (anyNew) rebuildLiveFlood(toGoal);

  printCellWalls(row, col);

  // Choose best move
  bool found;
  MazeHeading bestDir = chooseBestMove(row, col, live_flood, &found);
  if (!found) {
    logPrintf("[EXPLORE] Trapped at (%d,%d)!", row, col);
    applyMotorPWM(0, 0);
    botState = BOT_DONE;
    return;
  }

  issueMove(bestDir);
}

// ============================================================
// A* STEP  (Run 2+)
// Called once per cell entry.
// ============================================================
void stepAstar() {
  int row = ctGetRow();
  int col = ctGetCol();

  if (isGoalCell(row, col)) {
    applyMotorPWM(0, 0);
    logPrintf("[A*] Goal reached at (%d,%d)! Run %d complete.", row, col, runNumber);
    botState = BOT_DONE;
    broadcastMazeWebReport();
    return;
  }

  // Sense walls before moving
  bool anyNew = senseAndRecordWallsInternal(row, col, ctGetHeading());

  if (anyNew && newWallBlocksPath(row, col)) {
    logPrintf("[A*] New wall near (%d,%d) blocks path — replanning...", row, col);
    buildFloodFill();
    bool ok = planAstar(row, col);
    if (!ok) { logLine("[A*] No path after replan!"); applyMotorPWM(0, 0); botState = BOT_DONE; return; }
    logPrintf("[A*] Replanned: %d steps remaining.", pathLen);
  }

  if (pathIdx >= pathLen) {
    logLine("[A*] Path exhausted — replanning from current position.");
    bool ok = planAstar(row, col);
    if (!ok) { logLine("[A*] No path — halting."); applyMotorPWM(0, 0); botState = BOT_DONE; return; }
  }

  PathCell next = plannedPath[pathIdx++];
  int dr = next.row - row, dc = next.col - col;
  MazeHeading dir = HEADING_NORTH;
  if      (dr ==  1) dir = HEADING_NORTH;
  else if (dr == -1) dir = HEADING_SOUTH;
  else if (dc ==  1) dir = HEADING_EAST;
  else               dir = HEADING_WEST;

  issueMove(dir);
}

// ============================================================
// MAIN STATE MACHINE TICK
// Called every LOOP_INTERVAL_MS from loop().
// ============================================================
void tickStateMachine(float dt) {
  // Always update EKF
  noInterrupts();
  long curL = leftTicks, curR = rightTicks;
  interrupts();
  long dL = curL - prevLeftTicks, dR = curR - prevRightTicks;

  current_yaw_angle = readYawDegrees();
  ekfPredict(dL, dR);
  ekfUpdateYawDeg(current_yaw_angle);
  ekfTelemetry = ekfGetTelemetry();

  prevLeftTicks  = curL;
  prevRightTicks = curR;

  bool newCell = updateCellTracker();

  switch (botState) {

    case BOT_IDLE:
      applyMotorPWM(0, 0);
      break;

    case BOT_MOVING:
      // Drive forward using PID
      runDrivingPID(dt);

      // When CellTracker detects we've crossed into a new cell → stop,
      // record walls, make the next algorithmic decision.
      if (newCell) {
        applyMotorPWM(0, 0);
        delay(50);  // brief settle
        // Dispatch to correct phase handler
        if (currentPhase == PHASE_ASTAR) {
          stepAstar();
        } else {
          stepExplore();
        }
        // issueMove() sets botState=BOT_MOVING; if we're done it sets BOT_DONE.
      }
      break;

    case BOT_TURN_COOLDOWN:
      // Not used in this architecture (turns are synchronous in Turns.h)
      botState = BOT_MOVING;
      break;

    case BOT_DONE:
      applyMotorPWM(0, 0);
      break;
  }
}

// ============================================================
// TELEMETRY & WEB REPORT
// ============================================================
String buildMazeWebReport() {
  String report = "Visited maze cells\n";
  for (int r = MAZE_SIZE - 1; r >= 0; r--) {
    for (int c = 0; c < MAZE_SIZE; c++) {
      if (!wallMap[r][c].visited) {
        report += ".";
      } else {
        report += wallMap[r][c].north ? "N" : "-";
        report += wallMap[r][c].east  ? "E" : "-";
        report += wallMap[r][c].south ? "S" : "-";
        report += wallMap[r][c].west  ? "W" : "-";
      }
      if (c < MAZE_SIZE - 1) report += "  ";
    }
    if (r > 0) report += "\n";
  }
  return report;
}

void broadcastMazeWebReport() {
  logLine("MAZE:" + buildMazeWebReport());
}

void printTelemetry() {
  noInterrupts();
  long cL = leftTicks, cR = rightTicks;
  interrupts();
  const char *phaseStr =
    (currentPhase == PHASE_EXPLORE_TO_GOAL)  ? "EXP→GOAL" :
    (currentPhase == PHASE_EXPLORE_TO_START) ? "EXP→START" : "A*";
  const char *stateStr =
    botState == BOT_IDLE          ? "IDLE" :
    botState == BOT_MOVING        ? "MOVE" :
    botState == BOT_TURN_COOLDOWN ? "COOLDOWN" : "DONE";

  logPrintf("Enc:%ld/%ld PWM:%d/%d Yaw:%.1f Lidar:%d/%d/%d Cell:(%d,%d)%s Phase:%s St:%s",
    cL, cR, final_pwm_L, final_pwm_R,
    current_yaw_angle,
    current_lidars.left, current_lidars.front, current_lidars.right,
    ctGetRow(), ctGetCol(), headingName(ctGetHeading()),
    phaseStr, stateStr);
}

// ============================================================
// WEBSOCKET
// ============================================================
void webSocketEvent(uint8_t num, WStype_t type, uint8_t *payload, size_t length) {
  if (type == WStype_CONNECTED) replayLogHistory(num);
}

// ============================================================
// SETUP
// ============================================================
void setup() {
  Serial.begin(115200);
  delay(1000);

  WiFi.softAP(ssid, password);
  server.on("/", []() { server.send(200, "text/html", index_html); });
  server.begin();
  webSocket.begin();
  webSocket.onEvent(webSocketEvent);

  logLine(String("AP IP: ") + WiFi.softAPIP().toString());
  logLine("=================================");
  logLine("  MICROMOUSE  —  Iteration 1");
  logLine("  Floodfill Explore + A* Run");
  logLine("=================================");

  initMotors();
  initEncoders();
  initSensors();
  delay(5000);

  logLine("[!] Calibrating gyro...");
  delay(2000);
  calibrateGyro();
  logLine("Gyro OK.");

  resetYaw();
  current_yaw_angle = baseTargetYaw = targetYaw = 0.0f;

  ekfConfigure(TICKS_PER_REV, WHEEL_CIRCUMFERENCE_MM, TRACK_WIDTH_MM);
  ekfInit(0.0f, 0.0f, 0.0f);
  initCellTracker();
  initWallMap();

  // Initialise flood tables
  for (int r = 0; r < MAZE_SIZE; r++)
    for (int c = 0; c < MAZE_SIZE; c++) {
      flood_dist[r][c] = INF_DIST;
      live_flood[r][c] = INF_DIST;
    }

  resetEncoders();
  prevLeftTicks = prevRightTicks = 0;
  resetPIDIntegrals();
  resetWallPID();

  current_lidars = readLidars();

  unsigned long now = millis();
  lastLoopTime = lastLidarTime = lastPrintTime = now;

  // ── Begin Run 1 ──
  runNumber    = 1;
  currentPhase = PHASE_EXPLORE_TO_GOAL;
  robotFacing  = HEADING_NORTH;

  // Seed: sense walls at start cell, init live flood toward goal
  senseAndRecordWallsInternal(0, 0, HEADING_NORTH);
  rebuildLiveFlood(true);

  logLine("[RUN 1] Floodfill explore: start → goal → start");

  // Issue first move
  bool found;
  MazeHeading firstDir = chooseBestMove(0, 0, live_flood, &found);
  if (!found) {
    logLine("[!] Start cell is surrounded by walls — check lidar calibration.");
    botState = BOT_DONE;
  } else {
    issueMove(firstDir);
  }
}

// ============================================================
// MAIN LOOP
// ============================================================
void loop() {
  webSocket.loop();
  server.handleClient();

  unsigned long now = millis();

  // Lidar update (wall PID)
  if (now - lastLidarTime >= LIDAR_INTERVAL_MS) {
    float dt_lidar = (now - lastLidarTime) / 1000.0f;
    lastLidarTime  = now;
    current_lidars = readLidars();

    if (botState == BOT_MOVING) {
      runWallPIDLoop(dt_lidar);
    }
  }

  // Control loop
  if (now - lastLoopTime >= LOOP_INTERVAL_MS) {
    float dt       = (now - lastLoopTime) / 1000.0f;
    lastLoopTime   = now;
    tickStateMachine(dt);
  }

  // Telemetry
  if (now - lastPrintTime >= PRINT_INTERVAL_MS) {
    lastPrintTime = now;
    printTelemetry();
  }
}
