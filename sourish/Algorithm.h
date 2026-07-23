// ===========================================================================
// Algorithm.h
// Micromouse navigation: Floodfill Exploration (Run 1) + A* Speed Run (Run 2)
//
// Ported from iteration-1_algorithm.py.
//
// DEPENDENCIES
//   WallMap.h    — wallMap[][], recordWalls(), canMove(), MAZE_SIZE
//   CellTracker.h — MazeHeading, HEADING_NORTH/EAST/SOUTH/WEST
//   Turns.h      — turnCW90(), turnACW90(), turn180()
//   Micromouse.ino must provide:
//     extern DistanceData current_lidars;
//     void advanceOneCell();          (drive forward one full cell)
//     void logPrintf(const char*, ...);
// ===========================================================================

#ifndef ALGORITHM_H
#define ALGORITHM_H

#include <Arduino.h>
#include "Config.h"
#include "WallMap.h"
#include "CellTracker.h"
#include "Turns.h"

// ===========================================================================
// CONSTANTS
// ===========================================================================
static const int   ALGO_W    = MAZE_SIZE;
static const int   ALGO_H    = MAZE_SIZE;
static const int   ALGO_INF  = 32767;

// Goal = centre cell(s) of the maze.
// For a 6×6 maze the centre 2×2 block is (2,2),(3,2),(2,3),(3,3).
// Cells are stored as a flat array; goalCount ≤ 4.
static int goalCells[4][2];   // [i][0]=row  [i][1]=col
static int goalCount = 0;

// ===========================================================================
// RUNTIME NAVIGATION STATE
// (separate from the hardware CellTracker — this is the algorithm's own
//  view of position, used during commanded moves)
// ===========================================================================
static int          algo_row     = 0;
static int          algo_col     = 0;
static MazeHeading  algo_heading = HEADING_NORTH;

// ===========================================================================
// FLOOD-FILL DISTANCE TABLE
// flood_dist[row][col] = exact BFS distance to nearest goal (wall-aware).
// INF until buildFloodFill() is called.
// ===========================================================================
static int flood_dist[MAZE_SIZE][MAZE_SIZE];

// ===========================================================================
// DIRECTION HELPERS
//   Directions: 0=NORTH  1=EAST  2=SOUTH  3=WEST  (matches MazeHeading)
//
//   DR[d], DC[d] = row/col delta when moving in direction d.
//   row increases northward, col increases eastward.
// ===========================================================================
static const int DR[4] = {  1,  0, -1,  0 };  // N, E, S, W
static const int DC[4] = {  0,  1,  0, -1 };

static const MazeHeading DIR_TO_HEADING[4] = {
  HEADING_NORTH, HEADING_EAST, HEADING_SOUTH, HEADING_WEST
};

// Is cell (r,c) a goal?
static inline bool isGoalCell(int r, int c) {
  for (int i = 0; i < goalCount; i++)
    if (goalCells[i][0] == r && goalCells[i][1] == c)
      return true;
  return false;
}

// Can the robot move from (r,c) in absolute direction d (0-3)?
// Wraps wallMap walls using the existing canMove() with MazeHeading cast.
static inline bool algoCellOpen(int r, int c, int d) {
  return canMove(r, c, DIR_TO_HEADING[d]);
}

// ===========================================================================
// GOAL CELL INITIALISATION
// Centre 2×2 of an N×N maze (handles odd/even dimensions).
// ===========================================================================
void initGoalCells() {
  goalCount = 0;
  int cx = ALGO_H / 2;   // row centre
  int cy = ALGO_W / 2;   // col centre

  // For even dimensions include the 2×2 block; for odd just the single centre
  int rMin = (ALGO_H % 2 == 0) ? cx - 1 : cx;
  int rMax = cx;
  int cMin = (ALGO_W % 2 == 0) ? cy - 1 : cy;
  int cMax = cy;

  for (int r = rMin; r <= rMax && goalCount < 4; r++)
    for (int c = cMin; c <= cMax && goalCount < 4; c++) {
      goalCells[goalCount][0] = r;
      goalCells[goalCount][1] = c;
      goalCount++;
    }
}

// ===========================================================================
// WALL SENSING (bridge to hardware)
// Reads current_lidars (set by the main loop's Lidar poll) and stamps the
// walls into wallMap at the given cell.
// ===========================================================================
extern DistanceData current_lidars;   // defined in Micromouse.ino

// Sense all three walls from the current cell and record them.
// Returns true if any new wall was discovered.
static bool senseAndRecordWalls(int r, int c, MazeHeading heading) {
  bool frontBefore = *getWallMapping(r, c, heading).frontWall;
  bool leftBefore  = *getWallMapping(r, c, heading).leftWall;
  bool rightBefore = *getWallMapping(r, c, heading).rightWall;

  // Re-read lidars fresh before stamping
  current_lidars = readLidars();
  recordWalls(r, c, heading, current_lidars);

  bool frontAfter  = *getWallMapping(r, c, heading).frontWall;
  bool leftAfter   = *getWallMapping(r, c, heading).leftWall;
  bool rightAfter  = *getWallMapping(r, c, heading).rightWall;

  return (frontAfter != frontBefore) ||
         (leftAfter  != leftBefore)  ||
         (rightAfter != rightBefore);
}

// ===========================================================================
// MOVEMENT PRIMITIVES (bridge to hardware turn + drive functions)
// advanceOneCell() is provided by Micromouse.ino.
// ===========================================================================
extern void advanceOneCell();   // drive forward exactly one maze cell

static void algoTurnLeft() {
  turnACW90();
  algo_heading = (MazeHeading)((algo_heading + 3) % 4);
}

static void algoTurnRight() {
  turnCW90();
  algo_heading = (MazeHeading)((algo_heading + 1) % 4);
}

static void algoMoveForward() {
  advanceOneCell();
  algo_row += DR[algo_heading];
  algo_col += DC[algo_heading];
}

// Turn to face absolute direction d (0=N,1=E,2=S,3=W) via shortest rotation.
static void algoTurnToFace(int targetDir) {
  int rightTurns = ((int)targetDir - (int)algo_heading + 4) % 4;
  if      (rightTurns == 1) { algoTurnRight(); }
  else if (rightTurns == 2) { algoTurnRight(); algoTurnRight(); }  // U-turn
  else if (rightTurns == 3) { algoTurnLeft(); }
  // 0: already facing correct direction
}

// Turn to face neighbour (nr,nc) then drive there.
static void algoMoveToCell(int nr, int nc) {
  int dr = nr - algo_row;
  int dc = nc - algo_col;
  int targetDir;
  if      (dr ==  1) targetDir = 0;   // NORTH
  else if (dc ==  1) targetDir = 1;   // EAST
  else if (dr == -1) targetDir = 2;   // SOUTH
  else               targetDir = 3;   // WEST
  algoTurnToFace(targetDir);
  algoMoveForward();
}

// ===========================================================================
// MULTI-SOURCE BFS — live flood table
// Used during Run 1 exploration so the mouse always steers toward the
// closest target using the currently-known wall map.
// Returns a flat [MAZE_SIZE * MAZE_SIZE] array via the provided buffer.
// ===========================================================================
static void buildLiveFlood(int targets[][2], int tCount, int liveDist[MAZE_SIZE][MAZE_SIZE]) {
  // Initialise everything to INF
  for (int r = 0; r < MAZE_SIZE; r++)
    for (int c = 0; c < MAZE_SIZE; c++)
      liveDist[r][c] = ALGO_INF;

  // Simple queue using a fixed circular buffer (MAZE_SIZE² is small)
  static int qr[MAZE_SIZE * MAZE_SIZE];
  static int qc[MAZE_SIZE * MAZE_SIZE];
  int head = 0, tail = 0;

  for (int i = 0; i < tCount; i++) {
    int gr = targets[i][0], gc = targets[i][1];
    liveDist[gr][gc] = 0;
    qr[tail] = gr; qc[tail] = gc; tail++;
  }

  while (head != tail) {
    int r = qr[head], c = qc[head]; head++;
    for (int d = 0; d < 4; d++) {
      if (!algoCellOpen(r, c, d)) continue;
      int nr = r + DR[d], nc = c + DC[d];
      if (nr < 0 || nr >= MAZE_SIZE || nc < 0 || nc >= MAZE_SIZE) continue;
      if (liveDist[nr][nc] == ALGO_INF) {
        liveDist[nr][nc] = liveDist[r][c] + 1;
        qr[tail] = nr; qc[tail] = nc; tail++;
      }
    }
  }
}

// ===========================================================================
// FLOOD-FILL DISTANCE TABLE (post-exploration)
// Called once after Run 1 finishes. Populates the global flood_dist[][].
// ===========================================================================
void buildFloodFill() {
  logPrintf("[ALGO] Building flood-fill distance table...");

  buildLiveFlood(goalCells, goalCount, flood_dist);

  int reachable = 0;
  for (int r = 0; r < MAZE_SIZE; r++)
    for (int c = 0; c < MAZE_SIZE; c++)
      if (flood_dist[r][c] < ALGO_INF) reachable++;

  logPrintf("[ALGO] Flood-fill done. %d/%d cells reachable. Start→goal: %d steps.",
            reachable, MAZE_SIZE * MAZE_SIZE, flood_dist[0][0]);
}

// ===========================================================================
// RUN 1 — FLOODFILL-DRIVEN EXPLORATION
//
// One lap: navigate from start to goal, then navigate back to (0,0).
// At every cell the mouse senses walls and steers toward whichever open
// neighbour has the lowest live flood-fill distance to the current target.
// The live flood table is rebuilt whenever new walls are discovered.
// ===========================================================================

// Internal: navigate from current position to any cell in targets[].
// visited_label is printed in logs for phase identification.
static bool floodfillPhase(int targets[][2], int tCount, const char *label) {
  int liveDist[MAZE_SIZE][MAZE_SIZE];
  buildLiveFlood(targets, tCount, liveDist);

  int steps = 0;

  // Check if already at a target
  for (int i = 0; i < tCount; i++)
    if (algo_row == targets[i][0] && algo_col == targets[i][1])
      return true;

  while (true) {
    int r = algo_row, c = algo_col;

    // Sense walls; rebuild flood if new info
    bool newWall = senseAndRecordWalls(r, c, algo_heading);
    if (newWall) buildLiveFlood(targets, tCount, liveDist);

    // Pick the open neighbour with the lowest flood distance
    int bestDir = -1, bestVal = ALGO_INF;
    for (int d = 0; d < 4; d++) {
      if (!algoCellOpen(r, c, d)) continue;
      int nr = r + DR[d], nc = c + DC[d];
      if (nr < 0 || nr >= MAZE_SIZE || nc < 0 || nc >= MAZE_SIZE) continue;
      if (liveDist[nr][nc] < bestVal) {
        bestVal = liveDist[nr][nc];
        bestDir = d;
      }
    }

    if (bestDir == -1 || bestVal == ALGO_INF) {
      logPrintf("[ALGO] %s: trapped at (%d,%d)! No passable neighbour.", label, r, c);
      return false;
    }

    int nr = r + DR[bestDir], nc = c + DC[bestDir];
    algoMoveToCell(nr, nc);
    steps++;

    // Check arrival
    for (int i = 0; i < tCount; i++) {
      if (algo_row == targets[i][0] && algo_col == targets[i][1]) {
        logPrintf("[ALGO] %s: target reached in %d steps.", label, steps);
        return true;
      }
    }
  }
}

void runFloodfillExplore() {
  logPrintf("[ALGO] === RUN 1: Floodfill Explore (start -> goal -> start) ===");

  // Phase 1: Navigate to goal
  logPrintf("[ALGO]   Phase 1: start -> goal...");
  bool ok1 = floodfillPhase(goalCells, goalCount, "out(start->goal)");
  if (!ok1) {
    logPrintf("[ALGO]   Could not reach goal — maze may be disconnected.");
    return;
  }

  // Phase 2: Return to start
  logPrintf("[ALGO]   Phase 2: goal -> start...");
  static int startCell[1][2] = {{0, 0}};
  bool ok2 = floodfillPhase(startCell, 1, "back(goal->start)");
  if (!ok2) {
    logPrintf("[ALGO]   Could not return to start.");
    return;
  }

  logPrintf("[ALGO] === FLOODFILL EXPLORATION COMPLETE. Mouse at (%d,%d) ===",
            algo_row, algo_col);
}

// ===========================================================================
// RUN 2 — A* SPEED RUN  (flood-fill heuristic, live replanning)
//
// Plans the optimal path from start to goal using A*, follows it step by
// step, sensing walls before each move.  If a new wall is found that sits
// on the planned path the flood table is refreshed and A* replans instantly.
// ===========================================================================

// A* path storage: up to MAZE_SIZE² steps
static int astarPath[MAZE_SIZE * MAZE_SIZE][2];   // [i][0]=row [i][1]=col
static int astarPathLen  = 0;
static int astarPathHead = 0;   // index of next step to execute

// g_score / came_from tables (flat arrays for speed)
static int   astar_g[MAZE_SIZE][MAZE_SIZE];
static int   astar_from_r[MAZE_SIZE][MAZE_SIZE];
static int   astar_from_c[MAZE_SIZE][MAZE_SIZE];
static bool  astar_closed[MAZE_SIZE][MAZE_SIZE];

// Minimal binary min-heap for A* open set
// Each entry: {f, g, row, col}
struct AStarNode { int f, g, r, c; };
static AStarNode astarHeap[MAZE_SIZE * MAZE_SIZE];
static int       astarHeapSize = 0;

static void heapPush(AStarNode n) {
  int i = astarHeapSize++;
  astarHeap[i] = n;
  while (i > 0) {
    int p = (i - 1) / 2;
    if (astarHeap[p].f <= astarHeap[i].f) break;
    AStarNode tmp = astarHeap[p]; astarHeap[p] = astarHeap[i]; astarHeap[i] = tmp;
    i = p;
  }
}

static AStarNode heapPop() {
  AStarNode top = astarHeap[0];
  astarHeap[0]  = astarHeap[--astarHeapSize];
  int i = 0;
  while (true) {
    int l = 2*i+1, r2 = 2*i+2, s = i;
    if (l < astarHeapSize && astarHeap[l].f < astarHeap[s].f) s = l;
    if (r2 < astarHeapSize && astarHeap[r2].f < astarHeap[s].f) s = r2;
    if (s == i) break;
    AStarNode tmp = astarHeap[s]; astarHeap[s] = astarHeap[i]; astarHeap[i] = tmp;
    i = s;
  }
  return top;
}

// Plan A* from (sr,sc), store result in astarPath[].
// Returns number of steps (0 = no path).
static int astarPlan(int sr, int sc) {
  // Clear tables
  for (int r = 0; r < MAZE_SIZE; r++)
    for (int c = 0; c < MAZE_SIZE; c++) {
      astar_g[r][c]      = ALGO_INF;
      astar_from_r[r][c] = -1;
      astar_from_c[r][c] = -1;
      astar_closed[r][c] = false;
    }

  astarHeapSize = 0;
  astar_g[sr][sc] = 0;
  int h0 = (flood_dist[sr][sc] < ALGO_INF) ? flood_dist[sr][sc] : ALGO_INF;
  heapPush({h0, 0, sr, sc});

  while (astarHeapSize > 0) {
    AStarNode cur = heapPop();
    int r = cur.r, c = cur.c, g = cur.g;

    if (astar_closed[r][c]) continue;
    astar_closed[r][c] = true;

    // Goal check
    if (isGoalCell(r, c)) {
      // Reconstruct path (excluding start, including goal)
      int pathLen = 0;
      int pr = r, pc = c;
      // Count length first
      while (astar_from_r[pr][pc] != -1) {
        pathLen++;
        int nr2 = astar_from_r[pr][pc];
        int nc2 = astar_from_c[pr][pc];
        pr = nr2; pc = nc2;
      }
      // Store in forward order
      astarPathLen  = pathLen;
      astarPathHead = 0;
      pr = r; pc = c;
      for (int i = pathLen - 1; i >= 0; i--) {
        astarPath[i][0] = pr;
        astarPath[i][1] = pc;
        int nr2 = astar_from_r[pr][pc];
        int nc2 = astar_from_c[pr][pc];
        pr = nr2; pc = nc2;
      }
      logPrintf("[ALGO] A* path: %d steps.", pathLen);
      return pathLen;
    }

    // Expand neighbours
    for (int d = 0; d < 4; d++) {
      if (!algoCellOpen(r, c, d)) continue;
      int nr = r + DR[d], nc2 = c + DC[d];
      if (nr < 0 || nr >= MAZE_SIZE || nc2 < 0 || nc2 >= MAZE_SIZE) continue;
      if (astar_closed[nr][nc2]) continue;
      int ng = g + 1;
      if (ng < astar_g[nr][nc2]) {
        astar_g[nr][nc2]      = ng;
        astar_from_r[nr][nc2] = r;
        astar_from_c[nr][nc2] = c;
        int h = (flood_dist[nr][nc2] < ALGO_INF) ? flood_dist[nr][nc2] : ALGO_INF;
        heapPush({ng + h, ng, nr, nc2});
      }
    }
  }

  logPrintf("[ALGO] A* found no path — maze may be unsolvable!");
  return 0;
}

// Check if cell (r,c) appears in the current planned path (ahead of head).
static bool cellOnPath(int r, int c) {
  for (int i = astarPathHead; i < astarPathLen; i++)
    if (astarPath[i][0] == r && astarPath[i][1] == c)
      return true;
  return false;
}

void runAStarSpeedRun() {
  logPrintf("[ALGO] === RUN 2: A* Speed Run (flood-fill heuristic, live replanning) ===");

  // Initial plan
  logPrintf("[ALGO]   Planning initial path...");
  int pLen = astarPlan(algo_row, algo_col);
  if (pLen == 0) {
    logPrintf("[ALGO]   No path found. Aborting.");
    return;
  }
  logPrintf("[ALGO]   Path: %d steps. Starting run...", pLen);

  int totalSteps = 0;

  while (!isGoalCell(algo_row, algo_col)) {
    // Replan if path exhausted before reaching goal
    if (astarPathHead >= astarPathLen) {
      logPrintf("[ALGO]   Path exhausted — replanning...");
      if (!astarPlan(algo_row, algo_col)) { logPrintf("[ALGO]   No path. Aborting."); return; }
    }

    int r = algo_row, c = algo_col;

    // ── Sense walls BEFORE moving ─────────────────────────────────────────
    bool newWall = senseAndRecordWalls(r, c, algo_heading);

    if (newWall) {
      // Check whether any newly blocked neighbour sits on the planned path
      bool blockedOnPath = false;
      for (int d = 0; d < 4; d++) {
        // If there's now a wall in direction d...
        if (!algoCellOpen(r, c, d)) {
          int nr = r + DR[d], nc = c + DC[d];
          if (nr >= 0 && nr < MAZE_SIZE && nc >= 0 && nc < MAZE_SIZE)
            if (cellOnPath(nr, nc)) { blockedOnPath = true; break; }
        }
      }
      if (blockedOnPath) {
        logPrintf("[ALGO]   New wall at (%d,%d) blocks path — rebuilding + replanning...", r, c);
        buildFloodFill();   // refresh heuristic with updated walls
        if (!astarPlan(r, c)) { logPrintf("[ALGO]   No path after replan. Aborting."); return; }
        logPrintf("[ALGO]   Replanned: %d steps remaining.", astarPathLen - astarPathHead);
      }
    }

    // ── Move to next waypoint ─────────────────────────────────────────────
    int nr = astarPath[astarPathHead][0];
    int nc = astarPath[astarPathHead][1];
    astarPathHead++;

    algoMoveToCell(nr, nc);
    totalSteps++;

    if (isGoalCell(algo_row, algo_col))
      logPrintf("[ALGO]   Goal reached at (%d,%d) in %d steps!", algo_row, algo_col, totalSteps);
  }

  logPrintf("[ALGO] === A* RUN COMPLETE ===");
}

// ===========================================================================
// INIT
// Call once from setup(), AFTER initWallMap() and initCellTracker().
// ===========================================================================
void initAlgorithm() {
  algo_row     = 0;
  algo_col     = 0;
  algo_heading = HEADING_NORTH;

  initGoalCells();

  for (int r = 0; r < MAZE_SIZE; r++)
    for (int c = 0; c < MAZE_SIZE; c++)
      flood_dist[r][c] = ALGO_INF;

  logPrintf("[ALGO] Goal cells (%d): ", goalCount);
  for (int i = 0; i < goalCount; i++)
    logPrintf("  (%d,%d)", goalCells[i][0], goalCells[i][1]);
}

#endif // ALGORITHM_H
