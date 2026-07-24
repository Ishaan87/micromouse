#ifndef FLOODFILL_H
#define FLOODFILL_H

#include "WallMap.h"
#include "CellTracker.h"

// ============================================================
// FLOODFILL PATHFINDER
//
// Runs a standard BFS-based flood-fill on the completed
// WallMap to find the shortest path from any start cell
// to any goal cell.
//
// Output: a flat array of MazeHeading moves (NORTH/EAST/
// SOUTH/WEST) that the solver follows cell by cell.
//
// Usage:
//   1. Call ffComputePath(startRow, startCol, goalRow, goalCol)
//      after the survey phase is complete.
//   2. Read the planned moves with ffGetMove(index) and
//      ffGetPathLength().
//   3. The solver consumes moves one by one as it crosses
//      each cell boundary.
// ============================================================

// ==========================================
// CONSTANTS
// ==========================================
const int FF_MAX_CELLS = MAZE_SIZE * MAZE_SIZE;  // 25 for a 5×5 maze
const int FF_INF       = 9999;

// ==========================================
// INTERNAL FLOOD GRID
// Each cell holds its "distance" (in steps)
// from the goal. The BFS fills this backwards
// from goal → start, so the robot can follow
// the steepest descent toward zero.
// ==========================================
static int  ff_dist[MAZE_SIZE][MAZE_SIZE];
static bool ff_visited[MAZE_SIZE][MAZE_SIZE];

// ==========================================
// PLANNED PATH STORAGE
// Maximum path length = all cells in the maze.
// Stored as a sequence of absolute headings
// (the direction the robot faces to enter
// the next cell).
// ==========================================
static MazeHeading ff_path[FF_MAX_CELLS];
static int         ff_pathLength = 0;
static bool        ff_pathValid  = false;

// ==========================================
// SIMPLE BFS QUEUE  (no dynamic alloc)
// ==========================================
struct FFCell { int row; int col; };
static FFCell ff_queue[FF_MAX_CELLS];

// ==========================================
// FLOOD-FILL (BFS backwards from goal)
//
// Fills ff_dist[][] so every reachable cell
// holds its step-count distance to the goal.
// ==========================================
static void ff_bfsFromGoal(int goalRow, int goalCol) {
  // Init all distances to infinity
  for (int r = 0; r < MAZE_SIZE; r++)
    for (int c = 0; c < MAZE_SIZE; c++) {
      ff_dist[r][c]    = FF_INF;
      ff_visited[r][c] = false;
    }

  // Seed the queue with the goal cell
  int head = 0, tail = 0;
  ff_dist[goalRow][goalCol]    = 0;
  ff_visited[goalRow][goalCol] = true;
  ff_queue[tail++] = {goalRow, goalCol};

  // Standard BFS
  while (head < tail) {
    FFCell cur = ff_queue[head++];
    int r = cur.row, c = cur.col;
    int nextDist = ff_dist[r][c] + 1;

    // Try all four neighbours — only cross if no wall between them
    // NORTH neighbour
    if (r + 1 < MAZE_SIZE && !wallMap[r][c].north && !ff_visited[r+1][c]) {
      ff_dist[r+1][c]    = nextDist;
      ff_visited[r+1][c] = true;
      ff_queue[tail++]   = {r+1, c};
    }
    // SOUTH neighbour
    if (r - 1 >= 0 && !wallMap[r][c].south && !ff_visited[r-1][c]) {
      ff_dist[r-1][c]    = nextDist;
      ff_visited[r-1][c] = true;
      ff_queue[tail++]   = {r-1, c};
    }
    // EAST neighbour
    if (c + 1 < MAZE_SIZE && !wallMap[r][c].east && !ff_visited[r][c+1]) {
      ff_dist[r][c+1]    = nextDist;
      ff_visited[r][c+1] = true;
      ff_queue[tail++]   = {r, c+1};
    }
    // WEST neighbour
    if (c - 1 >= 0 && !wallMap[r][c].west && !ff_visited[r][c-1]) {
      ff_dist[r][c-1]    = nextDist;
      ff_visited[r][c-1] = true;
      ff_queue[tail++]   = {r, c-1};
    }
  }
}

// ==========================================
// PATH TRACE (greedy descent from start)
//
// After BFS fills ff_dist, walks from start
// toward goal by always moving to the
// neighbour with the smallest distance value.
// Records the heading taken at each step.
// ==========================================
static bool ff_tracePath(int startRow, int startCol, int goalRow, int goalCol) {
  ff_pathLength = 0;
  ff_pathValid  = false;

  int r = startRow, c = startCol;

  // Safety: if goal is unreachable from start, abort
  if (ff_dist[r][c] == FF_INF) {
    Serial.println("[FF] ERROR: Goal unreachable from start!");
    return false;
  }

  while (!(r == goalRow && c == goalCol)) {
    if (ff_pathLength >= FF_MAX_CELLS) {
      Serial.println("[FF] ERROR: Path overflow — possible loop!");
      return false;
    }

    int bestDist = FF_INF;
    int nextR = r, nextC = c;
    MazeHeading nextHeading = HEADING_NORTH;

    // Check all four neighbours; pick the one with the lowest flood value
    if (r + 1 < MAZE_SIZE && !wallMap[r][c].north && ff_dist[r+1][c] < bestDist) {
      bestDist = ff_dist[r+1][c]; nextR = r+1; nextC = c; nextHeading = HEADING_NORTH;
    }
    if (r - 1 >= 0 && !wallMap[r][c].south && ff_dist[r-1][c] < bestDist) {
      bestDist = ff_dist[r-1][c]; nextR = r-1; nextC = c; nextHeading = HEADING_SOUTH;
    }
    if (c + 1 < MAZE_SIZE && !wallMap[r][c].east && ff_dist[r][c+1] < bestDist) {
      bestDist = ff_dist[r][c+1]; nextR = r; nextC = c+1; nextHeading = HEADING_EAST;
    }
    if (c - 1 >= 0 && !wallMap[r][c].west && ff_dist[r][c-1] < bestDist) {
      bestDist = ff_dist[r][c-1]; nextR = r; nextC = c-1; nextHeading = HEADING_WEST;
    }

    if (bestDist == FF_INF) {
      Serial.println("[FF] ERROR: Trapped during trace — maze may be unconnected.");
      return false;
    }

    ff_path[ff_pathLength++] = nextHeading;
    r = nextR;
    c = nextC;
  }

  ff_pathValid = true;
  return true;
}

// ==========================================
// PUBLIC API
// ==========================================

// Run the full flood-fill and path trace.
// Call once after survey is complete.
// Returns true if a valid path was found.
bool ffComputePath(int startRow, int startCol, int goalRow, int goalCol) {
  Serial.printf("[FF] Computing path (%d,%d) → (%d,%d)...\n",
                startRow, startCol, goalRow, goalCol);

  ff_bfsFromGoal(goalRow, goalCol);
  bool ok = ff_tracePath(startRow, startCol, goalRow, goalCol);

  if (ok) {
    Serial.printf("[FF] Path found! %d steps:\n", ff_pathLength);
    for (int i = 0; i < ff_pathLength; i++) {
      Serial.printf("  step %2d: %s\n", i, headingName(ff_path[i]));
    }
  }
  return ok;
}

// Returns total number of moves in the planned path.
inline int ffGetPathLength() { return ff_pathLength; }

// Returns the planned heading for move index i.
// i is 0-based. Returns HEADING_NORTH if out of range.
inline MazeHeading ffGetMove(int i) {
  if (i < 0 || i >= ff_pathLength) return HEADING_NORTH;
  return ff_path[i];
}

// True if a valid path has been computed.
inline bool ffPathValid() { return ff_pathValid; }

// Print the flood distance grid to Serial for debugging.
void ffPrintDistGrid() {
  Serial.println("\n--- FLOOD DISTANCES (goal=0) ---");
  for (int r = MAZE_SIZE - 1; r >= 0; r--) {
    for (int c = 0; c < MAZE_SIZE; c++) {
      if (ff_dist[r][c] == FF_INF)
        Serial.print(" ## ");
      else
        Serial.printf("%3d ", ff_dist[r][c]);
    }
    Serial.println();
  }
  Serial.println("--------------------------------\n");
}

#endif // FLOODFILL_H
