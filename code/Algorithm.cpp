#include "Algorithm.h"
#include "API.h"
#include "Maze.h"
#include <queue>
#include <vector>
#include <string.h>

// ===========================================================================
// LOGGING  (Serial replaces Python's stderr)
// ===========================================================================
#define LOG(msg)      Serial.println(msg)
#define LOGF(...)     Serial.printf(__VA_ARGS__)

// ===========================================================================
// MOVEMENT HELPERS
// Exact translation of Python turn_left / turn_right / move_forward /
// turn_to_face / move_to_cell.
// ===========================================================================

static void doTurnLeft(MouseState& s) {
    turnLeft();
    s.facing = (s.facing + 3) % 4;   // NORTH→WEST→SOUTH→EAST
}

static void doTurnRight(MouseState& s) {
    turnRight();
    s.facing = (s.facing + 1) % 4;   // NORTH→EAST→SOUTH→WEST
}

static void doMoveForward(MouseState& s) {
    moveForward();
    s.x += DX[s.facing];
    s.y += DY[s.facing];
}

static void turnToFace(uint8_t targetDir, MouseState& s) {
    int rightTurns = (targetDir - s.facing + 4) % 4;
    if      (rightTurns == 1) doTurnRight(s);
    else if (rightTurns == 2) { doTurnRight(s); doTurnRight(s); }  // U-turn
    else if (rightTurns == 3) doTurnLeft(s);
    // 0 → already facing the right way
}

static void moveToCell(uint8_t nx, uint8_t ny, MouseState& s) {
    int8_t dx = (int8_t)nx - (int8_t)s.x;
    int8_t dy = (int8_t)ny - (int8_t)s.y;
    uint8_t targetDir;
    if      (dx ==  1) targetDir = EAST;
    else if (dx == -1) targetDir = WEST;
    else if (dy ==  1) targetDir = NORTH;
    else               targetDir = SOUTH;
    turnToFace(targetDir, s);
    doMoveForward(s);
}

// ===========================================================================
// WALL SENSING
// Converts relative sensor readings (front/left/right) to absolute
// compass directions and records any newly discovered walls.
// Returns true if at least one new wall was found.
// ===========================================================================
static uint8_t absDir(uint8_t facing, uint8_t relative) {
    // relative: 0=front, 1=right, 2=left  (matches sensor call order below)
    if (relative == 0) return facing;
    if (relative == 1) return (facing + 1) % 4;   // right
    return                     (facing + 3) % 4;   // left
}

static bool senseAndRecord(MouseState& s) {
    bool anyNew = false;
    // {relative_index, sensor_reading}
    bool readings[3] = { wallFront(), wallRight(), wallLeft() };
    uint8_t relDirs[3] = { 0, 1, 2 };   // front, right, left
    for (int i = 0; i < 3; i++) {
        if (readings[i]) {
            uint8_t dir = absDir(s.facing, relDirs[i]);
            if (recordWall(s.x, s.y, dir)) anyNew = true;
        }
    }
    return anyNew;
}

// ===========================================================================
// FLOODFILL PHASE
// Navigates from current position to any cell in targets[] by always
// stepping to the open neighbour with the lowest live flood value.
// Senses walls at every cell; rebuilds flood table on any new wall.
// Returns true if a target was reached.
// ===========================================================================
static bool floodfillPhase(MouseState& state,
                            const uint8_t targX[], const uint8_t targY[],
                            uint8_t targCount,
                            const char* label)
{
    // Working flood table for this phase
    int16_t live[MAZE_WIDTH][MAZE_HEIGHT];
    buildLiveFlood(targX, targY, targCount, live);

    int steps = 0;

    // Check if already at a target
    auto atTarget = [&]() {
        for (int i = 0; i < targCount; i++)
            if (state.x == targX[i] && state.y == targY[i]) return true;
        return false;
    };

    while (!atTarget()) {
        uint8_t x = state.x, y = state.y;

        // Sense walls; rebuild if anything new
        bool anyNew = false;
        bool readings[3] = { wallFront(), wallRight(), wallLeft() };
        uint8_t relDirs[3] = { 0, 1, 2 };
        for (int i = 0; i < 3; i++) {
            if (readings[i]) {
                uint8_t dir = absDir(state.facing, relDirs[i]);
                if (recordWall(x, y, dir)) anyNew = true;
            }
        }
        if (anyNew) buildLiveFlood(targX, targY, targCount, live);

        // Pick open neighbour with lowest flood distance
        uint8_t bestDir = 255;
        int16_t bestVal = INF_DIST;
        for (int d = 0; d < 4; d++) {
            if (hasWall(x, y, d)) continue;
            int8_t nx = x + DX[d];
            int8_t ny = y + DY[d];
            if (nx < 0 || nx >= MAZE_WIDTH || ny < 0 || ny >= MAZE_HEIGHT) continue;
            if (live[nx][ny] < bestVal) {
                bestVal = live[nx][ny];
                bestDir = d;
            }
        }

        if (bestDir == 255 || bestVal == INF_DIST) {
            LOGF("  %s: trapped at (%d,%d)!\n", label, x, y);
            return false;
        }

        uint8_t nx = x + DX[bestDir];
        uint8_t ny = y + DY[bestDir];
        moveToCell(nx, ny, state);
        steps++;

        if (!visited[nx][ny]) {
            visited[nx][ny] = true;
        }
    }

    LOGF("  %s: reached target in %d steps\n", label, steps);
    return true;
}

// ===========================================================================
// RUN 1 — FLOODFILL EXPLORATION
// Phase 1: start → goal
// Phase 2: goal  → start
// ===========================================================================
void floodfillExplore(MouseState& state) {
    LOG("=== RUN 1: Floodfill Explore (start->goal->start) ===");

    visited[state.x][state.y] = true;

    // Phase 1: navigate to goal
    LOG("  Phase 1: start -> goal...");
    bool ok = floodfillPhase(state, GOAL_X, GOAL_Y, NUM_GOALS, "Phase1");
    if (!ok) { LOG("  Could not reach goal. Maze disconnected?"); return; }
    LOGF("  Goal reached at (%d,%d). Coverage: %.1f%%\n",
         state.x, state.y, coverage());

    // Phase 2: navigate back to start
    LOG("  Phase 2: goal -> start...");
    const uint8_t startX[1] = {0};
    const uint8_t startY[1] = {0};
    ok = floodfillPhase(state, startX, startY, 1, "Phase2");
    if (!ok) { LOG("  Could not return to start."); return; }

    LOG("=== FLOODFILL EXPLORATION COMPLETE ===");
    LOGF("    Coverage: %.1f%%  Mouse at (%d,%d)\n", coverage(), state.x, state.y);
}

// ===========================================================================
// A* PLANNER
// Plans shortest path from (sx,sy) to any goal cell using flood_dist as
// the heuristic h(n).  Returns path as vector of {x,y} pairs, start
// exclusive, goal inclusive.  Empty vector = no path found.
//
// C++ translation notes:
//   Python heapq    → std::priority_queue (min-heap via greater<>)
//   Python dict     → flat arrays indexed by x*HEIGHT+y (256 cells max)
//   Python set      → bool closed[16][16]
// ===========================================================================
struct AStarNode {
    int16_t f, g;
    uint8_t x, y;
    bool operator>(const AStarNode& o) const { return f > o.f; }
};

struct Cell { uint8_t x, y; };

static std::vector<Cell> astarPlan(uint8_t sx, uint8_t sy) {
    // g_score and came_from indexed as [x][y]
    int16_t g_score[MAZE_WIDTH][MAZE_HEIGHT];
    uint8_t from_x[MAZE_WIDTH][MAZE_HEIGHT];
    uint8_t from_y[MAZE_WIDTH][MAZE_HEIGHT];
    bool    closed[MAZE_WIDTH][MAZE_HEIGHT];

    for (int x = 0; x < MAZE_WIDTH; x++)
        for (int y = 0; y < MAZE_HEIGHT; y++) {
            g_score[x][y] = INF_DIST;
            closed[x][y]  = false;
            from_x[x][y]  = 255;   // sentinel = no parent
            from_y[x][y]  = 255;
        }

    g_score[sx][sy] = 0;

    std::priority_queue<AStarNode, std::vector<AStarNode>, std::greater<AStarNode>> open;
    open.push({flood_dist[sx][sy], 0, sx, sy});

    while (!open.empty()) {
        AStarNode cur = open.top(); open.pop();
        uint8_t x = cur.x, y = cur.y;

        if (closed[x][y]) continue;
        closed[x][y] = true;

        // Goal reached — reconstruct path
        if (isGoal(x, y)) {
            std::vector<Cell> path;
            uint8_t cx = x, cy = y;
            while (!(cx == sx && cy == sy)) {
                path.push_back({cx, cy});
                uint8_t px = from_x[cx][cy];
                uint8_t py = from_y[cx][cy];
                cx = px; cy = py;
            }
            // Reverse so path goes from start to goal
            for (int i = 0, j = path.size()-1; i < j; i++, j--)
                std::swap(path[i], path[j]);
            LOGF("  A* path found: %d steps\n", (int)path.size());
            return path;
        }

        // Expand neighbours
        for (int d = 0; d < 4; d++) {
            if (hasWall(x, y, d)) continue;
            int8_t nx = x + DX[d];
            int8_t ny = y + DY[d];
            if (nx < 0 || nx >= MAZE_WIDTH || ny < 0 || ny >= MAZE_HEIGHT) continue;
            if (closed[nx][ny]) continue;

            int16_t new_g = g_score[x][y] + 1;
            if (new_g < g_score[nx][ny]) {
                g_score[nx][ny] = new_g;
                from_x[nx][ny]  = x;
                from_y[nx][ny]  = y;
                int16_t f = new_g + flood_dist[nx][ny];
                open.push({f, new_g, (uint8_t)nx, (uint8_t)ny});
            }
        }
    }

    LOG("  A* found no path!");
    return {};
}

// ===========================================================================
// RUN 2 — A* SPEED RUN WITH LIVE REPLANNING
//
// Plans the full route, then follows it step by step.
// Before every move, senses walls. If a new wall blocks the planned path,
// rebuilds flood_dist and replans from the current position.
// This prevents any crash from a wall missed during exploration.
// ===========================================================================
void astarRun(MouseState& state) {
    LOG("=== RUN 2: A* Speed Run (flood-fill heuristic, live replanning) ===");

    // Plan initial path
    LOG("  Planning initial path...");
    std::vector<Cell> path = astarPlan(state.x, state.y);
    if (path.empty()) { LOG("  No path found. Aborting."); return; }
    LOGF("  Path: %d steps. Starting run...\n", (int)path.size());

    // Build a quick lookup: is cell (x,y) on the current planned path?
    // Use a bool grid — O(1) lookup, no heap allocation per step.
    bool pathSet[MAZE_WIDTH][MAZE_HEIGHT];
    auto rebuildPathSet = [&]() {
        memset(pathSet, 0, sizeof(pathSet));
        for (auto& c : path) pathSet[c.x][c.y] = true;
    };
    rebuildPathSet();

    int totalSteps = 0;

    while (!isGoal(state.x, state.y)) {
        if (path.empty()) {
            LOG("  Path exhausted — replanning...");
            path = astarPlan(state.x, state.y);
            if (path.empty()) { LOG("  No path. Aborting."); return; }
            rebuildPathSet();
        }

        // --- Sense walls BEFORE moving ---
        uint8_t x = state.x, y = state.y;
        bool anyNew = false;
        bool readings[3] = { wallFront(), wallRight(), wallLeft() };
        uint8_t relDirs[3] = { 0, 1, 2 };
        for (int i = 0; i < 3; i++) {
            if (readings[i]) {
                uint8_t dir = absDir(state.facing, relDirs[i]);
                if (recordWall(x, y, dir)) anyNew = true;
            }
        }

        if (anyNew) {
            // Check if any newly blocked neighbour is on the planned path
            bool blockedOnPath = false;
            for (int d = 0; d < 4; d++) {
                if (!hasWall(x, y, d)) continue;
                int8_t nx = x + DX[d];
                int8_t ny = y + DY[d];
                if (nx < 0 || nx >= MAZE_WIDTH || ny < 0 || ny >= MAZE_HEIGHT) continue;
                if (pathSet[nx][ny]) { blockedOnPath = true; break; }
            }
            if (blockedOnPath) {
                LOGF("  New wall near (%d,%d) blocks path — replanning...\n", x, y);
                buildFloodFill();   // refresh heuristic with updated wall map
                path = astarPlan(x, y);
                if (path.empty()) { LOG("  No path after replan. Aborting."); return; }
                rebuildPathSet();
                LOGF("  Replanned: %d steps remaining.\n", (int)path.size());
            }
        }

        // --- Move to next waypoint ---
        Cell next = path.front();
        path.erase(path.begin());
        moveToCell(next.x, next.y, state);
        totalSteps++;

        if (isGoal(state.x, state.y)) {
            LOGF("  Goal reached at (%d,%d) in %d steps!\n",
                 state.x, state.y, totalSteps);
        }
    }

    LOG("=== A* RUN COMPLETE ===");
}

// ===========================================================================
// RESET HANDLER
// Preserves walls[][] and flood_dist[][] — keeps everything learned.
// Resets only logical position so Run 2 starts from (0,0).
// ===========================================================================
void handleReset(MouseState& state) {
    LOG("Reset detected. Preserving wall map.");
    ackReset();
    state.x      = 0;
    state.y      = 0;
    state.facing = NORTH;
}