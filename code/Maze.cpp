#include "Maze.h"
#include <string.h>

// ===========================================================================
// DIRECTION TABLES
// ===========================================================================
const int8_t DX[4]      = { 0,  1,  0, -1 };  // N E S W
const int8_t DY[4]      = { 1,  0, -1,  0 };
const uint8_t OPPOSITE[4] = { SOUTH, WEST, NORTH, EAST };

// ===========================================================================
// GOAL CELLS  (centre 2×2 of a 16×16 maze)
// ===========================================================================
const uint8_t GOAL_X[NUM_GOALS] = { 7, 8, 7, 8 };
const uint8_t GOAL_Y[NUM_GOALS] = { 7, 7, 8, 8 };

// ===========================================================================
// GLOBAL MAZE ARRAYS
// ===========================================================================
uint8_t  walls[MAZE_WIDTH][MAZE_HEIGHT];
bool     visited[MAZE_WIDTH][MAZE_HEIGHT];
int16_t  flood_dist[MAZE_WIDTH][MAZE_HEIGHT];

// ===========================================================================
// INIT
// ===========================================================================
void initMaze() {
    memset(walls,   0, sizeof(walls));
    memset(visited, 0, sizeof(visited));
    for (int x = 0; x < MAZE_WIDTH;  x++)
        for (int y = 0; y < MAZE_HEIGHT; y++)
            flood_dist[x][y] = INF_DIST;
}

void initBorderWalls() {
    for (int x = 0; x < MAZE_WIDTH; x++) {
        for (int y = 0; y < MAZE_HEIGHT; y++) {
            if (y == MAZE_HEIGHT - 1) setWallBit(x, y, NORTH);
            if (y == 0)               setWallBit(x, y, SOUTH);
            if (x == MAZE_WIDTH - 1)  setWallBit(x, y, EAST);
            if (x == 0)               setWallBit(x, y, WEST);
        }
    }
}

// ===========================================================================
// WALL RECORDING
// Stores the wall on both sides of the shared boundary.
// Returns true if this was new information.
// ===========================================================================
bool recordWall(uint8_t x, uint8_t y, uint8_t dir) {
    if (hasWall(x, y, dir)) return false;   // already knew this

    setWallBit(x, y, dir);

    // Mirror on the neighbouring cell
    int8_t nx = x + DX[dir];
    int8_t ny = y + DY[dir];
    if (nx >= 0 && nx < MAZE_WIDTH && ny >= 0 && ny < MAZE_HEIGHT) {
        setWallBit(nx, ny, OPPOSITE[dir]);
    }
    return true;   // new wall
}

// ===========================================================================
// GOAL TEST
// ===========================================================================
bool isGoal(uint8_t x, uint8_t y) {
    for (int i = 0; i < NUM_GOALS; i++)
        if (GOAL_X[i] == x && GOAL_Y[i] == y) return true;
    return false;
}

// ===========================================================================
// COVERAGE  (% of cells physically visited)
// ===========================================================================
float coverage() {
    int n = 0;
    for (int x = 0; x < MAZE_WIDTH;  x++)
        for (int y = 0; y < MAZE_HEIGHT; y++)
            if (visited[x][y]) n++;
    return (float)n / (MAZE_WIDTH * MAZE_HEIGHT) * 100.0f;
}

// ===========================================================================
// BFS FLOOD FILL — generic, writes into caller-supplied out_dist table
// Multi-source: seeds simultaneously from all target cells.
// Expands only through wall-free passages in the current walls[][] map.
// ===========================================================================
void buildLiveFlood(const uint8_t targX[], const uint8_t targY[],
                    uint8_t targCount,
                    int16_t out_dist[MAZE_WIDTH][MAZE_HEIGHT])
{
    // Reset output table
    for (int x = 0; x < MAZE_WIDTH;  x++)
        for (int y = 0; y < MAZE_HEIGHT; y++)
            out_dist[x][y] = INF_DIST;

    // Simple BFS queue — max cells = 16×16 = 256, fits easily in a small array
    uint8_t qx[MAZE_WIDTH * MAZE_HEIGHT];
    uint8_t qy[MAZE_WIDTH * MAZE_HEIGHT];
    int head = 0, tail = 0;

    // Seed all target cells with distance 0
    for (int i = 0; i < targCount; i++) {
        out_dist[targX[i]][targY[i]] = 0;
        qx[tail] = targX[i];
        qy[tail] = targY[i];
        tail++;
    }

    while (head != tail) {
        uint8_t x = qx[head];
        uint8_t y = qy[head];
        head++;

        for (int d = 0; d < 4; d++) {
            if (hasWall(x, y, d)) continue;
            int8_t nx = x + DX[d];
            int8_t ny = y + DY[d];
            if (nx < 0 || nx >= MAZE_WIDTH || ny < 0 || ny >= MAZE_HEIGHT) continue;
            int16_t newDist = out_dist[x][y] + 1;
            if (newDist < out_dist[nx][ny]) {
                out_dist[nx][ny] = newDist;
                qx[tail] = nx;
                qy[tail] = ny;
                tail++;
            }
        }
    }
}

// ===========================================================================
// BUILD FINAL FLOOD FILL
// Writes into the global flood_dist[][] used as A* heuristic in Run 2.
// Called once after exploration is complete.
// ===========================================================================
void buildFloodFill() {
    Serial.println("  Building flood-fill heuristic table...");
    buildLiveFlood(GOAL_X, GOAL_Y, NUM_GOALS, flood_dist);

    int reachable = 0;
    for (int x = 0; x < MAZE_WIDTH;  x++)
        for (int y = 0; y < MAZE_HEIGHT; y++)
            if (flood_dist[x][y] < INF_DIST) reachable++;

    Serial.print("  Flood-fill done. Reachable: ");
    Serial.print(reachable);
    Serial.print("/");
    Serial.print(MAZE_WIDTH * MAZE_HEIGHT);
    Serial.print("  Start(0,0)->goal: ");
    Serial.println(flood_dist[0][0]);
}