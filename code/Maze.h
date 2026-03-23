#ifndef MAZE_H
#define MAZE_H

#include <Arduino.h>
#include <stdint.h>

// ===========================================================================
// MAZE DIMENSIONS & GOAL
// ===========================================================================
#define MAZE_WIDTH  16
#define MAZE_HEIGHT 16
#define INF_DIST    30000   // Sentinel for unreachable cells (fits int16_t)

// Direction constants — indices into DX[], DY[], bit positions in walls[][]
#define NORTH 0
#define EAST  1
#define SOUTH 2
#define WEST  3

// Precomputed direction deltas
extern const int8_t DX[4];   // x-change for N/E/S/W
extern const int8_t DY[4];   // y-change for N/E/S/W
extern const uint8_t OPPOSITE[4];

// ===========================================================================
// MAZE STATE
// ===========================================================================
//
// walls[x][y] — bitmask of confirmed walls at cell (x,y).
//   Bit 0 = NORTH wall, Bit 1 = EAST, Bit 2 = SOUTH, Bit 3 = WEST
//   Use hasWall(x,y,dir) / setWallBit(x,y,dir) rather than raw bit ops.
//
// visited[x][y] — true once the mouse has physically stood on that cell.
//
// flood_dist[x][y] — wall-aware BFS distance to nearest goal cell.
//   Populated by buildFloodFill() after exploration. INF_DIST until then.
//
extern uint8_t  walls[MAZE_WIDTH][MAZE_HEIGHT];
extern bool     visited[MAZE_WIDTH][MAZE_HEIGHT];
extern int16_t  flood_dist[MAZE_WIDTH][MAZE_HEIGHT];

// Goal cells: centre 2×2 block of a 16×16 maze = (7,7),(8,7),(7,8),(8,8)
#define NUM_GOALS 4
extern const uint8_t GOAL_X[NUM_GOALS];
extern const uint8_t GOAL_Y[NUM_GOALS];

// ===========================================================================
// WALL BIT HELPERS
// ===========================================================================
inline bool hasWall(uint8_t x, uint8_t y, uint8_t dir) {
    return (walls[x][y] >> dir) & 1;
}
inline void setWallBit(uint8_t x, uint8_t y, uint8_t dir) {
    walls[x][y] |= (1 << dir);
}

// ===========================================================================
// FUNCTION DECLARATIONS
// ===========================================================================
void    initMaze();
void    initBorderWalls();
bool    recordWall(uint8_t x, uint8_t y, uint8_t dir);   // returns true if NEW
bool    isGoal(uint8_t x, uint8_t y);
float   coverage();

// BFS from a set of target cells — used during live exploration
// targets: array of {x,y} pairs, count: number of targets
// result written into out_dist[MAZE_WIDTH][MAZE_HEIGHT]
void    buildLiveFlood(const uint8_t targX[], const uint8_t targY[],
                       uint8_t targCount,
                       int16_t out_dist[MAZE_WIDTH][MAZE_HEIGHT]);

// BFS from GOAL_CELLS — writes into global flood_dist[][]
// Called once after exploration is complete
void    buildFloodFill();

#endif // MAZE_H