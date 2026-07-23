#ifndef WALL_MAP_H
#define WALL_MAP_H

#include "Sensors.h"
#include "CellTracker.h"

// ==========================================
// WALL MAP
// Stores which walls exist at every cell in
// a 16x16 maze. Walls are expressed in absolute
// cardinal directions (N/S/E/W), not relative
// to the robot.
//
// A wall value means:
//   true  = wall confirmed present
//   false = open / not yet confirmed
//
// Boundary walls (edges of maze) are pre-set
// to true in initWallMap().
// ==========================================

const int MAZE_SIZE = 6;

// ==========================================
// WALL DETECTION THRESHOLD
// Lidar reading below this → wall present.
// Tune to your maze corridor width.
// ==========================================
const int WALL_DETECT_MM = 110;

// ==========================================
// DATA STRUCTURES
// ==========================================
struct CellWalls {
  bool north;   // wall on north side of this cell
  bool south;
  bool east;
  bool west;
  bool visited; // has robot been here?
};

static CellWalls wallMap[MAZE_SIZE][MAZE_SIZE];

// ==========================================
// INIT
// Clears all walls to unknown (false),
// marks all cells unvisited, then pre-sets
// the outer boundary as solid walls.
// ==========================================
void initWallMap() {
  for (int r = 0; r < MAZE_SIZE; r++) {
    for (int c = 0; c < MAZE_SIZE; c++) {
      wallMap[r][c].north   = false;
      wallMap[r][c].south   = false;
      wallMap[r][c].east    = false;
      wallMap[r][c].west    = false;
      wallMap[r][c].visited = false;
    }
  }

  // Pre-set outer boundary walls
  for (int i = 0; i < MAZE_SIZE; i++) {
    wallMap[0][i].south          = true;  // bottom row, south edge
    wallMap[MAZE_SIZE-1][i].north = true;  // top row, north edge
    wallMap[i][0].west            = true;  // left col, west edge
    wallMap[i][MAZE_SIZE-1].east  = true;  // right col, east edge
  }

  // Starting cell always has a south wall (the back wall behind start)
  wallMap[0][0].south = true;
}

// ==========================================
// WALL CONSISTENCY ENFORCER
// When a wall is recorded on one side of a
// boundary, the neighbour cell's matching side
// must also reflect it. This keeps the map
// self-consistent for floodfill.
// ==========================================
void syncNeighbourWalls(int row, int col) {
  // North wall of (row,col) = South wall of (row+1,col)
  if (row + 1 < MAZE_SIZE)
    wallMap[row+1][col].south = wallMap[row][col].north;

  // South wall of (row,col) = North wall of (row-1,col)
  if (row - 1 >= 0)
    wallMap[row-1][col].north = wallMap[row][col].south;

  // East wall of (row,col) = West wall of (row,col+1)
  if (col + 1 < MAZE_SIZE)
    wallMap[row][col+1].west = wallMap[row][col].east;

  // West wall of (row,col) = East wall of (row,col-1)
  if (col - 1 >= 0)
    wallMap[row][col-1].east = wallMap[row][col].west;
}

// ==========================================
// HEADING → WALL MAPPING TABLE
//
// Physical sensor → absolute wall direction
// depends on which way the robot is facing.
//
//  Facing NORTH:  front=N,  left=W,  right=E
//  Facing EAST:   front=E,  left=N,  right=S
//  Facing SOUTH:  front=S,  left=E,  right=W
//  Facing WEST:   front=W,  left=S,  right=N
// ==========================================
struct WallMapping {
  bool* frontWall;
  bool* leftWall;
  bool* rightWall;
};

WallMapping getWallMapping(int row, int col, MazeHeading heading) {
  WallMapping m;
  switch (heading) {
    case HEADING_NORTH:
      m.frontWall = &wallMap[row][col].north;
      m.leftWall  = &wallMap[row][col].west;
      m.rightWall = &wallMap[row][col].east;
      break;
    case HEADING_EAST:
      m.frontWall = &wallMap[row][col].east;
      m.leftWall  = &wallMap[row][col].north;
      m.rightWall = &wallMap[row][col].south;
      break;
    case HEADING_SOUTH:
      m.frontWall = &wallMap[row][col].south;
      m.leftWall  = &wallMap[row][col].east;
      m.rightWall = &wallMap[row][col].west;
      break;
    case HEADING_WEST:
    default:
      m.frontWall = &wallMap[row][col].west;
      m.leftWall  = &wallMap[row][col].south;
      m.rightWall = &wallMap[row][col].north;
      break;
  }
  return m;
}

// ==========================================
// RECORD WALLS
// Call this immediately when the cell tracker
// fires ct_cellJustEntered = true.
//
// Reads current lidar data and stamps walls
// into wallMap at the given cell.
// ==========================================
void recordWalls(int row, int col, MazeHeading heading, DistanceData lidar) {

  // Clamp to maze bounds (safety)
  if (row < 0 || row >= MAZE_SIZE) return;
  if (col < 0 || col >= MAZE_SIZE) return;

  WallMapping m = getWallMapping(row, col, heading);

  // Write wall states based on lidar readings
  *m.frontWall = (lidar.front < WALL_DETECT_MM);
  *m.leftWall  = (lidar.left  < WALL_DETECT_MM);
  *m.rightWall = (lidar.right < WALL_DETECT_MM);

  // Mark cell visited
  wallMap[row][col].visited = true;

  // Keep neighbour cells consistent
  syncNeighbourWalls(row, col);
}

// ==========================================
// CONVENIENCE QUERY FUNCTIONS
// For use by the maze algorithm.
// ==========================================
inline bool hasWallNorth(int row, int col) { return wallMap[row][col].north; }
inline bool hasWallSouth(int row, int col) { return wallMap[row][col].south; }
inline bool hasWallEast (int row, int col) { return wallMap[row][col].east;  }
inline bool hasWallWest (int row, int col) { return wallMap[row][col].west;  }
inline bool cellVisited (int row, int col) { return wallMap[row][col].visited; }

// Can the robot move from (row,col) in the given heading?
bool canMove(int row, int col, MazeHeading heading) {
  switch (heading) {
    case HEADING_NORTH: return !wallMap[row][col].north && (row + 1 < MAZE_SIZE);
    case HEADING_EAST:  return !wallMap[row][col].east  && (col + 1 < MAZE_SIZE);
    case HEADING_SOUTH: return !wallMap[row][col].south && (row - 1 >= 0);
    case HEADING_WEST:  return !wallMap[row][col].west  && (col - 1 >= 0);
    default:            return false;
  }
}

// ==========================================
// SERIAL DEBUG — print one cell's walls
// ==========================================
void printCellWalls(int row, int col) {
  if (row < 0 || row >= MAZE_SIZE || col < 0 || col >= MAZE_SIZE) return;
  Serial.printf(
    "WALLS (%d,%d) visited=%d | N=%d E=%d S=%d W=%d\n",
    row, col,
    wallMap[row][col].visited,
    wallMap[row][col].north,
    wallMap[row][col].east,
    wallMap[row][col].south,
    wallMap[row][col].west
  );
}

// ==========================================
// SERIAL DEBUG — dump the visited portion
// of the map as a simple ASCII grid
// ==========================================
void printWallMapASCII() {
  Serial.println("\n--- WALL MAP (visited cells) ---");
  for (int r = MAZE_SIZE - 1; r >= 0; r--) {
    for (int c = 0; c < MAZE_SIZE; c++) {
      if (!wallMap[r][c].visited) {
        Serial.print(" .  ");
      } else {
        char buf[5];
        snprintf(buf, sizeof(buf), "%c%c%c%c",
          wallMap[r][c].north ? 'N' : '-',
          wallMap[r][c].east  ? 'E' : '-',
          wallMap[r][c].south ? 'S' : '-',
          wallMap[r][c].west  ? 'W' : '-'
        );
        Serial.print(buf);
        Serial.print(" ");
      }
    }
    Serial.println();
  }
  Serial.println("--------------------------------\n");
}

#endif
