#ifndef CELL_TRACKER_H
#define CELL_TRACKER_H

#include "ekf.h"

const float CELL_SIZE_MM        = 160.0f;  
const float CELL_ENTRY_MARGIN   = 20.0f;   

enum MazeHeading {
  HEADING_NORTH = 0,   
  HEADING_EAST  = 1,   
  HEADING_SOUTH = 2,   
  HEADING_WEST  = 3    
};

static int  ct_currentRow     = 0;
static int  ct_currentCol     = 0;
static int  ct_previousRow    = 0;
static int  ct_previousCol    = 0;
static MazeHeading ct_heading = HEADING_NORTH;
static bool ct_cellJustEntered = false;   

inline MazeHeading headingFromTheta(float theta_rad) {
  float deg = ekfRadToDeg(theta_rad);
  while (deg <    0.0f) deg += 360.0f;
  while (deg >= 360.0f) deg -= 360.0f;

  if (deg <  45.0f || deg >= 315.0f) return HEADING_NORTH;
  if (deg <  135.0f)                 return HEADING_EAST;
  if (deg <  225.0f)                 return HEADING_SOUTH;
  return HEADING_WEST;
}

inline const char* headingName(MazeHeading h) {
  switch (h) {
    case HEADING_NORTH: return "N";
    case HEADING_EAST:  return "E";
    case HEADING_SOUTH: return "S";
    case HEADING_WEST:  return "W";
    default:            return "?";
  }
}

void initCellTracker() {
  ct_currentRow      = 0;
  ct_currentCol      = 0;
  ct_previousRow     = 0;
  ct_previousCol     = 0;
  ct_heading         = HEADING_NORTH;
  ct_cellJustEntered = false;
}

bool updateCellTracker() {
  EKFState s = ekfGetState();

  // Unified floorf math ensures uniform 160x160 grid blocks
  int newRow = (int)floorf(s.x_mm / CELL_SIZE_MM);
  int newCol = (int)floorf(-s.y_mm / CELL_SIZE_MM); 

  ct_heading = headingFromTheta(s.theta_rad);
  ct_cellJustEntered = false;

  if (newRow != ct_currentRow || newCol != ct_currentCol) {
    // Get positive module wrapper for both axes
    float x_mod = fmodf(s.x_mm, CELL_SIZE_MM);
    if (x_mod < 0) x_mod += CELL_SIZE_MM;
    float y_mod = fmodf(-s.y_mm, CELL_SIZE_MM);
    if (y_mod < 0) y_mod += CELL_SIZE_MM;

    // Check margin relative to direction of travel
    bool pastMargin = false;
    if (ct_heading == HEADING_NORTH) pastMargin = (x_mod > CELL_ENTRY_MARGIN);
    else if (ct_heading == HEADING_SOUTH) pastMargin = (x_mod < (CELL_SIZE_MM - CELL_ENTRY_MARGIN));
    else if (ct_heading == HEADING_EAST) pastMargin = (y_mod > CELL_ENTRY_MARGIN);
    else if (ct_heading == HEADING_WEST) pastMargin = (y_mod < (CELL_SIZE_MM - CELL_ENTRY_MARGIN));

    if (pastMargin) {
      ct_previousRow     = ct_currentRow;
      ct_previousCol     = ct_currentCol;
      ct_currentRow      = newRow;
      ct_currentCol      = newCol;
      ct_cellJustEntered = true;
    }
  }

  return ct_cellJustEntered;
}

inline int         ctGetRow()     { return ct_currentRow;  }
inline int         ctGetCol()     { return ct_currentCol;  }
inline MazeHeading ctGetHeading() { return ct_heading;     }
inline int         ctGetPrevRow() { return ct_previousRow; }
inline int         ctGetPrevCol() { return ct_previousCol; }

void printCellState() {
  EKFState s = ekfGetState();
  Serial.printf(
    "CELL (%d,%d) heading=%s | EKF x=%.1f y=%.1f th=%.1f\n",
    ct_currentRow, ct_currentCol,
    headingName(ct_heading),
    s.x_mm, s.y_mm,
    ekfRadToDeg(s.theta_rad)
  );
}

#endif