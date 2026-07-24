#ifndef CELL_TRACKER_H
#define CELL_TRACKER_H

#include "ekf.h"

// ==========================================
// CELL TRACKER
// Translates EKF continuous pose (x_mm, y_mm,
// theta_rad) into discrete maze cells (row, col)
// and absolute heading (N/E/S/W).
//
// Coordinate convention:
//   row  → increases along EKF x_mm (forward)
//   col  → increases along EKF y_mm (rightward drift)
//   (0,0) = starting cell
// ==========================================

// ==========================================
// CONSTANTS
// ==========================================
const float CELL_SIZE_MM        = 155.0f;  // Change to 155.0f if your physical maze uses smaller cells
const float CELL_ENTRY_MARGIN   = 20.0f;   // must be this far past a cell boundary
                                            // before a transition is confirmed
                                            // (prevents false triggers near boundaries)

// ==========================================
// HEADING ENUM
// ==========================================
enum MazeHeading {
  HEADING_NORTH = 0,   // robot's initial forward direction
  HEADING_EAST  = 1,   // turned right once
  HEADING_SOUTH = 2,   // turned around
  HEADING_WEST  = 3    // turned left once
};

// ==========================================
// CELL TRACKER STATE
// ==========================================
static int  ct_currentRow     = 0;
static int  ct_currentCol     = 0;
static int  ct_previousRow    = 0;
static int  ct_previousCol    = 0;
static MazeHeading ct_heading = HEADING_NORTH;
static bool ct_cellJustEntered = false;   // true for ONE update cycle when
                                          // robot crosses into a new cell

// ==========================================
// HEADING HELPERS
// ==========================================

// Convert EKF theta (radians, wrapped -PI..PI)
// to the nearest cardinal heading.
// At start, theta=0 means facing North.
inline MazeHeading headingFromTheta(float theta_rad) {
  float deg = ekfRadToDeg(theta_rad);

  // Normalise to 0..360
  while (deg <    0.0f) deg += 360.0f;
  while (deg >= 360.0f) deg -= 360.0f;

  // Each quadrant is 90° wide, centred on 0/90/180/270
  if (deg <  45.0f || deg >= 315.0f) return HEADING_NORTH;
  if (deg <  135.0f)                 return HEADING_WEST;
  if (deg <  225.0f)                 return HEADING_SOUTH;
  return HEADING_EAST;
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

// ==========================================
// INIT
// Call once after ekfInit() in setup().
// ==========================================
void initCellTracker() {
  ct_currentRow      = 0;
  ct_currentCol      = 0;
  ct_previousRow     = 0;
  ct_previousCol     = 0;
  ct_heading         = HEADING_NORTH;
  ct_cellJustEntered = false;
}

// ==========================================
// UPDATE — call every control loop tick
// (inside runControlLoop, after ekfPredict
// and ekfUpdateYawDeg have already run)
//
// Returns true if robot just entered a new cell.
// The caller should read walls immediately when
// this returns true.
// ==========================================
bool updateCellTracker() {
  EKFState s = ekfGetState();

  // --- Compute cell indices from EKF pose ---
  // We use (pos + margin) / cell_size so the
  // transition fires slightly inside the new cell,
  // not right on the boundary edge.
  int newRow = (int)floorf(s.x_mm / CELL_SIZE_MM);
  int newCol = (int)roundf(-s.y_mm / CELL_SIZE_MM); // Changed to roundf
  // Note: col is centred so that small lateral
  // drift near zero doesn't flip the column.

  // --- Update heading from theta ---
  ct_heading = headingFromTheta(s.theta_rad);

  // --- Detect cell transition ---
  ct_cellJustEntered = false;

  if (newRow != ct_currentRow || newCol != ct_currentCol) {
    // Confirm the robot is genuinely INSIDE the new cell
    // (not just grazing the boundary)
    float posInCellX = fmodf(fabsf(s.x_mm), CELL_SIZE_MM);
    if (posInCellX > CELL_ENTRY_MARGIN || newRow != ct_currentRow) {
      ct_previousRow     = ct_currentRow;
      ct_previousCol     = ct_currentCol;
      ct_currentRow      = newRow;
      ct_currentCol      = newCol;
      ct_cellJustEntered = true;
    }
  }

  return ct_cellJustEntered;
}

// ==========================================
// GETTERS
// ==========================================
inline int         ctGetRow()     { return ct_currentRow;  }
inline int         ctGetCol()     { return ct_currentCol;  }
inline MazeHeading ctGetHeading() { return ct_heading;     }
inline int         ctGetPrevRow() { return ct_previousRow; }
inline int         ctGetPrevCol() { return ct_previousCol; }

// ==========================================
// SERIAL DEBUG
// ==========================================
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