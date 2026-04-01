#ifndef MAZE_H
#define MAZE_H

#include <Arduino.h>
#include "Sensors.h"

// ==========================================
// MAZE CONSTANTS & BITMASKS
// ==========================================
const int MAZE_SIZE = 5;

#define WALL_NORTH 1  // 0000 0001
#define WALL_EAST  2  // 0000 0010
#define WALL_SOUTH 4  // 0000 0100
#define WALL_WEST  8  // 0000 1000
#define VISITED    16 // 0001 0000

// Compass directions mapped to 0, 1, 2, 3
enum CompassDir {
  NORTH = 0,
  EAST  = 1,
  SOUTH = 2,
  WEST  = 3
};

// ==========================================
// LOCALIZATION STATE
// ==========================================
byte maze[MAZE_SIZE][MAZE_SIZE]; // The 16x16 map

int currentX = 0; // Starts at bottom-left (or top-left depending on your physical setup)
int currentY = 0; 

// ==========================================
// COMPASS HELPER
// ==========================================
// Translates your continuous unwrapped yaw into a clean 0-3 direction.
CompassDir getAbsoluteDirection(float currentYaw) {
  // 1. Divide by 90 and round to nearest integer to get the 'tick'
  int headingIndex = (int)round(currentYaw / 90.0);
  
  // 2. Safely wrap negatives to a positive 0-3 range
  int compassVal = (headingIndex % 4 + 4) % 4; 
  
  // NOTE: Depending on your BNO055 mounting, turning Right (East) might 
  // decrease your yaw instead of increasing it. 
  // If 0 is North and +90 is West, you will need to map these accordingly.
  // Assuming standard mathematical (ACW is positive): 0=N, 1=W, 2=S, 3=E
  // Assuming standard compass (CW is positive): 0=N, 1=E, 2=S, 3=W
  
  // Let's assume standard compass for this switch statement:
  switch (compassVal) {
    case 0: return NORTH;
    case 1: return EAST; // Change to WEST if your bot turns left to get positive degrees
    case 2: return SOUTH;
    case 3: return WEST; // Change to EAST if your bot turns left to get positive degrees
  }
  return NORTH;
}

// ==========================================
// MAPPING FUNCTION
// ==========================================
// Call this exactly once every time the robot enters a new 180mm cell.
void updateMap(DistanceData lidars, float currentYaw) {
  
  CompassDir facing = getAbsoluteDirection(currentYaw);
  
  // Mark the current cell as visited
  maze[currentX][currentY] |= VISITED;

  // Determine wall presence from LiDARs
  bool hasFront = (lidars.front < WALL_THRESHOLD);
  bool hasLeft  = (lidars.left  < WALL_THRESHOLD);
  bool hasRight = (lidars.right < WALL_THRESHOLD);

  // Translate relative walls (Front/Left/Right) to Absolute walls (N/E/S/W)
  // based on the direction the robot is currently facing.
  if (facing == NORTH) {
    if (hasFront) maze[currentX][currentY] |= WALL_NORTH;
    if (hasLeft)  maze[currentX][currentY] |= WALL_WEST;
    if (hasRight) maze[currentX][currentY] |= WALL_EAST;
    
    // The wall behind us is always open if we just drove in, 
    // but if you want to be safe, you can log it based on previous cell data.
  } 
  else if (facing == EAST) {
    if (hasFront) maze[currentX][currentY] |= WALL_EAST;
    if (hasLeft)  maze[currentX][currentY] |= WALL_NORTH;
    if (hasRight) maze[currentX][currentY] |= WALL_SOUTH;
  } 
  else if (facing == SOUTH) {
    if (hasFront) maze[currentX][currentY] |= WALL_SOUTH;
    if (hasLeft)  maze[currentX][currentY] |= WALL_EAST;
    if (hasRight) maze[currentX][currentY] |= WALL_WEST;
  } 
  else if (facing == WEST) {
    if (hasFront) maze[currentX][currentY] |= WALL_WEST;
    if (hasLeft)  maze[currentX][currentY] |= WALL_SOUTH;
    if (hasRight) maze[currentX][currentY] |= WALL_NORTH;
  }

  // NOTE: In a complete implementation, you should also update the cell 
  // adjacent to the wall you just detected so they share the data. 
  // e.g., If I detect a North wall at (0,0), cell (0,1) gets a South wall.
}

// ==========================================
// POSITION TRACKER
// ==========================================
// Call this function when the robot completes a forward move to the next cell
void moveToNextCell() {
  CompassDir facing = getAbsoluteDirection(readYawDegrees());
  
  if (facing == NORTH) currentY++;
  if (facing == SOUTH) currentY--;
  if (facing == EAST)  currentX++;
  if (facing == WEST)  currentX--;
}

#endif