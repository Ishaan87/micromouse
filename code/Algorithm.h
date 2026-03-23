#ifndef ALGORITHM_H
#define ALGORITHM_H

#include <Arduino.h>
#include "Maze.h"

// ===========================================================================
// MOUSE STATE
// Tracks logical position and heading inside the maze.
// (0,0) = bottom-left corner, facing NORTH at start.
// ===========================================================================
struct MouseState {
    uint8_t x;
    uint8_t y;
    uint8_t facing;   // NORTH / EAST / SOUTH / WEST
};

// ===========================================================================
// ALGORITHM ENTRY POINTS
// Called from main.cpp in sequence.
// ===========================================================================

// Run 1: floodfill-driven exploration (start→goal→start)
void floodfillExplore(MouseState& state);

// Run 2: A* speed run with live wall sensing and replanning
void astarRun(MouseState& state);

// Called when reset button is detected — preserves wall map, resets position
void handleReset(MouseState& state);

#endif // ALGORITHM_H