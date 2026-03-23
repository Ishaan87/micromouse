#ifndef API_H
#define API_H

#include <Arduino.h>
#include "Maze.h"

// ===========================================================================
// RESET BUTTON
// Connect a push button between this pin and GND.
// Internal pull-up is enabled — button press reads LOW.
// ===========================================================================
#define RESET_BUTTON_PIN 20

// ===========================================================================
// MOVEMENT TUNING
// Adjust these to match your physical mouse.
//
// ENCODER MATH:
//   N20 30:1 gearbox, 7 PPR on motor shaft
//   → 7 × 30 = 210 pulses per wheel revolution
//   Wheel diameter = 43mm → circumference = π × 43 = 135.09mm
//   Cell size = 180mm
//   → pulses per cell = (180 / 135.09) × 210 = 280 pulses
//
// TURN MATH:
//   Wheelbase (distance between wheel centres) — measure your robot.
//   Arc length for 90° turn = (π/2) × wheelbase
//   One wheel travels that arc while the other stays still (pivot turn),
//   or both wheels travel half that in opposite directions (point turn).
//   Adjust TARGET_TURN_DEG and TURN_TOLERANCE until 90° is accurate.
// ===========================================================================
#define PULSES_PER_CELL     280     // encoder pulses for one maze cell (180mm)
#define TARGET_TURN_DEG     90.0f   // degrees per 90° turn command
#define TURN_TOLERANCE      1.5f    // degrees — stop when within this of target
#define MOVE_BASE_SPEED     120     // PWM 0-255 for forward movement
#define TURN_BASE_SPEED     100     // PWM 0-255 for turning
#define WALL_THRESHOLD_MM   120     // lidar reading below this = wall present

// ===========================================================================
// API FUNCTION DECLARATIONS
// These mirror the MMS simulator API exactly.
// The algorithm calls these — never the hardware functions directly.
// ===========================================================================

// --- Maze size (hardcoded — no simulator to ask) ---
int  mazeWidth();
int  mazeHeight();

// --- Wall sensing (reads VL53L0X lidars) ---
bool wallFront();
bool wallLeft();
bool wallRight();

// --- Movement (encoder + IMU closed-loop, BLOCKING) ---
// Each function returns only after the physical movement is complete.
void moveForward();
void turnLeft();
void turnRight();

// --- Reset button ---
bool wasReset();
void ackReset();

// --- Display stubs (no-ops on hardware — MMS only) ---
inline void setWall(int x, int y, char dir)  { (void)x; (void)y; (void)dir; }
inline void clearWall(int x, int y, char dir){ (void)x; (void)y; (void)dir; }
inline void setColor(int x, int y, char col) { (void)x; (void)y; (void)col; }
inline void clearColor(int x, int y)         { (void)x; (void)y; }
inline void clearAllColor()                  { }
inline void setText(int x, int y, const char* t){ (void)x;(void)y;(void)t; }

// --- Hardware init (call once in setup(), before algorithm starts) ---
void initAPI();

#endif // API_H