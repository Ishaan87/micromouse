#include <Arduino.h>
#include "API.h"
#include "Maze.h"
#include "Algorithm.h"

// ===========================================================================
// MICROMOUSE — MAIN
//
// File structure:
//   Config.h       — pin definitions (unchanged from your original)
//   Motors.h/.cpp  — setMotorSpeeds(), stopMotors() (unchanged)
//   Encoders.h     — encoder ISRs, leftTicks/rightTicks (unchanged)
//   Sensors.h      — readLidars(), readGyroHeading() (unchanged)
//   Maze.h/.cpp    — walls[][], flood_dist[][], BFS flood fill
//   API.h/.cpp     — hardware implementation of moveForward(), turnLeft(),
//                    turnRight(), wallFront/Left/Right(), wasReset()
//   Algorithm.h/.cpp — floodfillExplore(), astarRun(), handleReset()
//   main.cpp       — this file: setup() + loop() state machine
//
// OPERATION:
//   Power on → calibrates gyro → waits 2 s → Run 1 starts automatically.
//   Run 1: mouse navigates start→goal→start using live floodfill.
//   After Run 1: press the reset button to start Run 2.
//   Run 2: A* speed run using flood_dist heuristic, with live replanning
//          if any previously unseen wall is detected mid-run.
//   After Run 2: press reset to run again (wall map is preserved).
// ===========================================================================

MouseState mouseState;
int        runNumber = 0;

void setup() {
    Serial.begin(115200);
    delay(500);   // let serial monitor connect

    Serial.println("=== Micromouse: Floodfill Explore + A* Speed Run ===");
    Serial.println("Initialising hardware...");

    // Initialise all hardware: motors, encoders, sensors, gyro calibration
    initAPI();

    // Initialise maze data structures and border walls
    initMaze();
    initBorderWalls();

    // Starting position and heading
    mouseState = {0, 0, NORTH};
    visited[0][0] = true;

    Serial.println("Ready. Starting Run 1 in 2 seconds...");
    delay(2000);
}

void loop() {
    // Check for reset button press at any time
    if (wasReset()) {
        handleReset(mouseState);
        // Small debounce — wait for button release
        while (wasReset()) delay(10);
        delay(200);
        return;
    }

    runNumber++;
    Serial.println("=======================================================");
    Serial.print("RUN "); Serial.println(runNumber);

    if (runNumber == 1) {
        // ── Run 1: floodfill exploration ─────────────────────────────────
        floodfillExplore(mouseState);

        // Build the final flood_dist heuristic table from the completed map
        buildFloodFill();

        Serial.println();
        Serial.println("Exploration complete. Press Reset button for speed run.");
        Serial.print("Map coverage: ");
        Serial.print(coverage(), 1);
        Serial.println("%");

        // Wait for reset button before continuing to Run 2
        while (!wasReset()) delay(10);
        handleReset(mouseState);
        while (wasReset()) delay(10);   // wait for release
        delay(200);

    } else {
        // ── Run 2+: A* speed run ─────────────────────────────────────────
        astarRun(mouseState);

        Serial.println();
        Serial.println("Speed run complete. Press Reset to run again.");

        while (!wasReset()) delay(10);
        handleReset(mouseState);
        while (wasReset()) delay(10);
        delay(200);
    }
}