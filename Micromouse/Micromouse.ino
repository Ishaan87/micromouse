#include <Arduino.h>
#include <WiFi.h>
#include <WebServer.h>
#include <WebSocketsServer.h>
#include <stdarg.h>
#include "Config.h"
#include "Motors.h"
#include "Encoders.h"
#include "Sensors.h"
#include "ekf.h"
#include "CellTracker.h"
#include "WallMap.h"
#include "Turns.h"

// ==========================================
// TUNING PARAMETERS
// WIRELESS WEB SERVER SETTINGS
// ==========================================
const char *ssid     = "Jerry";
const char *password = "mouse1234";

WebServer         server(80);
WebSocketsServer  webSocket(81);

// ==========================================
// HTML DASHBOARD
// ==========================================
const char index_html[] PROGMEM = R"rawliteral(
<!DOCTYPE html>
<html lang="en">
<head>
    <meta charset="UTF-8">
    <meta name="viewport" content="width=device-width, initial-scale=1.0">
    <title>Micromouse Serial Log</title>
    <style>
        body { margin: 0; padding: 12px; background: #000; color: #fff; font-family: Consolas, monospace; }
        pre { margin: 0; white-space: pre-wrap; word-break: break-word; font-size: 14px; line-height: 1.35; }
    </style>
</head>
<body>
    <pre id="serial-log"></pre>
    <script>
        var gateway = `ws://${window.location.hostname}:81/`;
        var websocket;
        const maxLogLines = 250;
        window.addEventListener('load', initWebSocket);
        function appendLogLine(line) {
            const log = document.getElementById('serial-log');
            const lines = log.textContent === '' ? [] : log.textContent.split('\n');
            lines.push(line);
            while (lines.length > maxLogLines) lines.shift();
            log.textContent = lines.join('\n');
            log.parentElement.scrollTop = log.parentElement.scrollHeight;
        }
        function initWebSocket() {
            websocket = new WebSocket(gateway);
            websocket.onopen    = () => { };
            websocket.onclose   = () => { setTimeout(initWebSocket, 2000); };
            websocket.onmessage = (event) => {
                let line = event.data;
                if (line.startsWith("MAZE:")) line = line.substring(5).trim();
                appendLogLine(line);
            };
        }
    </script>
</body>
</html>
)rawliteral";

<<<<<<< HEAD
// ==========================================
// PHASE DEFINITIONS
>>>>>>> cf170e537a06b5130a9b7b7b3701c0930bc2ed03
// ==========================================
// The robot runs through these phases in order:
//
//  PHASE_SURVEY  — right-hand-rule traversal, building the wall map
//  PHASE_RETURN  — floodfill back to (0,0) using the completed map
//  PHASE_SOLVE   — optimised speed-run along the shortest path to goal
//  PHASE_DONE    — finished; robot stops
//
// Transition logic:
//  SURVEY ends when the goal cell is reached (or all reachable
//  cells are visited — you can add that check later).
//  RETURN ends when the robot reaches (0,0).
//  SOLVE ends when the solver reports solverDone().
// ==========================================
enum Phase {
  PHASE_SURVEY,
  PHASE_RETURN,
  PHASE_SOLVE,
  PHASE_DONE
};

Phase currentPhase = PHASE_SURVEY;

// Goal coordinates (used for both survey target and solve destination)
const int GOAL_ROW = 4;
const int GOAL_COL = 4;

// ==========================================
// TUNING — SURVEY PHASE
// (conservative — safety over speed)
// ==========================================
float Kp_vel_L = 1.0f,  Ki_vel_L = 0.0f, Kd_vel_L = 0.1f;
float Kp_vel_R = 1.0f,  Ki_vel_R = 0.0f, Kd_vel_R = 0.1f;
float Kp_yaw   = 0.6f,  Ki_yaw   = 0.0f, Kd_yaw   = 0.06f;

float Kp_tunnel = 0.12f, Kd_tunnel = 0.08f;
float Kp_single = 0.06f, Kd_single = 0.12f;
float Kp_wall   = 0.12f, Kd_wall   = 0.08f, Ki_wall = 0.0f;

float baseTargetVelocity = 50.0f;
int   basePWM            = 35;

// ==========================================
// HEADING / YAW MANAGEMENT
// ==========================================
float baseTargetYaw    = 0.0f;
float correction_angle = 0.0f;
float targetYaw        = 0.0f;

// ==========================================
// TOLERANCES
// ==========================================
float vel_tolerance  = 0.5f;
float yaw_tolerance  = 0.5f;
float wall_tolerance = 10.0f;

// ==========================================
// GEOMETRY — 155mm cells
// ==========================================
const int   WALL_THRESHOLD         = 110;
const float SINGLE_WALL_TARGET_MM  = 63.0f;
const int   FRONT_STOP_MM          = 120;
const int   FRONT_HALT_MM          = 60;
const float CELL_SIZE_NAV_MM       = 155.0f;
const float CELL_HALF_MM           = CELL_SIZE_NAV_MM / 2.0f;
const float CENTRE_TOLERANCE_MM    = 15.0f;

// ==========================================
// EKF / ODOMETRY CONSTANTS
// ==========================================
const float TICKS_PER_REV          = 306.0f;
const float WHEEL_CIRCUMFERENCE_MM  = 144.5f;
const float TRACK_WIDTH_MM          = 72.0f;

// ==========================================
// DECISION THRESHOLDS
// ==========================================
const int WALL_MISSING_THRESHOLD   = 180;
const int FRONT_BLOCKED_THRESHOLD  = 70;
=======
// Goal coordinates for survey completion
const int GOAL_ROW = 5;
const int GOAL_COL = -5;
>>>>>>> fbb75a121313d6494b818cb3021db2a3b771a626

// ==========================================
// LOOP TIMING
// ==========================================
const int LOOP_INTERVAL_MS  = 20;
const int LIDAR_INTERVAL_MS = 50;
const int PRINT_INTERVAL_MS = 100;

unsigned long lastLoopTime  = 0;
unsigned long lastLidarTime = 0;
unsigned long lastPrintTime = 0;

const int LOG_HISTORY_SIZE = 300;
String logHistory[LOG_HISTORY_SIZE];
int logHistoryStart = 0;
int logHistoryCount = 0;

// ==========================================
// SHARED STATE & DEBUG VARIABLES
// ==========================================
float integral_vel_L = 0, integral_vel_R = 0;
float prev_error_vel_L = 0, prev_error_vel_R = 0;
float integral_yaw = 0,     prev_error_yaw = 0;
float current_yaw_angle = 0.0f;
float integral_wall = 0,    prev_error_wall = 0;

long prevLeftTicks  = 0;
long prevRightTicks = 0;
int  final_pwm_L    = 0;
int  final_pwm_R    = 0;

// Global variables for telemetry access
float debug_err_L = 0, debug_err_R = 0;
float debug_d_L   = 0, debug_d_R   = 0;

DistanceData current_lidars;
EKFTelemetry ekfTelemetry = {0.0f, 0.0f, 0.0f};

// ==========================================
// SURVEY-PHASE STATE MACHINE
// ==========================================
enum BotState {
  DRIVING,
  TURN_COOLDOWN
};

BotState      surveyState      = DRIVING;
unsigned long cooldownStartMs  = 0;
const unsigned long TURN_COOLDOWN_MS = 800;
bool surveyComplete = false;

bool rightWallWasPresent = true;
bool leftWallWasPresent  = true;
bool isStartup           = true;

// ==========================================
// FORWARD DECLARATIONS
// ==========================================
void runWallPIDLoop(float dt);
void runDrivingPID(float dt_motor);
void printTelemetry();
void resetPIDIntegrals();
void resetWallPID();
float frontBrakeScale();
float wrapAngleDegrees(float angle);
void appendLogHistory(const String &line);
void replayLogHistory(uint8_t clientNum);
void logLine(const String &line);
void logPrintf(const char *format, ...);
String buildMazeWebReport();
void broadcastMazeWebReport();
void executeDecisionAndTurn();
void runSurveyUpdate(float dt_motor, float dt_lidar, bool doMotor, bool doLidar);
void transitionToReturn();
void transitionToSolve();
=======
void runSurveyUpdate(float dt_motor, bool doMotor);
>>>>>>> fbb75a121313d6494b818cb3021db2a3b771a626

// ==========================================
// RESET HELPERS
// ==========================================
void resetPIDIntegrals() {
  integral_vel_L = 0; prev_error_vel_L = 0;
  integral_vel_R = 0; prev_error_vel_R = 0;
  integral_yaw   = 0; prev_error_yaw   = 0;
  integral_wall  = 0; prev_error_wall  = 0;
  prev_error_front = -1.0f; // Reset to prevent derivative spike
}

void resetWallPID() {
  integral_wall    = 0;
  prev_error_wall  = 0;
  correction_angle = 0.0f;
  targetYaw        = baseTargetYaw;
}

// ==========================================
// FRONT BRAKE SCALE (survey phase)
// ==========================================
// ==========================================
float calculateFrontPDBrake(float dt) {
  int d = current_lidars.front;

// ==========================================
// ALIGN TO CELL CENTRE (survey phase)
// Uses encoder odometry to reach exact midpoint
// ==========================================
void alignToCellCentre() {
  EKFState s = ekfGetState();
  float posInCell = fmodf(fabsf(s.x_mm), CELL_SIZE_NAV_MM);
  float remaining = CELL_HALF_MM - posInCell;

float wrapAngleDegrees(float angle) {
  while (angle > 180.0f) angle -= 360.0f;
  while (angle < -180.0f) angle += 360.0f;
  return angle;
}

void appendLogHistory(const String &line) {
  int writeIndex = (logHistoryStart + logHistoryCount) % LOG_HISTORY_SIZE;
  logHistory[writeIndex] = line;
  if (logHistoryCount < LOG_HISTORY_SIZE) {
    logHistoryCount++;
  } else {
    logHistoryStart = (logHistoryStart + 1) % LOG_HISTORY_SIZE;
  }
}

void replayLogHistory(uint8_t clientNum) {
  for (int i = 0; i < logHistoryCount; ++i) {
    int index = (logHistoryStart + i) % LOG_HISTORY_SIZE;
    webSocket.sendTXT(clientNum, logHistory[index]);
  }
}

void logLine(const String &line) {
  appendLogHistory(line);
  Serial.println(line);
  String message = line;
  webSocket.broadcastTXT(message);
}

void logPrintf(const char *format, ...) {
  char buffer[384];
  va_list args;
  va_start(args, format);
  vsnprintf(buffer, sizeof(buffer), format, args);
  va_end(args);
  logLine(String(buffer));
}

String buildMazeWebReport() {
  String report = "Visited maze cells\n";
  for (int r = MAZE_SIZE - 1; r >= 0; --r) {
    for (int c = 0; c < MAZE_SIZE; ++c) {
      if (!wallMap[r][c].visited) {
        report += ".";
      } else {
        report += wallMap[r][c].north ? "N" : "-";
        report += wallMap[r][c].east  ? "E" : "-";
        report += wallMap[r][c].south ? "S" : "-";
        report += wallMap[r][c].west  ? "W" : "-";
      }
      if (c < MAZE_SIZE - 1) report += "  ";
    }
    if (r > 0) report += "\n";
  }
  return report;
}

void broadcastMazeWebReport() {
  logLine("MAZE:" + buildMazeWebReport());
}

// ==========================================
// EXECUTE DECISION AND TURN (survey phase)
// Right-hand rule: right → straight → left → U-turn
// ==========================================
void executeDecisionAndTurn() {
  current_lidars = readLidars();
  delay(20);
  float turnDeltaDeg = 0.0f;
  float nextBaseTargetYaw = baseTargetYaw;

  bool rightOpen = (current_lidars.right > WALL_MISSING_THRESHOLD);
  bool frontOpen = (current_lidars.front > FRONT_BLOCKED_THRESHOLD);
  bool leftOpen  = (current_lidars.left  > WALL_MISSING_THRESHOLD);

  logPrintf("[DEC] R=%s F=%s L=%s",
    rightOpen ? "open" : "wall",
    frontOpen ? "open" : "wall",
    leftOpen  ? "open" : "wall");

  if      (rightOpen) { logLine("[DEC] TURN RIGHT");        turnDeltaDeg = -90.0f;  }
  else if (frontOpen) { logLine("[DEC] STRAIGHT");                             }
  else if (leftOpen)  { logLine("[DEC] TURN LEFT");         turnDeltaDeg = 90.0f;   }
  else                { logLine("[DEC] DEAD END - U-TURN"); turnDeltaDeg = -180.0f; }

  nextBaseTargetYaw = wrapAngleDegrees(baseTargetYaw + turnDeltaDeg);

  if (turnDeltaDeg == -90.0f) {
    turnCW90(nextBaseTargetYaw);
  } else if (turnDeltaDeg == 90.0f) {
    turnACW90(nextBaseTargetYaw);
  } else if (turnDeltaDeg == -180.0f) {
    turn180(nextBaseTargetYaw);
  }

  delay(100);
  baseTargetYaw = nextBaseTargetYaw;
  targetYaw     = baseTargetYaw;
  resetPIDIntegrals();
  resetWallPID();

  current_lidars      = readLidars();
  rightWallWasPresent = (current_lidars.right < WALL_MISSING_THRESHOLD);
  leftWallWasPresent  = (current_lidars.left  < WALL_MISSING_THRESHOLD);
}

// ==========================================
// WEBSOCKET EVENT HANDLER
// ==========================================
void webSocketEvent(uint8_t num, WStype_t type, uint8_t *payload, size_t length) {
  switch (type) {
<<<<<<< HEAD
    case WStype_DISCONNECTED: Serial.printf("[WS] #%u Disconnected\n", num); break;
    case WStype_CONNECTED:    Serial.printf("[WS] #%u Connected\n",    num); break;
>>>>>>> cf170e537a06b5130a9b7b7b3701c0930bc2ed03
=======
    case WStype_DISCONNECTED:
      Serial.printf("[WS] #%u Disconnected\n", num);
      break;
    case WStype_CONNECTED:
      Serial.printf("[WS] #%u Connected\n", num);
      replayLogHistory(num);
      break;
>>>>>>> fbb75a121313d6494b818cb3021db2a3b771a626
  }

  float error_front = (float)(d - FRONT_HALT_MM);

  if (error_front <= 0.0f) {
    prev_error_front = error_front;
    return 0.0f;
  }

  float d_front = 0.0f;
  if (dt > 0.0f && prev_error_front >= 0.0f) {
    d_front = (error_front - prev_error_front) / dt;
  }

  float pd_output = (Kp_front * error_front) + (Kd_front * d_front);
  prev_error_front = error_front;

  return constrain(pd_output, 0.0f, 1.0f);
}

// ==========================================
// SETUP
// ==========================================
void setup() {
  Serial.begin(115200);
  delay(1000);

    case WStype_DISCONNECTED:
      Serial.printf("[WS] #%u Disconnected\n", num);
      break;
    case WStype_CONNECTED:
      Serial.printf("[WS] #%u Connected\n", num);
      replayLogHistory(num);
      break;
  WiFi.softAP(ssid, password);
  server.on("/", []() { server.send(200, "text/html", index_html); });
  server.begin();
  webSocket.begin();
  webSocket.onEvent(webSocketEvent);

<<<<<<< HEAD
  Serial.print("AP IP: "); Serial.println(WiFi.softAPIP());
  Serial.println("=================================");
  Serial.println("  MICROMOUSE STARTUP");
>>>>>>> cf170e537a06b5130a9b7b7b3701c0930bc2ed03
  Serial.println("=================================");
=======
  logLine(String("AP IP: ") + WiFi.softAPIP().toString());
  logLine("=================================");
  logLine("  MICROMOUSE STARTUP");
  logLine("=================================");
>>>>>>> fbb75a121313d6494b818cb3021db2a3b771a626

  initMotors();
  initEncoders();
  initSensors();
  delay(5000);

  logLine("[!] Calibrating gyro...");
  delay(2000);
  calibrateGyro();
  logLine("Gyro OK.");

  resetYaw();
  current_yaw_angle = baseTargetYaw = targetYaw = 0.0f;

  ekfConfigure(TICKS_PER_REV, WHEEL_CIRCUMFERENCE_MM, TRACK_WIDTH_MM);
  ekfInit(0.0f, 0.0f, 0.0f);
  initCellTracker();
  initWallMap();

  resetEncoders();
  prevLeftTicks = prevRightTicks = 0;
  resetPIDIntegrals();
  resetWallPID();

  current_lidars      = readLidars();
  rightWallWasPresent = (current_lidars.right < WALL_MISSING_THRESHOLD);
  leftWallWasPresent  = (current_lidars.left  < WALL_MISSING_THRESHOLD);

  unsigned long now = millis();
=======
  surveyState    = DRIVING;
  surveyComplete = false;

  logLine("Setup complete - survey mode only");
>>>>>>> fbb75a121313d6494b818cb3021db2a3b771a626
}

// ==========================================
// MAIN LOOP
// ==========================================
void loop() {
  logLine(String("AP IP: ") + WiFi.softAPIP().toString());
  logLine("=================================");
  logLine("  MICROMOUSE STARTUP");
  logLine("=================================");
  webSocket.loop();
  server.handleClient();

>>>>>>> cf170e537a06b5130a9b7b7b3701c0930bc2ed03
  unsigned long now = millis();

  if (now - lastPrintTime >= PRINT_INTERVAL_MS) {
    lastPrintTime = now;
    printTelemetry();
  }

  bool doLidar = (now - lastLidarTime >= LIDAR_INTERVAL_MS);
  bool doMotor = (now - lastLoopTime  >= LOOP_INTERVAL_MS);

  if (doLidar) {
    float dt_lidar = (now - lastLidarTime) / 1000.0f;
    lastLidarTime  = now;
    current_lidars = readLidars();

    if (!surveyComplete && surveyState == DRIVING) {
      runWallPIDLoop(dt_lidar);
    }
  }

  if (doMotor) {
    float dt_motor = (now - lastLoopTime) / 1000.0f;
    lastLoopTime   = now;

  lastLoopTime = lastLidarTime = lastPrintTime = now;

  currentPhase = PHASE_SURVEY;
  surveyState  = DRIVING;

  Serial.println("Setup complete — PHASE: SURVEY");
        }
        break;

        }
        break;

    switch (currentPhase) {

      // ──────────────────────────────────────
      case PHASE_SURVEY:
        runSurveyUpdate(dt_motor, 0, true, false);
        break;

      // ──────────────────────────────────────
      // RETURN phase: floodfill path from
      // current position back to (0,0).
      // Reuses the solver with a different goal.
      case PHASE_RETURN:
        solverUpdate(dt_motor, 0, true, false);
        if (solverDone()) {
          Serial.println("[PHASE] RETURN complete → transitioning to SOLVE");
          transitionToSolve();
=======
    if (surveyComplete) {
      applyMotorPWM(0, 0);
    } else {
      runSurveyUpdate(dt_motor, true);
>>>>>>> fbb75a121313d6494b818cb3021db2a3b771a626
    }
  }
}

// ==========================================
// SURVEY UPDATE
// ==========================================
void runSurveyUpdate(float dt_motor, bool doMotor) {
  if (!doMotor) return;

  // EKF tick
  noInterrupts();
  long curL = leftTicks, curR = rightTicks;
  interrupts();
  long dL = curL - prevLeftTicks, dR = curR - prevRightTicks;

  current_yaw_angle = readYawDegrees();
  ekfPredict(dL, dR);
  ekfUpdateYawDeg(current_yaw_angle);
  ekfTelemetry = ekfGetTelemetry();

  bool newCell = updateCellTracker();
  if (newCell) {
    recordWalls(ctGetRow(), ctGetCol(), ctGetHeading(), current_lidars);
    printCellWalls(ctGetRow(), ctGetCol());
    printCellState();

    if (ctGetRow() == GOAL_ROW && ctGetCol() == GOAL_COL) {
      applyMotorPWM(0, 0);
      surveyComplete = true;
      surveyState    = TURN_COOLDOWN;
      logLine("");
      logLine("[SURVEY] Goal cell reached! Survey complete.");
      printWallMapASCII();
      broadcastMazeWebReport();
      prevLeftTicks  = curL;
      prevRightTicks = curR;
      return;
    }
  }

  prevLeftTicks  = curL;
  prevRightTicks = curR;

  // ── SURVEY STATE MACHINE ──
  if (surveyState == TURN_COOLDOWN) {
    if (millis() - cooldownStartMs >= TURN_COOLDOWN_MS) {
      current_lidars      = readLidars();
      rightWallWasPresent = (current_lidars.right < WALL_MISSING_THRESHOLD);
      leftWallWasPresent  = (current_lidars.left  < WALL_MISSING_THRESHOLD);
      surveyState         = DRIVING;
      logLine("[SURVEY] Cooldown over - DRIVING");
    } else {
    }

  } else {  // DRIVING
    bool rightOpen = (current_lidars.right > WALL_MISSING_THRESHOLD);
    bool leftOpen  = (current_lidars.left  > WALL_MISSING_THRESHOLD);
    bool frontOpen = (current_lidars.front > FRONT_BLOCKED_THRESHOLD);

    bool rightGapJustOpened = (rightOpen && rightWallWasPresent);
    bool mustDecide = (!frontOpen);

    if (mustDecide) {
      logLine("[SURVEY] Decision point");
      executeDecisionAndTurn();
      surveyState    = TURN_COOLDOWN;
      cooldownStartMs = millis();
    } else {
      runDrivingPID(dt_motor);
    }

    rightWallWasPresent = !rightOpen;
    leftWallWasPresent  = !leftOpen;
  }
}

// ==========================================
// DRIVING PID (survey phase)
// ==========================================
void runDrivingPID(float dt_motor) {
  noInterrupts();
  long cL = leftTicks, cR = rightTicks;
  interrupts();

  float vel_L = (float)(cL - prevLeftTicks);
  float vel_R = (float)(cR - prevRightTicks);

  // Heading correction
  // 1. Calculate the raw difference
  float error_yaw = targetYaw - current_yaw_angle;

  // 2. Wrap the difference to strictly be between -180.0 and +180.0
  while (error_yaw > 180.0f) {
    error_yaw -= 360.0f;
  }
  while (error_yaw <= -180.0f) {
    error_yaw += 360.0f;
  }
  if (fabsf(error_yaw) <= yaw_tolerance) { error_yaw = 0; prev_error_yaw = 0; }
  integral_yaw += error_yaw * dt_motor;
      // ──────────────────────────────────────
      case PHASE_SOLVE:
        solverUpdate(dt_motor, 0, true, false);
        if (solverDone()) {
          applyMotorPWM(0, 0);
          currentPhase = PHASE_DONE;
          Serial.println("[PHASE] SOLVE complete — ALL DONE!");
          printWallMapASCII();
  float heading_corr = (Kp_yaw * error_yaw) + (Ki_yaw * integral_yaw) + (Kd_yaw * deriv_yaw);
  prev_error_yaw    = error_yaw;

      // ──────────────────────────────────────
      case PHASE_DONE:
        applyMotorPWM(0, 0);
        break;
  float brakeFactor = frontBrakeScale();
>>>>>>> cf170e537a06b5130a9b7b7b3701c0930bc2ed03
  if (brakeFactor <= 0.0f) {
    applyMotorPWM(0, 0);
    final_pwm_L = 0; final_pwm_R = 0;
    debug_err_L = 0; debug_err_R = 0;
    debug_d_L = 0;   debug_d_R = 0;
    return;
  }

  float effVel = baseTargetVelocity * brakeFactor;
  int   effPWM = (int)(basePWM * brakeFactor);

  float tgt_L = effVel - heading_corr;
  float tgt_R = effVel + heading_corr;

  // Assign to global debug variables
  debug_err_L = tgt_L - vel_L;
  debug_err_R = tgt_R - vel_R;

  if (fabsf(debug_err_L) <= vel_tolerance) { debug_err_L = 0; prev_error_vel_L = 0; }
  if (fabsf(debug_err_R) <= vel_tolerance) { debug_err_R = 0; prev_error_vel_R = 0; }

  integral_vel_L += debug_err_L * dt_motor;
  integral_vel_R += debug_err_R * dt_motor;

  debug_d_L = (debug_err_L - prev_error_vel_L) / dt_motor;
  debug_d_R = (debug_err_R - prev_error_vel_R) / dt_motor;

  final_pwm_L = effPWM + (int)((Kp_vel_L * debug_err_L) + (Ki_vel_L * integral_vel_L) + (Kd_vel_L * debug_d_L));
  final_pwm_R = effPWM + (int)((Kp_vel_R * debug_err_R) + (Ki_vel_R * integral_vel_R) + (Kd_vel_R * debug_d_R));

  prev_error_vel_L = debug_err_L;
  prev_error_vel_R = debug_err_R;

  applyMotorPWM(final_pwm_L, final_pwm_R);
}

// ==========================================
// WALL CENTERING PID (survey phase)
// ==========================================
void runWallPIDLoop(float dt) {
  bool hasLeft  = (current_lidars.left  < WALL_THRESHOLD);
  bool hasRight = (current_lidars.right < WALL_THRESHOLD);
  
  float error_wall = 0.0f;

  if (hasLeft && hasRight) {
    Kp_wall    = Kp_tunnel; Kd_wall = Kd_tunnel;
    error_wall = (float)(current_lidars.left - current_lidars.right);
  } else if (hasLeft) {
    Kp_wall    = Kp_single; Kd_wall = Kd_single;
    error_wall = -(current_lidars.left - SINGLE_WALL_TARGET_MM);
  } else if (hasRight) {
    Kp_wall    = Kp_single; Kd_wall = Kd_single;
    error_wall = (current_lidars.right - SINGLE_WALL_TARGET_MM);
  } else {
    correction_angle = 0.0f; integral_wall = 0.0f; prev_error_wall = 0.0f;
    targetYaw        = baseTargetYaw;
    return;
  }

  if (fabsf(error_wall) <= wall_tolerance) { error_wall = 0.0f; integral_wall = 0.0f; }

  integral_wall    += error_wall * dt;
  float deriv_wall  = (error_wall - prev_error_wall) / dt;
  float deriv_yaw   = (error_yaw - prev_error_yaw) / dt_motor;

  correction_angle  = (correction_angle * 0.7f) + (corr * 0.3f);
  correction_angle  = constrain(correction_angle, -8.0f, 8.0f);
  prev_error_wall   = error_wall;
  targetYaw         = baseTargetYaw + correction_angle;
}

// ==========================================
// TELEMETRY
// ==========================================
void printTelemetry() {
  noInterrupts();
  long cL = leftTicks, cR = rightTicks;
  interrupts();

  // Braking 
  float brakeFactor = frontBrakeScale();
    baseTargetVelocity,
    cL, cR,
    final_pwm_L, final_pwm_R,
    current_yaw_angle,
  float corr        = (Kp_wall * error_wall) + (Ki_wall * integral_wall) + (Kd_wall * deriv_wall);
  );
  
  Serial.println(buf);
=======
  char buf[450]; 
  // snprintf(buf, sizeof(buf),
  //   "PWM:%d/%d | Yaw:%.1f | Lidar:%d/%d/%d | Cell:%d,%d | dL:%.2f dR:%.2f | eL:%.2f eR:%.2f",
  //   final_pwm_L, final_pwm_R,
  //   current_yaw_angle,
  //   current_lidars.left, current_lidars.front, current_lidars.right,
  //   ctGetRow(), ctGetCol(),
  //   debug_d_L, debug_d_R,
  //   debug_err_L, debug_err_R
  // );
  snprintf(buf, sizeof(buf),
    "Cell:%d,%d",
    ctGetRow(), ctGetCol()
  );

  logLine(String(buf));
>>>>>>> fbb75a121313d6494b818cb3021db2a3b771a626
}
