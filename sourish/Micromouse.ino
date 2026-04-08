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
#include "Algorithm.h"

// ==========================================
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
// SHARED STATE
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

DistanceData current_lidars;
EKFTelemetry ekfTelemetry = {0.0f, 0.0f, 0.0f};

// ==========================================
// TWO-RUN STATE MACHINE
//
//   RUN1_EXPLORING  — floodfill-driven goal + return (builds wall map)
//   RUN1_COMPLETE   — idle at start; auto-advances to Run 2
//   RUN2_SPEEDRUN   — A* speed run using flood-fill heuristic
//   DONE            — goal reached in Run 2; motors off
// ==========================================
enum MousePhase {
  RUN1_EXPLORING,
  RUN1_COMPLETE,
  RUN2_SPEEDRUN,
  DONE
};

MousePhase mousePhase = RUN1_EXPLORING;

// Delay between Run 1 completion and Run 2 start (ms)
const unsigned long RUN2_DELAY_MS   = 3000;
unsigned long       run1DoneMs      = 0;

bool rightWallWasPresent = true;
bool leftWallWasPresent  = true;

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
void advanceOneCell();          // used by Algorithm.h

// ==========================================
// RESET HELPERS
// ==========================================
void resetPIDIntegrals() {

  integral_vel_L = 0; prev_error_vel_L = 0;
  integral_vel_R = 0; prev_error_vel_R = 0;
  integral_yaw   = 0; prev_error_yaw   = 0;
  integral_wall  = 0; prev_error_wall  = 0;
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
float frontBrakeScale() {
  int d = current_lidars.front;
  if (d >= FRONT_STOP_MM) return 1.0f;
  if (d <= FRONT_HALT_MM) return 0.0f;
  return (float)(d - FRONT_HALT_MM) / (float)(FRONT_STOP_MM - FRONT_HALT_MM);
}

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
// WEBSOCKET EVENT HANDLER
// ==========================================
void webSocketEvent(uint8_t num, WStype_t type, uint8_t *payload, size_t length) {
  switch (type) {
    case WStype_DISCONNECTED:
      Serial.printf("[WS] #%u Disconnected\n", num);
      break;
    case WStype_CONNECTED:
      Serial.printf("[WS] #%u Connected\n", num);
      replayLogHistory(num);
      break;
  }
}

// ==========================================
// SETUP
// ==========================================
void setup() {
  Serial.begin(115200);
  delay(1000);

  WiFi.softAP(ssid, password);
  server.on("/", []() { server.send(200, "text/html", index_html); });
  server.begin();
  webSocket.begin();
  webSocket.onEvent(webSocketEvent);

  logLine(String("AP IP: ") + WiFi.softAPIP().toString());
  logLine("=================================");
  logLine("  MICROMOUSE STARTUP");
  logLine("=================================");

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
  initAlgorithm();   // computes goal cells, zeroes flood_dist

  resetEncoders();
  prevLeftTicks = prevRightTicks = 0;
  resetPIDIntegrals();
  resetWallPID();

  current_lidars      = readLidars();
  rightWallWasPresent = (current_lidars.right < WALL_MISSING_THRESHOLD);
  leftWallWasPresent  = (current_lidars.left  < WALL_MISSING_THRESHOLD);

  unsigned long now = millis();
  lastLoopTime = lastLidarTime = lastPrintTime = now;

  mousePhase = RUN1_EXPLORING;

  logLine("Setup complete — starting Run 1 (floodfill exploration)...");
}

// ==========================================
// MAIN LOOP
// ==========================================
void loop() {
  webSocket.loop();
  server.handleClient();

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

    // Wall centering only while mouse is mid-drive (not between phases)
    if (mousePhase == RUN1_EXPLORING || mousePhase == RUN2_SPEEDRUN) {
      runWallPIDLoop(dt_lidar);
    }
  }

  if (doMotor) {
    float dt_motor = (now - lastLoopTime) / 1000.0f;
    lastLoopTime   = now;

    switch (mousePhase) {

      // ── Run 1: blocking call — returns after explore + return to start ──
      case RUN1_EXPLORING:
        runFloodfillExplore();     // defined in Algorithm.h

        // Build the flood-fill distance table from the complete wall map
        buildFloodFill();

        printWallMapASCII();
        broadcastMazeWebReport();
        logPrintf("[MAIN] Run 1 complete. Pausing %lu ms before Run 2...",
                  RUN2_DELAY_MS);

        applyMotorPWM(0, 0);
        mousePhase = RUN1_COMPLETE;
        run1DoneMs = millis();
        break;

      // ── Brief pause so the operator can observe before the speed run ────
      case RUN1_COMPLETE:
        applyMotorPWM(0, 0);
        if (millis() - run1DoneMs >= RUN2_DELAY_MS) {
          logLine("[MAIN] Starting Run 2 (A* speed run)...");
          mousePhase = RUN2_SPEEDRUN;
        }
        break;

      // ── Run 2: blocking A* speed run ────────────────────────────────────
      case RUN2_SPEEDRUN:
        runAStarSpeedRun();        // defined in Algorithm.h
        applyMotorPWM(0, 0);
        logLine("[MAIN] Run 2 complete. Robot stopped.");
        mousePhase = DONE;
        break;

      // ── Done: sit still ─────────────────────────────────────────────────
      case DONE:
        applyMotorPWM(0, 0);
        break;
    }
  }
}

// ==========================================
// ADVANCE ONE CELL
// Blocking helper called by Algorithm.h.
// Drives the robot forward exactly one maze
// cell using encoder odometry + yaw PID.
// Wall centering continues via the Lidar
// interrupt path during the drive.
// ==========================================
void advanceOneCell() {
  // Number of encoder ticks for one cell width
  const int TARGET_TICKS = (int)(CELL_SIZE_NAV_MM /
                                 (WHEEL_CIRCUMFERENCE_MM / TICKS_PER_REV));

  noInterrupts();
  long startL = leftTicks, startR = rightTicks;
  interrupts();

  resetPIDIntegrals();

  while (true) {
    noInterrupts();
    long curL = leftTicks, curR = rightTicks;
    interrupts();

    long avgDone = ((curL - startL) + (curR - startR)) / 2;
    if (avgDone >= TARGET_TICKS) break;

    // Read fresh lidar for front-brake and wall centering
    current_lidars    = readLidars();
    current_yaw_angle = readYawDegrees();

    // EKF update
    long dL = curL - prevLeftTicks, dR = curR - prevRightTicks;
    ekfPredict(dL, dR);
    ekfUpdateYawDeg(current_yaw_angle);
    ekfTelemetry = ekfGetTelemetry();
    updateCellTracker();
    prevLeftTicks = curL; prevRightTicks = curR;

    float dt_motor = (float)LOOP_INTERVAL_MS / 1000.0f;
    runWallPIDLoop(dt_motor);
    runDrivingPID(dt_motor);

    delay(LOOP_INTERVAL_MS);
  }

  applyMotorPWM(0, 0);
  delay(100);   // brief settle before next command

  // Final EKF + cell tracker sync
  noInterrupts();
  long curL = leftTicks, curR = rightTicks;
  interrupts();
  long dL = curL - prevLeftTicks, dR = curR - prevRightTicks;
  current_yaw_angle = readYawDegrees();
  ekfPredict(dL, dR);
  ekfUpdateYawDeg(current_yaw_angle);
  ekfTelemetry = ekfGetTelemetry();
  updateCellTracker();
  prevLeftTicks = curL; prevRightTicks = curR;
}

// ==========================================
// DRIVING PID
// ==========================================
void runDrivingPID(float dt_motor) {
  noInterrupts();
  long cL = leftTicks, cR = rightTicks;
  interrupts();

  float vel_L = (float)(cL - prevLeftTicks);
  float vel_R = (float)(cR - prevRightTicks);

  float error_yaw = targetYaw - current_yaw_angle;
  if (fabsf(error_yaw) <= yaw_tolerance) { error_yaw = 0; prev_error_yaw = 0; }
  integral_yaw += error_yaw * dt_motor;
  float deriv_yaw   = (error_yaw - prev_error_yaw) / dt_motor;
  float heading_corr = (Kp_yaw * error_yaw) + (Ki_yaw * integral_yaw) + (Kd_yaw * deriv_yaw);
  prev_error_yaw    = error_yaw;

  float brakeFactor = frontBrakeScale();
  if (brakeFactor <= 0.0f) {
    applyMotorPWM(0, 0);
    final_pwm_L = 0; final_pwm_R = 0;
    integral_vel_L = 0; integral_vel_R = 0;
    prev_error_vel_L = 0; prev_error_vel_R = 0;
    return;
  }

  float effVel = baseTargetVelocity * brakeFactor;
  int   effPWM = (int)(basePWM * brakeFactor);

  float tgt_L = effVel - heading_corr;
  float tgt_R = effVel + heading_corr;

  float err_L = tgt_L - vel_L;
  float err_R = tgt_R - vel_R;

  if (fabsf(err_L) <= vel_tolerance) { err_L = 0; prev_error_vel_L = 0; }
  if (fabsf(err_R) <= vel_tolerance) { err_R = 0; prev_error_vel_R = 0; }

  integral_vel_L += err_L * dt_motor;
  integral_vel_R += err_R * dt_motor;

  float d_L = (err_L - prev_error_vel_L) / dt_motor;
  float d_R = (err_R - prev_error_vel_R) / dt_motor;

  final_pwm_L = effPWM + (int)((Kp_vel_L * err_L) + (Ki_vel_L * integral_vel_L) + (Kd_vel_L * d_L));
  final_pwm_R = effPWM + (int)((Kp_vel_R * err_R) + (Ki_vel_R * integral_vel_R) + (Kd_vel_R * d_R));

  prev_error_vel_L = err_L;
  prev_error_vel_R = err_R;

  applyMotorPWM(final_pwm_L, final_pwm_R);
}

// ==========================================
// WALL CENTERING PID
// Called while robot is driving between turns.
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
  float corr        = (Kp_wall * error_wall) + (Ki_wall * integral_wall) + (Kd_wall * deriv_wall);

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

  char buf[320];
  snprintf(buf, sizeof(buf),
    "Target: %.1f | Enc: %ld / %ld | PWM: %d / %d | Yaw: %.2f | Lidar: %d / %d / %d | Cell: %d/%d/%s | Phase: %s",
    baseTargetVelocity,
    cL, cR,
    final_pwm_L, final_pwm_R,
    current_yaw_angle,
    current_lidars.left, current_lidars.front, current_lidars.right,
    ctGetRow(), ctGetCol(), headingName(ctGetHeading()),
    mousePhase == DONE          ? "DONE"     :
    mousePhase == RUN2_SPEEDRUN ? "RUN2_A*"  :
    mousePhase == RUN1_COMPLETE ? "RUN1_DONE": "RUN1_EXPLORE"
  );

  logLine(String(buf));
}
