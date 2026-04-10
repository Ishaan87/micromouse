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
#include "Floodfill.h"
#include "Solver.h"
#include "Turns.h"

// ==========================================
// EXTERN MEMORY & CONFIG ALLOCATION
// ==========================================
uint8_t live_dist[MAZE_SIZE][MAZE_SIZE];
uint8_t flood_dist[MAZE_SIZE][MAZE_SIZE]; 
std::vector<MazeHeading> ff_path_moves;
CellWalls wallMap[MAZE_SIZE][MAZE_SIZE];

float Kp_vel_L = 1.0f;
float Ki_vel_L = 0.0f;
float Kd_vel_L = 0.1f;

float Kp_vel_R = 1.0f;
float Ki_vel_R = 0.0f;
float Kd_vel_R = 0.1f;

float Kp_yaw   = 0.6f;
float Ki_yaw   = 0.0f;
float Kd_yaw   = 0.06f;

float Kp_tunnel = 0.15f;
float Kd_tunnel = 0.08f;

float Kp_single = 0.10f;
float Kd_single = 0.12f;

float Kp_wall   = 0.19f;
float Kd_wall   = 0.08f;
float Ki_wall   = 0.0f;

float baseTargetVelocity = 50.0f;
int   basePWM            = 50;
float baseTargetYaw      = 0.0f;
float correction_angle   = 0.0f;
float targetYaw          = 0.0f;

float vel_tolerance      = 0.5f;
float yaw_tolerance      = 0.5f;
float wall_tolerance     = 10.0f;

// ==========================================
// WIRELESS WEB SERVER SETTINGS
// ==========================================
const char *ssid     = "Jerry";
const char *password = "mouse1234";

WebServer        server(80);
WebSocketsServer webSocket(81);

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

const int GOAL_ROW = 5;
const int GOAL_COL = -5;

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

float integral_vel_L = 0;
float prev_error_vel_L = 0;
float integral_vel_R = 0;
float prev_error_vel_R = 0;

float integral_yaw = 0;
float prev_error_yaw = 0;
float current_yaw_angle = 0.0f;

float integral_wall = 0;
float prev_error_wall = 0;

long prevLeftTicks  = 0;
long prevRightTicks = 0;
int  final_pwm_L    = 0;
int  final_pwm_R    = 0;

float debug_err_L = 0;
float debug_err_R = 0;
float debug_d_L = 0;
float debug_d_R = 0;

DistanceData current_lidars;
EKFTelemetry ekfTelemetry = {0.0f, 0.0f, 0.0f};

enum BotState {
  DRIVING,
  BRAKING_TO_CENTER,
  TURN_COOLDOWN
};

enum ExplorePhase { 
    EXPLORE_OUTBOUND, 
    EXPLORE_RETURN, 
    EXPLORE_DONE 
};

ExplorePhase  explorePhase     = EXPLORE_OUTBOUND;
BotState      surveyState      = DRIVING;
unsigned long cooldownStartMs  = 0;
const unsigned long TURN_COOLDOWN_MS = 800; 
bool surveyComplete = false;

int pendingTurn = 0; 

// ==========================================
// FORWARD DECLARATIONS
// ==========================================
void runWallPIDLoop(float dt);
void runDrivingPID(float dt_motor, float algorithmScale = 1.0f);
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
void executeSavedTurn(int headingDiff);
void runSurveyUpdate(float dt_motor, bool doMotor);

// ==========================================
// EKF DYNAMIC AXIS BRAKING
// ==========================================
float getEKFDistanceToCenter() {
  EKFState s = ekfGetState();
  MazeHeading h = ctGetHeading();
  
  auto posMod = [](float val, float mod) {
    float res = fmodf(val, mod);
    return (res < 0) ? res + mod : res;
  };

  float posInCell = 0.0f;
  
  if (h == HEADING_NORTH) {
      posInCell = posMod(s.x_mm, 160.0f);
  } else if (h == HEADING_SOUTH) {
      posInCell = posMod(-s.x_mm, 160.0f);
  } else if (h == HEADING_WEST) {
      posInCell = posMod(s.y_mm, 160.0f);
  } else if (h == HEADING_EAST) {
      posInCell = posMod(-s.y_mm, 160.0f);
  }

  return 80.0f - posInCell; 
}

void resetPIDIntegrals() {
  integral_vel_L = 0; 
  prev_error_vel_L = 0;
  integral_vel_R = 0; 
  prev_error_vel_R = 0;
  integral_yaw   = 0; 
  prev_error_yaw   = 0;
  integral_wall  = 0; 
  prev_error_wall  = 0;
}

void resetWallPID() {
  integral_wall = 0; 
  prev_error_wall = 0;
  correction_angle = 0.0f; 
  targetYaw = baseTargetYaw;
}

float frontBrakeScale() {
  int d = current_lidars.front;
  if (d >= FRONT_STOP_MM) {
      return 1.0f;
  }
  if (d <= FRONT_HALT_MM) {
      return 0.0f;
  }
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

void executeSavedTurn(int headingDiff) {
  float turnDeltaDeg = 0.0f;
  float nextBaseTargetYaw = baseTargetYaw;

  if (headingDiff == 1) { 
      logLine("[DEC] TURN RIGHT"); 
      turnDeltaDeg = -90.0f; 
  } else if (headingDiff == 3) { 
      logLine("[DEC] TURN LEFT"); 
      turnDeltaDeg = 90.0f; 
  } else if (headingDiff == 2) { 
      logLine("[DEC] DEAD END - U-TURN"); 
      turnDeltaDeg = -180.0f; 
  }

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
  targetYaw = baseTargetYaw;
  
  resetPIDIntegrals(); 
  resetWallPID();
}

void webSocketEvent(uint8_t num, WStype_t type, uint8_t *payload, size_t length) {
  if (type == WStype_CONNECTED) {
      replayLogHistory(num);
  }
}

void setup() {
  Serial.begin(115200);
  delay(1000);

  WiFi.softAP(ssid, password);
  server.on("/", []() { server.send(200, "text/html", index_html); });
  server.begin();
  
  webSocket.begin();
  webSocket.onEvent(webSocketEvent);

  initMotors();
  initEncoders();
  initSensors();
  delay(5000);

  calibrateGyro();
  resetYaw();
  current_yaw_angle = 0.0f;
  baseTargetYaw = 0.0f;
  targetYaw = 0.0f;

  ekfConfigure(TICKS_PER_REV, WHEEL_CIRCUMFERENCE_MM, TRACK_WIDTH_MM);
  ekfInit(0.0f, 0.0f, 0.0f); 
  
  initCellTracker();
  initWallMap();

  resetEncoders();
  prevLeftTicks = 0;
  prevRightTicks = 0;
  
  resetPIDIntegrals();
  resetWallPID();

  unsigned long now = millis();
  lastLoopTime = now;
  lastLidarTime = now;
  lastPrintTime = now;
  
  surveyState    = DRIVING;
  explorePhase   = EXPLORE_OUTBOUND;
  surveyComplete = false;

  logLine("Setup complete - Floodfill Mode Engaged");
}

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

  float dt_lidar = 0.0f;
  float dt_motor = 0.0f;

  if (doLidar) { 
      dt_lidar = (now - lastLidarTime) / 1000.0f; 
      lastLidarTime = now; 
  }
  
  if (doMotor) { 
      dt_motor = (now - lastLoopTime) / 1000.0f; 
      lastLoopTime = now; 
  }

  if (surveyComplete) {
    if (doMotor || doLidar) {
        solverUpdate(dt_motor, dt_lidar, doMotor, doLidar);
    }
  } else {
    if (doLidar) {
      current_lidars = readLidars();
      // Keep Wall PID running during braking
      if (surveyState == DRIVING || surveyState == BRAKING_TO_CENTER) {
          runWallPIDLoop(dt_lidar);
      }
    }
    if (doMotor) {
        runSurveyUpdate(dt_motor, true);
    }
  }
}

void runSurveyUpdate(float dt_motor, bool doMotor) {
  if (!doMotor) return;

  noInterrupts();
  long curL = leftTicks;
  long curR = rightTicks;
  interrupts();
  
  long dL = curL - prevLeftTicks;
  long dR = curR - prevRightTicks;
  
  current_yaw_angle = readYawDegrees();
  ekfPredict(dL, dR);
  ekfUpdateYawDeg(current_yaw_angle);
  ekfTelemetry = ekfGetTelemetry();

  bool newCell = updateCellTracker();
  
  if (newCell) {
    // Delete synchronous lidar poll here - async only!
    recordWalls(ctGetRow(), ctGetCol(), ctGetHeading(), current_lidars);

    if (ctGetRow() == GOAL_ROW && ctGetCol() == GOAL_COL) {
        surveyComplete = true; 
        explorePhase = EXPLORE_DONE; 
        surveyState = TURN_COOLDOWN;
        
        logLine("\n[SURVEY] Goal cell reached! Exploration complete.");
        ffBuildHeuristic({{GOAL_ROW, GOAL_COL}}); 
        
        if (ffComputePath(0, 0, {{GOAL_ROW, GOAL_COL}})) {
            solverInit(ctGetHeading()); 
        }
        return;
    }

    MazeHeading nextHeading = ffGetExploreMove(ctGetRow(), ctGetCol(), ctGetHeading());
    int headingDiff = ((int)nextHeading - (int)ctGetHeading() + 4) % 4;
    bool wallInFront = (current_lidars.front < FRONT_BLOCKED_THRESHOLD);

    if (headingDiff == 0 && !wallInFront) {
        surveyState = DRIVING; 
    } else {
        surveyState = BRAKING_TO_CENTER;
        pendingTurn = headingDiff;
    }
  }

  if (surveyState == TURN_COOLDOWN) {
    if (millis() - cooldownStartMs >= TURN_COOLDOWN_MS) {
        surveyState = DRIVING;
    }
  } else if (surveyState == DRIVING) {
    runDrivingPID(dt_motor, 1.0f); 
  } else if (surveyState == BRAKING_TO_CENTER) {
    float distanceLeft = getEKFDistanceToCenter();
    
    // Tolerances Applied: 5.0f for math, pure emergency 25mm for Lidar
    if (distanceLeft <= 5.0f || current_lidars.front <= 25) {
        applyMotorPWM(0, 0); 
        executeSavedTurn(pendingTurn);
        surveyState = TURN_COOLDOWN;
        cooldownStartMs = millis();
    } else {
        float totalBrakeDist = 80.0f - CELL_ENTRY_MARGIN; // 60.0f
        float slowDownFactor = distanceLeft / totalBrakeDist;
        
        // Anti-Stall Crawl Fix
        slowDownFactor = constrain(slowDownFactor, 0.35f, 1.0f);
        
        runDrivingPID(dt_motor, slowDownFactor); 
    }
  }

  prevLeftTicks  = curL;
  prevRightTicks = curR;
}

void runDrivingPID(float dt_motor, float algorithmScale) {
  noInterrupts();
  long cL = leftTicks;
  long cR = rightTicks;
  interrupts();

  float vel_L = (float)(cL - prevLeftTicks);
  float vel_R = (float)(cR - prevRightTicks);
  
  float error_yaw = wrapAngleDegrees(targetYaw - current_yaw_angle);
  
  if (fabsf(error_yaw) <= yaw_tolerance) { 
      error_yaw = 0; 
      prev_error_yaw = 0; 
  }
  
  integral_yaw += error_yaw * dt_motor;
  float deriv_yaw = (error_yaw - prev_error_yaw) / dt_motor;
  float heading_corr = (Kp_yaw * error_yaw) + (Ki_yaw * integral_yaw) + (Kd_yaw * deriv_yaw);
  prev_error_yaw = error_yaw;

  float lidarBrake = frontBrakeScale(); 
  float finalScale;

  // The Clash Fix applied to PID logic
  if (algorithmScale > 0.9f) {
      finalScale = min(lidarBrake, algorithmScale);
  } else {
      finalScale = algorithmScale;
  }

  if (finalScale <= 0.0f) {
    applyMotorPWM(0, 0);
    final_pwm_L = 0; 
    final_pwm_R = 0;
    integral_vel_L = 0; 
    integral_vel_R = 0; 
    prev_error_vel_L = 0; 
    prev_error_vel_R = 0;
    debug_err_L = 0; 
    debug_err_R = 0; 
    debug_d_L = 0; 
    debug_d_R = 0;
    return;
  }

  float effVel = baseTargetVelocity * finalScale;
  int   effPWM = (int)(basePWM * finalScale);

  float tgt_L = effVel - heading_corr;
  float tgt_R = effVel + heading_corr;
  
  debug_err_L = tgt_L - vel_L; 
  debug_err_R = tgt_R - vel_R;

  if (fabsf(debug_err_L) <= vel_tolerance) { 
      debug_err_L = 0; 
      prev_error_vel_L = 0; 
  }
  if (fabsf(debug_err_R) <= vel_tolerance) { 
      debug_err_R = 0; 
      prev_error_vel_R = 0; 
  }

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

void runWallPIDLoop(float dt) {
  bool hasLeft  = (current_lidars.left  < WALL_THRESHOLD);
  bool hasRight = (current_lidars.right < WALL_THRESHOLD);
  float error_wall = 0.0f;

  if (hasLeft && hasRight) { 
      Kp_wall = Kp_tunnel; 
      Kd_wall = Kd_tunnel; 
      error_wall = (float)(current_lidars.left - current_lidars.right); 
  } else if (hasLeft) { 
      Kp_wall = Kp_single; 
      Kd_wall = Kd_single; 
      error_wall = -(current_lidars.left - SINGLE_WALL_TARGET_MM); 
  } else if (hasRight) { 
      Kp_wall = Kp_single; 
      Kd_wall = Kd_single; 
      error_wall = (current_lidars.right - SINGLE_WALL_TARGET_MM); 
  } else { 
      correction_angle = 0.0f; 
      integral_wall = 0.0f; 
      prev_error_wall = 0.0f; 
      targetYaw = baseTargetYaw; 
      return; 
  }

  if (fabsf(error_wall) <= wall_tolerance) { 
      error_wall = 0.0f; 
      integral_wall = 0.0f; 
  }

  integral_wall += error_wall * dt;
  float deriv_wall = (error_wall - prev_error_wall) / dt;
  float corr = (Kp_wall * error_wall) + (Ki_wall * integral_wall) + (Kd_wall * deriv_wall);
  
  correction_angle  = (correction_angle * 0.7f) + (corr * 0.3f);
  correction_angle  = constrain(correction_angle, -8.0f, 8.0f);
  prev_error_wall   = error_wall;
  targetYaw         = baseTargetYaw + correction_angle;
}

void printTelemetry() {
  noInterrupts();
  long cL = leftTicks;
  long cR = rightTicks;
  interrupts();

  EKFState s = ekfGetState();
  float distToCenter = getEKFDistanceToCenter();

  char buf[450];
  snprintf(buf, sizeof(buf),
    "PWM:%d/%d | Yaw:%.1f | Lid:%d/%d/%d | Cell:%d,%d (%s) | Dist:%.1f | XY:%.1f,%.1f | Phs:%s",
    final_pwm_L, final_pwm_R,
    current_yaw_angle,
    current_lidars.left, current_lidars.front, current_lidars.right,
    ctGetRow(), ctGetCol(), headingName(ctGetHeading()),
    distToCenter,
    s.x_mm, s.y_mm,
    surveyComplete ? "SOLVE" : "SURVEY"
  );

  logLine(String(buf));
}