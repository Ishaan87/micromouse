#include <Arduino.h>
#include <WiFi.h>
#include <WebServer.h>
#include <WebSocketsServer.h>
#include "Config.h"
#include "Motors.h"
#include "Encoders.h"
#include "Sensors.h"
#include "ekf.h"
#include "CellTracker.h"
#include "WallMap.h"
#include "Turns.h"

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
    <title>Micromouse Dashboard</title>
    <style>
        body { font-family: 'Segoe UI', system-ui, sans-serif; background: #0f172a; color: #f8fafc; text-align: center; margin: 0; padding: 40px 20px; }
        h1 { color: #38bdf8; letter-spacing: 1px; }
        .grid { display: grid; grid-template-columns: repeat(auto-fit, minmax(250px, 1fr)); gap: 20px; max-width: 900px; margin: 30px auto; }
        .card { background: #1e293b; padding: 25px; border-radius: 12px; box-shadow: 0 4px 6px -1px rgba(0,0,0,0.3); border: 1px solid #334155; }
        h3 { margin-top: 0; color: #94a3b8; font-size: 1rem; text-transform: uppercase; }
        .val { font-size: 3.5rem; font-weight: 700; color: #10b981; margin: 10px 0 0 0; font-variant-numeric: tabular-nums; }
        .val.dual { font-size: 2.5rem; color: #38bdf8; }
        .phase { font-size: 1.5rem; font-weight: 700; color: #f59e0b; }
        #status { margin-top: 20px; font-weight: 600; font-size: 1.2rem; color: #f59e0b; }
        .connected { color: #10b981 !important; }
        .error { color: #ef4444 !important; }
        pre { margin: 0; text-align: left; white-space: pre-wrap; font-family: Consolas, monospace; font-size: 0.95rem; line-height: 1.35; color: #e2e8f0; }
    </style>
</head>
<body>
    <h1>Micromouse Control Center</h1>
    <div id="status">Connecting to Bot...</div>
    <div class="grid">
        <div class="card" style="grid-column: 1 / -1;"><h3>Phase</h3><div id="val-phase" class="val phase">SURVEY</div></div>
        <div class="card"><h3>Target Velocity</h3><div id="val-target" class="val">0.0</div></div>
        <div class="card"><h3>Current Yaw</h3><div id="val-yaw" class="val">0.00°</div></div>
        <div class="card" style="grid-column: 1 / -1;"><h3>LiDAR (Left / Front / Right)</h3><div id="val-lidar" class="val dual">0 / 0 / 0</div></div>
        <div class="card" style="grid-column: 1 / -1;"><h3>Encoder Ticks (Left / Right)</h3><div id="val-enc" class="val dual">0 / 0</div></div>
        <div class="card" style="grid-column: 1 / -1;"><h3>PWM Power (Left / Right)</h3><div id="val-pwm" class="val dual">0 / 0</div></div>
        <div class="card" style="grid-column: 1 / -1;"><h3>Cell (Row / Col / Heading)</h3><div id="val-cell" class="val dual">0 / 0 / N</div></div>
        <div class="card" style="grid-column: 1 / -1;"><h3>Explored Maze</h3><pre id="maze-output">Maze data will appear here after survey completes.</pre></div>
    </div>
    <script>
        var gateway = `ws://${window.location.hostname}:81/`;
        var websocket;
        window.addEventListener('load', initWebSocket);
        function initWebSocket() {
            websocket = new WebSocket(gateway);
            websocket.onopen    = () => { document.getElementById('status').innerText = "Status: Live"; document.getElementById('status').className = "connected"; };
            websocket.onclose   = () => { document.getElementById('status').innerText = "Disconnected. Reconnecting..."; document.getElementById('status').className = "error"; setTimeout(initWebSocket, 2000); };
            websocket.onmessage = (event) => {
                let line = event.data;
                if (line.startsWith("MAZE:")) {
                    document.getElementById('maze-output').textContent = line.substring(5).trim();
                    return;
                }
                if (!line.includes("Target:")) return;
                try {
                    const parts = line.split('|');
                    document.getElementById('val-target').innerText = parts[0].split(':')[1].trim();
                    document.getElementById('val-enc').innerText    = parts[1].split(':')[1].trim();
                    document.getElementById('val-pwm').innerText    = parts[2].split(':')[1].trim();
                    document.getElementById('val-yaw').innerText    = parts[3].split(':')[1].trim() + "°";
                    document.getElementById('val-lidar').innerText  = parts[4].split(':')[1].trim() + " mm";
                    document.getElementById('val-cell').innerText   = parts[5].split(':')[1].trim();
                    document.getElementById('val-phase').innerText  = parts[6] ? parts[6].split(':')[1].trim() : '';
                } catch (e) { }
            };
        }
    </script>
</body>
</html>
)rawliteral";

// Goal coordinates for survey completion
const int GOAL_ROW = 4;
const int GOAL_COL = 4;

// ==========================================
// LOOP TIMING
// ==========================================
const int LOOP_INTERVAL_MS  = 20;
const int LIDAR_INTERVAL_MS = 50;
const int PRINT_INTERVAL_MS = 100;

unsigned long lastLoopTime  = 0;
unsigned long lastLidarTime = 0;
unsigned long lastPrintTime = 0;

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
String buildMazeWebReport();
void broadcastMazeWebReport();
void executeDecisionAndTurn();
void runSurveyUpdate(float dt_motor, bool doMotor);

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
  webSocket.broadcastTXT("MAZE:" + buildMazeWebReport());
}

// ==========================================
// EXECUTE DECISION AND TURN (survey phase)
// Right-hand rule: right → straight → left → U-turn
// ==========================================
void executeDecisionAndTurn() {
  current_lidars = readLidars();
  delay(20);
  float turnDeltaDeg = 0.0f;

  bool rightOpen = (current_lidars.right > WALL_MISSING_THRESHOLD);
  bool frontOpen = (current_lidars.front > FRONT_BLOCKED_THRESHOLD);
  bool leftOpen  = (current_lidars.left  > WALL_MISSING_THRESHOLD);

  Serial.printf("[DEC] R=%s F=%s L=%s\n",
    rightOpen ? "open" : "wall",
    frontOpen ? "open" : "wall",
    leftOpen  ? "open" : "wall");

  if      (rightOpen) { Serial.println("[DEC] TURN RIGHT");        turnCW90();  turnDeltaDeg = -90.0f;  }
  else if (frontOpen) { Serial.println("[DEC] STRAIGHT");                                            }
  else if (leftOpen)  { Serial.println("[DEC] TURN LEFT");         turnACW90(); turnDeltaDeg = 90.0f;   }
  else                { Serial.println("[DEC] DEAD END — U-TURN"); turn180();   turnDeltaDeg = -180.0f; }

  delay(100);
  baseTargetYaw = wrapAngleDegrees(baseTargetYaw + turnDeltaDeg);
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
    case WStype_DISCONNECTED: Serial.printf("[WS] #%u Disconnected\n", num); break;
    case WStype_CONNECTED:
      Serial.printf("[WS] #%u Connected\n",    num);
      if (surveyComplete) {
        webSocket.sendTXT(num, "MAZE:" + buildMazeWebReport());
      }
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

  Serial.print("AP IP: "); Serial.println(WiFi.softAPIP());
  Serial.println("=================================");
  Serial.println("  MICROMOUSE STARTUP");
  Serial.println("=================================");

  initMotors();
  initEncoders();
  initSensors();
  delay(5000);

  Serial.println("[!] Calibrating gyro...");
  delay(2000);
  calibrateGyro();
  Serial.println("Gyro OK.");

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
  lastLoopTime = lastLidarTime = lastPrintTime = now;

  surveyState    = DRIVING;
  surveyComplete = false;

  Serial.println("Setup complete — survey mode only");
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

    if (!surveyComplete && surveyState == DRIVING) {
      runWallPIDLoop(dt_lidar);
    }
  }

  if (doMotor) {
    float dt_motor = (now - lastLoopTime) / 1000.0f;
    lastLoopTime   = now;

    if (surveyComplete) {
      applyMotorPWM(0, 0);
    } else {
      runSurveyUpdate(dt_motor, true);
    }
  }
}

// ==========================================
// SURVEY UPDATE
// Right-hand-rule traversal with wall mapping.
// Identical logic to the previous working ino
// but now neatly encapsulated so the main
// loop can switch cleanly to other phases.
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
      Serial.println("\n[SURVEY] Goal cell reached! Survey complete.");
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
      Serial.println("[SURVEY] Cooldown over — DRIVING");
    } else {
      runDrivingPID(dt_motor);
    }

  } else {  // DRIVING
    bool rightOpen = (current_lidars.right > WALL_MISSING_THRESHOLD);
    bool leftOpen  = (current_lidars.left  > WALL_MISSING_THRESHOLD);
    bool frontOpen = (current_lidars.front > FRONT_BLOCKED_THRESHOLD);

    bool rightGapJustOpened = (rightOpen && rightWallWasPresent);
    bool mustDecide = (!frontOpen) || (newCell && (rightGapJustOpened || rightOpen));

    if (mustDecide) {
      Serial.println("[SURVEY] Decision point");
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
// WALL CENTERING PID (survey phase)
// Only called when surveyState == DRIVING
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
    surveyComplete ? "SURVEY DONE" : "SURVEY"
  );

  webSocket.broadcastTXT(buf);
  Serial.println(buf);
}
