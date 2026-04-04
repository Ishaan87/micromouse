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
        #status { margin-top: 20px; font-weight: 600; font-size: 1.2rem; color: #f59e0b; }
        .connected { color: #10b981 !important; }
        .error { color: #ef4444 !important; }
    </style>
</head>
<body>
    <h1>Micromouse Control Center</h1>
    <div id="status">Connecting to Bot...</div>
    <div class="grid">
        <div class="card"><h3>Target Velocity</h3><div id="val-target" class="val">0.0</div></div>
        <div class="card"><h3>Current Yaw</h3><div id="val-yaw" class="val">0.00°</div></div>
        <div class="card" style="grid-column: 1 / -1;"><h3>LiDAR (Left / Front / Right)</h3><div id="val-lidar" class="val dual">0 / 0 / 0</div></div>
        <div class="card" style="grid-column: 1 / -1;"><h3>Encoder Ticks (Left / Right)</h3><div id="val-enc" class="val dual">0 / 0</div></div>
        <div class="card" style="grid-column: 1 / -1;"><h3>PWM Power (Left / Right)</h3><div id="val-pwm" class="val dual">0 / 0</div></div>
        <div class="card" style="grid-column: 1 / -1;"><h3>Cell (Row / Col / Heading)</h3><div id="val-cell" class="val dual">0 / 0 / N</div></div>
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
                if (!line.includes("Target:")) return;
                try {
                    const parts = line.split('|');
                    document.getElementById('val-target').innerText = parts[0].split(':')[1].trim();
                    document.getElementById('val-enc').innerText    = parts[1].split(':')[1].trim();
                    document.getElementById('val-pwm').innerText    = parts[2].split(':')[1].trim();
                    document.getElementById('val-yaw').innerText    = parts[3].split(':')[1].trim() + "°";
                    document.getElementById('val-lidar').innerText  = parts[4].split(':')[1].trim() + " mm";
                    document.getElementById('val-cell').innerText   = parts[5].split(':')[1].trim();
                } catch (e) { }
            };
        }
    </script>
</body>
</html>
)rawliteral";

// ==========================================
// TUNING PARAMETERS
// ==========================================
// Velocity PID
float Kp_vel_L = 1.0,  Ki_vel_L = 0.0, Kd_vel_L = 0.1;
float Kp_vel_R = 1.0,  Ki_vel_R = 0.0, Kd_vel_R = 0.1;

// Yaw PID
float Kp_yaw = 0.6,  Ki_yaw = 0.0,  Kd_yaw = 0.06;

// Wall-centering PID
float Kp_tunnel = 0.12, Kd_tunnel = 0.08;
float Kp_single = 0.06, Kd_single = 0.12;
float Kp_wall   = Kp_tunnel;
float Kd_wall   = Kd_tunnel;
float Ki_wall   = 0.0;

float baseTargetVelocity = 50.0;
int   basePWM            = 35;

// Heading management
float baseTargetYaw    = 0.0;
float correction_angle = 0.0;
float targetYaw        = 0.0;

// Tolerances
float vel_tolerance  = 0.5;
float yaw_tolerance  = 0.5;
float wall_tolerance = 10.0;

// ==========================================
// GEOMETRY  (155 mm cells)
// ==========================================
const int   WALL_THRESHOLD         = 110;   // lidar < this → wall present for PID
const float SINGLE_WALL_TARGET_MM  = 63.0f;
const int   FRONT_STOP_MM          = 70;    // start braking
const int   FRONT_HALT_MM          = 55;    // full stop

// ==========================================
// EKF / ODOMETRY CONSTANTS
// ==========================================
const float TICKS_PER_REV        = 306.0f;
const float WHEEL_CIRCUMFERENCE_MM = 144.5f;
const float TRACK_WIDTH_MM       = 72.0f;

// ==========================================
// REACTIVE SOLVER CONSTANTS
// ==========================================
const int WALL_MISSING_THRESHOLD  = 180;  
const int FRONT_BLOCKED_THRESHOLD = 70;   
const float PIVOT_OFFSET_MM = 77.5f;

// ==========================================
// SYSTEM VARIABLES
// ==========================================
const int LOOP_INTERVAL_MS  = 20;
const int LIDAR_INTERVAL_MS = 50;
const int PRINT_INTERVAL_MS = 100;

unsigned long lastLoopTime  = 0;
unsigned long lastLidarTime = 0;
unsigned long lastPrintTime = 0;

float integral_vel_L = 0, integral_vel_R = 0;
float prev_error_vel_L = 0, prev_error_vel_R = 0;
float integral_yaw = 0, prev_error_yaw = 0;
float current_yaw_angle = 0.0;
float integral_wall = 0, prev_error_wall = 0;

long prevLeftTicks  = 0;
long prevRightTicks = 0;
int  final_pwm_L    = 0;
int  final_pwm_R    = 0;

DistanceData current_lidars;
EKFTelemetry ekfTelemetry;

// ==========================================
// REACTIVE SOLVER STATE
// ==========================================
enum BotState {
  DRIVING,
  TURN_COOLDOWN,
  FINISHED
};

BotState      currentState      = DRIVING;
unsigned long cooldownStartMs   = 0;
const unsigned long TURN_COOLDOWN_MS = 600; 

bool rightWallWasPresent = true;
bool leftWallWasPresent  = true;

// ==========================================
// FORWARD DECLARATIONS
// ==========================================
void runWallPIDLoop(float dt);
void runDrivingPID(float dt_motor, bool frontBlocked);
void printTelemetry();
void driveForwardDistanceMM(float distanceMM);
void resetPIDIntegrals();
float frontBrakeScale();

// ==========================================
// RESET HELPERS
// ==========================================
void resetPIDIntegrals() {
  integral_vel_L = 0; prev_error_vel_L = 0;
  integral_vel_R = 0; prev_error_vel_R = 0;
  integral_yaw   = 0; prev_error_yaw   = 0;
  integral_wall  = 0; prev_error_wall  = 0;
}

// ==========================================
// BLOCKING HELPER: DRIVE DISTANCE
// ==========================================
void driveForwardDistanceMM(float distanceMM) {
  noInterrupts();
  long startLeft  = leftTicks;
  long startRight = rightTicks;
  interrupts();

  float targetTicks  = (distanceMM / WHEEL_CIRCUMFERENCE_MM) * TICKS_PER_REV;
  float yawRef       = readYawDegrees();

  while (true) {
    noInterrupts();
    long curLeft  = leftTicks;
    long curRight = rightTicks;
    interrupts();

    float avgMoved = ((curLeft - startLeft) + (curRight - startRight)) / 2.0f;
    if (avgMoved >= targetTicks) {
      applyMotorPWM(0, 0);
      break;
    }

    float yaw_err = yawRef - readYawDegrees();
    float corr    = Kp_yaw * yaw_err * 20.0f;
    applyMotorPWM(60 - (int)corr, 60 + (int)corr);
    delay(10);
  }
  delay(100); 
}

// ==========================================
// FRONT WALL BRAKE SCALE
// ==========================================
float frontBrakeScale() {
  int d = current_lidars.front;
  if (d >= FRONT_STOP_MM) return 1.0f;
  if (d <= FRONT_HALT_MM) return 0.0f;
  return (float)(d - FRONT_HALT_MM) / (float)(FRONT_STOP_MM - FRONT_HALT_MM);
}


// ==========================================
// WEBSOCKET EVENT HANDLER
// ==========================================
void webSocketEvent(uint8_t num, WStype_t type, uint8_t * payload, size_t length) {
  switch(type) {
    case WStype_DISCONNECTED:
      Serial.printf("[WS] Client #%u Disconnected\n", num);
      break;
    case WStype_CONNECTED:
      Serial.printf("[WS] Client #%u Connected!\n", num);
      break;
  }
}

// ==========================================
// SETUP
// ==========================================
void setup() {
  Serial.begin(115200);
  delay(1000);

  Serial.println("\nStarting Wi-Fi Access Point...");
  WiFi.softAP(ssid, password);
  server.on("/", []() { server.send(200, "text/html", index_html); });
  server.begin();
  webSocket.begin();
  webSocket.onEvent(webSocketEvent); // <--- ADD THIS LINE
  
  Serial.print("Connect to Wi-Fi: ");  Serial.println(ssid);
  Serial.print("Open browser to: http://"); Serial.println(WiFi.softAPIP());

  Serial.println("\n=================================");
  Serial.println("  MICROMOUSE SYSTEM STARTUP");
  Serial.println("=================================");

  initMotors();
  initEncoders();
  initSensors();
  delay(5000);

  Serial.println("\n[!] BNO055 CALIBRATION [!]");
  Serial.println("Keep bot still...");
  delay(2000);
  calibrateGyro();
  Serial.println("Gyro Calibration OK.");

  resetYaw();
  current_yaw_angle = 0.0;
  baseTargetYaw     = 0.0;
  targetYaw         = 0.0;

  ekfConfigure(TICKS_PER_REV, WHEEL_CIRCUMFERENCE_MM, TRACK_WIDTH_MM);
  ekfInit(0.0f, 0.0f, 0.0f);
  ekfTelemetry = ekfGetTelemetry();

  initCellTracker();
  initWallMap();

  resetEncoders();
  prevLeftTicks  = 0;
  prevRightTicks = 0;
  resetPIDIntegrals();

  current_lidars = readLidars();
  rightWallWasPresent = (current_lidars.right < WALL_MISSING_THRESHOLD);
  leftWallWasPresent  = (current_lidars.left  < WALL_MISSING_THRESHOLD);

  unsigned long now = millis();
  lastLoopTime  = now;
  lastLidarTime = now;
  lastPrintTime = now;
  currentState  = DRIVING;

  Serial.println("Setup complete. Running.");
}

// ==========================================
// MAIN LOOP
// ==========================================
void loop() {
  // ── 1. WI-FI SERVING ──
  webSocket.loop();
  server.handleClient();

  unsigned long now = millis();
  
  // ── 2. TELEMETRY (At the top so it never stutters) ──
  if (now - lastPrintTime >= PRINT_INTERVAL_MS) {
    lastPrintTime = now;
    printTelemetry();
  }

  bool doLidar = (now - lastLidarTime >= LIDAR_INTERVAL_MS);
  bool doMotor = (now - lastLoopTime  >= LOOP_INTERVAL_MS);

  // ── 3. LIDAR UPDATE ──
  if (doLidar) {
    float dt_lidar = (now - lastLidarTime) / 1000.0f;
    lastLidarTime  = now;
    current_lidars = readLidars();

    if (currentState == DRIVING) {
      runWallPIDLoop(dt_lidar);
    }
  }

  // ── 4. MOTOR / SOLVER UPDATE ──
  if (doMotor) {
    float dt_motor = (now - lastLoopTime) / 1000.0f;
    lastLoopTime   = now;

    // ── EKF ──
    noInterrupts();
    long currentLeftTicks  =  leftTicks;
    long currentRightTicks = rightTicks;
    interrupts();

    long deltaLeft  = currentLeftTicks  - prevLeftTicks;
    long deltaRight = currentRightTicks - prevRightTicks;

    current_yaw_angle = readYawDegrees();
    ekfPredict(deltaLeft, deltaRight);
    ekfUpdateYawDeg(current_yaw_angle);
    ekfTelemetry = ekfGetTelemetry();

    // ── CELL TRACKER ──
    bool newCell = updateCellTracker();
    if (newCell) {
      recordWalls(ctGetRow(), ctGetCol(), ctGetHeading(), current_lidars);
      printCellWalls(ctGetRow(), ctGetCol());
      printCellState();
    }

    // <-- ADD THIS GOAL CHECK HERE -->
    if (ctGetRow() == 4 && ctGetCol() == 4) {
      applyMotorPWM(0, 0);
      Serial.println("\n*** GOAL REACHED! ***");
      currentState = FINISHED;
    }

    // If finished, do absolutely nothing else.
    if (currentState == FINISHED) {
      applyMotorPWM(0, 0);
      return; 
    }

    prevLeftTicks  = currentLeftTicks;
    prevRightTicks = currentRightTicks;

    // ── COOLDOWN STATE ──
    if (currentState == TURN_COOLDOWN) {
      if (now - cooldownStartMs >= TURN_COOLDOWN_MS) {
        current_lidars      = readLidars();
        rightWallWasPresent = (current_lidars.right < WALL_MISSING_THRESHOLD);
        leftWallWasPresent  = (current_lidars.left  < WALL_MISSING_THRESHOLD);
        currentState        = DRIVING;
        Serial.println("[RX] Cooldown over — resuming normal drive.");
      } else {
        runDrivingPID(dt_motor, false);
        // We do NOT return here anymore, allowing the loop to finish cleanly
      }
    } 
    // ── DRIVING STATE ──
    else {
      bool rightOpen  = (current_lidars.right > WALL_MISSING_THRESHOLD);
      bool leftOpen   = (current_lidars.left  > WALL_MISSING_THRESHOLD);
      bool frontOpen  = (current_lidars.front > FRONT_BLOCKED_THRESHOLD);

      bool rightGapSustained = (rightOpen && !rightWallWasPresent);
      bool shouldDecide = (!frontOpen) || rightGapSustained;

      if (shouldDecide) {
        // applyMotorPWM(0, 0);
        Serial.println("\n[RX] INTERSECTION / DECISION POINT");

        if (frontOpen && PIVOT_OFFSET_MM > 0.0f) {
          Serial.println("[RX] Pivoting to cell centre...");
          driveForwardDistanceMM(PIVOT_OFFSET_MM);
        }

        current_lidars = readLidars();
        rightOpen  = (current_lidars.right > WALL_MISSING_THRESHOLD);
        leftOpen   = (current_lidars.left  > WALL_MISSING_THRESHOLD);
        frontOpen  = (current_lidars.front > FRONT_BLOCKED_THRESHOLD);

        if (rightOpen) {
          Serial.println("[RX] Decision: TURN RIGHT");
          turnCW90();
        } else if (frontOpen) {
          Serial.println("[RX] Decision: GO STRAIGHT");
        } else if (leftOpen) {
          Serial.println("[RX] Decision: TURN LEFT");
          turnACW90();
        } else {
          Serial.println("[RX] Decision: DEAD END — U-TURN");
          turn180();
        }

        baseTargetYaw = readYawDegrees();
        targetYaw     = baseTargetYaw;
        resetPIDIntegrals();

        currentState    = TURN_COOLDOWN;
        cooldownStartMs = millis();

      } else {
        runDrivingPID(dt_motor, !frontOpen);
      }

      rightWallWasPresent = !rightOpen;
      leftWallWasPresent  = !leftOpen;
    }
  }
}

// ==========================================
// DRIVING PID
// ==========================================
void runDrivingPID(float dt_motor, bool frontBlocked) {
  noInterrupts();
  long curLeft  =  leftTicks;
  long curRight = rightTicks;
  interrupts();

  float vel_L = (float)(curLeft  - prevLeftTicks);
  float vel_R = (float)(curRight - prevRightTicks);

  float error_yaw = targetYaw - current_yaw_angle;
  if (fabsf(error_yaw) <= yaw_tolerance) { error_yaw = 0; prev_error_yaw = 0; }
  integral_yaw += error_yaw * dt_motor;
  float deriv_yaw = (error_yaw - prev_error_yaw) / dt_motor;
  float heading_corr = (Kp_yaw * error_yaw) + (Ki_yaw * integral_yaw) + (Kd_yaw * deriv_yaw);
  prev_error_yaw = error_yaw;

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
// ==========================================
void runWallPIDLoop(float dt) {
  bool hasLeft  = (current_lidars.left  < WALL_THRESHOLD);
  bool hasRight = (current_lidars.right < WALL_THRESHOLD);
  float error_wall = 0.0f;

  if (hasLeft && hasRight) {
    Kp_wall    = Kp_tunnel;
    Kd_wall    = Kd_tunnel;
    error_wall = (float)(current_lidars.left - current_lidars.right);

  } else if (hasLeft && !hasRight) {
    Kp_wall    = Kp_single;
    Kd_wall    = Kd_single;
    error_wall = -(current_lidars.left - SINGLE_WALL_TARGET_MM);

  } else if (!hasLeft && hasRight) {
    Kp_wall    = Kp_single;
    Kd_wall    = Kd_single;
    error_wall = (current_lidars.right - SINGLE_WALL_TARGET_MM);

  } else {
    correction_angle = 0.0f;
    integral_wall    = 0.0f;
    prev_error_wall  = 0.0f;
    targetYaw        = baseTargetYaw;
    return;
  }

  if (fabsf(error_wall) <= wall_tolerance) {
    error_wall    = 0.0f;
    integral_wall = 0.0f;
  }

  integral_wall += error_wall * dt;
  float deriv_wall  = (error_wall - prev_error_wall) / dt;
  float target_corr = (Kp_wall * error_wall) + (Ki_wall * integral_wall) + (Kd_wall * deriv_wall);

  correction_angle = (correction_angle * 0.7f) + (target_corr * 0.3f);
  correction_angle = constrain(correction_angle, -8.0f, 8.0f);

  prev_error_wall = error_wall;
  targetYaw       = baseTargetYaw + correction_angle;
}

// ==========================================
// TELEMETRY
// ==========================================
void printTelemetry() {
  noInterrupts();
  long currLeft  = leftTicks;
  long currRight = rightTicks;
  interrupts();

  char buf[300];
  snprintf(buf, sizeof(buf),
    "Target: %.1f | Enc: %ld / %ld | PWM: %d / %d | Yaw: %.2f | Lidar: %d / %d / %d | Cell: %d/%d/%s",
    baseTargetVelocity,
    currLeft, currRight,
    final_pwm_L, final_pwm_R,
    current_yaw_angle,
    current_lidars.left, current_lidars.front, current_lidars.right,
    ctGetRow(), ctGetCol(), headingName(ctGetHeading())
  );

  webSocket.broadcastTXT(buf);
  Serial.println(buf);
}