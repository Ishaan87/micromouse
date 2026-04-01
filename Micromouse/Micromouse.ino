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

// Wall-centering PID (two gain sets: tunnel = both walls, single = one wall)
float Kp_tunnel = 0.12, Kd_tunnel = 0.08;
float Kp_single = 0.06, Kd_single = 0.12;
float Kp_wall   = Kp_tunnel;
float Kd_wall   = Kd_tunnel;
float Ki_wall   = 0.0;

float baseTargetVelocity = 15.0;
int   basePWM            = 50;

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
const int   FRONT_STOP_MM          = 110;   // start braking
const int   FRONT_HALT_MM          = 65;    // full stop

// ==========================================
// EKF / ODOMETRY CONSTANTS
// ==========================================
const float TICKS_PER_REV        = 306.0f;
const float WHEEL_CIRCUMFERENCE_MM = 144.5f;
const float TRACK_WIDTH_MM       = 72.0f;

// ==========================================
// REACTIVE SOLVER CONSTANTS
// ==========================================
// BUG FIX 1: These two thresholds were swapped/inconsistent between branches.
// WALL_THRESHOLD (110) is for PID centering.
// WALL_MISSING_THRESHOLD (180) is for detecting an open gap at an intersection.
const int WALL_MISSING_THRESHOLD  = 180;   // lidar > this → no wall, gap detected
const int FRONT_BLOCKED_THRESHOLD = 110;   // lidar < this → front wall present

// BUG FIX 2: PIVOT_OFFSET_MM was 77.5 (half a 155mm cell).
// The robot needs to be at the CENTER of the intersection to turn correctly,
// not half a cell ahead. The center of the current cell relative to where
// gap detection fires is approximately half a cell ahead.
// Tune this on your actual maze — start with 0 and increase if the robot
// turns short of the intersection centre.
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
// BUG FIX 3: The original code used edge-trigger detection
// (gap "just appeared") which caused the rotation loop:
//
//   1. Robot sees right gap → triggers turn → turns right
//   2. After turn, lidar reads the NEW corridor as another "gap just appeared"
//      because rightWallWasPresent was still true from before the turn
//   3. Triggers another turn → robot spins forever
//
// FIX: Track a cooldown period after any turn. During cooldown,
// gap detection is suppressed so the robot drives into the new
// corridor before looking for the next intersection.
//
// Also added TURN_DEBOUNCE_MS: minimum time between turns.

enum BotState {
  DRIVING,
  TURN_COOLDOWN   // replaces the old ALIGNING_FOR_TURN state
};

BotState      currentState      = DRIVING;
unsigned long cooldownStartMs   = 0;
const unsigned long TURN_COOLDOWN_MS = 600; // ms to ignore gaps after a turn

// These track whether a wall was CONTINUOUSLY absent,
// not just absent this one cycle (extra debounce)
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
// Drives forward a fixed distance using
// encoder ticks. Used for pivot offset.
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
  delay(100); // brief settle pause
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

  // Force a fresh lidar read before we start
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
  webSocket.loop();
  server.handleClient();

  unsigned long now = millis();
  if (now - lastPrintTime >= PRINT_INTERVAL_MS) {
    lastPrintTime = now;
    printTelemetry();
  }

  bool doLidar = (now - lastLidarTime >= LIDAR_INTERVAL_MS);
  bool doMotor = (now - lastLoopTime  >= LOOP_INTERVAL_MS);
  bool doPrint = (now - lastPrintTime >= PRINT_INTERVAL_MS);


  // ── LIDAR UPDATE ──────────────────────────────────────────
  if (doLidar) {
    float dt_lidar = (now - lastLidarTime) / 1000.0f;
    lastLidarTime  = now;
    current_lidars = readLidars();

    // Only run wall-centering PID when driving normally
    if (currentState == DRIVING) {
      runWallPIDLoop(dt_lidar);
    }
  }

  // ── MOTOR / SOLVER UPDATE ─────────────────────────────────
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

    prevLeftTicks  = currentLeftTicks;
    prevRightTicks = currentRightTicks;

    // ── COOLDOWN STATE ────────────────────────────────────────
    // BUG FIX 3 continued: while in cooldown just drive forward,
    // suppress all turn decisions.
    if (currentState == TURN_COOLDOWN) {
      if (now - cooldownStartMs >= TURN_COOLDOWN_MS) {
        // Re-sample lidar to get the actual new-corridor reading
        current_lidars      = readLidars();
        rightWallWasPresent = (current_lidars.right < WALL_MISSING_THRESHOLD);
        leftWallWasPresent  = (current_lidars.left  < WALL_MISSING_THRESHOLD);
        currentState        = DRIVING;
        Serial.println("[RX] Cooldown over — resuming normal drive.");
      } else {
        // During cooldown: drive straight, no turning
        runDrivingPID(dt_motor, false);
        return;
      }
    }

    // ── REACTIVE SOLVER (DRIVING STATE) ──────────────────────
    bool rightOpen  = (current_lidars.right > WALL_MISSING_THRESHOLD);
    bool leftOpen   = (current_lidars.left  > WALL_MISSING_THRESHOLD);
    bool frontOpen  = (current_lidars.front > FRONT_BLOCKED_THRESHOLD);

    // BUG FIX 4: Old code used gap-edge-trigger detection.
    // This fired when a wall DISAPPEARED this cycle vs last cycle.
    // Problem: after turning, both walls appear "new" because the
    // entire scene changed, causing immediate re-triggering.
    //
    // New approach: trigger a turn decision when:
    //   (a) the front is blocked, OR
    //   (b) the right is open AND was open last cycle too
    //       (sustained, not just a one-frame flicker)
    //
    // This is much more robust than edge detection.

    bool rightGapSustained = (rightOpen && !rightWallWasPresent);
    // Note: rightWallWasPresent is now updated every cycle below,
    // so rightGapSustained becomes true only when the right has been
    // open for at least one full lidar cycle before this one.

    bool shouldDecide = (!frontOpen) || rightGapSustained;

    if (shouldDecide) {
      applyMotorPWM(0, 0);
      Serial.println("\n[RX] INTERSECTION / DECISION POINT");

      // Pivot forward to cell centre if the front is still open
      if (frontOpen && PIVOT_OFFSET_MM > 0.0f) {
        Serial.println("[RX] Pivoting to cell centre...");
        driveForwardDistanceMM(PIVOT_OFFSET_MM);
      }

      // Re-read lidar after the pivot — scene may have changed
      current_lidars = readLidars();
      rightOpen  = (current_lidars.right > WALL_MISSING_THRESHOLD);
      leftOpen   = (current_lidars.left  > WALL_MISSING_THRESHOLD);
      frontOpen  = (current_lidars.front > FRONT_BLOCKED_THRESHOLD);

      // Right-hand rule priority: RIGHT > STRAIGHT > LEFT > U-TURN
      if (rightOpen) {
        Serial.println("[RX] Decision: TURN RIGHT");
        turnCW90();
      } else if (frontOpen) {
        Serial.println("[RX] Decision: GO STRAIGHT");
        // No physical turn needed
      } else if (leftOpen) {
        Serial.println("[RX] Decision: TURN LEFT");
        turnACW90();
      } else {
        Serial.println("[RX] Decision: DEAD END — U-TURN");
        turn180();
      }

      // Snap yaw reference to settled heading
      baseTargetYaw = readYawDegrees();
      targetYaw     = baseTargetYaw;
      resetPIDIntegrals();

      // Enter cooldown — suppresses re-triggering for TURN_COOLDOWN_MS
      currentState    = TURN_COOLDOWN;
      cooldownStartMs = millis();

    } else {
      // ── NORMAL DRIVING PID ──
      runDrivingPID(dt_motor, !frontOpen);
    }

    // Update wall presence history for next cycle
    rightWallWasPresent = !rightOpen;
    leftWallWasPresent  = !leftOpen;
  }

  // ── TELEMETRY ─────────────────────────────────────────────
  if (doPrint) {
    lastPrintTime = now;
    // printTelemetry();
  }
}

// ==========================================
// DRIVING PID
// Extracted from the loop so both the normal
// path and the cooldown path share the same code.
// ==========================================
void runDrivingPID(float dt_motor, bool frontBlocked) {
  noInterrupts();
  long curLeft  =  leftTicks;
  long curRight = rightTicks;
  interrupts();

  float vel_L = (float)(curLeft  - prevLeftTicks);
  float vel_R = (float)(curRight - prevRightTicks);

  // Yaw PID
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

  // Low-pass filter on correction to smooth out lidar noise
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
