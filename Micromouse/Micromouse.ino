#include <Arduino.h>
#include "Config.h"
#include "Motors.h"
#include "Encoders.h"
#include "Sensors.h"
#include "ekf.h"
#include "CellTracker.h"
#include "WallMap.h"
#include "Turns.h"
#include "Floodfill.h"   // ← new
#include "Solver.h"      // ← new

// ==========================================
<<<<<<< HEAD
// TUNING PARAMETERS
=======
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
                    document.getElementById('val-phase').innerText  = parts[6] ? parts[6].split(':')[1].trim() : '';
                } catch (e) { }
            };
        }
    </script>
</body>
</html>
)rawliteral";

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

<<<<<<< HEAD
// Yaw PID
float Kp_yaw = 0.75,  Ki_yaw = 0.0,  Kd_yaw = 0.06;

// Wall-centering PID
float Kp_tunnel = 0.15, Kd_tunnel = 0.08;
float Kp_single = 0.06, Kd_single = 0.12;
float Kp_wall   = Kp_tunnel;
float Kd_wall   = Kd_tunnel;
float Ki_wall   = 0.0;

// Front Wall Braking PD
float Kp_front = 0.009;   
float Kd_front = 0.001;  
float prev_error_front = -1.0f; // Initialized to -1 to prevent math spikes

float baseTargetVelocity = 25.0;
int   basePWM            = 125;
=======
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
>>>>>>> cf170e537a06b5130a9b7b7b3701c0930bc2ed03

// ==========================================
// HEADING / YAW MANAGEMENT
// ==========================================
float baseTargetYaw    = 0.0f;
float correction_angle = 0.0f;
float targetYaw        = 0.0f;

// ==========================================
// TOLERANCES
// ==========================================
<<<<<<< HEAD
const int   WALL_THRESHOLD         = 110;
const float SINGLE_WALL_TARGET_MM  = 63.0f;
const int   FRONT_STOP_MM          = 150; // start braking
const int   FRONT_HALT_MM          = 60;  // full stop
=======
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
>>>>>>> cf170e537a06b5130a9b7b7b3701c0930bc2ed03

// ==========================================
// EKF / ODOMETRY CONSTANTS
// ==========================================
const float TICKS_PER_REV          = 306.0f;
<<<<<<< HEAD
const float WHEEL_CIRCUMFERENCE_MM = 144.5f;
const float TRACK_WIDTH_MM         = 72.0f;
=======
const float WHEEL_CIRCUMFERENCE_MM  = 144.5f;
const float TRACK_WIDTH_MM          = 72.0f;
>>>>>>> cf170e537a06b5130a9b7b7b3701c0930bc2ed03

// ==========================================
// DECISION THRESHOLDS
// ==========================================
<<<<<<< HEAD
const int WALL_MISSING_THRESHOLD  = 180;  
const int FRONT_BLOCKED_THRESHOLD = 70;
const float PIVOT_OFFSET_MM       = 77.5f;
=======
const int WALL_MISSING_THRESHOLD   = 180;
const int FRONT_BLOCKED_THRESHOLD  = 70;
>>>>>>> cf170e537a06b5130a9b7b7b3701c0930bc2ed03

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
// SHARED STATE (read by Solver.h via extern)
// ==========================================
float integral_vel_L = 0, integral_vel_R = 0;
float prev_error_vel_L = 0, prev_error_vel_R = 0;
<<<<<<< HEAD

float integral_yaw = 0, prev_error_yaw = 0;
float current_yaw_angle = 0.0;

float integral_wall = 0, prev_error_wall = 0;
=======
float integral_yaw = 0,     prev_error_yaw = 0;
float current_yaw_angle = 0.0f;
float integral_wall = 0,    prev_error_wall = 0;
>>>>>>> cf170e537a06b5130a9b7b7b3701c0930bc2ed03

long prevLeftTicks  = 0;
long prevRightTicks = 0;
int  final_pwm_L    = 0;
int  final_pwm_R    = 0;

DistanceData current_lidars;

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
<<<<<<< HEAD
float calculateFrontPDBrake(float dt);
=======
void resetWallPID();
float frontBrakeScale();
void alignToCellCentre();
void executeDecisionAndTurn();
void runSurveyUpdate(float dt_motor, float dt_lidar, bool doMotor, bool doLidar);
void transitionToReturn();
void transitionToSolve();
>>>>>>> cf170e537a06b5130a9b7b7b3701c0930bc2ed03

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

<<<<<<< HEAD
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

    // SAFETY ABORT
    int current_front_dist = readLidars().front;
    if (current_front_dist <= FRONT_HALT_MM) {
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
// FRONT WALL PD BRAKE SCALE
=======
void resetWallPID() {
  integral_wall    = 0;
  prev_error_wall  = 0;
  correction_angle = 0.0f;
  targetYaw        = baseTargetYaw;
}

// ==========================================
// FRONT BRAKE SCALE (survey phase)
>>>>>>> cf170e537a06b5130a9b7b7b3701c0930bc2ed03
// ==========================================
float calculateFrontPDBrake(float dt) {
  int d = current_lidars.front;

<<<<<<< HEAD
  if (d >= FRONT_STOP_MM) {
    prev_error_front = -1.0f; 
    return 1.0f;
=======
// ==========================================
// ALIGN TO CELL CENTRE (survey phase)
// Uses encoder odometry to reach exact midpoint
// ==========================================
void alignToCellCentre() {
  EKFState s = ekfGetState();
  float posInCell = fmodf(fabsf(s.x_mm), CELL_SIZE_NAV_MM);
  float remaining = CELL_HALF_MM - posInCell;

  if (remaining <= CENTRE_TOLERANCE_MM) {
    Serial.printf("[ALN] Already centred (posInCell=%.1f). Skipping.\n", posInCell);
    return;
  }

  Serial.printf("[ALN] Driving %.1fmm to centre.\n", remaining);
  float targetTicks = (remaining / WHEEL_CIRCUMFERENCE_MM) * TICKS_PER_REV;

  noInterrupts();
  long startLeft  = leftTicks;
  long startRight = rightTicks;
  interrupts();

  float yawRef = readYawDegrees();

  while (true) {
    noInterrupts();
    long cL = leftTicks, cR = rightTicks;
    interrupts();

    float moved = ((cL - startLeft) + (cR - startRight)) / 2.0f;
    if (moved >= targetTicks) { applyMotorPWM(0, 0); delay(50); break; }

    float yaw_err = yawRef - readYawDegrees();
    float corr    = Kp_yaw * yaw_err * 20.0f;
    applyMotorPWM(55 - (int)corr, 55 + (int)corr);
    delay(10);
  }
}

// ==========================================
// EXECUTE DECISION AND TURN (survey phase)
// Right-hand rule: right → straight → left → U-turn
// ==========================================
void executeDecisionAndTurn() {
  current_lidars = readLidars();
  delay(20);

  bool rightOpen = (current_lidars.right > WALL_MISSING_THRESHOLD);
  bool frontOpen = (current_lidars.front > FRONT_BLOCKED_THRESHOLD);
  bool leftOpen  = (current_lidars.left  > WALL_MISSING_THRESHOLD);

  Serial.printf("[DEC] R=%s F=%s L=%s\n",
    rightOpen ? "open" : "wall",
    frontOpen ? "open" : "wall",
    leftOpen  ? "open" : "wall");

  if      (rightOpen) { Serial.println("[DEC] TURN RIGHT");        turnCW90();  }
  else if (frontOpen) { Serial.println("[DEC] STRAIGHT");                       }
  else if (leftOpen)  { Serial.println("[DEC] TURN LEFT");         turnACW90(); }
  else                { Serial.println("[DEC] DEAD END — U-TURN"); turn180();   }

  delay(100);
  baseTargetYaw = readYawDegrees();
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
    case WStype_CONNECTED:    Serial.printf("[WS] #%u Connected\n",    num); break;
>>>>>>> cf170e537a06b5130a9b7b7b3701c0930bc2ed03
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

<<<<<<< HEAD
  Serial.println("\n=================================");
  Serial.println("  STATIONARY PRIORITY FOLLOWER");
=======
  WiFi.softAP(ssid, password);
  server.on("/", []() { server.send(200, "text/html", index_html); });
  server.begin();
  webSocket.begin();
  webSocket.onEvent(webSocketEvent);

  Serial.print("AP IP: "); Serial.println(WiFi.softAPIP());
  Serial.println("=================================");
  Serial.println("  MICROMOUSE STARTUP");
>>>>>>> cf170e537a06b5130a9b7b7b3701c0930bc2ed03
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

  resetEncoders();
  prevLeftTicks = prevRightTicks = 0;
  resetPIDIntegrals();
  resetWallPID();

  current_lidars      = readLidars();
  rightWallWasPresent = (current_lidars.right < WALL_MISSING_THRESHOLD);
  leftWallWasPresent  = (current_lidars.left  < WALL_MISSING_THRESHOLD);

  unsigned long now = millis();
<<<<<<< HEAD
  lastLoopTime  = now;
  lastLidarTime = now;
  lastPrintTime = now;
  
  currentState  = DRIVING;
  isStartup     = true; 

  Serial.println("Setup complete.");
=======
  lastLoopTime = lastLidarTime = lastPrintTime = now;

  currentPhase = PHASE_SURVEY;
  surveyState  = DRIVING;

  Serial.println("Setup complete — PHASE: SURVEY");
>>>>>>> cf170e537a06b5130a9b7b7b3701c0930bc2ed03
}

// ==========================================
// MAIN LOOP
// ==========================================
void loop() {
<<<<<<< HEAD
=======
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

    // Wall PID only during survey straight-driving
    if (currentPhase == PHASE_SURVEY && surveyState == DRIVING) {
      runWallPIDLoop(dt_lidar);
    }
    // Solver handles its own wall PID internally
  }

  if (doMotor) {
    float dt_motor = (now - lastLoopTime) / 1000.0f;
    lastLoopTime   = now;

<<<<<<< HEAD
    noInterrupts();
    long currentLeftTicks  =  leftTicks;
    long currentRightTicks = rightTicks;
    interrupts();

    current_yaw_angle = readYawDegrees();
    prevLeftTicks  = currentLeftTicks;
    prevRightTicks = currentRightTicks;

    if (currentState == TURN_COOLDOWN) {
      if (now - cooldownStartMs >= TURN_COOLDOWN_MS) {
        current_lidars      = readLidars();
        rightWallWasPresent = (current_lidars.right < WALL_MISSING_THRESHOLD);
        leftWallWasPresent  = (current_lidars.left  < WALL_MISSING_THRESHOLD);
        currentState        = DRIVING;
      } else {
        runDrivingPID(dt_motor, false);
      }
    } 
    else {
      bool rightOpen  = (current_lidars.right > WALL_MISSING_THRESHOLD);
      bool leftOpen   = (current_lidars.left  > WALL_MISSING_THRESHOLD);
      bool frontOpen  = (current_lidars.front > FRONT_BLOCKED_THRESHOLD);

      // Trigger stops based on falling edges or reaching a front wall
      bool rightGapJustDetected = (rightOpen && !rightWallWasPresent);
      bool stoppedAtFrontWall   = (current_lidars.front <= FRONT_HALT_MM + 5);
      bool frontIsClear         = (current_lidars.front >= FRONT_STOP_MM);

      // IMMEDIATE TRIGGER: Only stop for an open intersection if the front is clear
      // (If a front wall is there, let the PD brake naturally stop us)
      bool clearIntersectionDetected = rightGapJustDetected && frontIsClear;

      bool shouldDecide = stoppedAtFrontWall || clearIntersectionDetected || isStartup;

      if (shouldDecide) {
        Serial.println("\n[RX] DECISION MATRIX TRIGGERED");

        // Center the bot in the intersection ONLY if we found an open right gap
        if (clearIntersectionDetected && !isStartup && PIVOT_OFFSET_MM > 0.0f) {
          driveForwardDistanceMM(PIVOT_OFFSET_MM);
=======
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
>>>>>>> cf170e537a06b5130a9b7b7b3701c0930bc2ed03
        }
        break;

<<<<<<< HEAD
        // KILL MOMENTUM & SETTLE
        applyMotorPWM(0, 0);
        delay(300); 

        // ── READ FRESH STATIONARY DATA ──
        // This is exactly what you asked for: Decision based purely on the lidars AFTER stopping.
        current_lidars = readLidars();
        
        bool priorityRight = (current_lidars.right > WALL_MISSING_THRESHOLD);
        bool priorityLeft  = (current_lidars.left  > WALL_MISSING_THRESHOLD);
        
        // Front priority is only valid if we didn't just stop because of a front wall
        bool priorityFront = (current_lidars.front > FRONT_BLOCKED_THRESHOLD) && !stoppedAtFrontWall;

        if (priorityRight) {
          Serial.println("[RX] Priority: TURN RIGHT");
          turnCW90();
        } else if (priorityFront) {
          Serial.println("[RX] Priority: GO STRAIGHT");
        } else if (priorityLeft) {
          Serial.println("[RX] Priority: TURN LEFT");
          turnACW90();
        } else {
          Serial.println("[RX] Priority: DEAD END — U-TURN");
          turn180();
=======
      // ──────────────────────────────────────
      case PHASE_SOLVE:
        solverUpdate(dt_motor, 0, true, false);
        if (solverDone()) {
          applyMotorPWM(0, 0);
          currentPhase = PHASE_DONE;
          Serial.println("[PHASE] SOLVE complete — ALL DONE!");
          printWallMapASCII();
>>>>>>> cf170e537a06b5130a9b7b7b3701c0930bc2ed03
        }
        break;

<<<<<<< HEAD
        // ── WIPE MEMORY FOR NEXT CORRIDOR ──
        isStartup = false;
        
        baseTargetYaw = readYawDegrees();
        targetYaw     = baseTargetYaw;
        resetPIDIntegrals();

        noInterrupts();
        prevLeftTicks  = leftTicks;
        prevRightTicks = rightTicks;
        interrupts();
        lastLoopTime   = millis();

        currentState    = TURN_COOLDOWN;
        cooldownStartMs = millis();

      } else {
        // Safe to keep driving; PD Brake is fully in charge of stopping
        runDrivingPID(dt_motor, !frontOpen);
      }

      // Record wall state for falling edge detection on the next loop
      rightWallWasPresent = !rightOpen;
      leftWallWasPresent  = !leftOpen;
=======
      // ──────────────────────────────────────
      case PHASE_DONE:
        applyMotorPWM(0, 0);
        break;
>>>>>>> cf170e537a06b5130a9b7b7b3701c0930bc2ed03
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
void runSurveyUpdate(float dt_motor, float /*dt_lidar*/, bool doMotor, bool /*doLidar*/) {
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

    // ── SURVEY GOAL CHECK ──
    // Survey ends when we first reach the goal cell.
    // At that point the wall map is good enough to plan a return path.
    if (ctGetRow() == GOAL_ROW && ctGetCol() == GOAL_COL) {
      applyMotorPWM(0, 0);
      Serial.println("\n[SURVEY] Goal cell reached! Transitioning to RETURN phase.");
      prevLeftTicks  = curL;
      prevRightTicks = curR;
      transitionToReturn();
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
      alignToCellCentre();
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
// PHASE TRANSITIONS
// ==========================================

// Called when survey goal is reached.
// Computes a return path from current cell → (0,0)
// and hands off to the solver.
void transitionToReturn() {
  Serial.println("[PHASE] Computing RETURN path...");

  int curRow = ctGetRow();
  int curCol = ctGetCol();

  bool ok = ffComputePath(curRow, curCol, 0, 0);
  if (!ok) {
    Serial.println("[PHASE] ERROR: No return path found! Staying put.");
    currentPhase = PHASE_DONE;
    return;
  }

  ffPrintDistGrid();

  // Reset odometry and PID for the new run
  resetPIDIntegrals();
  resetWallPID();
  resetEncoders();
  prevLeftTicks = prevRightTicks = 0;

  // Reset EKF to current cell centre (good-enough re-localisation)
  float cellCentreX = (curRow + 0.5f) * CELL_SIZE_NAV_MM;
  float cellCentreY = (curCol + 0.5f) * CELL_SIZE_NAV_MM;
  ekfInit(cellCentreX, cellCentreY, ekfGetState().theta_rad);

  solverInit(ctGetHeading());
  currentPhase = PHASE_RETURN;
  Serial.println("[PHASE] RETURN started.");
}

// Called when the return run reaches (0,0).
// Waits briefly, re-orients to NORTH if needed,
// then computes the optimal solve path and starts it.
void transitionToSolve() {
  Serial.println("[PHASE] At origin. Preparing SOLVE run...");

  // Orient to NORTH before solving so the path headings align
  MazeHeading h = ctGetHeading();
  if (h != HEADING_NORTH) {
    Serial.printf("[PHASE] Re-orienting from %s to NORTH\n", headingName(h));
    // Compute how many right turns needed to face NORTH
    int turns = ((int)HEADING_NORTH - (int)h + 4) % 4;
    for (int i = 0; i < turns; i++) {
      if (turns == 1)      turnCW90();   // 1 right turn
      else if (turns == 3) turnACW90();  // 1 left turn (== 3 rights)
      else                 turn180();    // 2 turns
      delay(200);
      break;  // one call handles it — turn functions do full angles
    }
    delay(200);
  }

  baseTargetYaw = readYawDegrees();
  targetYaw     = baseTargetYaw;
  resetPIDIntegrals();
  resetWallPID();
  resetEncoders();
  prevLeftTicks = prevRightTicks = 0;

  // Re-init EKF at origin
  ekfInit(0.0f, 0.0f, 0.0f);
  initCellTracker();

  // Compute shortest path from (0,0) to goal
  bool ok = ffComputePath(0, 0, GOAL_ROW, GOAL_COL);
  if (!ok) {
    Serial.println("[PHASE] ERROR: No solve path found!");
    currentPhase = PHASE_DONE;
    return;
  }

  ffPrintDistGrid();

  // 3-second pause so you can see the serial output before the fast run
  Serial.println("[PHASE] SOLVE starts in 3 seconds...");
  delay(3000);

  solverInit(HEADING_NORTH);
  currentPhase = PHASE_SOLVE;
  Serial.println("[PHASE] SOLVE started — go!");
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
<<<<<<< HEAD
  
  float deriv_yaw = (error_yaw - prev_error_yaw) / dt_motor;
=======
  float deriv_yaw   = (error_yaw - prev_error_yaw) / dt_motor;
>>>>>>> cf170e537a06b5130a9b7b7b3701c0930bc2ed03
  float heading_corr = (Kp_yaw * error_yaw) + (Ki_yaw * integral_yaw) + (Kd_yaw * deriv_yaw);
  prev_error_yaw    = error_yaw;

<<<<<<< HEAD
  float brakeFactor = calculateFrontPDBrake(dt_motor);

=======
  float brakeFactor = frontBrakeScale();
>>>>>>> cf170e537a06b5130a9b7b7b3701c0930bc2ed03
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
<<<<<<< HEAD

  float target_corr = (Kp_wall * error_wall) + (Ki_wall * integral_wall) + (Kd_wall * deriv_wall);
=======
  float corr        = (Kp_wall * error_wall) + (Ki_wall * integral_wall) + (Kd_wall * deriv_wall);
>>>>>>> cf170e537a06b5130a9b7b7b3701c0930bc2ed03

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

<<<<<<< HEAD
  char buf[200];

  snprintf(buf, sizeof(buf),
    "Target: %.1f | Enc: %ld / %ld | PWM: %d / %d | Yaw: %.2f | Lidar: %d / %d / %d",
=======
  const char* phaseStr =
    (currentPhase == PHASE_SURVEY) ? "SURVEY" :
    (currentPhase == PHASE_RETURN) ? "RETURN" :
    (currentPhase == PHASE_SOLVE)  ? "SOLVE"  : "DONE";

  char buf[320];
  snprintf(buf, sizeof(buf),
    "Target: %.1f | Enc: %ld / %ld | PWM: %d / %d | Yaw: %.2f | Lidar: %d / %d / %d | Cell: %d/%d/%s | Phase: %s",
>>>>>>> cf170e537a06b5130a9b7b7b3701c0930bc2ed03
    baseTargetVelocity,
    cL, cR,
    final_pwm_L, final_pwm_R,
    current_yaw_angle,
<<<<<<< HEAD
    current_lidars.left, current_lidars.front, current_lidars.right
=======
    current_lidars.left, current_lidars.front, current_lidars.right,
    ctGetRow(), ctGetCol(), headingName(ctGetHeading()),
    phaseStr
>>>>>>> cf170e537a06b5130a9b7b7b3701c0930bc2ed03
  );
  
  Serial.println(buf);
}
