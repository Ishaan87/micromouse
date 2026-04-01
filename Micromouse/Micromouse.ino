#include <Arduino.h>
#include <WiFi.h>
#include <WebServer.h>
#include <WebSocketsServer.h>
#include "Config.h"
#include "Motors.h"
#include "Encoders.h"
#include "Sensors.h"
#include "Turns.h" // Assumes you have your turnCW90(), turnACW90(), turn180() here

// ==========================================
// WIRELESS WEB SERVER SETTINGS
// ==========================================
const char *ssid = "Jerry";
const char *password = "mouse1234"; 

WebServer server(80);                
WebSocketsServer webSocket(81);

// ==========================================
// THE HTML DASHBOARD (Embedded)
// ==========================================
const char index_html[] PROGMEM = R"rawliteral(
<!DOCTYPE html>
<html lang="en">
<head>
    <meta charset="UTF-8">
    <meta name="viewport" content="width=device-width, initial-scale=1.0">
    <title>Varun's Micromouse Dashboard</title>
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
    <h1>🐭 Wireless Control Center</h1>
    <div id="status">Connecting to Bot...</div>

    <div class="grid">
        <div class="card">
            <h3>Target Velocity</h3>
            <div id="val-target" class="val">0.0</div>
        </div>
        <div class="card">
            <h3>Current Yaw</h3>
            <div id="val-yaw" class="val">0.00°</div>
        </div>
        <div class="card" style="grid-column: 1 / -1;">
            <h3>LiDAR (Left / Front / Right)</h3>
            <div id="val-lidar" class="val dual">0 / 0 / 0</div>
        </div>
        <div class="card" style="grid-column: 1 / -1;">
            <h3>Encoder Ticks (Left / Right)</h3>
            <div id="val-enc" class="val dual">0 / 0</div>
        </div>
        <div class="card" style="grid-column: 1 / -1;">
            <h3>PWM Power (Left / Right)</h3>
            <div id="val-pwm" class="val dual">0 / 0</div>
        </div>
    </div>

    <script>
        var gateway = `ws://${window.location.hostname}:81/`;
        var websocket;
        
        window.addEventListener('load', initWebSocket);
        function initWebSocket() {
            websocket = new WebSocket(gateway);
            websocket.onopen = onOpen;
            websocket.onclose = onClose;
            websocket.onmessage = onMessage;
        }

        function onOpen(event) {
            document.getElementById('status').innerText = "Status: Live Telemetry Connected";
            document.getElementById('status').className = "connected";
        }

        function onClose(event) {
            document.getElementById('status').innerText = "Status: Disconnected. Reconnecting...";
            document.getElementById('status').className = "error";
            setTimeout(initWebSocket, 2000); 
        }

        function onMessage(event) {
            let line = event.data;
            if (!line.includes("Target:")) return;
            try {
                const parts = line.split('|');
                document.getElementById('val-target').innerText = parts[0].split(':')[1].trim();
                document.getElementById('val-enc').innerText = parts[1].split(':')[1].trim();
                document.getElementById('val-pwm').innerText = parts[2].split(':')[1].trim();
                document.getElementById('val-yaw').innerText = parts[3].split(':')[1].trim() + "°";
                document.getElementById('val-lidar').innerText = parts[4].split(':')[1].trim() + " mm";
            } catch (e) { }
        }
    </script>
</body>
</html>
)rawliteral";


// ==========================================
// TUNING PARAMETERS
// ==========================================
// 1. Inner Loop (Velocity PID)
float Kp_vel_L = 1.0, Ki_vel_L = 0.0, Kd_vel_L = 0.1;
float Kp_vel_R = 1.0, Ki_vel_R = 0.0, Kd_vel_R = 0.1; 

// 2. Middle Loop (Yaw PID)
float Kp_yaw = 0.6, Ki_yaw = 0.0, Kd_yaw = 0.06; 

// 3. Outer Loop (Wall Centering PID)
float Kp_tunnel = 0.12, Kd_tunnel = 0.08;
float Kp_single = 0.06, Kd_single = 0.12; 
float Kp_wall = Kp_tunnel, Kd_wall = Kd_tunnel, Ki_wall = 0.0;
        
float baseTargetVelocity = 15.0;
int basePWM = 50; 

// Heading Management
float baseTargetYaw = 0.0;     
float correction_angle = 0.0;
float targetYaw = 0.0;         

float vel_tolerance = 0.5; 
float yaw_tolerance = 0.5;
float wall_tolerance = 10.0;    

// 155mm Cell Geometry Adjustments
const int WALL_THRESHOLD = 110;
const float SINGLE_WALL_TARGET_MM = 63.0;
const int FRONT_STOP_MM = 110;
const int FRONT_HALT_MM = 65;

// ==========================================
// REACTIVE MAZE SOLVER VARIABLES
// ==========================================
enum BotState { 
  DRIVING, 
  ALIGNING_FOR_TURN 
};

BotState currentState = DRIVING;

// Gap detection thresholds
const int WALL_MISSING_THRESHOLD = 180; 
const int FRONT_BLOCKED_THRESHOLD = 110; 

// Memory to detect when a gap *opens up*
bool rightWallWasPresent = true;
bool leftWallWasPresent = true;

// Physical Constants for the Pivot
const float TICKS_PER_REV = 306.0f;
const float WHEEL_CIRCUMFERENCE_MM = 144.5f;
const float PIVOT_OFFSET_MM = 77.5f; // Half of a 155mm cell

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
int final_pwm_L = 0;
int final_pwm_R = 0;

DistanceData current_lidars;

// Forward Declarations
void runWallPIDLoop(float dt);
void printTelemetry();
void driveForwardDistanceMM(float distanceMM);
void resetPIDIntegrals();

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
  
  Serial.print("Connect to Wi-Fi: "); Serial.println(ssid);
  Serial.print("Open browser to: http://"); Serial.println(WiFi.softAPIP());

  Serial.println("\n=================================");
  Serial.println("  MICROMOUSE REACTIVE STARTUP");
  Serial.println("=================================");
  
  initMotors();
  initEncoders();
  initSensors();
  delay(5000);

  Serial.println("\n[!] PREPARING BNO055 CALIBRATION [!]");
  Serial.println("Please ensure the bot is perfectly still.");
  delay(2000);
  calibrateGyro();
  Serial.println("Gyro Calibration Successful.");

  resetYaw();
  current_yaw_angle = 0.0;
  baseTargetYaw = 0.0;
  targetYaw = 0.0;

  resetEncoders();
  prevLeftTicks  = 0;
  prevRightTicks = 0;
  resetPIDIntegrals();

  unsigned long now = millis();
  lastLoopTime  = now;
  lastLidarTime = now;
  lastPrintTime = now;
}

// ==========================================
// MAIN LOOP
// ==========================================
void loop() {
  webSocket.loop();
  server.handleClient();
  
  unsigned long currentTime = millis();
  
  bool doLidar = (currentTime - lastLidarTime >= LIDAR_INTERVAL_MS);
  bool doMotor = (currentTime - lastLoopTime >= LOOP_INTERVAL_MS);
  bool doPrint = (currentTime - lastPrintTime >= PRINT_INTERVAL_MS);

  float dt_lidar = doLidar ? (currentTime - lastLidarTime) / 1000.0 : 0.0;
  float dt_motor = doMotor ? (currentTime - lastLoopTime) / 1000.0 : 0.0;

  // --- REACTIVE SOLVER LOGIC ---
  if (doLidar) {
    lastLidarTime = currentTime;
    current_lidars = readLidars();
    if (currentState == DRIVING) {
        runWallPIDLoop(dt_lidar);
    }
  }

  if (doMotor) {
    lastLoopTime = currentTime;
    
    // Read wall states
    bool rightOpen = (current_lidars.right > WALL_MISSING_THRESHOLD);
    bool leftOpen  = (current_lidars.left > WALL_MISSING_THRESHOLD);
    bool frontOpen = (current_lidars.front > FRONT_BLOCKED_THRESHOLD);

    // Detect if we hit a decision point (a gap just appeared, or front is blocked)
    bool newlyFoundRightGap = (rightOpen && !rightWallWasPresent);
    bool newlyFoundLeftGap  = (leftOpen && !leftWallWasPresent);
    bool forcedToDecide     = (!frontOpen); 

    if (currentState == DRIVING) {
      if (newlyFoundRightGap || forcedToDecide || (newlyFoundLeftGap && !frontOpen)) {
        
        applyMotorPWM(0, 0); 
        currentState = ALIGNING_FOR_TURN;
        Serial.println("\n[!] INTERSECTION DETECTED [!]");

        // 1. Pivot Offset: Drive to the center of the cell if the front is open
        if (frontOpen) {
          Serial.println("Pushing forward to center of intersection...");
          driveForwardDistanceMM(PIVOT_OFFSET_MM);
        }

        // 2. Right-Hand Rule Priority: Right > Straight > Left > U-Turn
        if (rightOpen) {
          Serial.println("Decision: TURN RIGHT");
          turnCW90();
        } 
        else if (frontOpen) {
          Serial.println("Decision: GO STRAIGHT");
          // Do nothing, just pass through
        } 
        else if (leftOpen) {
          Serial.println("Decision: TURN LEFT");
          turnACW90();
        } 
        else {
          Serial.println("Decision: DEAD END - U-TURN");
          turn180();
        }

        // 3. Reset heading targets to current heading
        baseTargetYaw = readYawDegrees();
        targetYaw = baseTargetYaw;
        resetPIDIntegrals();
        
        // 4. Force a fresh Lidar read so we don't immediately trigger another turn
        current_lidars = readLidars();
        rightWallWasPresent = (current_lidars.right <= WALL_MISSING_THRESHOLD);
        leftWallWasPresent  = (current_lidars.left <= WALL_MISSING_THRESHOLD);

        currentState = DRIVING; 
        
      } else {
        // --- NORMAL DRIVING PID ---
        noInterrupts();
        long currentLeftTicks  = leftTicks;
        long currentRightTicks = -rightTicks; // Keep an eye on this negation!
        interrupts();

        float vel_L = (float)(currentLeftTicks  - prevLeftTicks);
        float vel_R = (float)(currentRightTicks - prevRightTicks);
        
        prevLeftTicks  = currentLeftTicks;
        prevRightTicks = currentRightTicks;

        current_yaw_angle = readYawDegrees();

        // Yaw PID
        float error_yaw = targetYaw - current_yaw_angle;
        if (abs(error_yaw) <= yaw_tolerance) { error_yaw = 0; prev_error_yaw = 0; }
        integral_yaw += error_yaw * dt_motor;
        float deriv_yaw = (error_yaw - prev_error_yaw) / dt_motor;
        float heading_correction_vel = (Kp_yaw * error_yaw) + (Ki_yaw * integral_yaw) + (Kd_yaw * deriv_yaw);
        prev_error_yaw = error_yaw;

        float brakeFactor = 1.0; 
        if (!frontOpen) {
            // Simple brake ramp if approaching a wall but not yet at threshold
            int d = current_lidars.front;
            if (d <= FRONT_HALT_MM) brakeFactor = 0.0;
            else if (d < FRONT_STOP_MM) brakeFactor = (float)(d - FRONT_HALT_MM) / (float)(FRONT_STOP_MM - FRONT_HALT_MM);
        }

        if (brakeFactor <= 0.0) {
          applyMotorPWM(0, 0);
          final_pwm_L = 0; final_pwm_R = 0;
          integral_vel_L = 0; integral_vel_R = 0;
          return;
        }

        float effectiveVelocity = baseTargetVelocity * brakeFactor;
        int   effectiveBasePWM  = (int)(basePWM * brakeFactor);

        float target_vel_L = effectiveVelocity - heading_correction_vel;
        float target_vel_R = effectiveVelocity + heading_correction_vel;

        // Velocity PID
        float error_vel_L = target_vel_L - vel_L;
        float error_vel_R = target_vel_R - vel_R;
        
        if (abs(error_vel_L) <= vel_tolerance) { error_vel_L = 0; prev_error_vel_L = 0; }
        if (abs(error_vel_R) <= vel_tolerance) { error_vel_R = 0; prev_error_vel_R = 0; }

        integral_vel_L += error_vel_L * dt_motor;
        integral_vel_R += error_vel_R * dt_motor;

        float deriv_vel_L = (error_vel_L - prev_error_vel_L) / dt_motor;
        float deriv_vel_R = (error_vel_R - prev_error_vel_R) / dt_motor;

        final_pwm_L = effectiveBasePWM + (int)((Kp_vel_L * error_vel_L) + (Ki_vel_L * integral_vel_L) + (Kd_vel_L * deriv_vel_L));
        final_pwm_R = effectiveBasePWM + (int)((Kp_vel_R * error_vel_R) + (Ki_vel_R * integral_vel_R) + (Kd_vel_R * deriv_vel_R));

        prev_error_vel_L = error_vel_L;
        prev_error_vel_R = error_vel_R;

        applyMotorPWM(final_pwm_L, final_pwm_R);
      }
    }

    // Update history for the edge-trigger detection
    if (currentState == DRIVING) {
        rightWallWasPresent = !rightOpen;
        leftWallWasPresent  = !leftOpen;
    }
  }

  if (doPrint) {
    lastPrintTime = currentTime;
    printTelemetry();
  }
}

// ==========================================
// WALL CENTERING PID
// ==========================================
void runWallPIDLoop(float dt) {
  bool hasLeft  = current_lidars.left  < WALL_THRESHOLD;
  bool hasRight = current_lidars.right < WALL_THRESHOLD;
  float error_wall = 0.0;

  if (hasLeft && hasRight) {
    Kp_wall = Kp_tunnel; Kd_wall = Kd_tunnel;
    error_wall = (float)(current_lidars.left - current_lidars.right);
  } else if (hasLeft) {
    Kp_wall = Kp_single; Kd_wall = Kd_single;
    error_wall = -(current_lidars.left - SINGLE_WALL_TARGET_MM);
  } else if (hasRight) {
    Kp_wall = Kp_single; Kd_wall = Kd_single;
    error_wall = (current_lidars.right - SINGLE_WALL_TARGET_MM);
  } else {
    correction_angle = 0.0; integral_wall = 0.0; prev_error_wall = 0.0;
    targetYaw = baseTargetYaw;
    return;
  }

  if (abs(error_wall) <= wall_tolerance) { error_wall = 0.0; integral_wall = 0.0; }

  integral_wall += error_wall * dt;
  float deriv_wall = (error_wall - prev_error_wall) / dt;
  float target_correction = (Kp_wall * error_wall) + (Ki_wall * integral_wall) + (Kd_wall * deriv_wall);
  
  correction_angle = (correction_angle * 0.7) + (target_correction * 0.3);
  correction_angle = constrain(correction_angle, -8.0, 8.0);

  prev_error_wall = error_wall;
  targetYaw = baseTargetYaw + correction_angle;
}

// ==========================================
// BLOCKING HELPER: DRIVE DISTANCE
// ==========================================
void driveForwardDistanceMM(float distanceMM) {
  noInterrupts();
  long startLeft = leftTicks;
  long startRight = -rightTicks; // Assuming negation matches your setup
  interrupts();

  float targetTicks = (distanceMM / WHEEL_CIRCUMFERENCE_MM) * TICKS_PER_REV;
  float currentYawRef = readYawDegrees();

  while (true) {
    noInterrupts();
    long currentLeft = leftTicks;
    long currentRight = -rightTicks;
    interrupts();

    float avgTicksMoved = ((currentLeft - startLeft) + (currentRight - startRight)) / 2.0f;
    
    if (avgTicksMoved >= targetTicks) {
      applyMotorPWM(0, 0);
      break;
    }
    
    // Very simple proportional heading lock for the short hop
    float yaw_err = currentYawRef - readYawDegrees();
    float corr = Kp_yaw * yaw_err * 20.0f; // Multiplier to translate to PWM scale
    
    int hopPWM = 60; // Safe hopping speed
    applyMotorPWM(hopPWM - corr, hopPWM + corr);
    delay(10);
  }
}

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
// TELEMETRY
// ==========================================
void printTelemetry() {
  noInterrupts();
  long currLeft  = leftTicks;
  long currRight = rightTicks;
  interrupts();

  char telemetryString[220];
  snprintf(telemetryString, sizeof(telemetryString), 
           "Target: %.1f | Enc: %ld / %ld | PWM: %d / %d | Yaw: %.2f | TargetYaw: %.2f | Lidar: %d / %d / %d", 
           baseTargetVelocity, currLeft, currRight, final_pwm_L, final_pwm_R, current_yaw_angle,
           targetYaw, current_lidars.left, current_lidars.front, current_lidars.right);
  webSocket.broadcastTXT(telemetryString);
  Serial.println(telemetryString);
}