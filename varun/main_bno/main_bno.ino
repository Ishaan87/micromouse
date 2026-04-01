#include <Arduino.h>
#include <WiFi.h>
#include <WebServer.h>
#include <WebSocketsServer.h>
#include "Config.h"
#include "Motors.h"
#include "Encoders.h"
#include "Sensors.h"

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
// Tunnel Constants (Both walls present) - More aggressive
float Kp_tunnel = 0.12; 
float Kd_tunnel = 0.08;

// Single Wall Constants (Only one wall) - More gentle/dampened
float Kp_single = 0.06; 
float Kd_single = 0.12; // Higher D helps stop the jitter

// Current active constants (will be swapped by logic)
float Kp_wall = Kp_tunnel;
float Kd_wall = Kd_tunnel;
float Ki_wall = 0.0;


        
float baseTargetVelocity = 15.0;
int basePWM = 50; 

// Heading Management
float baseTargetYaw = 0.0;     // The absolute heading we want (0 = forward, set by resetYaw())
float correction_angle = 0.0;  // The adjustment given by LiDARs
float targetYaw = 0.0;         // baseTargetYaw + correction_angle

float vel_tolerance = 0.5; 
float yaw_tolerance = 0.5; 
float wall_tolerance = 10.0;    // mm
const int WALL_THRESHOLD = 150;

const float SINGLE_WALL_TARGET_MM = 63.0; // desired distance when only one wall visible
const int   FRONT_STOP_MM         = 150;  // begin decelerating below this distance
const int   FRONT_HALT_MM         = 75;   // full stop below this distance

// ==========================================
// SYSTEM VARIABLES
// ==========================================
const int LOOP_INTERVAL_MS  = 20;   // 50Hz  — fast motor/gyro control
const int LIDAR_INTERVAL_MS = 50;   // 20Hz  — lidar + wall PID
const int PRINT_INTERVAL_MS = 100;  // 10Hz  — telemetry

unsigned long lastLoopTime  = 0;
unsigned long lastLidarTime = 0;
unsigned long lastPrintTime = 0;

float integral_vel_L = 0, integral_vel_R = 0;
float prev_error_vel_L = 0, prev_error_vel_R = 0;

float integral_yaw = 0, prev_error_yaw = 0;

// current_yaw_angle is now read directly from the BNO055 quaternion
// via readYawDegrees() — no more integration drift.
float current_yaw_angle = 0.0;

float integral_wall = 0, prev_error_wall = 0;

long prevLeftTicks  = 0;
long prevRightTicks = 0;

int final_pwm_L = 0;
int final_pwm_R = 0;

DistanceData current_lidars;

void runControlLoop(float dt);
void runWallPIDLoop(float dt);
void printTelemetry();

void setup() {
  Serial.begin(115200);
  delay(1000);

  Serial.println("\nStarting Wi-Fi Access Point...");
  WiFi.softAP(ssid, password);
  server.on("/", []() {
    server.send(200, "text/html", index_html);
  });
  server.begin();
  webSocket.begin();
  
  Serial.print("Connect to Wi-Fi: "); Serial.println(ssid);
  Serial.print("Open browser to: http://"); Serial.println(WiFi.softAPIP());

  Serial.println("\n=================================");
  Serial.println("  MICROMOUSE SYSTEM STARTUP");
  Serial.println("=================================");
  
  initMotors();
  initEncoders();
  initSensors();
  delay(5000); 
  
  Serial.println("\n[!] PREPARING BNO055 CALIBRATION [!]");
  Serial.println("Please ensure the bot is perfectly still.");
  delay(2000);
  calibrateGyro();   // Waits for BNO055 gyro to reach cal level 3
  Serial.println("Gyro Calibration Successful.");

  // Zero the heading: robot's current orientation becomes 0 degrees
  resetYaw();
  current_yaw_angle = 0.0;

  resetEncoders();
  prevLeftTicks  = 0;
  prevRightTicks = 0;
  integral_vel_L = 0;
  integral_vel_R = 0;
  integral_yaw   = 0;
  integral_wall  = 0;
  
  unsigned long now = millis();
  lastLoopTime  = now;
  lastLidarTime = now;
  lastPrintTime = now;
}

void loop() {
  webSocket.loop();
  server.handleClient();
  
  unsigned long currentTime = millis();
  
  // 1. Medium Loop: LiDAR & Wall PID
  if (currentTime - lastLidarTime >= LIDAR_INTERVAL_MS) {
    float dt_lidar = (currentTime - lastLidarTime) / 1000.0;
    lastLidarTime = currentTime;
    runWallPIDLoop(dt_lidar);
  }

  // 2. Fast Loop: Yaw, Encoders, Motors
  if (currentTime - lastLoopTime >= LOOP_INTERVAL_MS) {
    float dt_motor = (currentTime - lastLoopTime) / 1000.0;
    lastLoopTime = currentTime;
    runControlLoop(dt_motor);
  }

  // 3. Slow Loop: Telemetry
  if (currentTime - lastPrintTime >= PRINT_INTERVAL_MS) {
    lastPrintTime = currentTime;
    printTelemetry();
  }
}

// ==========================================
// WALL CENTERING PID (Updates Target Yaw)
// Three cases:
//   1. Both walls visible  → centre between them
//   2. One wall visible    → maintain SINGLE_WALL_TARGET_MM from it
//   3. No walls            → hold current heading, reset integral
// ==========================================
void runWallPIDLoop(float dt) {
  current_lidars = readLidars();

  bool hasLeft  = current_lidars.left  < WALL_THRESHOLD;
  bool hasRight = current_lidars.right < WALL_THRESHOLD;

  float error_wall = 0.0;

  // --- GAIN SCHEDULING LOGIC ---
  if (hasLeft && hasRight) {
    // Tunnel Case: Use firmer constants
    Kp_wall = Kp_tunnel;
    Kd_wall = Kd_tunnel;
    error_wall = (float)(current_lidars.left - current_lidars.right);

  } else if (hasLeft) {
    // Single Wall Left: Use gentle constants
    Kp_wall = Kp_single;
    Kd_wall = Kd_single;
    error_wall = -(current_lidars.left - SINGLE_WALL_TARGET_MM);

  } else if (hasRight) {
    // Single Wall Right: Use gentle constants
    Kp_wall = Kp_single;
    Kd_wall = Kd_single;
    error_wall = (current_lidars.right - SINGLE_WALL_TARGET_MM);

  } else {
    // No Walls: Reset and return
    correction_angle = 0.0;
    integral_wall    = 0.0;
    prev_error_wall  = 0.0;
    targetYaw = baseTargetYaw;
    return;
  }

  // Deadzone to prevent micro-jitters when near target
  if (abs(error_wall) <= wall_tolerance) {
    error_wall = 0.0;
  }

  // PID Calculation
  integral_wall += error_wall * dt;
  float deriv_wall = (error_wall - prev_error_wall) / dt;

  float target_correction = (Kp_wall * error_wall) + (Ki_wall * integral_wall) + (Kd_wall * deriv_wall);
  
  // SMOOTHING FILTER (Crucial for Single Wall Jitter)
  // This prevents the targetYaw from jumping instantly
  correction_angle = (correction_angle * 0.7) + (target_correction * 0.3);
  
  // Constrain the steering authority
  correction_angle = constrain(correction_angle, -8.0, 8.0);

  prev_error_wall = error_wall;
  targetYaw = baseTargetYaw + correction_angle;
}

// ==========================================
// FRONT WALL DECELERATION
// - Returns a velocity scale 0.0..1.0
//   based on front lidar distance.
//   1.0 = full speed, 0.0 = full stop.
// ==========================================
float frontBrakeScale() {
  int d = current_lidars.front;

  if (d >= FRONT_STOP_MM)  return 1.0;                                         // nothing ahead — full speed
  if (d <= FRONT_HALT_MM)  return 0.0;                                         // too close — stop

  // Linear ramp between FRONT_HALT_MM and FRONT_STOP_MM
  return (float)(d - FRONT_HALT_MM) / (float)(FRONT_STOP_MM - FRONT_HALT_MM);
}

// ==========================================
// MOTOR & YAW CONTROL LOOP
// ==========================================
void runControlLoop(float dt) {
  // Encoder velocity (ticks per loop)
  noInterrupts();
  long currentLeftTicks  = leftTicks;
  long currentRightTicks = -rightTicks;
  interrupts();

  float vel_L = (float)(currentLeftTicks  - prevLeftTicks);
  float vel_R = (float)(currentRightTicks - prevRightTicks);
  
  prevLeftTicks  = currentLeftTicks;
  prevRightTicks = currentRightTicks;

  // --- REPLACED: was current_yaw_angle += readGyroHeading() * dt ---
  // Now reads absolute, drift-free yaw directly from the BNO055 quaternion.
  // No integration error accumulates over time.
  current_yaw_angle = readYawDegrees();

  // Middle Loop (Yaw PID)
  float error_yaw = targetYaw - current_yaw_angle;
  
  if (abs(error_yaw) <= yaw_tolerance) { error_yaw = 0; prev_error_yaw = 0; }

  integral_yaw += error_yaw * dt;
  float deriv_yaw = (error_yaw - prev_error_yaw) / dt;
  
  float heading_correction_vel = (Kp_yaw * error_yaw) + (Ki_yaw * integral_yaw) + (Kd_yaw * deriv_yaw);
  prev_error_yaw = error_yaw;

  // Scale target velocity down if front wall is close
  float brakeFactor = frontBrakeScale();

  // Hard stop — bypass PID entirely and kill motors
  if (brakeFactor <= 0.0) {
    applyMotorPWM(0, 0);
    final_pwm_L = 0;
    final_pwm_R = 0;
    // Reset velocity integrals so there's no windup while stopped
    integral_vel_L = 0;
    integral_vel_R = 0;
    prev_error_vel_L = 0;
    prev_error_vel_R = 0;
    return;
  }

  float effectiveVelocity = baseTargetVelocity * brakeFactor;
  int   effectiveBasePWM  = (int)(basePWM * brakeFactor);

  // Positive correction = turning ACW/Left → left slows, right speeds up
  float target_vel_L = effectiveVelocity - heading_correction_vel;
  float target_vel_R = effectiveVelocity + heading_correction_vel;

  // Inner Loop (Velocity PID)
  float error_vel_L = target_vel_L - vel_L;
  float error_vel_R = target_vel_R - vel_R;
  
  if (abs(error_vel_L) <= vel_tolerance) { error_vel_L = 0; prev_error_vel_L = 0; }
  if (abs(error_vel_R) <= vel_tolerance) { error_vel_R = 0; prev_error_vel_R = 0; }

  integral_vel_L += error_vel_L * dt;
  integral_vel_R += error_vel_R * dt;

  float deriv_vel_L = (error_vel_L - prev_error_vel_L) / dt;
  float deriv_vel_R = (error_vel_R - prev_error_vel_R) / dt;

  final_pwm_L = effectiveBasePWM + (int)((Kp_vel_L * error_vel_L) + (Ki_vel_L * integral_vel_L) + (Kd_vel_L * deriv_vel_L));
  final_pwm_R = effectiveBasePWM + (int)((Kp_vel_R * error_vel_R) + (Ki_vel_R * integral_vel_R) + (Kd_vel_R * deriv_vel_R));

  prev_error_vel_L = error_vel_L;
  prev_error_vel_R = error_vel_R;

  applyMotorPWM(final_pwm_L, final_pwm_R);
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
