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
const char *ssid = "Varun_Mouse";
const char *password = "mouse1234"; 

WebServer server(80);                // Hosts the HTML Dashboard on port 80
WebSocketsServer webSocket(81);      // Streams the live data on port 81

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
            <h3>Yaw Angle</h3>
            <div id="val-yaw" class="val">0.00°</div>
        </div>
        <div class="card" style="grid-column: 1 / -1;">
            <h3>PWM Power (Left / Right)</h3>
            <div id="val-pwm" class="val dual">0 / 0</div>
        </div>
        <div class="card" style="grid-column: 1 / -1;">
            <h3>Encoder Ticks (Left / Right)</h3>
            <div id="val-enc" class="val dual">0 / 0</div>
        </div>
    </div>

    <script>
        // Connect to the WebSocket port automatically
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
            setTimeout(initWebSocket, 2000); // Auto-reconnect
        }

        function onMessage(event) {
            // Looking for: Target: 15.0 | Enc: 100 / 102 | PWM: 45 / 46 | Yaw: 0.05
            let line = event.data;
            if (!line.includes("Target:")) return;

            try {
                const parts = line.split('|');
                document.getElementById('val-target').innerText = parts[0].split(':')[1].trim();
                document.getElementById('val-enc').innerText = parts[1].split(':')[1].trim();
                document.getElementById('val-pwm').innerText = parts[2].split(':')[1].trim();
                document.getElementById('val-yaw').innerText = parts[3].split(':')[1].trim() + "°";
            } catch (e) { }
        }
    </script>
</body>
</html>
)rawliteral";


// ==========================================
// TUNING PARAMETERS
// ==========================================
float Kp_vel = 7.0;  
float Ki_vel = 0.0;  
float Kd_vel = 0.1;  

float Kp_yaw = 0.75;  
float Ki_yaw = 0.00; 
float Kd_yaw = 0.05; 

float maxVelocity = 25.0;        
float baseTargetVelocity = 0.0;  
float targetYaw = 0.0;           

float vel_tolerance = 0.0; 
float yaw_tolerance = 0.0; 

// ==========================================
// SYSTEM VARIABLES
// ==========================================
const int LOOP_INTERVAL_MS = 20;  // 50Hz control loop
const int PRINT_INTERVAL_MS = 100; // 10Hz telemetry print loop

unsigned long lastLoopTime = 0;
unsigned long lastPrintTime = 0;

float integral_vel_L = 0, integral_vel_R = 0;
float prev_error_vel_L = 0, prev_error_vel_R = 0;

float integral_yaw = 0, prev_error_yaw = 0;
float current_yaw_angle = 0.0;

long prevLeftTicks = 0;
long prevRightTicks = 0;

int final_pwm_L = 0;
int final_pwm_R = 0;

void waitForStartSignal();
void runControlLoop(float dt);
void printTelemetry();

void setup() {
  Serial.begin(115200);
  delay(1000); 

  // --- START WI-FI & WEB SERVER ---
  Serial.println("\nStarting Wi-Fi Access Point...");
  WiFi.softAP(ssid, password);
  
  // When a browser asks for the root page ("/"), send the HTML string
  server.on("/", []() {
    server.send(200, "text/html", index_html);
  });
  
  server.begin();
  webSocket.begin();
  
  Serial.print("Connect to Wi-Fi: "); Serial.println(ssid);
  Serial.print("Open browser to: http://"); Serial.println(WiFi.softAPIP());
  // --------------------------------

  Serial.println("\n=================================");
  Serial.println("  MICROMOUSE SYSTEM STARTUP");
  Serial.println("=================================");
  
  initMotors();
  initEncoders();
  initSensors();
  delay(5000); 
  
  Serial.println("\n[!] PREPARING GYRO CALIBRATION [!]");
  Serial.println("Please ensure the bot is perfectly still on a flat surface.");
  delay(2000);
  calibrateGyro(); 
  Serial.println("Gyro Calibration Successful.");

  // waitForStartSignal(); // Uncomment when ready to use lidars for start

  resetEncoders();
  prevLeftTicks = 0;
  prevRightTicks = 0;
  integral_vel_L = 0;
  integral_vel_R = 0;
  current_yaw_angle = 0.0; 
  
  lastLoopTime = millis();
  lastPrintTime = millis();
}

void loop() {
  // CRITICAL: Keep the web server and websockets alive
  webSocket.loop();
  server.handleClient();

  unsigned long currentTime = millis();
  
  if (currentTime - lastLoopTime >= LOOP_INTERVAL_MS) {
    float dt = (currentTime - lastLoopTime) / 1000.0; 
    lastLoopTime = currentTime;
    runControlLoop(dt);
  }

  if (currentTime - lastPrintTime >= PRINT_INTERVAL_MS) {
    lastPrintTime = currentTime;
    printTelemetry();
  }
}

void runControlLoop(float dt) {
  if (baseTargetVelocity < maxVelocity) {
    baseTargetVelocity += 1; 
  }

  noInterrupts();
  long currentLeftTicks = leftTicks;
  long currentRightTicks = rightTicks;
  interrupts();

  float vel_L = (float)(currentLeftTicks - prevLeftTicks);
  float vel_R = (float)(currentRightTicks - prevRightTicks);
  
  prevLeftTicks = currentLeftTicks;
  prevRightTicks = currentRightTicks;

  // Outer Loop (Yaw)
  float yaw_rate = readGyroHeading(); 
  current_yaw_angle += yaw_rate * dt; 
  float error_yaw = targetYaw - current_yaw_angle;
  
  if (abs(error_yaw) <= yaw_tolerance) { error_yaw = 0; prev_error_yaw = 0; }

  integral_yaw += error_yaw * dt;
  float deriv_yaw = (error_yaw - prev_error_yaw) / dt;
  float heading_correction_vel = (Kp_yaw * error_yaw) + (Ki_yaw * integral_yaw) + (Kd_yaw * deriv_yaw);
  prev_error_yaw = error_yaw;

  float target_vel_L = baseTargetVelocity - heading_correction_vel;
  float target_vel_R = baseTargetVelocity + heading_correction_vel;

  // Inner Loop (Velocity)
  float error_vel_L = target_vel_L - vel_L;
  float error_vel_R = target_vel_R - vel_R;

  if (abs(error_vel_L) <= vel_tolerance) { error_vel_L = 0; prev_error_vel_L = 0; }
  if (abs(error_vel_R) <= vel_tolerance) { error_vel_R = 0; prev_error_vel_R = 0; }

  integral_vel_L += error_vel_L * dt;
  integral_vel_R += error_vel_R * dt;

  float deriv_vel_L = (error_vel_L - prev_error_vel_L) / dt;
  float deriv_vel_R = (error_vel_R - prev_error_vel_R) / dt;

  final_pwm_L = (int)((Kp_vel * error_vel_L) + (Ki_vel * integral_vel_L) + (Kd_vel * deriv_vel_L));
  final_pwm_R = (int)((Kp_vel * error_vel_R) + (Ki_vel * integral_vel_R) + (Kd_vel * deriv_vel_R));

  prev_error_vel_L = error_vel_L;
  prev_error_vel_R = error_vel_R;

  setMotorSpeeds(final_pwm_L, final_pwm_R);
}

void printTelemetry() {
  noInterrupts();
  long currLeft = leftTicks;
  long currRight = rightTicks;
  interrupts();

  // 1. Create the formatted string
  char telemetryString[150];
  snprintf(telemetryString, sizeof(telemetryString), 
           "Target: %.1f | Enc: %ld / %ld | PWM: %d / %d | Yaw: %.2f", 
           baseTargetVelocity, currLeft, currRight, final_pwm_L, final_pwm_R, current_yaw_angle);

  // 2. Broadcast it instantly over WebSockets to any connected browser
  webSocket.broadcastTXT(telemetryString);
}

void waitForStartSignal() {
  // Implementation kept same as before
  delay(5000); 
}