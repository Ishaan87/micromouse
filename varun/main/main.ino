#include <Arduino.h>
#include <WiFi.h>
#include <WebServer.h>
#include "Config.h"
#include "Motors.h"
#include "Encoders.h"
#include "Sensors.h"

// ==========================================
// TUNING PARAMETERS (You will need to adjust these!)
// ==========================================
// Velocity PID Constants
float Kp_vel = 5.0;
float Ki_vel = 0.0;
float Kd_vel = 1.0;

// Heading (Yaw) PID Constants
float Kp_yaw = 2.0;
float Ki_yaw = 0.0;
float Kd_yaw = 0.5;

// Target settings
float targetVelocity = 15.0; // Target ticks per loop interval
float targetYaw = 0.0;       // 0 degrees = straight forward

// ==========================================
// SYSTEM VARIABLES
// ==========================================
const int LOOP_INTERVAL_MS = 20; // 50Hz control loop
const unsigned long DISTANCE_SAMPLE_INTERVAL_MS = 80;
const unsigned long IDLE_TELEMETRY_INTERVAL_MS = 250;
unsigned long lastLoopTime = 0;
unsigned long lastDistanceSampleTime = 0;
unsigned long lastIdleTelemetryTime = 0;

// Integral and Derivative state variables
float integral_vel_L = 0, integral_vel_R = 0;
float prev_error_vel_L = 0, prev_error_vel_R = 0;

float integral_yaw = 0, prev_error_yaw = 0;
float current_yaw_angle = 0.0;

long prevLeftTicks = 0;
long prevRightTicks = 0;

bool motionEnabled = false;
WebServer server(80);

struct TelemetrySample {
  unsigned long ms;
  long leftTicks;
  long rightTicks;
  float targetVelL;
  float targetVelR;
  float velL;
  float velR;
  float yawRate;
  float yawAngle;
  int pwmL;
  int pwmR;
  int frontMm;
  int leftMm;
  int rightMm;
  bool running;
};

DistanceData lastDistances = {999, 999, 999};
TelemetrySample latestTelemetry = {0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 999, 999, 999, false};
TelemetrySample telemetryLog[TELEMETRY_LOG_CAPACITY];
int telemetryLogCount = 0;
int telemetryLogHead = 0;

// ==========================================
// FUNCTION PROTOTYPES
// ==========================================
void runControlLoop(float dt);
void connectControlAccessPoint();
void configureWebServer();
void handleRoot();
void handleStatus();
void handleLogs();
void handleStart();
void handleStop();
void resetControlState();
void updateDistanceDataIfDue();
void recordTelemetry(float targetVelL, float targetVelR, float velL, float velR, float yawRate, int pwmL, int pwmR);
void pushTelemetry(const TelemetrySample& sample);
void setPWMSpeed();
String telemetryToJson(const TelemetrySample& sample);
String logsToJson();

// ==========================================
// SETUP
// ==========================================
void setup() {
  Serial.begin(115200);
  delay(1000);

  Serial.println("Initializing Subsystems...");
  initMotors();
  initEncoders();
  initSensors();
  
  // Calibrate Gyro while the bot is perfectly still
  calibrateGyro();
  connectControlAccessPoint();
  configureWebServer();
  resetControlState();

  Serial.println("Telemetry CSV header:");
  Serial.println("ms,running,leftTicks,rightTicks,targetVelL,targetVelR,velL,velR,yawRate,yawAngle,pwmL,pwmR,frontMm,leftMm,rightMm");
  Serial.println("Open the control page at http://192.168.4.1/");
}

// ==========================================
// MAIN LOOP
// ==========================================
void loop() {
  server.handleClient();
  unsigned long currentTime = millis();

  updateDistanceDataIfDue();
  
  // Run the PID calculations strictly at the defined interval
  if (motionEnabled && currentTime - lastLoopTime >= LOOP_INTERVAL_MS) {
    float dt = (currentTime - lastLoopTime) / 1000.0; // Delta time in seconds
    lastLoopTime = currentTime;
    
    runControlLoop(dt);
  } else if (!motionEnabled && currentTime - lastIdleTelemetryTime >= IDLE_TELEMETRY_INTERVAL_MS) {
    lastIdleTelemetryTime = currentTime;
    recordTelemetry(0.0, 0.0, 0.0, 0.0, 0.0, 0, 0);
  }
}

void setPWMSpeed(float heading_correction, float yaw_rate, float dt){
  noInterrupts();
  long currentLeftTicks = leftTicks;
  long currentRightTicks = rightTicks;
  interrupts();

  float vel_L = (float)(currentLeftTicks - prevLeftTicks);
  float vel_R = (float)(currentRightTicks - prevRightTicks);
  
  prevLeftTicks = currentLeftTicks;
  prevRightTicks = currentRightTicks;

  float targetVelL = targetVelocity - heading_correction;
  float targetVelR = targetVelocity + heading_correction;

  // 4. Wheel speed PID converts desired wheel speed into PWM.
  float error_vel_L = targetVelL - vel_L;
  float error_vel_R = targetVelR - vel_R;

  integral_vel_L += error_vel_L * dt;
  integral_vel_R += error_vel_R * dt;

  float deriv_vel_L = (error_vel_L - prev_error_vel_L) / dt;
  float deriv_vel_R = (error_vel_R - prev_error_vel_R) / dt;

  int final_pwm_L = (int)((Kp_vel * error_vel_L) + (Ki_vel * integral_vel_L) + (Kd_vel * deriv_vel_L));
  int final_pwm_R = (int)((Kp_vel * error_vel_R) + (Ki_vel * integral_vel_R) + (Kd_vel * deriv_vel_R));

  prev_error_vel_L = error_vel_L;
  prev_error_vel_R = error_vel_R;

  setMotorSpeeds(final_pwm_L, final_pwm_R);
  recordTelemetry(targetVelL, targetVelR, vel_L, vel_R, yaw_rate, final_pwm_L, final_pwm_R);
}


// ==========================================
// CONTROL LOOP (VELOCITY + HEADING PID)
// ==========================================
void runControlLoop(float dt) {
  // 1. Calculate current wheel velocities (ticks per dt)
  // Disable interrupts briefly to safely read volatile 32-bit integers
    // 2. Heading PID Calculation
  // Read angular velocity and integrate it to track the absolute angle
  float yaw_rate = readGyroHeading(); 
  current_yaw_angle += yaw_rate * dt; 

  float error_yaw = targetYaw - current_yaw_angle;
  integral_yaw += error_yaw * dt;
  float deriv_yaw = (error_yaw - prev_error_yaw) / dt;

  // Heading correction value
  float heading_correction = (Kp_yaw * error_yaw) + (Ki_yaw * integral_yaw) + (Kd_yaw * deriv_yaw);
  prev_error_yaw = error_yaw;

  setPWMSpeed(heading_correction, yaw_rate, dt);

  // 3. Yaw PID shapes the desired wheel speeds.
  
}

// ==========================================
// WEB + TELEMETRY
// ==========================================
void connectControlAccessPoint() {
  WiFi.mode(WIFI_AP);
  bool apStarted = WiFi.softAP(WIFI_AP_SSID, WIFI_AP_PASSWORD);

  if (!apStarted) {
    Serial.println("Failed to start WiFi access point");
    while (true) {
      delay(1000);
    }
  }

  Serial.print("AP SSID: ");
  Serial.println(WIFI_AP_SSID);
  Serial.print("AP IP: ");
  Serial.println(WiFi.softAPIP());
}

void configureWebServer() {
  server.on("/", HTTP_GET, handleRoot);
  server.on("/api/status", HTTP_GET, handleStatus);
  server.on("/api/logs", HTTP_GET, handleLogs);
  server.on("/api/start", HTTP_POST, handleStart);
  server.on("/api/stop", HTTP_POST, handleStop);
  server.begin();
}

void handleRoot() {
  static const char PAGE[] PROGMEM = R"HTML(
<!DOCTYPE html>
<html lang="en">
<head>
  <meta charset="utf-8">
  <meta name="viewport" content="width=device-width, initial-scale=1">
  <title>Micromouse Control</title>
  <style>
    :root {
      --bg: #f2efe8;
      --panel: #fffaf2;
      --ink: #1d2a32;
      --accent: #c4512d;
      --accent-dark: #7e2e18;
      --line: #d8cfbf;
      --good: #2b6e4b;
      font-family: "Segoe UI", Tahoma, sans-serif;
    }
    body {
      margin: 0;
      background: radial-gradient(circle at top, #fffaf2 0%, #efe4cf 45%, #d7c1a1 100%);
      color: var(--ink);
    }
    main {
      max-width: 1080px;
      margin: 0 auto;
      padding: 24px;
    }
    .hero, .panel {
      background: rgba(255, 250, 242, 0.92);
      border: 1px solid var(--line);
      border-radius: 20px;
      box-shadow: 0 14px 36px rgba(74, 54, 34, 0.12);
      padding: 20px;
      margin-bottom: 18px;
      backdrop-filter: blur(6px);
    }
    h1, h2 {
      margin-top: 0;
    }
    .controls, .stats {
      display: grid;
      grid-template-columns: repeat(auto-fit, minmax(180px, 1fr));
      gap: 12px;
    }
    button, a {
      border: 0;
      border-radius: 999px;
      padding: 12px 16px;
      font: inherit;
      text-decoration: none;
      text-align: center;
      cursor: pointer;
    }
    .start {
      background: var(--good);
      color: white;
    }
    .stop {
      background: var(--accent);
      color: white;
    }
    .ghost {
      background: transparent;
      border: 1px solid var(--line);
      color: var(--ink);
    }
    .stat {
      background: var(--panel);
      border: 1px solid var(--line);
      border-radius: 16px;
      padding: 14px;
    }
    .label {
      font-size: 0.86rem;
      opacity: 0.7;
    }
    .value {
      font-size: 1.5rem;
      font-weight: 700;
      margin-top: 6px;
    }
    table {
      width: 100%;
      border-collapse: collapse;
      font-size: 0.9rem;
    }
    th, td {
      padding: 8px 6px;
      border-bottom: 1px solid var(--line);
      text-align: left;
    }
    .status-running {
      color: var(--good);
      font-weight: 700;
    }
    .status-stopped {
      color: var(--accent-dark);
      font-weight: 700;
    }
    @media (max-width: 720px) {
      main {
        padding: 14px;
      }
      .hero, .panel {
        padding: 16px;
      }
      table {
        display: block;
        overflow-x: auto;
      }
    }
  </style>
</head>
<body>
  <main>
    <section class="hero">
      <h1>Micromouse Control Deck</h1>
      <p>Start or stop the robot, inspect live telemetry, and download the session log as CSV.</p>
      <div class="controls">
        <button class="start" onclick="sendCommand('/api/start')">Start Motion</button>
        <button class="stop" onclick="sendCommand('/api/stop')">Stop Motion</button>
        <a class="ghost" href="/api/logs" target="_blank" rel="noreferrer">Open JSON Log</a>
        <button class="ghost" onclick="downloadCsv()">Download CSV</button>
      </div>
      <p id="statusText">Connecting...</p>
    </section>

    <section class="panel">
      <h2>Live Telemetry</h2>
      <div class="stats" id="stats"></div>
    </section>

    <section class="panel">
      <h2>Recent Samples</h2>
      <table>
        <thead>
          <tr>
            <th>ms</th>
            <th>run</th>
            <th>L ticks</th>
            <th>R ticks</th>
            <th>target L</th>
            <th>target R</th>
            <th>yaw</th>
            <th>yaw rate</th>
            <th>PWM L</th>
            <th>PWM R</th>
            <th>front</th>
            <th>left</th>
            <th>right</th>
          </tr>
        </thead>
        <tbody id="logBody"></tbody>
      </table>
    </section>
  </main>
  <script>
    const stats = [
      ['state', 'Run State'],
      ['leftTicks', 'Left Encoder'],
      ['rightTicks', 'Right Encoder'],
      ['targetVelL', 'Target Left Vel'],
      ['targetVelR', 'Target Right Vel'],
      ['velL', 'Left Velocity'],
      ['velR', 'Right Velocity'],
      ['yawAngle', 'Yaw Angle'],
      ['yawRate', 'Yaw Rate'],
      ['pwmL', 'Left PWM'],
      ['pwmR', 'Right PWM'],
      ['frontMm', 'Front Lidar'],
      ['leftMm', 'Left Lidar'],
      ['rightMm', 'Right Lidar']
    ];

    async function sendCommand(path) {
      await fetch(path, { method: 'POST' });
      await refreshAll();
    }

    function renderStats(sample) {
      const container = document.getElementById('stats');
      container.innerHTML = stats.map(([key, label]) => {
        const raw = key === 'state' ? (sample.running ? 'RUNNING' : 'STOPPED') : sample[key];
        const value = typeof raw === 'number' ? Number(raw).toFixed(2).replace(/\\.00$/, '') : raw;
        return `<div class="stat"><div class="label">${label}</div><div class="value">${value}</div></div>`;
      }).join('');
      document.getElementById('statusText').innerHTML = `Robot is <span class="${sample.running ? 'status-running' : 'status-stopped'}">${sample.running ? 'RUNNING' : 'STOPPED'}</span> at ${sample.ms} ms`;
    }

    function renderLogs(samples) {
      const recent = samples.slice(-20).reverse();
      document.getElementById('logBody').innerHTML = recent.map((sample) => `
        <tr>
          <td>${sample.ms}</td>
          <td>${sample.running ? 'Y' : 'N'}</td>
          <td>${sample.leftTicks}</td>
          <td>${sample.rightTicks}</td>
          <td>${Number(sample.targetVelL).toFixed(2)}</td>
          <td>${Number(sample.targetVelR).toFixed(2)}</td>
          <td>${Number(sample.yawAngle).toFixed(2)}</td>
          <td>${Number(sample.yawRate).toFixed(2)}</td>
          <td>${sample.pwmL}</td>
          <td>${sample.pwmR}</td>
          <td>${sample.frontMm}</td>
          <td>${sample.leftMm}</td>
          <td>${sample.rightMm}</td>
        </tr>`).join('');
    }

    async function downloadCsv() {
      const response = await fetch('/api/logs');
      const payload = await response.json();
      const header = ['ms','running','leftTicks','rightTicks','targetVelL','targetVelR','velL','velR','yawRate','yawAngle','pwmL','pwmR','frontMm','leftMm','rightMm'];
      const rows = payload.samples.map((sample) => header.map((key) => sample[key]));
      const csv = [header.join(','), ...rows.map((row) => row.join(','))].join('\\n');
      const blob = new Blob([csv], { type: 'text/csv' });
      const url = URL.createObjectURL(blob);
      const link = document.createElement('a');
      link.href = url;
      link.download = 'micromouse-telemetry.csv';
      link.click();
      URL.revokeObjectURL(url);
    }

    async function refreshAll() {
      const [statusRes, logsRes] = await Promise.all([
        fetch('/api/status'),
        fetch('/api/logs')
      ]);
      const statusPayload = await statusRes.json();
      const logsPayload = await logsRes.json();
      renderStats(statusPayload.sample);
      renderLogs(logsPayload.samples);
    }

    refreshAll();
    setInterval(refreshAll, 1000);
  </script>
</body>
</html>
)HTML";

  server.send_P(200, "text/html", PAGE);
}

void handleStatus() {
  server.send(200, "application/json", String("{\"sample\":") + telemetryToJson(latestTelemetry) + "}");
}

void handleLogs() {
  server.send(200, "application/json", logsToJson());
}

void handleStart() {
  resetControlState();
  motionEnabled = true;
  lastLoopTime = millis();
  lastIdleTelemetryTime = millis();
  Serial.println("Motion started from web interface");
  server.send(200, "application/json", "{\"ok\":true,\"running\":true}");
}

void handleStop() {
  motionEnabled = false;
  stopMotors();
  recordTelemetry(0.0, 0.0, 0.0, 0.0, 0.0, 0, 0);
  Serial.println("Motion stopped from web interface");
  server.send(200, "application/json", "{\"ok\":true,\"running\":false}");
}

void resetControlState() {
  stopMotors();
  resetEncoders();
  prevLeftTicks = 0;
  prevRightTicks = 0;
  integral_vel_L = 0;
  integral_vel_R = 0;
  prev_error_vel_L = 0;
  prev_error_vel_R = 0;
  integral_yaw = 0;
  prev_error_yaw = 0;
  current_yaw_angle = 0.0;
  telemetryLogCount = 0;
  telemetryLogHead = 0;
  latestTelemetry = {millis(), 0, 0, 0, 0, 0, 0, 0, 0, 0, lastDistances.front, lastDistances.left, lastDistances.right, false};
}

void updateDistanceDataIfDue() {
  unsigned long now = millis();
  if (now - lastDistanceSampleTime < DISTANCE_SAMPLE_INTERVAL_MS) {
    return;
  }

  lastDistanceSampleTime = now;
  lastDistances = readLidars();
}

void recordTelemetry(float targetVelL, float targetVelR, float velL, float velR, float yawRate, int pwmL, int pwmR) {
  noInterrupts();
  long currentLeftTicks = leftTicks;
  long currentRightTicks = rightTicks;
  interrupts();

  TelemetrySample sample = {
    millis(),
    currentLeftTicks,
    currentRightTicks,
    targetVelL,
    targetVelR,
    velL,
    velR,
    yawRate,
    current_yaw_angle,
    pwmL,
    pwmR,
    lastDistances.front,
    lastDistances.left,
    lastDistances.right,
    motionEnabled
  };

  latestTelemetry = sample;
  pushTelemetry(sample);

  Serial.printf(
    "%lu,%d,%ld,%ld,%.2f,%.2f,%.2f,%.2f,%.2f,%.2f,%d,%d,%d,%d,%d\n",
    sample.ms,
    sample.running ? 1 : 0,
    sample.leftTicks,
    sample.rightTicks,
    sample.targetVelL,
    sample.targetVelR,
    sample.velL,
    sample.velR,
    sample.yawRate,
    sample.yawAngle,
    sample.pwmL,
    sample.pwmR,
    sample.frontMm,
    sample.leftMm,
    sample.rightMm
  );
}

void pushTelemetry(const TelemetrySample& sample) {
  telemetryLog[telemetryLogHead] = sample;
  telemetryLogHead = (telemetryLogHead + 1) % TELEMETRY_LOG_CAPACITY;
  if (telemetryLogCount < TELEMETRY_LOG_CAPACITY) {
    telemetryLogCount++;
  }
}

String telemetryToJson(const TelemetrySample& sample) {
  String json = "{";
  json += "\"ms\":" + String(sample.ms);
  json += ",\"running\":" + String(sample.running ? "true" : "false");
  json += ",\"leftTicks\":" + String(sample.leftTicks);
  json += ",\"rightTicks\":" + String(sample.rightTicks);
  json += ",\"targetVelL\":" + String(sample.targetVelL, 2);
  json += ",\"targetVelR\":" + String(sample.targetVelR, 2);
  json += ",\"velL\":" + String(sample.velL, 2);
  json += ",\"velR\":" + String(sample.velR, 2);
  json += ",\"yawRate\":" + String(sample.yawRate, 2);
  json += ",\"yawAngle\":" + String(sample.yawAngle, 2);
  json += ",\"pwmL\":" + String(sample.pwmL);
  json += ",\"pwmR\":" + String(sample.pwmR);
  json += ",\"frontMm\":" + String(sample.frontMm);
  json += ",\"leftMm\":" + String(sample.leftMm);
  json += ",\"rightMm\":" + String(sample.rightMm);
  json += "}";
  return json;
}

String logsToJson() {
  String json = "{\"samples\":[";
  for (int i = 0; i < telemetryLogCount; i++) {
    int index = (telemetryLogHead - telemetryLogCount + i + TELEMETRY_LOG_CAPACITY) % TELEMETRY_LOG_CAPACITY;
    json += telemetryToJson(telemetryLog[index]);
    if (i < telemetryLogCount - 1) {
      json += ",";
    }
  }
  json += "]}";
  return json;
}
