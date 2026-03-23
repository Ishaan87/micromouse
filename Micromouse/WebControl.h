#ifndef WEB_CONTROL_H
#define WEB_CONTROL_H

#include <WiFi.h>
#include <WebServer.h>

#include "Config.h"
#include "PID.h"

enum ManualCommand {
  CMD_NONE,
  CMD_FORWARD,
  CMD_BACKWARD,
  CMD_TURN_LEFT,
  CMD_TURN_RIGHT,
  CMD_TURN_AROUND,
  CMD_STOP
};

WebServer webServer(WEB_CONTROL_PORT);
ManualCommand pendingCommand = CMD_NONE;
ManualCommand activeCommand = CMD_NONE;
int manualSpeed = WEB_DEFAULT_SPEED;
bool webControlReady = false;
bool motionBusy = false;

const char WEB_CONTROL_PAGE[] PROGMEM = R"rawliteral(
<!DOCTYPE html>
<html lang="en">
<head>
  <meta charset="UTF-8">
  <meta name="viewport" content="width=device-width, initial-scale=1.0">
  <title>Micromouse Control</title>
  <style>
    :root {
      --bg1: #f4efe6;
      --bg2: #d7e4db;
      --panel: rgba(255, 252, 247, 0.92);
      --ink: #182126;
      --muted: #54606a;
      --accent: #d96941;
      --accent-dark: #b44c2d;
      --secondary: #2d5c53;
      --line: rgba(24, 33, 38, 0.12);
      --shadow: 0 18px 40px rgba(23, 35, 31, 0.14);
    }

    * { box-sizing: border-box; }

    body {
      margin: 0;
      min-height: 100vh;
      padding: 20px;
      font-family: "Segoe UI", Tahoma, sans-serif;
      color: var(--ink);
      background:
        radial-gradient(circle at top left, rgba(217, 105, 65, 0.18), transparent 32%),
        radial-gradient(circle at bottom right, rgba(49, 101, 84, 0.18), transparent 28%),
        linear-gradient(135deg, var(--bg1), var(--bg2));
    }

    .panel {
      width: min(100%, 860px);
      margin: 0 auto;
      background: var(--panel);
      border: 1px solid var(--line);
      border-radius: 28px;
      box-shadow: var(--shadow);
      backdrop-filter: blur(12px);
      padding: 24px;
    }

    h1 {
      margin: 0 0 8px;
      font-size: 2rem;
    }

    p {
      margin: 0;
      color: var(--muted);
      line-height: 1.5;
    }

    .top {
      display: grid;
      gap: 18px;
      grid-template-columns: 1.1fr 0.9fr;
      margin-top: 22px;
    }

    .card {
      background: rgba(255, 255, 255, 0.72);
      border: 1px solid var(--line);
      border-radius: 20px;
      padding: 18px;
    }

    .status {
      display: grid;
      gap: 10px;
      font-weight: 600;
    }

    .grid {
      margin-top: 16px;
      display: grid;
      grid-template-columns: repeat(3, 1fr);
      gap: 12px;
    }

    button {
      border: 0;
      border-radius: 18px;
      min-height: 76px;
      font-size: 1rem;
      font-weight: 700;
      color: white;
      background: linear-gradient(180deg, var(--accent), var(--accent-dark));
      box-shadow: 0 12px 24px rgba(180, 76, 45, 0.22);
      transition: transform 120ms ease, box-shadow 120ms ease, opacity 120ms ease;
    }

    button.secondary {
      background: linear-gradient(180deg, #3e6f65, #2a5149);
      box-shadow: 0 12px 24px rgba(42, 81, 73, 0.22);
    }

    button.stop {
      background: linear-gradient(180deg, #2b3138, #171d23);
      box-shadow: 0 12px 24px rgba(23, 29, 35, 0.22);
    }

    button:active {
      transform: translateY(2px) scale(0.98);
    }

    .slider-wrap {
      margin-top: 6px;
    }

    .slider-head {
      display: flex;
      justify-content: space-between;
      align-items: center;
      margin-bottom: 10px;
      font-weight: 600;
    }

    input[type="range"] {
      width: 100%;
      accent-color: var(--accent);
    }

    pre {
      margin: 0;
      min-height: 260px;
      max-height: 420px;
      overflow: auto;
      padding: 14px;
      border-radius: 16px;
      background: #161c22;
      color: #e7eef6;
      font: 0.92rem/1.45 Consolas, monospace;
      white-space: pre-wrap;
    }

    @media (max-width: 760px) {
      .top {
        grid-template-columns: 1fr;
      }
    }
  </style>
</head>
<body>
  <main class="panel">
    <h1>Micromouse PID Control</h1>
    <p>Each button runs a closed-loop motion: forward and backward move one cell in a straight line, left and right turn 90 degrees, and U-turn rotates 180 degrees.</p>

    <div class="top">
      <section class="card">
        <div class="slider-wrap">
          <div class="slider-head">
            <span>Speed / Turn Cap</span>
            <span id="speedValue">120</span>
          </div>
          <input id="speed" type="range" min="60" max="150" value="120">
        </div>

        <div class="grid">
          <div></div>
          <button data-command="forward">Forward 1 Cell</button>
          <div></div>

          <button class="secondary" data-command="turn_left">Left 90</button>
          <button class="stop" id="stopButton">Stop</button>
          <button class="secondary" data-command="turn_right">Right 90</button>

          <button class="secondary" data-command="turn_around">U-Turn 180</button>
          <button data-command="backward">Backward 1 Cell</button>
          <div></div>
        </div>
      </section>

      <section class="card status">
        <div id="statusText">State: idle</div>
        <div id="commandText">Command: none</div>
        <div id="headingText">Heading: 0</div>
        <div id="ticksText">Ticks: 0 / 0</div>
        <div id="wheelText">Wheels: L 0 | R 0</div>
        <div id="sensorText">Sensors: F 0 | L 0 | R 0</div>
      </section>
    </div>

    <section class="card" style="margin-top: 18px;">
      <div style="font-weight: 700; margin-bottom: 10px;">Feedback Logs</div>
      <pre id="logs">Waiting for logs...</pre>
    </section>
  </main>

  <script>
    const speedEl = document.getElementById("speed");
    const speedValueEl = document.getElementById("speedValue");
    const logsEl = document.getElementById("logs");
    const statusText = document.getElementById("statusText");
    const commandText = document.getElementById("commandText");
    const headingText = document.getElementById("headingText");
    const ticksText = document.getElementById("ticksText");
    const wheelText = document.getElementById("wheelText");
    const sensorText = document.getElementById("sensorText");
    const stopButton = document.getElementById("stopButton");
    const commandButtons = [...document.querySelectorAll("[data-command]")];

    speedValueEl.textContent = speedEl.value;
    speedEl.addEventListener("input", () => {
      speedValueEl.textContent = speedEl.value;
    });

    async function sendCommand(command) {
      const params = new URLSearchParams({
        command,
        speed: speedEl.value
      });

      const response = await fetch("/api/move?" + params.toString(), { method: "POST" });
      if (!response.ok) {
        throw new Error("Request failed");
      }
    }

    async function refreshStatus() {
      const response = await fetch("/api/status");
      const data = await response.json();
      statusText.textContent = "State: " + data.state;
      commandText.textContent = "Command: " + data.command;
      headingText.textContent = "Heading: " + data.heading.toFixed(1) + " deg";
      ticksText.textContent = "Ticks: " + data.left_ticks + " / " + data.right_ticks;
      wheelText.textContent = "Wheels: L " + data.left_speed.toFixed(1) + " | R " + data.right_speed.toFixed(1);
      sensorText.textContent = "Sensors: F " + data.front + " | L " + data.left + " | R " + data.right;
    }

    async function refreshLogs() {
      const response = await fetch("/api/logs");
      const data = await response.json();
      logsEl.textContent = data.length ? data.join("\n") : "No logs yet.";
      logsEl.scrollTop = logsEl.scrollHeight;
    }

    async function issueCommand(command) {
      try {
        await sendCommand(command);
        await refreshStatus();
      } catch (error) {
        statusText.textContent = "State: request failed";
      }
    }

    commandButtons.forEach((button) => {
      button.addEventListener("click", () => issueCommand(button.dataset.command));
    });

    stopButton.addEventListener("click", () => issueCommand("stop"));

    async function refreshAll() {
      try {
        await Promise.all([refreshStatus(), refreshLogs()]);
      } catch (error) {
        statusText.textContent = "State: disconnected";
      }
    }

    refreshAll();
    setInterval(refreshAll, 500);
  </script>
</body>
</html>
)rawliteral";

const char* commandToText(ManualCommand command) {
  switch (command) {
    case CMD_FORWARD:
      return "forward";
    case CMD_BACKWARD:
      return "backward";
    case CMD_TURN_LEFT:
      return "turn_left";
    case CMD_TURN_RIGHT:
      return "turn_right";
    case CMD_TURN_AROUND:
      return "turn_around";
    case CMD_STOP:
      return "stop";
    default:
      return "none";
  }
}

ManualCommand parseCommand(const String &value) {
  if (value == "forward") return CMD_FORWARD;
  if (value == "backward") return CMD_BACKWARD;
  if (value == "turn_left") return CMD_TURN_LEFT;
  if (value == "turn_right") return CMD_TURN_RIGHT;
  if (value == "turn_around") return CMD_TURN_AROUND;
  if (value == "stop") return CMD_STOP;
  return CMD_NONE;
}

void motionServiceWebCallback() {
  webServer.handleClient();
}

void appendStatusJson(String &body) {
  DistanceData distances = readLidars();
  long leftTicks = 0;
  long rightTicks = 0;
  readTicks(leftTicks, rightTicks);

  body += "\"state\":\"";
  body += motionBusy ? "busy" : "idle";
  body += "\",\"command\":\"";
  body += commandToText(activeCommand == CMD_NONE ? pendingCommand : activeCommand);
  body += "\",\"speed\":";
  body += manualSpeed;
  body += ",\"heading\":";
  body += String(currentHeading, 2);
  body += ",\"left_ticks\":";
  body += leftTicks;
  body += ",\"right_ticks\":";
  body += rightTicks;
  body += ",\"left_speed\":";
  body += String(measuredLeftSpeedMMPS, 2);
  body += ",\"right_speed\":";
  body += String(measuredRightSpeedMMPS, 2);
  body += ",\"front\":";
  body += distances.front;
  body += ",\"left\":";
  body += distances.left;
  body += ",\"right\":";
  body += distances.right;
}

void respondWithStatus() {
  String body = "{";
  appendStatusJson(body);
  body += "}";
  webServer.send(200, "application/json", body);
}

void handleRoot() {
  webServer.send_P(200, "text/html", WEB_CONTROL_PAGE);
}

void handleMove() {
  ManualCommand requested = parseCommand(webServer.arg("command"));

  if (webServer.hasArg("speed")) {
    manualSpeed = constrain(webServer.arg("speed").toInt(), 60, 150);
  }

  if (requested == CMD_STOP) {
    pendingCommand = CMD_NONE;
    activeCommand = CMD_STOP;
    requestMotionAbort();
    appendControlLog("Stop requested from web");
  } else if (motionBusy) {
    appendControlLog(String("Ignored while busy: ") + commandToText(requested));
  } else {
    pendingCommand = requested;
    appendControlLog(String("Queued command: ") + commandToText(requested));
  }

  respondWithStatus();
}

void handleStatus() {
  respondWithStatus();
}

void handleLogs() {
  webServer.send(200, "application/json", getControlLogsJson());
}

void startAccessPoint() {
  WiFi.mode(WIFI_AP);

  bool started = WiFi.softAP(AP_SSID, AP_PASSWORD);
  if (started) {
    webControlReady = true;
    appendControlLog(String("Hotspot ready. SSID: ") + AP_SSID);
    appendControlLog(String("Password: ") + AP_PASSWORD);
    appendControlLog(String("Open http://") + WiFi.softAPIP().toString());
  } else {
    webControlReady = false;
    appendControlLog("Failed to start hotspot. Web control disabled.");
  }
}

void runPendingCommand() {
  if (pendingCommand == CMD_NONE || motionBusy) {
    return;
  }

  activeCommand = pendingCommand;
  pendingCommand = CMD_NONE;
  motionBusy = true;

  switch (activeCommand) {
    case CMD_FORWARD:
      moveForwardOneCellPID(manualSpeed);
      break;
    case CMD_BACKWARD:
      moveBackwardOneCellPID(manualSpeed);
      break;
    case CMD_TURN_LEFT:
      turnLeft90PID(manualSpeed);
      break;
    case CMD_TURN_RIGHT:
      turnRight90PID(manualSpeed);
      break;
    case CMD_TURN_AROUND:
      turnAroundPID(manualSpeed);
      break;
    default:
      break;
  }

  motionBusy = false;
  activeCommand = CMD_NONE;
}

void initWebControl() {
  setMotionServiceCallback(motionServiceWebCallback);
  startAccessPoint();
  if (!webControlReady) return;

  webServer.on("/", HTTP_GET, handleRoot);
  webServer.on("/api/move", HTTP_POST, handleMove);
  webServer.on("/api/status", HTTP_GET, handleStatus);
  webServer.on("/api/logs", HTTP_GET, handleLogs);
  webServer.begin();
}

void handleWebControl() {
  if (!webControlReady) {
    stopMotors();
    return;
  }

  webServer.handleClient();
  runPendingCommand();
}

#endif
