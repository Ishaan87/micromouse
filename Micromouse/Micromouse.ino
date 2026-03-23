// ==========================================
// MICROMOUSE - Main File
// ==========================================
// File structure:
//   Config.h    - all pins and tuning constants
//   Motors.h    - motor control functions
//   Encoders.h  - encoder reading + speed calc
//   Sensors.h   - lidar + IMU reading
//   PID.h       - all movement + PID functions
//   WebControl.h - WiFi + browser control interface
// ==========================================

#include "Config.h"
#include "Encoders.h"
#include "Motors.h"
#include "Sensors.h"
#include "PID.h"
#include "WebControl.h"

void setup() {
  Serial.begin(115200);
  delay(1000);

  initMotors();
  initEncoders();
  initSensors();
  calibrateGyro();

  Serial.println("=== Micromouse Ready ===");

  resetEncoders();
  lastTimeMicros = micros();
  initWebControl();
}

void loop() {
  handleWebControl();

  static unsigned long lastStatusPrint = 0;
  if (millis() - lastStatusPrint >= 500) {
    lastStatusPrint = millis();

    DistanceData d = readLidars();
    long l, r;
    readTicks(l, r);

    Serial.print("F:"); Serial.print(d.front);
    Serial.print(" L:"); Serial.print(d.left);
    Serial.print(" R:"); Serial.print(d.right);
    Serial.print(" | EncL:"); Serial.print(l);
    Serial.print(" EncR:"); Serial.print(r);
    Serial.print(" | Cmd:"); Serial.print(commandToText(activeCommand == CMD_NONE ? pendingCommand : activeCommand));
    Serial.print(" | Speed:"); Serial.println(manualSpeed);
  }
}
