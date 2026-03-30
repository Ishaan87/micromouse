// ==========================================
// MICROMOUSE - Main File
// ==========================================
// File structure:
//   Config.h   - pins and tuning constants
//   Motors.h   - per-wheel motor speed control
//   Encoders.h - encoder reading and speed calculation
//   Sensors.h  - lidar and IMU reading
//   PID.h      - wheel-speed PID helpers
// ==========================================

#include "Config.h"
#include "Encoders.h"
#include "Motors.h"
#include "Sensors.h"
#include "PID.h"

void setup() {
  Serial.begin(115200);
  delay(1000);

  initMotors();
  initEncoders();
  initSensors();
  calibrateGyro();

  Serial.println("=== Micromouse Ready ===");

  resetEncoders();
  startWheelSpeedControl();

  // Example:
  driveForwardSpeed(5.0);
}

void loop() {
  // Example continuous drive:
  driveForwardSpeed(5.0);

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
    Serial.print(" | WheelL:"); Serial.print(measuredLeftSpeedMMPS, 1);
    Serial.print(" WheelR:"); Serial.println(measuredRightSpeedMMPS, 1);
  }
}
