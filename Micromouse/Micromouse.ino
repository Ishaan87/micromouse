#include "Config.h"
#include "Encoders.h"
#include "Motors.h"

void setup() {
  Serial.begin(115200);
  delay(1000);
  
  initEncoders();
  initMotors();
  resetEncoders();
  Serial.println("=== Encoder Test ===");
  Serial.println("Spin wheels by hand or run motors to see readings");
  Serial.println("----------------------------------------------------");
}

void loop() {
  EncoderData e = readEncoders();
  moveBackward(255);

  Serial.println("--- Encoder Readings ---");

  Serial.print("Raw Ticks     →  Left: "); Serial.print(e.leftTicks);
  Serial.print("   Right: ");              Serial.println(e.rightTicks);

  Serial.print("Speed (RPM)   →  Left: "); Serial.print(e.leftSpeedRPM);
  Serial.print("   Right: ");              Serial.println(e.rightSpeedRPM);

  Serial.print("Speed (mm/s)  →  Left: "); Serial.print(e.leftSpeedMMPS);
  Serial.print("   Right: ");              Serial.println(e.rightSpeedMMPS);

  Serial.print("Distance (mm) →  Left: "); Serial.print(e.leftDistanceMM);
  Serial.print("   Right: ");              Serial.println(e.rightDistanceMM);

  Serial.println("----------------------------------------");
  delay(300);
}