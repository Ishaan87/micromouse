#include <Arduino.h>
#include "Config.h"
#include "Encoders.h"
#include "Motors.h"
#include "Sensors.h"
#include "Turns.h"

// Any object closer than this (in mm) will trigger a turn
const int HAND_TRIGGER_DIST_MM = 40; 

void setup() {
  Serial.begin(115200);
  delay(1000); 
  
  Serial.println("Initializing Micromouse...");
  applyMotorPWM(0, 0);
  
  // Initialize all hardware
  initSensors();
  delay(5000);
  calibrateGyro(); // Wait for BNO055 to settle
  resetYaw();      // Zero out the unwrapped yaw heading
  
  initMotors();
  initEncoders();
  
  Serial.println("========================================");
  Serial.println("Ready! Use Lidars as buttons:");
  Serial.println("  - Touch RIGHT Lidar -> Turn Right (CW 90)");
  Serial.println("  - Touch LEFT Lidar  -> Turn Left (ACW 90)");
  Serial.println("  - Touch FRONT Lidar -> Turn Around (180)");
  Serial.println("========================================");
}

void loop() {
  // 1. Read all LIDARs
  DistanceData lidars = readLidars();
  
  // 2. Check Right LIDAR (Turn Right / CW)
  if (lidars.right < HAND_TRIGGER_DIST_MM && lidars.right > 0) {
    Serial.println("RIGHT trigger detected! Moving hand away...");
    applyMotorPWM(0, 0); 
    delay(2000);         
    
    Serial.println("Executing: turnCW90()");
    turnCW90();
    
    Serial.println("Done. Waiting for next command.");
    delay(1000); // Debounce
  }
  // 3. Check Left LIDAR (Turn Left / ACW)
  else if (lidars.left < HAND_TRIGGER_DIST_MM && lidars.left > 0) {
    Serial.println("LEFT trigger detected! Moving hand away...");
    applyMotorPWM(0, 0); 
    delay(2000);         
    
    Serial.println("Executing: turnACW90()");
    turnACW90();
    
    Serial.println("Done. Waiting for next command.");
    delay(1000); // Debounce
  }
  // 4. Check Front LIDAR (Turn Around / 180)
  else if (lidars.front < HAND_TRIGGER_DIST_MM && lidars.front > 0) {
    Serial.println("FRONT trigger detected! Moving hand away...");
    applyMotorPWM(0, 0); 
    delay(2000);         
    
    Serial.println("Executing: turn180()");
    turn180();
    
    Serial.println("Done. Waiting for next command.");
    delay(1000); // Debounce
  }
  
  // Small delay to prevent spamming the I2C bus
  delay(20);
}