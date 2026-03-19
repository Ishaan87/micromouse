#ifndef SENSORS_H
#define SENSORS_H

#include <Adafruit_VL53L0X.h>
#include <Adafruit_MPU6050.h>
#include <Adafruit_Sensor.h>
#include <Wire.h>
#include "Config.h"

// --- Sensor Objects ---
Adafruit_VL53L0X lox_front = Adafruit_VL53L0X();
Adafruit_VL53L0X lox_left  = Adafruit_VL53L0X();
Adafruit_VL53L0X lox_right = Adafruit_VL53L0X();
Adafruit_MPU6050 mpu;

float gyroZ_bias = 0.0; // The calibration offset

struct DistanceData {
  int front, left, right;
};

void initSensors() {
  if (!mpu.begin()) {
    Serial.println("CRITICAL ERROR: Failed to find MPU6050 chip");
    while(1);
  }
  mpu.setGyroRange(MPU6050_RANGE_500_DEG); 
  
  pinMode(XSHUT_FRONT, OUTPUT);
  pinMode(XSHUT_LEFT, OUTPUT);
  pinMode(XSHUT_RIGHT, OUTPUT);

  digitalWrite(XSHUT_FRONT, LOW);
  digitalWrite(XSHUT_LEFT, LOW);
  digitalWrite(XSHUT_RIGHT, LOW);
  delay(10);

  digitalWrite(XSHUT_FRONT, HIGH); delay(10);
  lox_front.begin(0x30);

  digitalWrite(XSHUT_LEFT, HIGH); delay(10);
  lox_left.begin(0x31);

  digitalWrite(XSHUT_RIGHT, HIGH); delay(10);
  lox_right.begin(0x32);
}

// Automatically finds the resting error of your specific gyro
void calibrateGyro() {
  Serial.print("Calibrating Gyro... Do NOT move the bot! ");
  float totalZ = 0;
  int numSamples = 500;
  
  for (int i = 0; i < numSamples; i++) {
    sensors_event_t a, g, temp;
    mpu.getEvent(&a, &g, &temp);
    totalZ += (g.gyro.x * 57.2958);
    delay(2); // Brief pause to let the sensor update
  }
  
  gyroZ_bias = totalZ / numSamples;
  Serial.println("Done!");
  Serial.print("Gyro Bias: "); Serial.println(gyroZ_bias);
}

// Fetches Gyro Z-axis and instantly removes the bias error
float readGyroZ() {
  sensors_event_t a, g, temp;
  mpu.getEvent(&a, &g, &temp);
  
  float rawZ = g.gyro.x* 57.2958; 
  float adjustedZ = rawZ - gyroZ_bias;

  // --- ADD THE DEADZONE ---
  // If the rotation is microscopic, force it to zero to stop drift
  if (abs(adjustedZ) < 0.15) { 
    return 0.0; 
  }
  
  return adjustedZ;
}


#endif