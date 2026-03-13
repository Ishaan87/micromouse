#ifndef SENSORS_H
#define SENSORS_H

#include <Adafruit_VL53L0X.h>
#include <Adafruit_MPU6050.h>
#include <Adafruit_Sensor.h>
#include <Wire.h>
#include "Config.h"

// Sensor objects
Adafruit_VL53L0X lox_front = Adafruit_VL53L0X();
Adafruit_VL53L0X lox_left  = Adafruit_VL53L0X();
Adafruit_VL53L0X lox_right = Adafruit_VL53L0X();
Adafruit_MPU6050 mpu;

// Struct to hold all lidar readings
struct DistanceData {
  int front, left, right;
};

void initSensors() {
  Wire.begin(9,8); // SDA, SCL for Romeo Mini ESP32-C3

  // Step 1 — shut ALL sensors down first
  pinMode(XSHUT_FRONT, OUTPUT);
  pinMode(XSHUT_LEFT,  OUTPUT);
  pinMode(XSHUT_RIGHT, OUTPUT);

  digitalWrite(XSHUT_FRONT, LOW);
  digitalWrite(XSHUT_LEFT,  LOW);
  digitalWrite(XSHUT_RIGHT, LOW);
  delay(50);

  // Step 2 — wake front, move from 0x29 → 0x30
  digitalWrite(XSHUT_FRONT, HIGH);
  delay(50);
  if (!lox_front.begin(0x30)) {
    Serial.println("Front lidar NOT found!");
  } else {
    Serial.println("Front lidar OK at 0x30");
  }

  // Step 3 — wake left, move from 0x29 → 0x31
  digitalWrite(XSHUT_LEFT, HIGH);
  delay(50);
  if (!lox_left.begin(0x31)) {
    Serial.println("Left lidar NOT found!");
  } else {
    Serial.println("Left lidar OK at 0x31");
  }

  // Step 4 — wake right, move from 0x29 → 0x32
  digitalWrite(XSHUT_RIGHT, HIGH);
  delay(50);
  if (!lox_right.begin(0x32)) {
    Serial.println("Right lidar NOT found!");
  } else {
    Serial.println("Right lidar OK at 0x32");
  }

  // Step 5 — initialize MPU6050
  if (!mpu.begin()) {
    Serial.println("MPU6050 NOT found!");
  } else {
    Serial.println("MPU6050 OK at 0x68");
  }
}

DistanceData readLidars() {
  VL53L0X_RangingMeasurementData_t measure;
  DistanceData data;

  lox_front.rangingTest(&measure, false);
  data.front = (measure.RangeStatus != 4) ? measure.RangeMilliMeter : 999;

  lox_left.rangingTest(&measure, false);
  data.left = (measure.RangeStatus != 4) ? measure.RangeMilliMeter : 999;

  lox_right.rangingTest(&measure, false);
  data.right = (measure.RangeStatus != 4) ? measure.RangeMilliMeter : 999;

  return data;
}

// Read gyro Z axis (yaw) from MPU6050
// Returns rotation in degrees per second
float readGyroZ() {
  sensors_event_t a, g, temp;
  mpu.getEvent(&a, &g, &temp);
  return g.gyro.z; // rad/s
}

#endif