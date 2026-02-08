#include <Adafruit_VL53L0X.h>
#include <Adafruit_MPU6050.h>
#include <Adafruit_Sensor.h>
#include <Wire.h>
#include "Config.h"

// Objects for sensors
Adafruit_VL53L0X lox_front = Adafruit_VL53L0X();
Adafruit_VL53L0X lox_left  = Adafruit_VL53L0X();
Adafruit_VL53L0X lox_right = Adafruit_VL53L0X();
Adafruit_MPU6050 mpu;

void initSensors() {
  // 1. Initialize XSHUT pins
  pinMode(XSHUT_FRONT, OUTPUT);
  pinMode(XSHUT_LEFT, OUTPUT);
  pinMode(XSHUT_RIGHT, OUTPUT);

  // Shutdown all Lidars
  digitalWrite(XSHUT_FRONT, LOW);
  digitalWrite(XSHUT_LEFT, LOW);
  digitalWrite(XSHUT_RIGHT, LOW);
  delay(10);

  // 2. Wake up and address Lidars one by one
  digitalWrite(XSHUT_FRONT, HIGH); delay(10);
  lox_front.begin(0x30);

  digitalWrite(XSHUT_LEFT, HIGH); delay(10);
  lox_left.begin(0x31);

  digitalWrite(XSHUT_RIGHT, HIGH); delay(10);
  lox_right.begin(0x32);

  // 3. Initialize IMU (MPU6050)
  if (!mpu.begin()) {
    Serial.println("Failed to find MPU6050 chip");
  }
}

// Helper struct to get all distances at once
struct DistanceData {
  int front, left, right;
};

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