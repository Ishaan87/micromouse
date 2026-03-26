#ifndef SENSORS_H
#define SENSORS_H

#include <Adafruit_VL53L0X.h>
#include <Adafruit_MPU6050.h>
#include <Adafruit_Sensor.h>
#include <Wire.h>
#include "Config.h"

Adafruit_VL53L0X lox_front = Adafruit_VL53L0X();
Adafruit_VL53L0X lox_left  = Adafruit_VL53L0X();
Adafruit_VL53L0X lox_right = Adafruit_VL53L0X();
Adafruit_MPU6050 mpu;

float gyro_bias = 0.0;

struct DistanceData {
  int front, left, right;
};

void initSensors() {
  Serial.println("initsensors");
  // Start I2C using your custom ESP32-C3 pins
  Wire.begin(I2C_SCL, I2C_SDA);
  Serial.println("wirebegin");

  // 1. Initialize MPU6050
  if (!mpu.begin()) {
    Serial.println("CRITICAL ERROR: Failed to find MPU6050 chip");
    while(1);
  }
  mpu.setGyroRange(MPU6050_RANGE_500_DEG);

  
  // 2. Initialize XSHUT pins to multiplex the LiDARs
  pinMode(XSHUT_FRONT, OUTPUT);
  pinMode(XSHUT_LEFT, OUTPUT);
  pinMode(XSHUT_RIGHT, OUTPUT);

  digitalWrite(XSHUT_FRONT, LOW);
  digitalWrite(XSHUT_LEFT, LOW);
  digitalWrite(XSHUT_RIGHT, LOW); 
  delay(10);

  // Wake up and assign new I2C addresses sequentially
  digitalWrite(XSHUT_FRONT, HIGH); delay(10);
  lox_front.begin(0x30);

  digitalWrite(XSHUT_LEFT, HIGH); delay(10);
  lox_left.begin(0x31);

  digitalWrite(XSHUT_RIGHT, HIGH); delay(10);
  lox_right.begin(0x32);
  Serial.println("done");
}

void calibrateGyro() {
  Serial.print("Calibrating Gyro... Do NOT move the bot! ");
  float total = 0;
  int numSamples = 500;
  
  for (int i = 0; i < numSamples; i++) {
    sensors_event_t a, g, temp;
    mpu.getEvent(&a, &g, &temp);
    
    // NOTE: Using .x because the sensor is mounted vertically! 
    // Change to .y if your $90^\circ$ turns are still unresponsive.
    total += (g.gyro.x * 57.2958);
    delay(2); 
  }
  
  gyro_bias = total / numSamples;
  Serial.println("Done!");
}

float readGyroHeading() {
  sensors_event_t a, g, temp;
  mpu.getEvent(&a, &g, &temp);
  
  // Read the vertical axis and subtract the resting error
  float raw_Velocity = g.gyro.x * 57.2958; 
  float adjusted_Velocity = raw_Velocity - gyro_bias;

  // DEADZONE: Ignore microscopic vibrations to prevent integration drift
  if (abs(adjusted_Velocity) < 0.2) { 
    return 0.0; 
  }
  
  return adjusted_Velocity;
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

#endif