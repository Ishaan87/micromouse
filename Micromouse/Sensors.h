#ifndef SENSORS_H
#define SENSORS_H

#include <Adafruit_VL53L0X.h>
#include <Adafruit_MPU6050.h>
#include <Adafruit_Sensor.h>
#include <Wire.h>
#include "Config.h"

// ==========================================
// SENSOR OBJECTS
// ==========================================
Adafruit_VL53L0X lox_front = Adafruit_VL53L0X();
Adafruit_VL53L0X lox_left  = Adafruit_VL53L0X();
Adafruit_VL53L0X lox_right = Adafruit_VL53L0X();
Adafruit_MPU6050 mpu;

// ==========================================
// GYRO CALIBRATION STATE
// ==========================================
float gyroZ_offset = 0;

#define GYRO_DEADZONE 0.02  // rad/s — tune this value

// ==========================================
// DISTANCE DATA STRUCT
// ==========================================
struct DistanceData {
  int front;   // mm, 999 = nothing detected
  int left;
  int right;
};

// ==========================================
// LIDAR INIT
// — shuts all down first, wakes one by one
//   assigns unique I2C addresses
// ==========================================
void initLidars() {
  pinMode(XSHUT_FRONT, OUTPUT);
  pinMode(XSHUT_LEFT,  OUTPUT);
  pinMode(XSHUT_RIGHT, OUTPUT);

  // shut all down
  digitalWrite(XSHUT_FRONT, LOW);
  digitalWrite(XSHUT_LEFT,  LOW);
  digitalWrite(XSHUT_RIGHT, LOW);
  delay(50);

  // front → 0x30
  digitalWrite(XSHUT_FRONT, HIGH); delay(50);
  if (!lox_front.begin(0x30)) {
    Serial.println("Front lidar NOT found!");
  } else {
    Serial.println("Front lidar OK (0x30)");
  }

  // left → 0x31
  digitalWrite(XSHUT_LEFT, HIGH); delay(50);
  if (!lox_left.begin(0x31)) {
    Serial.println("Left lidar NOT found!");
  } else {
    Serial.println("Left lidar OK (0x31)");
  }

  // right → 0x32
  digitalWrite(XSHUT_RIGHT, HIGH); delay(50);
  if (!lox_right.begin(0x32)) {
    Serial.println("Right lidar NOT found!");
  } else {
    Serial.println("Right lidar OK (0x32)");
  }
}

// ==========================================
// MPU6050 INIT
// ==========================================
void initIMU() {
  if (!mpu.begin()) {
    Serial.println("MPU6050 NOT found!");
    return;
  }

  mpu.setGyroRange(MPU6050_RANGE_500_DEG);
  mpu.setAccelerometerRange(MPU6050_RANGE_4_G);
  mpu.setFilterBandwidth(MPU6050_BAND_21_HZ);

  Serial.println("MPU6050 OK (0x68)");
}

// ==========================================
// GYRO CALIBRATION
// — keep robot perfectly still during this
// ==========================================
void calibrateGyro() {
  Serial.println("Calibrating gyro — keep still...");

  float sum   = 0;
  int samples = 1000;

  for (int i = 0; i < samples; i++) {
    sensors_event_t a, g, temp;
    mpu.getEvent(&a, &g, &temp);
    sum += g.gyro.z;   // change to x or y if MPU mounted differently
    delay(2);
  }

  gyroZ_offset = sum / samples;

  Serial.print("Gyro offset: ");
  Serial.println(gyroZ_offset, 6);
  Serial.println("Calibration done!");
}

// ==========================================
// INIT ALL SENSORS
// ==========================================
void initSensors() {
  Wire.begin(SCL_PIN, SDA_PIN);
  initLidars();
  initIMU();
}

// ==========================================
// READ ALL LIDARS
// ==========================================
DistanceData readLidars() {
  VL53L0X_RangingMeasurementData_t measure;
  DistanceData data;

  lox_front.rangingTest(&measure, false);
  data.front = (measure.RangeStatus != 4) ? measure.RangeMilliMeter : 999;

  lox_left.rangingTest(&measure, false);
  data.left  = (measure.RangeStatus != 4) ? measure.RangeMilliMeter : 999;

  lox_right.rangingTest(&measure, false);
  data.right = (measure.RangeStatus != 4) ? measure.RangeMilliMeter : 999;

  return data;
}

// ==========================================
// READ GYRO — returns rad/s on yaw axis
// ==========================================
float readGyroHeading() {
  sensors_event_t a, g, temp;
  mpu.getEvent(&a, &g, &temp);

  // subtract calibration offset
  float gyroVal = g.gyro.z - gyroZ_offset;

  // apply deadzone — kills residual drift when still
  if (abs(gyroVal) < GYRO_DEADZONE) return 0.0;

  return gyroVal;
}

// ==========================================
// READ ACCELEROMETER — for crash detection
// ==========================================
float readAccelMagnitude() {
  sensors_event_t a, g, temp;
  mpu.getEvent(&a, &g, &temp);

  return sqrt(a.acceleration.x * a.acceleration.x +
              a.acceleration.y * a.acceleration.y +
              a.acceleration.z * a.acceleration.z);
}

// ==========================================
// WALL DETECTION HELPERS
// ==========================================
bool wallFront() { return readLidars().front < WALL_THRESHOLD; }
bool wallLeft()  { return readLidars().left  < WALL_THRESHOLD; }
bool wallRight() { return readLidars().right < WALL_THRESHOLD; }

#endif
