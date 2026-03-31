#ifndef SENSORS_H
#define SENSORS_H

#include <Adafruit_VL53L0X.h>
#include <Adafruit_BNO055.h>
#include <Adafruit_Sensor.h>
#include <utility/imumaths.h>
#include <Wire.h>
#include "Config.h"

// ==========================================
// SENSOR OBJECTS
// ==========================================
Adafruit_VL53L0X lox_front = Adafruit_VL53L0X();
Adafruit_VL53L0X lox_left  = Adafruit_VL53L0X();
Adafruit_VL53L0X lox_right = Adafruit_VL53L0X();

// BNO055: sensor ID 55, default I2C address 0x28
Adafruit_BNO055 bno = Adafruit_BNO055(55, 0x28, &Wire);

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
// - shuts all down first, wakes one by one
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
  delay(150);

  // front → 0x30
  pinMode(XSHUT_FRONT, INPUT); delay(50);
  if (!lox_front.begin(0x30)) {
    Serial.println("Front lidar NOT found!");
  } else {
    Serial.println("Front lidar OK (0x30)");
  }
  delay(150);

  // left → 0x31
  pinMode(XSHUT_LEFT, INPUT); delay(50);
  if (!lox_left.begin(0x31)) {
    Serial.println("Left lidar NOT found!");
  } else {
    Serial.println("Left lidar OK (0x31)");
  }
  delay(150);

  // right → 0x32
  pinMode(XSHUT_RIGHT, INPUT); delay(50);
  if (!lox_right.begin(0x32)) {
    Serial.println("Right lidar NOT found!");
  } else {
    Serial.println("Right lidar OK (0x32)");
  }
}

// ==========================================
// BNO055 INIT
// ==========================================
void initIMU() {
  if (!bno.begin()) {
    Serial.println("CRITICAL ERROR: BNO055 NOT found!");
    return;
  }

  // Use external 32.768 kHz crystal for better accuracy
  bno.setExtCrystalUse(true);

  Serial.println("BNO055 OK (0x28)");
}

// ==========================================
// CALIBRATE GYRO
// - BNO055 handles sensor fusion internally.
//   Waits for gyro to reach calibration level
//   3, then continues. Kept so main.ino
//   compiles unchanged.
// ==========================================
void calibrateGyro() {
  uint8_t sys, gyro, accel, mag = 0;

  Serial.println("Waiting for BNO055 gyro calibration...");

  unsigned long start = millis();
  while (gyro < 3) {
    bno.getCalibration(&sys, &gyro, &accel, &mag);
    Serial.printf("  Sys:%d  Gyro:%d  Accel:%d  Mag:%d\n", sys, gyro, accel, mag);
    delay(500);

    // Timeout after 10s — continue anyway (level 2 is usually fine)
    if (millis() - start > 10000) {
      Serial.println("Calibration timeout — continuing.");
      break;
    }
  }
}

// ==========================================
// INIT ALL SENSORS
// ==========================================
void initSensors() {
  Wire.begin(I2C_SCL, I2C_SDA);
  initIMU();
  initLidars();
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
// READ GYRO HEADING
// - Returns Z-axis angular velocity in
//   degrees/sec. Available for the yaw rate
//   derivative term if needed.
// ==========================================
float readGyroHeading() {
  imu::Vector<3> gyro = bno.getVector(Adafruit_BNO055::VECTOR_GYROSCOPE);
  // Negate if turns appear inverted during testing
  return gyro.z();
}

// ==========================================
// ABSOLUTE YAW — INTERNAL HELPERS
// ==========================================

// Converts current quaternion to raw yaw in degrees (-180..+180)
float _rawYawDegrees() {
  imu::Quaternion q = bno.getQuat();
  float yaw = atan2(2.0 * (q.w() * q.z() + q.x() * q.y()),
                    1.0 - 2.0 * (q.y() * q.y() + q.z() * q.z()));
  return yaw * 57.2958; // radians → degrees
}

// ==========================================
// UNWRAPPED CONTINUOUS YAW
// - Tracks cumulative rotation with no ±180
//   wraparound. Grows -inf to +inf.
// - Call resetYaw() once after calibration
//   to zero the heading.
// - Call readYawDegrees() every control loop
//   iteration — delta is computed each call
//   so don't skip calls or the delta will
//   be large and may misdetect a wraparound.
// ==========================================
static float last_raw_yaw  = 0.0;
static float yaw_unwrapped = 0.0;

void resetYaw() {
  last_raw_yaw  = _rawYawDegrees();
  yaw_unwrapped = 0.0;
}

float readYawDegrees() {
  float raw = _rawYawDegrees();

  // Take the shortest-path delta to handle the -180/+180 crossover
  float delta = raw - last_raw_yaw;
  if (delta >  180.0) delta -= 360.0;
  if (delta < -180.0) delta += 360.0;

  yaw_unwrapped += delta;
  last_raw_yaw   = raw;

  return yaw_unwrapped;
}

// ==========================================
// READ ACCELEROMETER — for crash detection
// ==========================================
float readAccelMagnitude() {
  imu::Vector<3> accel = bno.getVector(Adafruit_BNO055::VECTOR_ACCELEROMETER);
  return sqrt(accel.x() * accel.x() +
              accel.y() * accel.y() +
              accel.z() * accel.z());
}

// ==========================================
// WALL DETECTION HELPERS
// ==========================================
// Note: WALL_THRESHOLD must be defined in Config.h or main.ino
// extern const int WALL_THRESHOLD;

// bool wallFront() { return readLidars().front < WALL_THRESHOLD; }
// bool wallLeft()  { return readLidars().left  < WALL_THRESHOLD; }
// bool wallRight() { return readLidars().right < WALL_THRESHOLD; }

#endif