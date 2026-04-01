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
float gyro_bias = 0.0;

// Deadzone to prevent microscopic vibrations from causing integration drift
#define GYRO_DEADZONE 0.25  // degrees/s 

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
// MPU6050 INIT
// ==========================================
void initIMU() {
  if (!mpu.begin()) {
    Serial.println("CRITICAL ERROR: MPU6050 NOT found!");
    return;
    }

  mpu.setGyroRange(MPU6050_RANGE_500_DEG);
  mpu.setAccelerometerRange(MPU6050_RANGE_4_G);
  mpu.setFilterBandwidth(MPU6050_BAND_21_HZ);

  Serial.println("MPU6050 OK (0x68)");
}

// ==========================================
// GYRO CALIBRATION
// - keep robot perfectly still during this
// ==========================================
void calibrateGyro() {
  float sum   = 0;
  int samples = 1000;

  for (int i = 0; i < samples; i++) {
    sensors_event_t a, g, temp;
    mpu.getEvent(&a, &g, &temp);
    
    // NOTE: Using .x and converting to degrees/sec based on your main.ino logic
    sum += (g.gyro.x * 57.2958);   
    delay(2);
  }

  gyro_bias = sum / samples;
}

// ==========================================
// INIT ALL SENSORS
// ==========================================
void initSensors() {
  // CRITICAL FIX: I2C_SDA must come before I2C_SCL
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
// READ GYRO
// ==========================================
float readGyroHeading() {
  sensors_event_t a, g, temp;
  mpu.getEvent(&a, &g, &temp);

  // Read the vertical axis (X) and convert to degrees/sec
  float raw_Velocity = g.gyro.x * 57.2958; 
  float adjusted_Velocity = raw_Velocity - gyro_bias;

  // Apply deadzone to kill residual drift when still
  if (abs(adjusted_Velocity) < GYRO_DEADZONE) { 
    return 0.0; 
  }

  return adjusted_Velocity;
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
// Note: WALL_THRESHOLD must be defined in your Config.h or main.ino
// extern const int WALL_THRESHOLD; 

// bool wallFront() { return readLidars().front < WALL_THRESHOLD; }
// bool wallLeft()  { return readLidars().left  < WALL_THRESHOLD; }
// bool wallRight() { return readLidars().right < WALL_THRESHOLD; }

#endif