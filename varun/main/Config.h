#ifndef CONFIG_H
#define CONFIG_H

#include <Arduino.h>

// ==========================================
// MOTOR DRIVER PINS (Romeo Mini ESP32-C3)
// ==========================================
const int M1_PWM = 0;  // Left Motor Speed (0-255)
const int M1_DIR = 1;  // Left Motor Direction (HIGH/LOW)
const int M2_PWM = 2;  // Right Motor Speed (0-255)
const int M2_DIR = 10; // Right Motor Direction (HIGH/LOW)
const bool LEFT_MOTOR_INVERTED = false;
const bool RIGHT_MOTOR_INVERTED = true;

// ==========================================
// ENCODER PINS
// ==========================================
// Note: ESP32-C3 supports interrupts on all GPIO pins.
const int ENC_L_A = 3; // Left Encoder Phase A
const int ENC_L_B = 4; // Left Encoder Phase B
const int ENC_R_A = 5; // Right Encoder Phase A
const int ENC_R_B = 6; // Right Encoder Phase B
const bool LEFT_ENCODER_INVERTED = false;
const bool RIGHT_ENCODER_INVERTED = false;
const unsigned long ENCODER_GLITCH_FILTER_US = 80;

// ==========================================
// I2C & LIDAR XSHUT PINS
// ==========================================
const int I2C_SDA = 8; // Your MPU6050 & Lidar SDA pin
const int I2C_SCL = 9; // Your MPU6050 & Lidar SCL pin

const int XSHUT_FRONT = 7; // Front VL53L0X
const int XSHUT_LEFT  = 18; // Left VL53L0X
const int XSHUT_RIGHT = 19; // Right VL53L0X

// ==========================================
// WEB CONTROL
// ==========================================
constexpr char WIFI_AP_SSID[] = "Micromouse-Control";
constexpr char WIFI_AP_PASSWORD[] = "micromouse123";
constexpr int TELEMETRY_LOG_CAPACITY = 300;

#endif
