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

// ==========================================
// ENCODER PINS
// ==========================================
// Note: ESP32-C3 supports interrupts on all GPIO pins.
const int ENC_L_A = 6; // Left Encoder Phase A
const int ENC_L_B = 7; // Left Encoder Phase B
const int ENC_R_A = 20; // Right Encoder Phase A
const int ENC_R_B = 21; // Right Encoder Phase B

// ==========================================
// I2C & LIDAR XSHUT PINS
// ==========================================
const int I2C_SDA = 8; // Your MPU6050 & Lidar SDA pin
const int I2C_SCL = 9; // Your MPU6050 & Lidar SCL pin

const int XSHUT_FRONT = 5; // Front VL53L0X
const int XSHUT_LEFT  = 4; // Left VL53L0X
const int XSHUT_RIGHT = 3; // Right VL53L0X


const int WALL_THRESHOLD = 150;



// ==========================================
// ROBOT PHYSICAL CONSTANTS & LIMITS
// ==========================================
const float TICKS_PER_REV = 360.0;          // UPDATE THIS: Ticks for 1 full wheel rotation
const float WHEEL_CIRCUMFERENCE_MM = 100.0; // UPDATE THIS: Pi * Wheel Diameter in mm

const float CELL_SIZE_MM = 180.0;           // Standard Micromouse cell size

// The Magic Number: How many ticks to move exactly one cell
const long TICKS_PER_CELL = (long)((CELL_SIZE_MM / WHEEL_CIRCUMFERENCE_MM) * TICKS_PER_REV);



// const float MAX_WHEEL_SPEED_MMPS = 500.0;
// const float MIN_WHEEL_SPEED_MMPS = 50.0;
// const float MOTOR_INTEGRAL_LIMIT = 200.0;
// const float MOTOR_PID_ALPHA = 0.2;

// // Inner PID loop constants for Motors.h
// const float Kp_motor_speed = 1.0;
// const float Ki_motor_speed = 0.1;
// const float Kd_motor_speed = 0.05;

#endif