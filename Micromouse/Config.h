#ifndef CONFIG_H
#define CONFIG_H

// ==========================================
// MOTOR PINS
// ==========================================
const int M1_PWM = 0;
const int M1_DIR = 1;
const int M2_PWM = 2;
const int M2_DIR = 10;

// ==========================================
// MOTOR / ENCODER POLARITY
// Set to -1 if that side is reversed on your robot.
// ==========================================
const int LEFT_MOTOR_SIGN = 1;
const int RIGHT_MOTOR_SIGN = 1;
const int LEFT_ENCODER_SIGN = 1;
const int RIGHT_ENCODER_SIGN = 1;

// ==========================================
// ENCODER PINS
// ==========================================
const int ENC_L_A = 6;
const int ENC_L_B = 7;
const int ENC_R_A = 20;
const int ENC_R_B = 21;

// ==========================================
// LIDAR XSHUT PINS
// ==========================================
const int XSHUT_FRONT = 5;
const int XSHUT_LEFT  = 3;
const int XSHUT_RIGHT = 4;

// ==========================================
// I2C PINS
// ==========================================
const int SDA_PIN = 8;
const int SCL_PIN = 9;

// ==========================================
// WHEEL SPECS
// ==========================================
const int   PPR                    = 7;
const int   GEAR_RATIO             = 30;
const int   TICKS_PER_REV          = PPR * GEAR_RATIO;
const float WHEEL_DIAMETER_MM      = 34.0;
const float WHEEL_CIRCUMFERENCE_MM = 3.14159 * WHEEL_DIAMETER_MM;

// ==========================================
// SENSOR THRESHOLD
// ==========================================
const int   WALL_THRESHOLD         = 120;

// ==========================================
// MOTOR SPEED PID
// ==========================================
const float Kp_motor_speed         = 0.5;
const float Ki_motor_speed         = 0.0;
const float Kd_motor_speed         = 0.08;
const float MOTOR_PID_ALPHA        = 0.25;
const float MOTOR_INTEGRAL_LIMIT   = 180.0;

// ==========================================
// MOTOR SPEED LIMITS
// ==========================================
const float MAX_WHEEL_SPEED_MMPS   = 280.0;
const float MIN_WHEEL_SPEED_MMPS   = 10.0;

#endif
