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
// MAZE GEOMETRY
// ==========================================
const int   CELL_SIZE_MM           = 180;
const long  TARGET_TICKS           = 800;
const int   IDEAL_SIDE_DIST        = 45;
const int   IDEAL_STOP_DIST        = 45;
const int   WALL_THRESHOLD         = 120;

// ==========================================
// STRAIGHT-LINE PID
// ==========================================
const float Kp_dist_encoder        = 0.5;
const float Kd_dist_encoder        = 0.02;
const float Kp_dist_lidar          = 2.0;
const float Kp_heading_hold        = 1.8;
const float Kd_heading_hold        = 0.08;
const float Kp_straight_sensor     = 0.7;
const float Kd_straight_sensor     = 0.08;
const float alpha_D                = 0.3;

// ==========================================
// TURN PID
// ==========================================
const float Kp_turn                = 3.0;
const float Kd_turn                = 0.2;
const float TURN_THRESHOLD_DEG     = 2.0;
const int   TURN_SPEED_MIN         = 50;
const int   TURN_SPEED_MAX         = 150;

// ==========================================
// MOTOR SPEED PID
// ==========================================
const float Kp_motor_speed         = 1.3;
const float Ki_motor_speed         = 0.18;
const float Kd_motor_speed         = 0.02;
const float MOTOR_PID_ALPHA        = 0.25;
const float MOTOR_INTEGRAL_LIMIT   = 180.0;

// ==========================================
// GENERAL
// ==========================================
const int   BASE_SPEED             = 120;
const float MAX_WHEEL_SPEED_MMPS   = 280.0;
const float MIN_WHEEL_SPEED_MMPS   = 70.0;

// ==========================================
// WEB CONTROL
// ==========================================
const char* AP_SSID                = "Micromouse-Control";
const char* AP_PASSWORD            = "micromouse123";
const int   WEB_CONTROL_PORT       = 80;
const int   WEB_DEFAULT_SPEED      = 120;
const int   WEB_COMMAND_TIMEOUT_MS = 400;

#endif
