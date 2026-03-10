#ifndef CONFIG_H
#define CONFIG_H

// --- Motor Pins (Romeo Mini ESP32-C3 Internal) ---
const int M1_PWM = 0;
const int M1_DIR = 1;
const int M2_PWM = 2;
const int M2_DIR = 10;

// --- Encoder Pins ---
const int ENC_L_A = 6;  // Motor Left Phase A
const int ENC_L_B = 7;  // Motor Left Phase B
const int ENC_R_A = 20; // Motor Right Phase A
const int ENC_R_B = 21; // Motor Right Phase B

// --- Lidar XSHUT Pins ---
const int XSHUT_FRONT = 4;
const int XSHUT_LEFT  = 5;
const int XSHUT_RIGHT = 3;

#endif