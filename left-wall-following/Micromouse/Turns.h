#ifndef TURNS_H
#define TURNS_H

#include "Config.h"
#include "Motors.h"
#include "Sensors.h"

// ==========================================
// TURN TUNING CONSTANTS (Split PD)
// ==========================================
// LEFT TURN (ACW) - Currently UNDERSHOOTING
// Needs more minimum push at the end, or less braking.
const float TURN_KP_ACW = 3.0;        
const float TURN_KD_ACW = 0.08;       // Lowered to brake less
const int MIN_TURN_PWM_ACW = 65;      // Raised to prevent stalling early

// RIGHT TURN (CW) - Currently OVERSHOOTING
// Needs more braking at the end, or less minimum push.
const float TURN_KP_CW = 3.0;         
const float TURN_KD_CW = 0.10;        // Raised to brake harder
const int MIN_TURN_PWM_CW = 65;       // Lowered so it doesn't push past the target

const int MAX_TURN_PWM = 255;     
const float TURN_TOLERANCE = 0.5; 

// ==========================================
// CORE TURNING LOGIC (PD-LOOP)
// ==========================================
float wrapTurnAngleDegrees(float angle) {
  while (angle > 180.0f) angle -= 360.0f;
  while (angle <= -180.0f) angle += 360.0f;
  return angle;
}

void turnToGlobalYaw(float targetYawGlobal) {
  targetYawGlobal = wrapTurnAngleDegrees(targetYawGlobal);

  float lastError = 0.0;
  unsigned long lastTime = millis();

  while (true) {
    float currentYaw = readYawDegrees();
    float error = wrapTurnAngleDegrees(targetYawGlobal - currentYaw);

    if (abs(error) <= TURN_TOLERANCE) {
      break; 
    }

    bool isTurningLeft = (error < 0.0f);

    unsigned long now = millis();
    float dt = (now - lastTime) / 1000.0;
    if (dt <= 0.0) dt = 0.001;            

    float derivative = (error - lastError) / dt;

    // Dynamically grab the correct constants based on direction
    float kp = isTurningLeft ? TURN_KP_ACW : TURN_KP_CW;
    float kd = isTurningLeft ? TURN_KD_ACW : TURN_KD_CW;
    int min_pwm = isTurningLeft ? MIN_TURN_PWM_ACW : MIN_TURN_PWM_CW;

    float pd_output = (error * kp) + (derivative * kd);

    int pwm = abs((int)pd_output);
    if (pwm < min_pwm) pwm = min_pwm;
    if (pwm > MAX_TURN_PWM) pwm = MAX_TURN_PWM;

    if (pd_output > 0) {
      applyMotorPWM(-pwm, -pwm); 
    } else {
      applyMotorPWM(pwm, pwm); 
    }

    lastError = error;
    lastTime = now;
    delay(10);
  }

  applyMotorPWM(0, 0);
  delay(150); 
}

// ==========================================
// MICROMOUSE MOVEMENT COMMANDS
// ==========================================

void turnCW90(float targetYawGlobal) { turnToGlobalYaw(targetYawGlobal); }
void turnACW90(float targetYawGlobal) { turnToGlobalYaw(targetYawGlobal); }
void turn180(float targetYawGlobal) { turnToGlobalYaw(targetYawGlobal); }

#endif
