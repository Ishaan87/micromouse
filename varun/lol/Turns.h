#ifndef TURNS_H
#define TURNS_H

#include "Config.h"
#include "Motors.h"
#include "Sensors.h"

// ==========================================
// TURN TUNING CONSTANTS (Split PD)
// ==========================================
// LEFT TURN (ACW) - Currently UNDERSHOOTING
const float TURN_KP_ACW = 3.0;        
const float TURN_KD_ACW = 0.08;       
const int MIN_TURN_PWM_ACW = 65;      

// RIGHT TURN (CW) - Currently OVERSHOOTING
const float TURN_KP_CW = 3.0;         
const float TURN_KD_CW = 0.10;        
const int MIN_TURN_PWM_CW = 65;       

const int MAX_TURN_PWM = 160;     
const float TURN_TOLERANCE = 0.5; 

// ==========================================
// CORE TURNING LOGIC (PD-LOOP)
// ==========================================
void turnAngle(float angleDelta) {
  float startYaw = readYawDegrees();
  float targetYaw = startYaw + angleDelta;

  float lastError = 0.0;
  unsigned long lastTime = millis();

  // Figure out which way we are turning before the loop starts
  bool isTurningLeft = (angleDelta > 0);

  while (true) {
    float currentYaw = readYawDegrees();
    float error = targetYaw - currentYaw;

    if (abs(error) <= TURN_TOLERANCE) {
      break; 
    }

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
      applyMotorPWM(-pwm, pwm); 
    } else {
      applyMotorPWM(pwm, -pwm); 
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
void turnCW90() { turnAngle(-90.0); }
void turnACW90() { turnAngle(90.0); }
void turn180() { turnAngle(-180.0); }

#endif