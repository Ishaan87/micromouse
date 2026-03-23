#include <Arduino.h>
#include "Config.h"
#include "Motors.h"
#include "Encoders.h"
#include "Sensors.h"

// ==========================================
// TUNING PARAMETERS (You will need to adjust these!)
// ==========================================
// Velocity PID Constants
float Kp_vel = 5.0;
float Ki_vel = 0.0;
float Kd_vel = 1.0;

// Heading (Yaw) PID Constants
float Kp_yaw = 2.0;
float Ki_yaw = 0.0;
float Kd_yaw = 0.5;

// Target settings
float targetVelocity = 15.0; // Target ticks per loop interval
float targetYaw = 0.0;       // 0 degrees = straight forward

// ==========================================
// SYSTEM VARIABLES
// ==========================================
const int LOOP_INTERVAL_MS = 20; // 50Hz control loop
unsigned long lastLoopTime = 0;

// Integral and Derivative state variables
float integral_vel_L = 0, integral_vel_R = 0;
float prev_error_vel_L = 0, prev_error_vel_R = 0;

float integral_yaw = 0, prev_error_yaw = 0;
float current_yaw_angle = 0.0;

long prevLeftTicks = 0;
long prevRightTicks = 0;

// ==========================================
// FUNCTION PROTOTYPES
// ==========================================
void waitForStartSignal();
void runControlLoop(float dt);

// ==========================================
// SETUP
// ==========================================
void setup() {
  Serial.begin(115200);
  delay(1000);

  Serial.println("Initializing Subsystems...");
  initMotors();
  initEncoders();
  initSensors();
  
  // Calibrate Gyro while the bot is perfectly still
  calibrateGyro();

  // Block execution until you trigger the start sequence
  waitForStartSignal();

  // Reset trackers right before moving
  resetEncoders();
  prevLeftTicks = 0;
  prevRightTicks = 0;
  lastLoopTime = millis();
}

// ==========================================
// MAIN LOOP
// ==========================================
void loop() {
  unsigned long currentTime = millis();
  
  // Run the PID calculations strictly at the defined interval
  if (currentTime - lastLoopTime >= LOOP_INTERVAL_MS) {
    float dt = (currentTime - lastLoopTime) / 1000.0; // Delta time in seconds
    lastLoopTime = currentTime;
    
    runControlLoop(dt);
  }
}

// ==========================================
// CONTROL LOOP (VELOCITY + HEADING PID)
// ==========================================
void runControlLoop(float dt) {
  // 1. Calculate current wheel velocities (ticks per dt)
  // Disable interrupts briefly to safely read volatile 32-bit integers
  noInterrupts();
  long currentLeftTicks = leftTicks;
  long currentRightTicks = rightTicks;
  interrupts();

  float vel_L = (float)(currentLeftTicks - prevLeftTicks);
  float vel_R = (float)(currentRightTicks - prevRightTicks);
  
  prevLeftTicks = currentLeftTicks;
  prevRightTicks = currentRightTicks;

  // 2. Velocity PID Calculations
  float error_vel_L = targetVelocity - vel_L;
  float error_vel_R = targetVelocity - vel_R;

  integral_vel_L += error_vel_L * dt;
  integral_vel_R += error_vel_R * dt;

  float deriv_vel_L = (error_vel_L - prev_error_vel_L) / dt;
  float deriv_vel_R = (error_vel_R - prev_error_vel_R) / dt;

  float base_pwm_L = (Kp_vel * error_vel_L) + (Ki_vel * integral_vel_L) + (Kd_vel * deriv_vel_L);
  float base_pwm_R = (Kp_vel * error_vel_R) + (Ki_vel * integral_vel_R) + (Kd_vel * deriv_vel_R);

  prev_error_vel_L = error_vel_L;
  prev_error_vel_R = error_vel_R;

  // 3. Heading PID Calculation
  // Read angular velocity and integrate it to track the absolute angle
  float yaw_rate = readGyroHeading(); 
  current_yaw_angle += yaw_rate * dt; 

  float error_yaw = targetYaw - current_yaw_angle;
  integral_yaw += error_yaw * dt;
  float deriv_yaw = (error_yaw - prev_error_yaw) / dt;

  // Heading correction value
  float heading_correction = (Kp_yaw * error_yaw) + (Ki_yaw * integral_yaw) + (Kd_yaw * deriv_yaw);
  prev_error_yaw = error_yaw;

  // 4. Combine and Apply Speeds
  // If the bot drifts left (positive yaw), heading_correction becomes negative.
  // We subtract it from the left wheel and add to the right to correct back to 0.
  int final_pwm_L = (int)(base_pwm_L - heading_correction);
  int final_pwm_R = (int)(base_pwm_R + heading_correction);

  setMotorSpeeds(final_pwm_L, final_pwm_R);
}

// ==========================================
// START SEQUENCE LOGIC
// ==========================================
void waitForStartSignal() {
  Serial.println("Ready. Block LEFT and RIGHT lidars (< 50mm) to arm...");
  
  bool isArmed = false;

  while (true) {
    DistanceData lidars = readLidars();

    // Check if hands are covering both side lidars
    if (!isArmed && lidars.left < 50 && lidars.right < 50) {
      isArmed = true;
      Serial.println("ARMED! Remove hands to start.");
    }

    // If armed, wait for hands to be removed (hysteresis > 100mm)
    if (isArmed && lidars.left > 100 && lidars.right > 100) {
      Serial.println("Starting in 5 seconds...");
      
      // Flash motors or LED here if you want a visual cue
      delay(5000); 
      
      Serial.println("GO!");
      return; // Exit loop and begin main program
    }

    delay(50); // Small delay to prevent I2C spamming
  }
}