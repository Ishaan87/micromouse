#include <Arduino.h>
#include "Config.h"
#include "Motors.h"
#include "Encoders.h"
#include "Sensors.h"

// ==========================================
// TUNING PARAMETERS
// ==========================================
// Velocity PID Constants
float Kp_vel = 5.0;
float Ki_vel = 0.1;
float Kd_vel = 1.0;

// Heading (Yaw) PID Constants
float Kp_yaw = 2.0;
float Ki_yaw = 0.0;
float Kd_yaw = 0.5;

// Target settings
float targetVelocity = 15.0; // Target ticks per loop interval
float targetYaw = 0.0;       // 0 degrees = straight forward

// Tolerances (Deadbands)
float vel_tolerance = 1.0;   // Ticks per interval tolerance
float yaw_tolerance = 0.5;   // Degrees tolerance

// ==========================================
// SYSTEM VARIABLES
// ==========================================
const int LOOP_INTERVAL_MS = 20;  // 50Hz control loop
const int PRINT_INTERVAL_MS = 100; // 10Hz telemetry print loop

unsigned long lastLoopTime = 0;
unsigned long lastPrintTime = 0;

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
void printTelemetry();

// ==========================================
// SETUP
// ==========================================
void setup() {
  Serial.begin(115200);
  delay(1000); // Give serial monitor time to connect

  Serial.println("\n=================================");
  Serial.println("  MICROMOUSE SYSTEM STARTUP");
  Serial.println("=================================");
  
  Serial.println("[1/3] Initializing Motors...");
  initMotors();
  
  Serial.println("[2/3] Initializing Encoders...");
  initEncoders();
  
  Serial.println("[3/3] Initializing Sensors (I2C & Lidar XSHUT)...");
  initSensors();
  
  Serial.println(">>> Sensor Init Complete. <<<");
  
  // Calibrate Gyro
  Serial.println("\n[!] PREPARING GYRO CALIBRATION [!]");
  Serial.println("Please ensure the bot is perfectly still on a flat surface.");
  delay(1000);
  calibrateGyro(); 
  Serial.println("Gyro Calibration Successful.");

  // Block execution until you trigger the start sequence
  waitForStartSignal();

  // Reset trackers right before moving
  resetEncoders();
  prevLeftTicks = 0;
  prevRightTicks = 0;
  lastLoopTime = millis();
  lastPrintTime = millis();
}

// ==========================================
// MAIN LOOP
// ==========================================
void loop() {
  unsigned long currentTime = millis();
  
  // 1. Run the PID calculations strictly at the defined interval
  if (currentTime - lastLoopTime >= LOOP_INTERVAL_MS) {
    float dt = (currentTime - lastLoopTime) / 1000.0; // Delta time in seconds
    lastLoopTime = currentTime;
    
    runControlLoop(dt);
  }

  // 2. Print telemetry at a slower rate to avoid lagging the controller
  if (currentTime - lastPrintTime >= PRINT_INTERVAL_MS) {
    lastPrintTime = currentTime;
    printTelemetry();
  }
}

// ==========================================
// CONTROL LOOP (VELOCITY + HEADING PID)
// ==========================================
void runControlLoop(float dt) {
  // --- 1. Get Velocity ---
  noInterrupts();
  long currentLeftTicks = leftTicks;
  long currentRightTicks = rightTicks;
  interrupts();

  float vel_L = (float)(currentLeftTicks - prevLeftTicks);
  float vel_R = (float)(currentRightTicks - prevRightTicks);
  
  prevLeftTicks = currentLeftTicks;
  prevRightTicks = currentRightTicks;

  // --- 2. Velocity PID Calculations with Deadband ---
  float error_vel_L = targetVelocity - vel_L;
  float error_vel_R = targetVelocity - vel_R;

  // Apply Tolerance/Deadband
  if (abs(error_vel_L) <= vel_tolerance) {
    error_vel_L = 0;
    prev_error_vel_L = 0; // Prevent derivative kick when exiting deadband
  }
  if (abs(error_vel_R) <= vel_tolerance) {
    error_vel_R = 0;
    prev_error_vel_R = 0;
  }

  integral_vel_L += error_vel_L * dt;
  integral_vel_R += error_vel_R * dt;

  float deriv_vel_L = (error_vel_L - prev_error_vel_L) / dt;
  float deriv_vel_R = (error_vel_R - prev_error_vel_R) / dt;

  float base_pwm_L = (Kp_vel * error_vel_L) + (Ki_vel * integral_vel_L) + (Kd_vel * deriv_vel_L);
  float base_pwm_R = (Kp_vel * error_vel_R) + (Ki_vel * integral_vel_R) + (Kd_vel * deriv_vel_R);

  prev_error_vel_L = error_vel_L;
  prev_error_vel_R = error_vel_R;

  // --- 3. Heading PID Calculation with Deadband ---
  float yaw_rate = readGyroHeading(); 
  current_yaw_angle += yaw_rate * dt; 

  float error_yaw = targetYaw - current_yaw_angle;
  
  // Apply Yaw Tolerance
  if (abs(error_yaw) <= yaw_tolerance) {
    error_yaw = 0;
    prev_error_yaw = 0; 
  }

  integral_yaw += error_yaw * dt;
  float deriv_yaw = (error_yaw - prev_error_yaw) / dt;

  float heading_correction = (Kp_yaw * error_yaw) + (Ki_yaw * integral_yaw) + (Kd_yaw * deriv_yaw);
  prev_error_yaw = error_yaw;

  // --- 4. Combine and Apply Speeds ---
  int final_pwm_L = (int)(base_pwm_L - heading_correction);
  int final_pwm_R = (int)(base_pwm_R + heading_correction);

  setMotorSpeeds(final_pwm_L, final_pwm_R);
}

// ==========================================
// TELEMETRY PRINTING
// ==========================================
void printTelemetry() {
  DistanceData lidars = readLidars();
  
  noInterrupts();
  long currLeft = leftTicks;
  long currRight = rightTicks;
  interrupts();

  Serial.print("Lidar[L/F/R]: ");
  Serial.print(lidars.left); Serial.print(" / ");
  Serial.print(lidars.front); Serial.print(" / ");
  Serial.print(lidars.right);
  
  Serial.print("  |  Enc[L/R]: ");
  Serial.print(currLeft); Serial.print(" / ");
  Serial.print(currRight);

  Serial.print("  |  Yaw(deg): ");
  Serial.println(current_yaw_angle, 2);
}

// ==========================================
// START SEQUENCE LOGIC
// ==========================================
void waitForStartSignal() {
  Serial.println("\n=================================");
  Serial.println("       SYSTEM READY TO ARM       ");
  Serial.println("=================================");
  Serial.println("To arm: Block LEFT and RIGHT lidars (< 50mm) simultaneously.");
  
  bool isArmed = false;

  while (true) {
    DistanceData lidars = readLidars();

    if (!isArmed && lidars.left < 50 && lidars.right < 50) {
      isArmed = true;
      Serial.println("\n[!] ARMED! Remove hands to commence countdown...");
      delay(500); // Debounce
    }

    if (isArmed && lidars.left > 100 && lidars.right > 100) {
      Serial.println("\nStarting in 5 seconds...");
      for(int i = 5; i > 0; i--) {
        Serial.print(i); Serial.println("...");
        delay(1000);
      }
      
      Serial.println("GO!");
      return; 
    }

    delay(50); // Small delay to prevent I2C spamming while waiting
  }
}