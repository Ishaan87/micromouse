#include <Arduino.h>
#include "Config.h"
#include "Motors.h"
#include "Encoders.h"
#include "Sensors.h"

// ==========================================
// TUNING PARAMETERS
// ==========================================
// Velocity PID Constants (Inner Loop -> Outputs PWM)
float Kp_vel = 4;  // Pushes the motor to the target speed
float Ki_vel = 0.0; // BUILDS UP to hold the steady-state PWM power
float Kd_vel = 0.1;  // Dampens sudden spikes

// Heading (Yaw) PID Constants (Outer Loop -> Outputs Velocity Ticks)
float Kp_yaw = 0.0;  // 1 degree of error = 0.5 ticks of speed difference
float Ki_yaw = 0.0;  // Keep 0 to prevent windup on turns
float Kd_yaw = 0.0;  // Helps smooth out the steering

// Target settings
float baseTargetVelocity = 1.0; // Base forward speed (ticks per 20ms)
float targetYaw = 0.0;           // 0 degrees = straight forward

// Tolerances (Set to 0 for tuning to prevent deadband jitter)
float vel_tolerance = 0.0; 
float yaw_tolerance = 0.0; 

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

// Global PWM outputs carried over between loops
int final_pwm_L = 0;
int final_pwm_R = 0;

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
  delay(1000); 

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
  
  Serial.println("\n[!] PREPARING GYRO CALIBRATION [!]");
  Serial.println("Please ensure the bot is perfectly still on a flat surface.");
  delay(2000);
  calibrateGyro(); 
  Serial.println("Gyro Calibration Successful.");

  // waitForStartSignal();

  // Reset everything right before the very first loop
  resetEncoders();
  prevLeftTicks = 0;
  prevRightTicks = 0;
  integral_vel_L = 0;
  integral_vel_R = 0;
  
  lastLoopTime = millis();
  lastPrintTime = millis();
}

// ==========================================
// MAIN LOOP
// ==========================================
void loop() {
  unsigned long currentTime = millis();
  
  // 1. 50Hz Fast Control Loop
  if (currentTime - lastLoopTime >= LOOP_INTERVAL_MS) {
    float dt = (currentTime - lastLoopTime) / 1000.0; 
    lastLoopTime = currentTime;
    
    runControlLoop(dt);
  }

  // 2. 10Hz Slow Telemetry Loop
  if (currentTime - lastPrintTime >= PRINT_INTERVAL_MS) {
    lastPrintTime = currentTime;
    printTelemetry();
  }
}

// ==========================================
// CONTROL LOOP (CASCADED PID)
// ==========================================
void runControlLoop(float dt) {
  // --- 1. Read Actual Velocities ---
  noInterrupts();
  long currentLeftTicks = leftTicks;
  long currentRightTicks = rightTicks;
  interrupts();

  float vel_L = (float)(currentLeftTicks - prevLeftTicks);
  float vel_R = (float)(currentRightTicks - prevRightTicks);
  
  prevLeftTicks = currentLeftTicks;
  prevRightTicks = currentRightTicks;

  // --- 2. Heading PID (Outer Loop) ---
  float yaw_rate = readGyroHeading(); 
  current_yaw_angle += yaw_rate * dt; 

  float error_yaw = targetYaw - current_yaw_angle;
  
  if (abs(error_yaw) <= yaw_tolerance) {
    error_yaw = 0;
    prev_error_yaw = 0; 
  }

  integral_yaw += error_yaw * dt;
  float deriv_yaw = (error_yaw - prev_error_yaw) / dt;

  // Output is the difference in ticks required to straighten out
  float heading_correction_vel = (Kp_yaw * error_yaw) + (Ki_yaw * integral_yaw) + (Kd_yaw * deriv_yaw);
  prev_error_yaw = error_yaw;

  // Calculate dynamic target velocities for each wheel
  float target_vel_L = baseTargetVelocity - heading_correction_vel;
  float target_vel_R = baseTargetVelocity + heading_correction_vel;

  // --- 3. Velocity PID (Inner Loop) ---
  float error_vel_L = target_vel_L - vel_L;
  float error_vel_R = target_vel_R - vel_R;

  if (abs(error_vel_L) <= vel_tolerance) {
    error_vel_L = 0;
    prev_error_vel_L = 0; 
  }
  if (abs(error_vel_R) <= vel_tolerance) {
    error_vel_R = 0;
    prev_error_vel_R = 0;
  }

  integral_vel_L += error_vel_L * dt;
  integral_vel_R += error_vel_R * dt;

  float deriv_vel_L = (error_vel_L - prev_error_vel_L) / dt;
  float deriv_vel_R = (error_vel_R - prev_error_vel_R) / dt;

  // Calculate required electrical power (PWM)
  final_pwm_L = (int)((Kp_vel * error_vel_L) + (Ki_vel * integral_vel_L) + (Kd_vel * deriv_vel_L));
  final_pwm_R = (int)((Kp_vel * error_vel_R) + (Ki_vel * integral_vel_R) + (Kd_vel * deriv_vel_R));

  prev_error_vel_L = error_vel_L;
  prev_error_vel_R = error_vel_R;

  // --- 4. Apply Speeds ---
  setMotorSpeeds(final_pwm_L, final_pwm_R);
}

// ==========================================
// TELEMETRY PRINTING
// ==========================================
void printTelemetry() {
  noInterrupts();
  long currLeft = leftTicks;
  long currRight = rightTicks;
  interrupts();

  Serial.print("TargetVel: ");
  Serial.print(baseTargetVelocity);
  
  Serial.print("  |  Enc[L/R]: ");
  Serial.print(currLeft); Serial.print(" / ");
  Serial.print(currRight);

  Serial.print("  |  PWM[L/R]: ");
  Serial.print(final_pwm_L); Serial.print(" / ");
  Serial.print(final_pwm_R);

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
      delay(500); 
    }

    if (isArmed && lidars.left > 100 && lidars.right > 100) {
      Serial.println("\nStarting in 3 seconds...");
      for(int i = 3; i > 0; i--) {
        Serial.print(i); Serial.println("...");
        delay(1000);
      }
      
      Serial.println("GO!");
      return; 
    }

    delay(50); 
  }
}