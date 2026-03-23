#include "Config.h"
#include "Encoders.h"
#include "Motors.h"
#include "Sensors.h"

// ==========================================
// TUNING PARAMETERS (You will need to adjust these!)
// ==========================================

// --- Rotational (Steering) PD Constants ---
float Kp_imu = 3.5;       
float Kd_imu = 0.2;       // NEW: The "shock absorber" for steering
float Kp_wall = 0.4;      
float Kd_wall = 0.05;     // NEW: The "shock absorber" for wall centering
float alpha_D = 0.3;      // NEW: Low-Pass Filter strength (0.0 to 1.0)

// --- Translational (Distance) P Constants ---
float Kp_dist_encoder = 0.5; 
float Kp_dist_lidar = 2.0;   

// --- Maze Geometry Constants ---
const int IDEAL_SIDE_DIST = 45;  
const int IDEAL_STOP_DIST = 45;  
const int WALL_THRESHOLD = 120;  
const long TARGET_TICKS = 800;   

// ==========================================
// STATE VARIABLES
// ==========================================
float currentHeading = 0.0;
unsigned long lastTimeMicros = 0;
bool isMoving = true; 

// --- Memory for the Derivative Term ---
float last_headingError = 0.0;
float filtered_D_heading = 0.0;

float last_positionError = 0.0;
float filtered_D_position = 0.0;

void setup() {
  Serial.begin(115200);
  
  initMotors();
  initEncoders();
  initSensors(); 
  
  calibrateGyro(); 
  
  Serial.println("Starting in 2 seconds...");
  delay(2000); 

  resetEncoders(); 
  lastTimeMicros = micros();
}

void loop() {
  if (!isMoving) {
    stopMotors();
    Serial.println("Target Reached. Stopped.");
    delay(1000); 
    return; 
  }

  unsigned long currentMicros = micros();
  // Prevent divide-by-zero if loop runs impossibly fast
  float dt = max((currentMicros - lastTimeMicros) / 1000000.0, 0.0001); 
  lastTimeMicros = currentMicros;

  float gyroVelocity = readGyroHeading();
  currentHeading += (gyroVelocity * dt); 

  DistanceData lidars = readLidars();
  long currentTicks = (leftTicks + rightTicks) / 2; 

  // ==========================================
  // LOOP A: TRANSLATIONAL PID (Speed)
  // ==========================================
  int baseSpeed = 0;
  float distanceError = 0;

  if (lidars.front < WALL_THRESHOLD) {
    distanceError = lidars.front - IDEAL_STOP_DIST;
    baseSpeed = distanceError * Kp_dist_lidar;
  } else {
    distanceError = TARGET_TICKS - currentTicks;
    baseSpeed = distanceError * Kp_dist_encoder;
  }

  baseSpeed = constrain(baseSpeed, -100, 150); 

  if (abs(distanceError) < 5) {
    isMoving = false; 
  }

  // ==========================================
  // LOOP B: ROTATIONAL PD (Steering)
  // ==========================================
  float targetHeading = 0.0; 
  float positionError = 0.0;

  bool hasLeft = (lidars.left < WALL_THRESHOLD);
  bool hasRight = (lidars.right < WALL_THRESHOLD);

  if (hasLeft && hasRight) {
    positionError = lidars.left - lidars.right; 
  } else if (hasLeft && !hasRight) {
    positionError = lidars.left - IDEAL_SIDE_DIST;
  } else if (hasRight && !hasLeft) {
    positionError = IDEAL_SIDE_DIST - lidars.right; 
  }

  // --- Filtered D-Term for Wall Centering ---
  float raw_D_position = (positionError - last_positionError) / dt;
  filtered_D_position = (alpha_D * raw_D_position) + ((1.0 - alpha_D) * filtered_D_position);
  last_positionError = positionError;

  // Calculate target heading using both P and D
  targetHeading = (positionError * Kp_wall) + (filtered_D_position * Kd_wall);
  targetHeading = constrain(targetHeading, -10.0, 10.0);

  // --- Filtered D-Term for IMU Heading ---
  float headingError = targetHeading - currentHeading;
  
  float raw_D_heading = (headingError - last_headingError) / dt;
  filtered_D_heading = (alpha_D * raw_D_heading) + ((1.0 - alpha_D) * filtered_D_heading);
  last_headingError = headingError;

  // Final Steering Command
  int correctionSpeed = (int)((Kp_imu * headingError) + (Kd_imu * filtered_D_heading));

  // ==========================================
  // THE MIXER
  // ==========================================
  int leftMotorSpeed = baseSpeed - correctionSpeed;
  int rightMotorSpeed = baseSpeed + correctionSpeed;

  setMotorSpeeds(leftMotorSpeed, rightMotorSpeed);
}