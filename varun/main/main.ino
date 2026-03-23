#include "Config.h"
#include "Sensors.h"

float currentHeading = 0.0;
unsigned long lastTime = 0;

void setup() {
  Serial.begin(115200);
  
  // Initialize the I2C bus and sensors
  Wire.begin(9,8); 
  initSensors();
  calibrateGyro();
  
  Serial.println("MPU6050 Initialized. Keep the bot perfectly still...");
  delay(2000); // Give the gyro time to settle
  
  lastTime = millis();
}

void loop() {
  unsigned long currentTime = millis();
  float dt = (currentTime - lastTime) / 1000.0; // Time delta in seconds
  lastTime = currentTime;

  // Read the angular velocity and integrate it into heading
  float gyroZ = readGyroZ();
  currentHeading += (gyroZ * dt);

  // Print results to the Serial Plotter/Monitor
  Serial.print("Velocity(deg/s): "); 
  Serial.print(gyroZ);
  Serial.print("\t Yaw Angle(deg): "); 
  Serial.println(currentHeading);
  
  delay(10); // Small delay to prevent serial spam
}