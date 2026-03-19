#include <DFRobot_BMX160.h>
#include <Wire.h>

DFRobot_BMX160 bmx160;

float yaw          = 0;
float gyroZ_offset = 0;
unsigned long lastTime = 0;

float magX_offset = 0;
float magY_offset = 0;

#define ALPHA 0.98

// ─── Better calibration — more samples + deadzone ─────────
void calibrateGyro() {
  Serial.println("Calibrating — keep still for 3 seconds...");

  float sum     = 0;
  int samples   = 1000;   // more samples = better average

  for (int i = 0; i < samples; i++) {
    sBmx160SensorData_t magn, gyro, accel;
    bmx160.getAllData(&magn, &gyro, &accel);
    sum += gyro.z;
    delay(3);
  }

  gyroZ_offset = sum / samples;

  Serial.print("Gyro Z offset: ");
  Serial.println(gyroZ_offset, 6);  // print 6 decimal places
  Serial.println("Calibration done!");
}

// ─── Normalize any angle to 0-360 range ───────────────────
float normalizeAngle(float angle) {
  while (angle < 0)   angle += 360.0;
  while (angle > 360) angle -= 360.0;
  return angle;
}

// ─── Deadzone — ignore tiny gyro values when still ────────
// anything below this threshold treated as zero
#define GYRO_DEADZONE 0.003  // rad/s — tune this value

float applyDeadzone(float value) {
  if (abs(value) < GYRO_DEADZONE) return 0.0;
  return value;
}

// ─── Update yaw ───────────────────────────────────────────
void updateYaw() {
  sBmx160SensorData_t magn, gyro, accel;
  bmx160.getAllData(&magn, &gyro, &accel);

  float dt  = (millis() - lastTime) / 1000.0;
  lastTime  = millis();

  // subtract offset AND apply deadzone
  float gyroZ = gyro.z - gyroZ_offset;
  gyroZ       = applyDeadzone(gyroZ);  // ← kills residual drift

  // gyro integration
  float gyroYaw = yaw + gyroZ * dt * (180.0 / 3.14159);

  // magnetometer yaw
  float mx     = magn.x - magX_offset;
  float my     = magn.y - magY_offset;
  float magYaw = atan2(my, mx) * (180.0 / 3.14159);
  magYaw       = normalizeAngle(magYaw);  // ← normalize mag too

  // complementary filter
  float rawFused = ALPHA * gyroYaw + (1.0 - ALPHA) * magYaw;

  // ← normalize final yaw to 0-360
  yaw = normalizeAngle(rawFused);
}

void setup() {
  Serial.begin(115200);
  Wire.begin(9, 8);  // SDA=9, SCL=8
  delay(1000);

  if (!bmx160.begin()) {
    Serial.println("BMX160 not found!");
    while (1);
  }

  bmx160.setGyroRange(eGyroRange_500DPS);
  bmx160.setAccelRange(eAccelRange_4G);
  Serial.println("BMX160 found!");

  calibrateGyro();

  // initialize yaw from magnetometer at startup
  // so it starts at a meaningful value not 0
  sBmx160SensorData_t magn, gyro, accel;
  bmx160.getAllData(&magn, &gyro, &accel);
  float mx = magn.x - magX_offset;
  float my = magn.y - magY_offset;
  yaw      = normalizeAngle(atan2(my, mx) * (180.0 / 3.14159));

  lastTime = millis();
  Serial.print("Initial yaw: ");
  Serial.println(yaw);
}

void loop() {
  updateYaw();

  Serial.print("Yaw: ");
  Serial.println(yaw);

  delay(50);
}