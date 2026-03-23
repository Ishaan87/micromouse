#include <Adafruit_MPU6050.h>
#include <Adafruit_Sensor.h>
#include <Wire.h>

Adafruit_MPU6050 mpu;

void setup() {
  Serial.begin(115200);
  Wire.begin(9, 8);  // SDA=9, SCL=8
  delay(1000);

  if (!mpu.begin()) {
    Serial.println("MPU6050 not found!");
    while (1);
  }

  mpu.setGyroRange(MPU6050_RANGE_500_DEG);
  mpu.setAccelRange(MPU6050_ACCEL_RANGE_4_G);
  mpu.setFilterBandwidth(MPU6050_BAND_21_HZ);

  Serial.println("MPU6050 found!");
  Serial.println("Rotate robot LEFT/RIGHT and watch which axis changes");
}

void loop() {
  sensors_event_t a, g, temp;
  mpu.getEvent(&a, &g, &temp);

  Serial.print("GyroX: "); Serial.print(g.gyro.x);
  Serial.print("  GyroY: "); Serial.print(g.gyro.y);
  Serial.print("  GyroZ: "); Serial.println(g.gyro.z);

  delay(100);
}