#include "imu.h"

static unsigned long timer;
static float dt;
static float yaw = 0;


void MPU_begin(){
// --- Initialize MPU6050 ---
  Wire.begin(); // ESP32 defaults SDA=21, SCL=22

  if (!mpu.begin()) {
    Serial.println("Failed to find MPU6050 chip. Check wiring.");
    while (true)
    {
      delay(1000);
    }
  }
  Serial.println("MPU6050 Found!");

  // Configure Sensor Settings
  mpu.setAccelerometerRange(MPU6050_RANGE_2_G);
  mpu.setGyroRange(MPU6050_RANGE_500_DEG);
  mpu.setFilterBandwidth(MPU6050_BAND_21_HZ);
  timer = millis();
}

void reset_yaw(){
    yaw=0;
}

float update_yaw(){
    unsigned long currentMillis = millis();
    dt = (currentMillis - timer) / 1000.0; // Convert to seconds
    timer = currentMillis;

    // Get sensor data
    sensors_event_t a, g, temp;
    mpu.getEvent(&a, &g, &temp);

    // Get gyro rates in deg/s
    float gyroZ = g.gyro.z * RAD_TO_DEG;

    // Integrate gyro for yaw (this will drift, but is fine for a short turn)
    yaw = yaw + gyroZ * dt;

    return yaw;
}