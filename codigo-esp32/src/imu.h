#ifndef IMU_H
#define IMU_H

#include <Adafruit_MPU6050.h>
#include <Adafruit_Sensor.h>


static Adafruit_MPU6050 mpu;

void MPU_begin();
void reset_yaw();
float update_yaw();
#endif