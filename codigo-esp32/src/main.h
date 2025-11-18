#ifndef MAIN_H
#define MAIN_H

#include <Arduino.h>
#include <WiFi.h>
#include <TinyGPS++.h>
#include <HardwareSerial.h>
#include <HCSR04.h>
#include <math.h>
#include <algorithm>
#include <cstdlib>
#include "BluetoothSerial.h"

#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "freertos/semphr.h"

// Variáveis de tempo de compilação e pinos
#include "pins.h"
#include "params.h"


// Headers das funcionalidades
#include "motor.h"
#include "drive.h"
#include "gps.h"
#include "encoder.h"
#include "web.h"
#include "imu.h"

//FINITE MACHINE STATE

enum MachineState
{
  Idle,
  SaveP0,
  DriveForward,
  SaveP1,
  RouteCalc,
  DriveToGoal,
  Evaluation,
  Done,
};

static MachineState state;

#endif // MAIN_H