#include "drive.h"

// ===== Controle diferencial básico =====

static float distance=0;
unsigned long timer; 
float dt;

// Curva com diferencial: base ∈ [0..255]; steer ∈ [-1..+1] (esq negativo, dir positivo)
void resetDistance(){
  distance=0;
}

void resetTimer(){
  timer=millis();
}


bool metersStraight(float distanceToGo, float kmh){
  setMotor(0,255);
  setMotor(1,255);
  unsigned long currentMillis = millis();
  dt = (currentMillis - timer) / 1000.0; // Convert to seconds
  timer = currentMillis;

  distance += kmh*dt/(3.6);
  Serial.print(kmh);
  Serial.print(" ");
  Serial.println(distance);
  if(distance >= distanceToGo){
    stopMotors();
    return true;
  }
  return false;
}


void driveStraight(float yaw, float yaw_ref){
  float correction = (yaw- yaw_ref)*2;
  setMotor(1,190-correction);
  setMotor(2,190+correction);
}

// Giro no lugar: sinal ∈ {-1, +1}
void turnInPlace(int sign, uint8_t pwm) {
  int left  = (sign > 0) ? +pwm : -pwm;  // +: gira pra direita
  int right = (sign > 0) ? -pwm : +pwm;
  setMotor(1, left);
  setMotor(2, right);
}

float Kp = 3;  // Proportional gain - controls reaction speed
float Ki = 0.35;  // Integral gain - eliminates small steady-state errors
float Kd = 2.5;  // Derivative gain - dampens overshoot

// PID state variables
float pidError = 0;
float pidLastError = 0;
float pidIntegral = 0;
float pidDerivative = 0;



float timeInRange=0;

bool turnByAngle(float yaw_ref,float yaw)
{
  unsigned long currentMillis = millis();
  dt = (currentMillis - timer) / 1000.0; // Convert to seconds
  timer = currentMillis;

  pidError = yaw_ref - yaw;
  Serial.print(pidError);
  Serial.print(" ");
  Serial.println(timeInRange);
  if (abs(pidError) < 5) {
    timeInRange+=dt;
  }else{
    timeInRange = 0;
  }

  // if(abs(pidError) < 30){
  //   Ki = 0.3f;
  // }else{
  //   Ki = 0.1f;  // Integral gain - eliminates small steady-state errors
  // }

  if(timeInRange>5){
    // Serial.println("Target angle reached.");
    stopMotors();
    pidIntegral = 0; // Reset for next turn
    pidLastError = 0;
    return true; 
  }

  // Integral (with anti-windup)
  pidIntegral = pidIntegral + (pidError * dt);

  // Derivative
  pidDerivative = (pidError - pidLastError) / dt;

  // Save error for next loop
  pidLastError = pidError;

  // --- 4. Calculate PID Output ---
  int outputSpeed = (Kp * pidError) + (Ki * pidIntegral) + (Kd * pidDerivative);

  // Constrain output to the max speed
  outputSpeed = constrain(outputSpeed, -185, 185);



  // --- 5. Apply Motor Speed ---
  // turnInPlace(outputSpeed, abs(outputSpeed)); // Use the new function to set motor speeds
  setMotor(1, outputSpeed);
  setMotor(2, -outputSpeed);


  return false; // No, the turn is not yet complete
}