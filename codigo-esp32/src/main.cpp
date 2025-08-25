#include <Arduino.h>

// Mapeamento de pinos
#define M1_EN   14  // ENA - PWM motor A
#define M1_IN1  27  // IN1 motor A
#define M1_IN2  26  // IN2 motor A

#define M2_EN   32  // ENB - PWM motor B
#define M2_IN1  25  // IN3 motor B
#define M2_IN2  33  // IN4 motor B

// Canais PWM do ESP32
#define CH_M1   0
#define CH_M2   1

// Config PWM
const int PWM_FREQ = 20000;   // 20 kHz
const int PWM_RES  = 10;      // 10 bits (0..1023)

// Função utilitária: controla direção e velocidade (−255..+255)
void setMotor(int motor, int speed) {
  int in1, in2, ch;
  if (motor == 1) { in1 = M1_IN1; in2 = M1_IN2; ch = CH_M1; }
  else            { in1 = M2_IN1; in2 = M2_IN2; ch = CH_M2; }

  if (speed <= -255) speed = -255;
  if (speed >=  255) speed =  255;

  if (speed == 0) {
    digitalWrite(in1, LOW);
    digitalWrite(in2, LOW);
    ledcWrite(ch, 0);
    return;
  }

  if (speed > 0) {        // Frente
    digitalWrite(in1, HIGH);
    digitalWrite(in2, HIGH);
  } else {                 // Ré
    digitalWrite(in1, LOW);
    digitalWrite(in2, LOW);
  }

  int duty = map(abs(speed), 0, 255, 0, (1 << PWM_RES) - 1);
  ledcWrite(ch, duty);
}

void setup() {
  // Direção
  pinMode(M1_IN1, OUTPUT);
  pinMode(M1_IN2, OUTPUT);
  pinMode(M2_IN1, OUTPUT);
  pinMode(M2_IN2, OUTPUT);

  // PWM
  ledcSetup(CH_M1, PWM_FREQ, PWM_RES);
  ledcAttachPin(M1_EN, CH_M1);

  ledcSetup(CH_M2, PWM_FREQ, PWM_RES);
  ledcAttachPin(M2_EN, CH_M2);

  // 100% PARA FRENTE nos dois motores
  setMotor(1, 255);
  setMotor(2, 255);
}

void loop() {
  // Nada: mantém 100% para frente
}
