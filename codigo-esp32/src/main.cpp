#include <Arduino.h>
#include <WiFi.h>
#include <WebServer.h>

// Replace with your network credentials
const char *ssid = "integrador2";
const char *password = "passwordbigger2";

// Create a web server object that listens on port 80
WebServer server(80);

// Pin mapping from your original code
#define M1_EN 14  // ENA - PWM motor A
#define M1_IN1 27 // IN1 motor A
#define M1_IN2 26 // IN2 motor A

#define M2_EN 32  // ENB - PWM motor B
#define M2_IN1 25 // IN3 motor B
#define M2_IN2 33 // IN4 motor B

// ESP32 PWM channels
#define CH_M1 0
#define CH_M2 1

// PWM configuration
const int PWM_FREQ = 20000; // 20 kHz
const int PWM_RES = 10;     // 10 bits (0..1023)

// ==== User settings ====
const uint8_t HALL_LEFT = 19;        // change to your pin
const uint8_t HALL_RIGHT = 18;
const float   R = 0.03f;            // wheel radius [m]
const uint8_t MAGNETS_PER_REV = 1;  // number of magnets on the wheel
const unsigned long TIMEOUT_US = 500000; // 0.5 seconds in microseconds

// Derived
const float PERIMETER = 2.0f * PI * R;
const float DIST_PER_PULSE = PERIMETER / MAGNETS_PER_REV;

// ISR-shared
volatile unsigned long lastPulseTimeUs = 0;
volatile unsigned long periodUs = 0;

float kmh = 0.0f;

void hallISR() {
  unsigned long now = millis();
  periodUs = now - lastPulseTimeUs;
  lastPulseTimeUs = now;
  Serial.println("detectou!");
}

// Função utilitária: controla direção e velocidade (−255..+255)
void setMotor(int motor, int speed) {
  int in1, in2, ch;
  if (motor == 1) {
    in1 = M1_IN1;
    in2 = M1_IN2;
    ch = CH_M1;
  } else {
    in1 = M2_IN1;
    in2 = M2_IN2;
    ch = CH_M2;
  }

  // Ensure speed is within the valid range
  speed = constrain(speed, -255, 255);

  if (speed == 0) {
    // Stop the motor by setting both direction pins low and PWM duty to 0
    digitalWrite(in1, LOW);
    digitalWrite(in2, LOW);
    ledcWrite(ch, 0);
    return;
  }

  if (speed > 0) { // Frente
    digitalWrite(in1, HIGH);
    digitalWrite(in2, LOW);
  } else { // Ré
    digitalWrite(in1, LOW);
    digitalWrite(in2, HIGH);
  }

  // Map the absolute speed value to a PWM duty cycle
  int duty = map(abs(speed), 0, 255, 0, (1 << PWM_RES) - 1);
  ledcWrite(ch, duty);
}

void handleForward() {
  server.sendHeader("Access-Control-Allow-Origin", "*");
  Serial.println("Received PUT request for /forward");
  setMotor(1, 200
  ); // Motor 1 forward
  setMotor(2, 200
  ); // Motor 2 forward
  server.send(200, "text/plain", "Moving forward.");
}

void handleBackward() {
  server.sendHeader("Access-Control-Allow-Origin", "*");
  Serial.println("Received PUT request for /backward");
  setMotor(1, -255); // Motor 1 backward
  setMotor(2, -255); // Motor 2 backward
  server.send(200, "text/plain", "Moving backward.");
}

void handleLeft() {
  server.sendHeader("Access-Control-Allow-Origin", "*");
  Serial.println("Received PUT request for /backward");
  setMotor(1, 255); // Motor 1 backward
  setMotor(2, -255); // Motor 2 backward
  server.send(200, "text/plain", "Moving backward.");
}

void handleRight() {
  server.sendHeader("Access-Control-Allow-Origin", "*");
  Serial.println("Received PUT request for /backward");
  setMotor(1, -255); // Motor 1 backward
  setMotor(2, 255); // Motor 2 backward
  server.send(200, "text/plain", "Moving backward.");
}

void handleStop() {
  server.sendHeader("Access-Control-Allow-Origin", "*");
  Serial.println("Received PUT request for /stop");
  setMotor(1, 0); // Stop motor 1
  setMotor(2, 0); // Stop motor 2
  server.send(200, "text/plain", "Stopping.");
}

void handleNotFound() {
  server.send(404, "text/plain", "Not found here");
}

void handleSpeed() {
  server.sendHeader("Access-Control-Allow-Origin", "*");
  char floatBuffer[10]; // Adjust size based on expected float range and precision
  dtostrf(kmh, 4, 2, floatBuffer); 
  server.send(200, "text/plain", floatBuffer);
}

void setup() {
  // Start serial communication for debugging
  Serial.begin(9600);
  while (!Serial) {
    ; // wait for serial port to connect
  }

  // Configure motor control pins
  pinMode(M1_IN1, OUTPUT);
  pinMode(M1_IN2, OUTPUT);
  pinMode(M2_IN1, OUTPUT);
  pinMode(M2_IN2, OUTPUT);

  // Configure PWM channels
  ledcSetup(CH_M1, PWM_FREQ, PWM_RES);
  ledcAttachPin(M1_EN, CH_M1);
  ledcSetup(CH_M2, PWM_FREQ, PWM_RES);
  ledcAttachPin(M2_EN, CH_M2);

  // Connect to Wi-Fi
  // Serial.print("Connecting to Wi-Fi...");
  // WiFi.begin(ssid, password);
  // while (WiFi.status() != WL_CONNECTED) {
  //   delay(500);
  //   Serial.print(".");
  // }

  // Connect to Wi-Fi
  Serial.print("Setting AP (Access Point)...");
  WiFi.softAP(ssid, password);

  IPAddress IP = WiFi.softAPIP();
  Serial.print("AP IP address: ");
  Serial.println(IP);

  // Define the server's request handlers for specific endpoints and HTTP methods
  server.on("/forward", HTTP_GET, handleForward);
  server.on("/backward", HTTP_GET, handleBackward);
  server.on("/right", HTTP_GET, handleRight);
  server.on("/left", HTTP_GET, handleLeft);
  server.on("/stop", HTTP_GET, handleStop);
  server.on("/speed", HTTP_GET, handleSpeed);

  // Set a handler for any other requests
  // Set a handler for any other requests
  server.onNotFound(handleNotFound);
  // Start the server
  server.begin();
  Serial.println("HTTP server started.");

  pinMode(HALL_RIGHT, INPUT_PULLUP);
  attachInterrupt(digitalPinToInterrupt(HALL_RIGHT), hallISR, FALLING);
}

void loop() {
  server.handleClient();

  unsigned long timeSinceLastPulse;

  // Safely read the shared variables
  noInterrupts();
  unsigned long latestPeriod = periodUs;
  unsigned long lastTime = lastPulseTimeUs;
  interrupts();

  // If a significant amount of time has passed since the last pulse, assume speed is zero.
  timeSinceLastPulse = micros() - lastTime;
  if (timeSinceLastPulse > TIMEOUT_US) {
    kmh = 0.0f;
  } else {
    // If we have a recent pulse, calculate the speed.
    if (latestPeriod > 0) {
      float s = latestPeriod * 1e-3f; // microseconds -> seconds
      kmh = (DIST_PER_PULSE / s) * 3.6f; // m/s -> km/h
    }
  }

  // Serial.println(kmh);
}
