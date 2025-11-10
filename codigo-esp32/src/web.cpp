#include "web.h"
#include "drive.h"

const char *ssid = "integrador2";
const char *password = "passwordbigger2";
static WebServer server(80);


void handleForward()
{
  server.sendHeader("Access-Control-Allow-Origin", "*");
  Serial.println("Received PUT request for /forward");
  setMotor(1, -190);
  setMotor(2, -190);
  server.send(200, "text/plain", "Moving forward.");
}

void handleBackward()
{
  server.sendHeader("Access-Control-Allow-Origin", "*");
  Serial.println("Received PUT request for /backward");
  setMotor(1, 190);
  setMotor(2, 190);
  server.send(200, "text/plain", "Moving backward.");
}

void handleLeft()
{
  server.sendHeader("Access-Control-Allow-Origin", "*");
  Serial.println("Received PUT request for /backward");
  setMotor(1, 190);
  setMotor(2, -190);
  server.send(200, "text/plain", "Moving backward.");
}

void handleRight()
{
  server.sendHeader("Access-Control-Allow-Origin", "*");
  Serial.println("Received PUT request for /backward");
  setMotor(1, -190);
  setMotor(2, 190);
  server.send(200, "text/plain", "Moving backward.");
}

void handleStop()
{
  server.sendHeader("Access-Control-Allow-Origin", "*");
  Serial.println("Received PUT request for /stop");
  setMotor(1, 0);
  setMotor(2, 0);
  server.send(200, "text/plain", "Stopping.");
}

void handleNotFound()
{
  server.send(404, "text/plain", "Not found here");
}

void handleSpeed()
{
  server.sendHeader("Access-Control-Allow-Origin", "*");
  char floatBuffer[10];
  dtostrf(0, 4, 2, floatBuffer);
  server.send(200, "text/plain", floatBuffer);
}


void web_begin(){
    Serial.print("Setting AP (Access Point)...");
    WiFi.softAP(ssid, password);

    IPAddress IP = WiFi.softAPIP();
    Serial.print("AP IP address: ");
    Serial.println(IP);

    // Define the server's request handlers for specific endpoints and HTTP methods
    server.on("/speed", HTTP_GET, handleSpeed);

    // Set a handler for any other requests
    // Set a handler for any other requests
    server.onNotFound(handleNotFound);
    // Start the server
    server.begin();
    Serial.println("HTTP server started.");
}

