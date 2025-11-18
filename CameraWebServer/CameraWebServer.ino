#include "camera_pins.h"
#include <WiFi.h>        // <-- From your original list
#include "esp_camera.h"  // <-- From your original list
// We remove WebSocketsClient.h and ArduinoJson.h as they are not usable for this server

const char* ssid = "ESP32-Camera-Stream";
const char* password = "password123";

// Create a WiFiServer object on port 80 (HTTP)
WiFiServer server(80);

// --- Your original setupCamera() function ---
// I've copied it here for completeness
void setupCamera() {
  camera_config_t config;
  config.ledc_channel = LEDC_CHANNEL_0;
  config.ledc_timer = LEDC_TIMER_0;
  config.pin_d0 = Y2_GPIO_NUM;
  config.pin_d1 = Y3_GPIO_NUM;
  config.pin_d2 = Y4_GPIO_NUM;
  config.pin_d3 = Y5_GPIO_NUM;
  config.pin_d4 = Y6_GPIO_NUM;
  config.pin_d5 = Y7_GPIO_NUM;
  config.pin_d6 = Y8_GPIO_NUM;
  config.pin_d7 = Y9_GPIO_NUM;
  config.pin_xclk = XCLK_GPIO_NUM;
  config.pin_pclk = PCLK_GPIO_NUM;
  config.pin_vsync = VSYNC_GPIO_NUM;
  config.pin_href = HREF_GPIO_NUM;
  config.pin_sscb_sda = SIOD_GPIO_NUM;
  config.pin_sscb_scl = SIOC_GPIO_NUM;
  config.pin_pwdn = PWDN_GPIO_NUM;
  config.pin_reset = RESET_GPIO_NUM;
  config.xclk_freq_hz = 20000000;
  config.pixel_format = PIXFORMAT_JPEG;

    config.frame_size = FRAMESIZE_UXGA;
    config.jpeg_quality = 8;
    config.fb_count = 1;
  esp_err_t err = esp_camera_init(&config);
  if (err != ESP_OK) {
    Serial.printf("Camera init failed with error 0x%x", err);
    return;
  }

  sensor_t * s = esp_camera_sensor_get();
  if (s->id.PID == OV3660_PID) {
    s->set_vflip(s, 1); 
    s->set_brightness(s, 1); 
    s->set_saturation(s, -2); 
  }
  // Set initial resolution
  s->set_framesize(s,FRAMESIZE_240X240);
}

// --- setup() ---
// Creates the AP and starts the server
void setup() {
  Serial.begin(115200);

  // Start Camera
  setupCamera();

  // Start Access Point
  Serial.print("Creating AP: ");
  Serial.println(ssid);
  WiFi.softAP(ssid, password);

  Serial.print("AP IP address: ");
  Serial.println(WiFi.softAPIP()); // Default is 192.168.4.1

  // Start the server
  server.begin();
  Serial.println("Server started. MJPEG stream is ready.");
}

// --- loop() ---
// This is now a simple, blocking web server.
void loop() {
  WiFiClient client = server.available(); // Listen for incoming clients

  if (client) { // If a new client connects...
    Serial.println("New Client Connected.");
    String currentLine = "";                // make a String to hold incoming data
    while (client.connected()) {            // loop while the client's connected
      if (client.available()) {             // if there's bytes to read
        char c = client.read();             // read a byte
        if (c == '\n') {                    // if the byte is a newline character
          // if the current line is blank, you got two newline characters in a row.
          // that's the end of the client HTTP request, so send a response:
          if (currentLine.length() == 0) {
            
            // --- This is the MJPEG stream ---
            // Send the HTTP header for a streaming response
            client.println("HTTP/1.1 200 OK");
            client.println("Content-Type: multipart/x-mixed-replace; boundary=--frame");
            client.println(); // End of header

            // Now, loop forever, sending frames
            while (client.connected()) {
              camera_fb_t * fb = NULL;
              fb = esp_camera_fb_get();
              if (!fb) {
                Serial.println("Camera capture failed");
                continue;
              }

              // Send the frame boundary
              client.println("--frame");
              // Send the frame headers
              client.println("Content-Type: image/jpeg");
              client.print("Content-Length: ");
              client.println(fb->len);
              client.println();
              
              // Send the actual image data
              client.write(fb->buf, fb->len);
              
              // Send trailing newline
              client.println(); 

              esp_camera_fb_return(fb);
              
              delay(1); // Small delay to allow other tasks
            }
            // The client has disconnected
            break;
          } else { // if you got a newline, then clear currentLine
            currentLine = "";
          }
        } else if (c != '\r') {  // if you got anything else but a carriage return...
          currentLine += c;      // add it to the end of the currentLine
        }
      }
    }
    // Close the connection
    client.stop();
    Serial.println("Client Disconnected.");
  }
}