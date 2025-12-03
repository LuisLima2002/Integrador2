#include "camera_pins.h"
#include <WiFi.h>
#include "esp_camera.h"

// --- ENCODER CONFIGURATION ---
#define ENCODER_PIN 13       
#define MEASURE_INTERVAL 1000 
 
// --- WIFI CONFIGURATION ---
const char* ssid = "ESP32-Camera-Stream";
const char* password = "password123";

// --- GLOBAL VARIABLES ---
WiFiServer server(80);
volatile unsigned long pulseCount = 0;
unsigned long lastMeasureTime = 0;
float kmh = 0;

// --- HTML WEBPAGE ---
const char index_html[] PROGMEM = R"rawliteral(
<!DOCTYPE html>
<html>
<head>
  <meta name="viewport" content="width=device-width, initial-scale=1">
  <title>ESP32 CAM Robot</title>
  <style>
    body { font-family: Arial; text-align: center; margin: 0px; padding: 0px; background-color: #222; color: white; }
    h1 { margin-top: 30px; }
    #container { margin-top: 20px; }
    
    /* MODIFIED: Added transform property to flip the image */
    img { 
      width: 90%; 
      max-width: 640px; 
      height: auto; 
      border: 4px solid #444; 
      border-radius: 8px;
      transform: rotate(180deg); /* Rotates image 180 degrees */
    }

    .info { margin-top: 20px; color: #fff; font-size: 24px; font-weight: bold;}
    .unit { font-size: 16px; color: #aaa; }
    .btn { background-color: #d32f2f; border: none; color: white; padding: 10px 20px; text-decoration: none; display: inline-block; margin: 4px 2px; cursor: pointer; border-radius: 4px;}
  </style>
</head>
<body>
  <h1>ESP32 Robot Stream</h1>
  <div id="container">
    <img src="/stream" id="video_stream">
  </div>
  
  <p class="info">Speed: <span id="speedVal">0.0</span> <span class="unit">Kmh</span></p>
  
  <p><button class="btn" onclick="location.reload()">Refresh Stream</button></p>

  <script>
    // Updated comment to match the actual interval (1000ms)
    // This script asks the ESP32 for the speed every 1000ms
    setInterval(function() {
      fetch('/speed')
        .then(response => response.text())
        .then(data => {
          document.getElementById('speedVal').innerText = data;
        })
        .catch(error => console.log(error));
    }, 1000); 
  </script>
</body>
</html>
)rawliteral";

// --- INTERRUPT SERVICE ROUTINE ---
void IRAM_ATTR readEncoder() {
  pulseCount++;
}

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
  config.jpeg_quality = 12; 
  config.fb_count = 1;

  esp_err_t err = esp_camera_init(&config);
  if (err != ESP_OK) {
    Serial.printf("Camera init failed with error 0x%x", err);
    return;
  }
  
  sensor_t * s = esp_camera_sensor_get();
  
  // FIX: Force Flip for all sensors (fixes upside down issue)
  s->set_vflip(s, 1);   // Flip Vertical
  s->set_hmirror(s, 0); // Disable Mirror (unless needed)
  
  // Specific settings for OV3660
  if (s->id.PID == OV3660_PID) {
    s->set_brightness(s, 1);
    s->set_saturation(s, -2);
  }
  
  s->set_framesize(s, FRAMESIZE_240X240);
}

void setup() {
  Serial.begin(115200);

  // Setup Encoder
  pinMode(ENCODER_PIN, INPUT_PULLUP);
  attachInterrupt(digitalPinToInterrupt(ENCODER_PIN), readEncoder, RISING);

  // Setup Camera
  setupCamera();

  // Setup WiFi AP
  WiFi.softAP(ssid, password);
  Serial.print("AP IP: ");
  Serial.println(WiFi.softAPIP());

  server.begin();
}

void loop() {
  // Check for new clients
  WiFiClient client = server.available();

  if (client) {
    String currentLine = "";
    String requestLine = ""; 
    boolean isFirstLine = true;
    
    // Read the request headers
    while (client.connected()) {
      if (client.available()) {
        char c = client.read();
        
        if (isFirstLine) {
          if (c == '\n') isFirstLine = false;
          else if (c != '\r') requestLine += c;
        }

        if (c == '\n') {
          if (currentLine.length() == 0) {
            
            // --- ROUTING LOGIC ---
            
            // 1. DATA ROUTE: Returns just the speed number
            if (requestLine.indexOf("GET /speed") >= 0) {
              client.println("HTTP/1.1 200 OK");
              client.println("Content-Type: text/plain");
              client.println("Connection: close");
              client.println("Access-Control-Allow-Origin: *");
              client.println();
              client.print(kmh);
              client.stop(); // Close immediately
            }
            
            // 2. STREAM ROUTE: MJPEG Stream
            else if (requestLine.indexOf("GET /stream") >= 0) {
              
              client.println("HTTP/1.1 200 OK");
              client.println("Content-Type: multipart/x-mixed-replace; boundary=--frame");
              client.println();

              // MJPEG Loop
              while (client.connected()) {
                
                // --- MULTI-CLIENT HACK ---
                // Inside this video loop, we quickly check if the browser 
                // is asking for the speed on a DIFFERENT connection.
                WiFiClient speedClient = server.available();
                if (speedClient) {
                   // Clean buffer
                   while(speedClient.available()) speedClient.read();
                   // Send response
                   speedClient.println("HTTP/1.1 200 OK");
                   speedClient.println("Content-Type: text/plain");
                   speedClient.println("Connection: close");
                   speedClient.println("Access-Control-Allow-Origin: *");
                   speedClient.println();
                   speedClient.print(kmh);
                   speedClient.stop();
                }
                // -------------------------

                // Calculate Speed
                unsigned long currentMillis = millis();
                if (currentMillis - lastMeasureTime >= MEASURE_INTERVAL) {
                  noInterrupts();
                  unsigned long tempCount = pulseCount;
                  pulseCount = 0;
                  interrupts();
                  float window_s = MEASURE_INTERVAL / 1000.0f;
                  float rpm = (tempCount / (float)20) * (60.0f / window_s);

                  kmh = 10 * rpm * 60.0f * 2.0f * 3.1416f * 3e-5f;

                }

                camera_fb_t * fb = esp_camera_fb_get();
                if (!fb) {
                  Serial.println("Cam fail");
                  continue;
                }

                client.println("--frame");
                client.println("Content-Type: image/jpeg");
                client.print("Content-Length: ");
                client.println(fb->len);
                client.println();
                
                client.write(fb->buf, fb->len);
                client.println();
                
                esp_camera_fb_return(fb);
              }
            } 
            
            // 3. DEFAULT ROUTE: The HTML Page
            else {
              client.println("HTTP/1.1 200 OK");
              client.println("Content-Type: text/html");
              client.println("Connection: close");
              client.println();
              client.println(index_html);
              client.stop();
            }
            break;
            
          } else {
            currentLine = "";
          }
        } else if (c != '\r') {
          currentLine += c;
        }
      }
    }
  }
}