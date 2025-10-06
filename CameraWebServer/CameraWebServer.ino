#include "camera_pins.h"
#include <WiFi.h>
#include "esp_camera.h"
#include <WebSocketsClient.h>
#include <ArduinoJson.h>
using namespace std;
      
const char* ssid = "integrador2";
const char* password = "passwordbigger2";

WebSocketsClient webSocket;


framesize_t convert(const char *str)
{
    if(strcmp(str,"FRAMESIZE_96X96")==0) return FRAMESIZE_96X96;
    else if(strcmp(str,"FRAMESIZE_240X240")==0) return FRAMESIZE_240X240;
    else if(strcmp(str,"FRAMESIZE_QQVGA")==0) return FRAMESIZE_QQVGA;
    else if(strcmp(str,"FRAMESIZE_UXGA")==0) return FRAMESIZE_UXGA;
}

void webSocketEvent(WStype_t type, uint8_t * payload, size_t length) {
  const uint8_t size = JSON_OBJECT_SIZE(5);
  StaticJsonDocument<size> json;
  deserializeJson(json, payload);
  const char *action = json["action"];

  switch(type) {
    case WStype_DISCONNECTED:
      Serial.printf("[WSc] Disconnected!\n");
      break;
    case WStype_CONNECTED: {
      Serial.printf("[WSc] Connected to url: %s\n", payload);
    }
      break;
    case WStype_TEXT:
      Serial.printf("[WSc] get text: %s\n", payload);
      if(strcmp(action,"resolution")==0){
        Serial.printf("Changing resolution\n");
        const char *value = json["value"];
        framesize_t resolution = convert(value);
        sensor_t * s = esp_camera_sensor_get();
        delay(1000);
        s->set_framesize(s,resolution);
        delay(1000);
      }
      break;

}
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
  s->set_framesize(s,FRAMESIZE_240X240);
}

void setup() {
  Serial.begin(115200);

  Serial.println(ssid);
  Serial.println(password);
  WiFi.begin(ssid, password);
  int j =0;
  while (WiFi.status() != WL_CONNECTED) {
    delay(500);
    Serial.println(j);
    j++;
  }
  Serial.println("");
  Serial.println("WiFi connected");

  Serial.print("Camera Ready! Use 'http://");
  Serial.print(WiFi.localIP());
  Serial.println("' to connect");


  webSocket.begin("192.168.4.1", 3000, "/jpgstream_server");
  webSocket.onEvent(webSocketEvent);
  webSocket.setReconnectInterval(5000);
  webSocket.enableHeartbeat(15000, 3000, 2);
  setupCamera();

}


void sendImage(){
int64_t fr_start = esp_timer_get_time();
  webSocket.loop();
  camera_fb_t * fb = NULL;

  // Take Picture with Camera
  fb = esp_camera_fb_get();  
  if(!fb) {
    Serial.println("Camera capture failed");
    return;
  }     
  webSocket.sendBIN(fb->buf,fb->len);
  esp_camera_fb_return(fb); 
  int64_t fr_end = esp_timer_get_time();
  // Serial.printf("Image sent. %ums. FPS: %u\n", (uint32_t)((fr_end - fr_start)/1000),(uint32_t)(1000000/((fr_end - fr_start))));
}


void loop() {
  sendImage();
  webSocket.loop();
}