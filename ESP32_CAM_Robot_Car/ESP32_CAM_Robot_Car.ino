/*
  ESP32_CAM_Drilling_Machine
  app_httpd.cpp
  Uses TB6600 Motor Driver for Stepper Motor
  Uses BTS7960 Motor Driver for DC Motor
  
  
*/

#include "esp_wifi.h"
#include "esp_camera.h"
#include <WiFi.h>
#include "soc/soc.h"
#include "soc/rtc_cntl_reg.h"
#include <AccelStepper.h>

// Setup Access Point Credentials
const char* ssid1 = "ESP32-CAM Drill";
const char* password1 = "VerticalDrillingMachine123";
int counter;
int set1, set2 = 0;

#define DIR 13
#define PUL 14

extern volatile unsigned int  motor_speed;



extern void drill_stop();
extern void drill_setup();
extern uint8_t up;
extern uint8_t down;
extern uint8_t drill;
extern uint8_t standby;
extern uint8_t stop_drill;
extern volatile unsigned long previous_time;        
extern volatile unsigned long move_interval; 



#define CAMERA_MODEL_AI_THINKER
#define PWDN_GPIO_NUM     32
#define RESET_GPIO_NUM    -1
#define XCLK_GPIO_NUM      0
#define SIOD_GPIO_NUM     26
#define SIOC_GPIO_NUM     27

#define Y9_GPIO_NUM       35
#define Y8_GPIO_NUM       34
#define Y7_GPIO_NUM       39
#define Y6_GPIO_NUM       36
#define Y5_GPIO_NUM       21
#define Y4_GPIO_NUM       19
#define Y3_GPIO_NUM       18
#define Y2_GPIO_NUM        5
#define VSYNC_GPIO_NUM    25
#define HREF_GPIO_NUM     23
#define PCLK_GPIO_NUM     22

void startCameraServer();

void setup() 
{
  
  
  WRITE_PERI_REG(RTC_CNTL_BROWN_OUT_REG, 0); // Prevent the brownout error by silencing it
  
  Serial.begin(115200);
  Serial.setDebugOutput(true);
  Serial.println();

  pinMode(PUL, OUTPUT);
  pinMode(DIR, OUTPUT);

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
  // We initialize with high specs in order to pre-allocate larger buffers 
  if(psramFound()){
    config.frame_size = FRAMESIZE_QVGA;
    config.jpeg_quality = 10;
    config.fb_count = 2;
  } else {
    config.frame_size = FRAMESIZE_QVGA;
    config.jpeg_quality = 12;
    config.fb_count = 1;
  }

  // camera init
  esp_err_t err = esp_camera_init(&config);
  if (err != ESP_OK) {
    Serial.printf("Camera init failed with error 0x%x", err);
    return;
  }

  // Drop down frame size for higher initial frame rate
  sensor_t * s = esp_camera_sensor_get();
  s->set_framesize(s, FRAMESIZE_QVGA);
  s->set_vflip(s, 1);
  s->set_hmirror(s, 1);

  WiFi.softAP(ssid1, password1);
  IPAddress myIP = WiFi.softAPIP();
  Serial.print("AP IP address: ");
  Serial.println(myIP);
  
  startCameraServer();

  ledcSetup(7, 5000, 8);
  ledcAttachPin(4, 7);  // Pin 4 is the LED
  drill_setup();
  
  for (int i=0;i<5;i++) 
  {
    ledcWrite(7,10);  // The LED is flashed 5 times at startup
    delay(50);
    ledcWrite(7,0);
    delay(50);    
  }
  digitalWrite(33,LOW);
      
  previous_time = millis();
}

void loop() {
  if(up)
  
  { digitalWrite(DIR, HIGH);
  for (int i = 0; i < 5 * 700; i++) {
    // These four lines result in 1 step:
    digitalWrite(PUL, HIGH);
    delayMicroseconds(50);
    digitalWrite(PUL, LOW);
    delayMicroseconds(50);
  }
  
   // Daca adaugi up = 0 dupa loop, va face o rotatie rapida si se va opri.

  }
  if(down)
  
  { digitalWrite(DIR, LOW);
    for (int i = 0; i < 5 * 1000; i++) {
    // These four lines result in 1 step:
    digitalWrite(PUL, LOW);
    delayMicroseconds(50);
    digitalWrite(PUL, HIGH);
    delayMicroseconds(50);
  }

      // Daca adaugi up = 0 dupa loop, va face o rotatie rapida si se va opri.

  }
  if (standby)
  {
    up = 0;
    down = 0;
    standby = 0;
  }
  /*if(counter !=0)
  {
    digitalWrite(STEP, HIGH);
    delayMicroseconds(50);
    digitalWrite(STEP, LOW);
    delayMicroseconds(50);
*/
  
  if(drill)
  {
    /*unsigned long currentMillis = millis();
    if (currentMillis - previous_time >= move_interval) {
      previous_time = currentMillis;
      drill_stop();
      char rsp[32];
      sprintf(rsp,"SPPED: %d",motor_speed);
      Serial.println("Stop");
      drill=0;
    }*/
  }
  delay(1);
  yield();
}