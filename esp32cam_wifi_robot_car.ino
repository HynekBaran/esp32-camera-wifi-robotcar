// esp32cam_wifi_robot_car_l293d
// https://www.instructables.com/DIY-ESP32-Camera-Motor-Shield-Wifi-Camera-Robot-Ca/
// https://www.bluino.com/2021/01/membuat-robot-mobil-rc-kamera.html
// https://play.google.com/store/apps/details?id=com.bluino.esp32camerawifirobotcar&hl=cs&gl=US

// Board: ESP32 / AI_THINKER ESP32-CAM

// Works also with generic ESP32 module (without video)

// OTA: 
// Partition scheme Minimal SPIFFS (Large APPS with OTA)
// add the code bellow to the file 
// boards.local.txt
// in the same dir as boards.txt, probably ~/Library/Arduino15/packages/esp32/hardware/esp32/1.0.6/boards.local.txt
// (follow More preferences... link in Arduino Preferences/Settings window to find proper location)
//    esp32cam.menu.PartitionScheme.huge_app=Huge APP (3MB No OTA/1MB SPIFFS)
//    esp32cam.menu.PartitionScheme.huge_app.build.partitions=huge_app
//    esp32cam.menu.PartitionScheme.huge_app.upload.maximum_size=3145728
//    esp32cam.menu.PartitionScheme.min_spiffs=Minimal SPIFFS (Large APPS with OTA)
//    esp32cam.menu.PartitionScheme.min_spiffs.build.partitions=min_spiffs
//    esp32cam.menu.PartitionScheme.min_spiffs.upload.maximum_size=1966080
// or edit boards.txt
// esp32cam.upload.maximum_size=1966080
// esp32cam.build.partitions=min_spiffs

#include "esp_camera.h"
#include <WiFi.h>
#include <ArduinoOTA.h>

/* Wifi Crdentials */
String sta_ssid = "FatManD";     // set Wifi network you want to connect to
String sta_password = "kakadubkorela";   // set password for Wifi network

/* define CAMERA_MODEL_AI_THINKER */
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

/* Defining motor and servo pins */
extern int MOTOR_L_F = 12; // left motor Forward
extern int MOTOR_L_B = 13; // left motor Backward
extern int MOTOR_R_B = 14; // right motor Backward
extern int MOTOR_R_F = 15; // right motor Forward


extern int ledVal = 20;  // setting bright of flash LED 0-255

extern int ledPin = 4;  // set digital pin GPIO4 as LED pin (use biult-in LED)
extern int buzzerPin = 2;  // set digital pin GPIO2 as LED pin (use Active Buzzer)
extern int servoPin = 2;  // set digital pin GPIO2 as servo pin (use SG90)

unsigned long previousMillis = 0;

void startCameraServer();

void initServo() {
  ledcSetup(8, 50, 16); /*50 hz PWM, 16-bit resolution and range from 3250 to 6500 */
  ledcAttachPin(servoPin, 8);
}

void initLed() {
  ledcSetup(7, 5000, 8); /* 5000 hz PWM, 8-bit resolution and range from 0 to 255 */
  ledcAttachPin(ledPin, 7);
}

void setup() {
  Serial.begin(115200);         // set up seriamonitor at 115200 bps
  Serial.setDebugOutput(true);
  Serial.println();
  Serial.println("*ESP32 Camera Remote Control - L293D Bluino Shield*");
  Serial.println("--------------------------------------------------------");

  // Set all the motor control pin to Output
  pinMode(MOTOR_L_F, OUTPUT);
  pinMode(MOTOR_L_B, OUTPUT);
  pinMode(MOTOR_R_B, OUTPUT);
  pinMode(MOTOR_R_F, OUTPUT);

  pinMode(ledPin, OUTPUT); // set the LED pin as an Output
  pinMode(buzzerPin, OUTPUT); // set the buzzer pin as an Output
  pinMode(servoPin, OUTPUT); // set the servo pin as an Output

  // Initial state - turn off motors, LED & buzzer
  digitalWrite(MOTOR_L_F, LOW);
  digitalWrite(MOTOR_L_B, LOW);
  digitalWrite(MOTOR_R_B, LOW);
  digitalWrite(MOTOR_R_F, LOW);
  digitalWrite(ledPin, LOW);
  digitalWrite(buzzerPin, LOW);
  digitalWrite(servoPin, LOW);

  /* Initializing Servo and LED */
  initServo();
  initLed();

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
  //init with high specs to pre-allocate larger buffers
  if (psramFound()) {
    config.frame_size = FRAMESIZE_UXGA;
    config.jpeg_quality = 10;
    config.fb_count = 2;
  } else {
    config.frame_size = FRAMESIZE_SVGA;
    config.jpeg_quality = 12;
    config.fb_count = 1;
  }

  // camera init
  esp_err_t cam_err = esp_camera_init(&config);
  if (cam_err != ESP_OK) {
    // camera not available, lets continue without it...
    Serial.printf("esp_camera init failed with error 0x%x.\n", cam_err);
  } else {
    // drop down frame size for higher initial frame rate
    sensor_t * s = esp_camera_sensor_get();
    s->set_framesize(s, FRAMESIZE_QVGA);
  }

  // Set NodeMCU Wifi hostname based on chip mac address
  char chip_id[15];
  snprintf(chip_id, 15, "%04X", (uint16_t)(ESP.getEfuseMac() >> 32));
  String hostname = "esp32cam-" + String(chip_id);

  Serial.println();
  Serial.println("Hostname: " + hostname);

  // first, set NodeMCU as STA mode to connect with a Wifi network
  WiFi.mode(WIFI_STA);
  WiFi.begin(sta_ssid.c_str(), sta_password.c_str());
  Serial.println("");
  Serial.print("Connecting to: ");
  Serial.println(sta_ssid);
  Serial.print("Password: ");
  Serial.println(sta_password);

  // try to connect with Wifi network about 10 seconds
  unsigned long currentMillis = millis();
  previousMillis = currentMillis;
  while (WiFi.status() != WL_CONNECTED && currentMillis - previousMillis <= 10000) {
    delay(500);
    Serial.print(".");
    currentMillis = millis();
  }

  // if failed to connect with Wifi network set NodeMCU as AP mode
  IPAddress myIP;
  if (WiFi.status() == WL_CONNECTED) {
    Serial.println("");
    Serial.println("*WiFi-STA-Mode*");
    Serial.print("IP: ");
    myIP = WiFi.localIP();
    Serial.println(myIP);
    delay(2000);
  } else {
    WiFi.mode(WIFI_AP);
    WiFi.softAP(hostname.c_str());
    myIP = WiFi.softAPIP();
    Serial.println("");
    Serial.println("WiFi failed connected to " + sta_ssid);
    Serial.println("");
    Serial.println("*WiFi-AP-Mode*");
    Serial.print("AP IP address: ");
    Serial.println(myIP);
    delay(2000);
  }

  // Start camera web server to get realtime view and react to commands
  startCameraServer();
  Serial.print("Camera Ready! Use 'http://");
  Serial.print(myIP);
  Serial.println("' to connect ");

  // OTA
  ArduinoOTA
    .onStart([]() {
      String type;
      if (ArduinoOTA.getCommand() == U_FLASH)
        type = "sketch";
      else // U_SPIFFS
        type = "filesystem";

      // NOTE: if updating SPIFFS this would be the place to unmount SPIFFS using SPIFFS.end()
      Serial.println("Start updating " + type);
    })
    .onEnd([]() {
      Serial.println("\nEnd");
    })
    .onProgress([](unsigned int progress, unsigned int total) {
      Serial.printf("Progress: %u%%\r", (progress / (total / 100)));
    })
    .onError([](ota_error_t error) {
      Serial.printf("Error[%u]: ", error);
      if (error == OTA_AUTH_ERROR) Serial.println("Auth Failed");
      else if (error == OTA_BEGIN_ERROR) Serial.println("Begin Failed");
      else if (error == OTA_CONNECT_ERROR) Serial.println("Connect Failed");
      else if (error == OTA_RECEIVE_ERROR) Serial.println("Receive Failed");
      else if (error == OTA_END_ERROR) Serial.println("End Failed");
    });
  ArduinoOTA.begin();   // enable to receive update/upload firmware via Wifi OTA
}

void loop() {
  // put your main code here, to run repeatedly:
  ArduinoOTA.handle();
}
