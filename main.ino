// Blynk
#define BLYNK_PRINT Serial
#include <BlynkSimpleEsp32.h>

// Blynk define for project
#define BLYNK_TEMPLATE_ID "---------------"
#define BLYNK_DEVICE_NAME "---------------"
#define BLYNK_AUTH_TOKEN "---------------"

// Line notify
#include <TridentTD_LineNotify.h>

// WiFi
#include <WiFi.h>
#include <WiFiClient.h>

// ESP
#include "esp_camera.h"
#include "esp_system.h"

// Timer
hw_timer_t *timer = NULL;

// Reset module function
void IRAM_ATTR resetModule() {
  ets_printf("reboot\n");
  esp_restart();
}

// Define connection
#define SSID        "---------------"   // WiFi name
#define PASSWORD    "---------------"   // PASSWORD
#define LINE_TOKEN  "---------------" // LINE TOKEN

// Pin definition for CAMERA_MODEL_AI_THINKER
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

// PIN define
int Led_Flash = 4;
int BuzzerPin = 13;
int PIRPin = 12;

// On-Off value
int onOff = 0;

// Timer
boolean startTimer = false;
unsigned long time_now = 0;
int time_capture = 0;

// Blynk read-write virtual pin 0
BLYNK_WRITE(V0)
{
  onOff = param.asInt();
}

void setup() {

  Serial.begin(115200);
  while (!Serial) {
    ;
  }
  // Setup pinMode
  pinMode(Led_Flash, OUTPUT);
  pinMode(BuzzerPin, OUTPUT);
  pinMode(PIRPin, INPUT);
  
  //  WiFi.begin(SSID, PASSWORD);
  //  Serial.printf("WiFi connecting to %s\n",  SSID);
  //  while (WiFi.status() != WL_CONNECTED) {
  //    Serial.print(".");
  //    delay(400);
  //  }
  //  Serial.printf("\nWiFi connected\nIP : ");
  //  Serial.println(WiFi.localIP());
  
  // Set Line token
  LINE.setToken(LINE_TOKEN);
  // Connect to blynk server
  Blynk.begin(BLYNK_AUTH_TOKEN, SSID , PASSWORD, "blynk.cloud", 80);

  // Setup timer
  timer = timerBegin(0, 80, true); //timer 0, div 80Mhz
  timerAttachInterrupt(timer, &resetModule, true);
  timerAlarmWrite(timer, 20000000, false); //set time in us 15s
  timerAlarmEnable(timer); //enable interrupt

  // Camera config
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

  if (psramFound()) {
    config.frame_size = FRAMESIZE_VGA;
    config.jpeg_quality = 10;
    config.fb_count = 2;
    config.grab_mode = CAMERA_GRAB_LATEST;
  }     else {
    config.frame_size = FRAMESIZE_QQVGA;
    config.jpeg_quality = 12;
    config.fb_count = 1;
    config.grab_mode = CAMERA_GRAB_LATEST;
  }

  // Init Camera
  esp_err_t err = esp_camera_init(&config);
  if (err != ESP_OK) {
    Serial.printf("Camera init failed with error 0x%x", err);
    return;
  }
}
void loop() {
  // Run Blynk
  Blynk.run();
  timerWrite(timer, 0);
//  long tme = millis();
  // Check if On and PIR detected
  if (digitalRead(PIRPin) == 1 && startTimer != true && onOff == 1) {
    digitalWrite(BuzzerPin, HIGH);
    Camera_capture();
    Serial.println("OK");
    startTimer = true;
  // Check if On but PIR not detected
  } else if (digitalRead(PIRPin) == 0 && onOff == 1) {
    startTimer = false;
    time_capture = 0;
  }

  // Buzzer write LOW when PIR not detected
  if (digitalRead(PIRPin) == 0)
  {
    digitalWrite(BuzzerPin, LOW);
  }
  delay(500);
}

// Camera capture function
void Camera_capture() {
  // Flash
  digitalWrite(Led_Flash, HIGH);
  delay(100);
  digitalWrite(Led_Flash, LOW);
  delay(100);
  digitalWrite(Led_Flash, HIGH);

  // Take Picture with Camera
  camera_fb_t * fb = esp_camera_fb_get();
  if (!fb) {
    Serial.println("Camera capture failed");
    return;
  }
  digitalWrite(Led_Flash, LOW);
  // Send picture to Line
  Send_line(fb->buf, fb->len);
  esp_camera_fb_return(fb);
}

// Send picture to Line function
void Send_line(uint8_t *image_data, size_t   image_size) {
  LINE.notifyPicture("แจ้งเตือนจาก ESP32 Cam", image_data, image_size);
}
