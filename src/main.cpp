/***********************************************************************************
 * 
 * ESP32-CAM
 * 
 * Sketch to test different sensors connected to the ESP32-Cam.
 * - Light Sensor (I2C)                 : TSL2561
 * - PIR Sensor (binary)                : AM312
 * - Temperature (Wire)                 : DS18B20
 * - Switch                             : e.g. limit or reed switch 
 * 
 * Also hosts a web service to view the live camera stream from e.g. a browser.
 * 
 * Fritzing:
 *   ESP32Cam_GateMonitor (bottom).fzz
 * 
 * Issues:
 * - Remap I2C pins, as default I2C GPIO's are used by camera.
 * - Use different interrupt creation commands (in setup).
 * - Change "" struct name in Adafruit libraries, as struct with same name also exists in esp camera lib.
 * - Sequence of library inludes (camera before Arduino) helps to point to Arduino libs to fix duplicate struct name. 
 * - Sequence of initialization in setup() seems important.
 *
***********************************************************************************/
#include <Arduino.h>
#include <WiFi.h>
#include <soc/soc.h>
#include <soc/rtc_cntl_reg.h>
#include <esp_camera.h>
#include <esp_timer.h>
#include <img_converters.h>
#include <fb_gfx.h>
#include <esp_http_server.h>
#include <Adafruit_Sensor.h>
#include <Adafruit_TSL2561_U.h>
#include <OneWire.h>
#include <DallasTemperature.h>
#include "configuration.h"

Adafruit_TSL2561_Unified sensorLux = Adafruit_TSL2561_Unified(TSL2561_ADDR_FLOAT, 12345);
OneWire wireBus(PIN_TEMP);
DallasTemperature sensorTemp(&wireBus);
WiFiClient wifiClient;
httpd_handle_t stream_httpd = NULL;

volatile bool motionDetected = false;                   // Motion detect flag (true = motion detected)
volatile bool switchChangedState = false;               // Switch state change flag (true if changed)
volatile unsigned long lastSwitchChange = 0;            // Last timestamp when switch changed status
volatile unsigned long lastMovementDetected = 0;        // Last timestamp when movement was detected

portMUX_TYPE switchMux = portMUX_INITIALIZER_UNLOCKED;
portMUX_TYPE pirMux = portMUX_INITIALIZER_UNLOCKED;


/**************************************************************************
 * BlinkLED
 * - blink the onboard LED.
 **************************************************************************/
void BlinkLED (int LoopCnt) {
  for (int i=0;i<LoopCnt;i++) {
    // Blink LED enabled, so do it.
    digitalWrite(PIN_BOARD_LED, LOW);
    delay(60);
    digitalWrite(PIN_BOARD_LED, HIGH);
    if (i<LoopCnt-1) delay(100);
  }
}

/**************************************************************************
 * GetLuxReading
 * - Get current light value from Lux sensor
 **************************************************************************/
void GetLuxReading() {
  sensors_event_t event;

  sensorLux.getEvent(&event);
  if (event.light) {
    int valueLux =  event.light;
    Serial.print (" - Illuminence: "); Serial.println( valueLux);
  }
  else {
    // If event.light = 0 lux the sensor is probably saturated and no reliable data could be generated! 
    Serial.println(">> Lux sensor overload!");
  }
}

/**************************************************************************
 * configureLuxSensor
 * - Set Gain and Integration Time of Lux sensor
 **************************************************************************/
void configureLuxSensor()
{
  sensorLux.setGain(TSL2561_GAIN_16X);                              // 16x gain ... use in low light to boost sensitivity 
  //sensorLux.setGain(TSL2561_GAIN_1X);                             // No gain ... use in bright light to avoid sensor saturation 
  sensorLux.enableAutoRange(true);                                  // Auto-gain ... switches automatically between 1x and 16x 

  // Changing the integration time gives better sensor resolution (402ms = 16-bit data) 
  //sensorLux.setIntegrationTime(TSL2561_INTEGRATIONTIME_402MS);    // 16-bit data but slowest conversions 
  sensorLux.setIntegrationTime(TSL2561_INTEGRATIONTIME_101MS);      // medium resolution and speed 
  //sensorLux.setIntegrationTime(TSL2561_INTEGRATIONTIME_13MS);     // fast but low resolution 

  Serial.print  ("- Lux sensor info: "); sensorLux.printSensorDetails();
  Serial.println("");
}

/**************************************************************************
 * isrSwitchChange
 * - interrupt routine called when the switch is opened/closed.
 **************************************************************************/
static void IRAM_ATTR isrSwitchChange (void *arg) {
  portENTER_CRITICAL_ISR(&switchMux);
  if ( millis() - lastSwitchChange > debounceSwitch ) {
    switchChangedState = true;
    lastSwitchChange = millis();                  // Keep FIRST change as debounce time reference
  }
  //lastSwitchChange = millis();                  // Keep LAST change as debounce time reference
  portEXIT_CRITICAL(&switchMux); 
}

/**************************************************************************
 * isrDetectMovement
 * - interrupt routine called when the PIR detected movement.
 **************************************************************************/
static void IRAM_ATTR isrDetectMovement(void *arg) {
  //Serial.println("MOTION DETECTED!!!");
  portENTER_CRITICAL_ISR(&pirMux);
  if ( millis() - lastMovementDetected > debounceMovement ) {
    motionDetected = true;
    lastMovementDetected = millis();
  portEXIT_CRITICAL(&pirMux); 
  }
}

/**************************************************************************
 * stream_handler ~
 **************************************************************************/
static esp_err_t stream_handler(httpd_req_t *req) {
  camera_fb_t * fb = NULL;
  esp_err_t res = ESP_OK;
  size_t _jpg_buf_len = 0;
  uint8_t * _jpg_buf = NULL;
  char * part_buf[64];

  res = httpd_resp_set_type(req, _STREAM_CONTENT_TYPE);
  if (res != ESP_OK) {
    Serial.print(">>> http stream handler error - "); Serial.println(res);
    return res;
  }

  Serial.println("StreamHandler started");

  while (true) {
    fb = esp_camera_fb_get();
    if (!fb) {
      Serial.println(">>> Camera capture failed");
      res = ESP_FAIL;
    } else {
      if(fb->width > 400){
        if(fb->format != PIXFORMAT_JPEG){
          bool jpeg_converted = frame2jpg(fb, 80, &_jpg_buf, &_jpg_buf_len);
          esp_camera_fb_return(fb);
          fb = NULL;
          if(!jpeg_converted){
            Serial.println(">>> JPEG compression failed");
            res = ESP_FAIL;
          }
        } else {
          _jpg_buf_len = fb->len;
          _jpg_buf = fb->buf;
        }
      }
    }
    if(res == ESP_OK){
      size_t hlen = snprintf((char *)part_buf, 64, _STREAM_PART, _jpg_buf_len);
      res = httpd_resp_send_chunk(req, (const char *)part_buf, hlen);
    }
    if(res == ESP_OK){
      res = httpd_resp_send_chunk(req, (const char *)_jpg_buf, _jpg_buf_len);
    }
    if(res == ESP_OK){
      res = httpd_resp_send_chunk(req, _STREAM_BOUNDARY, strlen(_STREAM_BOUNDARY));
    }
    if(fb){
      esp_camera_fb_return(fb);
      fb = NULL;
      _jpg_buf = NULL;
    } else if(_jpg_buf){
      free(_jpg_buf);
      _jpg_buf = NULL;
    }
    if(res != ESP_OK){
//      Serial.print(">>> StreamHandler error - "); Serial.println(res);
      break;
    }
    //Serial.printf("MJPG: %uB\n",(uint32_t)(_jpg_buf_len));
  }

  Serial.println("StreamHandler stopped");
  return res;
}

/**************************************************************************
 * cam_StartServer ~
***************************************************************************/
void cam_StartServer() {
  httpd_config_t config = HTTPD_DEFAULT_CONFIG();
  config.server_port = 80;

  httpd_uri_t index_uri = {
    .uri       = "/",
    .method    = HTTP_GET,
    .handler   = stream_handler,
    .user_ctx  = NULL
  };
  
  if (httpd_start(&stream_httpd, &config) == ESP_OK) {
    httpd_register_uri_handler(stream_httpd, &index_uri);
    Serial.printf("Started web server on port: '%d'\n", config.server_port);
  } else {
    Serial.printf(">>> Web server start failed");
  }
}

/**************************************************************************
 * cam_Setup ~
***************************************************************************/
void cam_Setup() {
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
  
  if ( psramFound() ) {
    config.frame_size = FRAMESIZE_UXGA;       // FRAMESIZE_ + QVGA|CIF|VGA|SVGA|XGA|SXGA|UXGA
    config.jpeg_quality = 10;
    config.fb_count = 2;
  } else {
    config.frame_size = FRAMESIZE_SVGA;
    config.jpeg_quality = 12;
    config.fb_count = 1;
  }
  
  // Camera init
  esp_err_t err = esp_camera_init(&config);
  if (err != ESP_OK) {
    Serial.printf(">>> Camera init failed with error 0x%x", err);
  }

}

/**************************************************************************
 * setup
***************************************************************************/
void setup() {
  WRITE_PERI_REG(RTC_CNTL_BROWN_OUT_REG, 0);      // Prevent brownout
  esp_err_t res = 0;

  Serial.begin(115200);
  Serial.setDebugOutput(false);
  Serial.println("\n--- SETUP STARTED ---\n");

  pinMode(PIN_SWITCH, INPUT_PULLUP);
  pinMode(PIN_FLASH_LED, OUTPUT);
  pinMode(PIN_BOARD_LED, OUTPUT);
  digitalWrite(PIN_FLASH_LED, LOW);

  cam_Setup();

  // Wi-Fi connection
  Serial.print("Connecting WiFi to: "); Serial.println(ssid);
  WiFi.begin(ssid, password);
  while (WiFi.status() != WL_CONNECTED) {
    delay(500);
    Serial.print(".");
  }
  if ( WiFi.isConnected() ) {
    Serial.print("\nWiFi connected.\n - RSSI: "); Serial.print(WiFi.RSSI()); Serial.print(", Local IP: "); Serial.println(WiFi.localIP() ); 
  } else {
    Serial.print("\n>>> WiFi connection failed.");
  }


  // Start streaming web server
  cam_StartServer();


  // Initiate and set the Lux light sensor.
  Wire.setPins(I2C_SDA, I2C_SCL);               // Map SDA and SCL to new pins
  if ( sensorLux.begin() ) {
    configureLuxSensor();
    Serial.println("Setup: TSL2561 initiated and configured.");
  } else {
    Serial.println(">>> Setup: TSL2561 initialization failed!");
  }


  // Set up the OneWire bus with the Temperature sensor
  sensorTemp.begin();
  Serial.println("Setup: Temperature sensor initiated.");


  // PIR Motion Sensor interrupt (rising edge).
  res = gpio_isr_handler_add( (gpio_num_t) pinPIR, &isrDetectMovement, (void *) pinPIR);  
  if (res != ESP_OK) {
    Serial.printf(">>> Setup: PIR interrupt handler add failed (err=0x%x) \r\n", res); 
  }
  res = gpio_set_intr_type( (gpio_num_t)pinPIR, GPIO_INTR_POSEDGE);
  if (res != ESP_OK) {
    Serial.printf(">>> Setup: PIR set interrupt type failed (err=0x%x) \r\n", res);
  }

  // Switch sensor interrupt (both edges - rising/opening and falling/closing)
  res = gpio_isr_handler_add( (gpio_num_t)pinSwitch, &isrSwitchChange, (void *) pinSwitch);
  if (res != ESP_OK) {
    Serial.printf(">>> Switch handler add failed with error 0x%x \r\n", res);
  }
  res = gpio_set_intr_type((gpio_num_t)pinSwitch, GPIO_INTR_ANYEDGE);
  if (res != ESP_OK) {
    Serial.printf(">>> Switch set intr type failed with error 0x%x \r\n", res);
  }
  Serial.println("Setup: Interrupts configured.");


  // Show some board info
  Serial.println("ESP32-Cam Detail ------------------");
  esp_chip_info_t espInfo;
  esp_chip_info(&espInfo);
  Serial.print("\t- Nr of cores: \t\t"); Serial.println(espInfo.cores);
  Serial.print("\t- ESP Model: \t\t"); Serial.println( espInfo.model );
  Serial.print("\t- Revision: \t\t"); Serial.println( espInfo.revision );
  Serial.print("\t- IDF version: \t\t"); Serial.println( esp_get_idf_version() );
  Serial.print("\t- PSRAM: \t\t"); if (psramFound()) { Serial.println("Yes"); } else { Serial.println("No"); };
  Serial.print("\t- Free Heap Memory: \t"); Serial.println( esp_get_free_heap_size() ); 
  Serial.print("\t- Core temperature: \t"); Serial.println( temperatureRead() );

  Serial.println("----------------------------------- \nSetup done\n");
  BlinkLED(3);
  delay(1000);
 
}

/**************************************************************************
 * loop
 * - Display Temperature and Illumination readings at regular intervals
 * - Display notification when PIR detected movement
 * - Display notification when Switch changed state
***************************************************************************/
void loop() {
  static int lastLuxReading = 0;
  static int lastTempReading = 0;

  // --- LIGHT reading ---
  if ( (lastLuxReading == 0) || (millis() - lastLuxReading > luxReadInterval) ) {
    //GetLuxReading();
    sensors_event_t event;
    sensorLux.getEvent(&event);
    int valueLux =  event.light;
    Serial.print (" - Illuminence: "); Serial.println( valueLux);
    BlinkLED(1);
    lastLuxReading = millis();
  }

  // --- TEMPERATURE reading ---
  if ( (lastTempReading == 0) || (millis() - lastTempReading > tempReadInterval) ) {
    sensorTemp.requestTemperatures();
    float curTemp = sensorTemp.getTempCByIndex(0);
    Serial.print(" - Temperature: "); Serial.println(curTemp);
    BlinkLED(1);
    lastTempReading = millis();
  }

  // --- PIR Motion detection ---
  if (motionDetected) {
    Serial.println(" - Motion Detected");
    BlinkLED(1);
    portENTER_CRITICAL_ISR(&pirMux);
    motionDetected = false;
    portEXIT_CRITICAL_ISR(&pirMux);
  }

  // --- SWITCH changes ---
  if (switchChangedState ) {
    portENTER_CRITICAL_ISR(&switchMux);
    if ( (millis() - lastSwitchChange > debounceSwitch) ) {
      // The switch changed, and we waited long enough after the first change/bounce.
      Serial.print(" - Switch changed: "); 
      switch ( digitalRead(PIN_SWITCH) ) {
        case 0:     Serial.println("CLOSED"); break;
        case 1:     Serial.println("OPEN"); break;
        default :   Serial.println("UNKNOWN??");
      }
      BlinkLED(1);
      switchChangedState = false;
    }
    portEXIT_CRITICAL_ISR(&switchMux);
  }
}


