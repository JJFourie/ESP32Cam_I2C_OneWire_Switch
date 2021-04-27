const char* ssid = "YOUR SSID";                     // WLAN WiFi name
const char* password = "YOUR PWD";                  // WiFi password

const unsigned int luxReadInterval = 20000;         // Duration between light level measurements (msec)
const unsigned int tempReadInterval = 20000;        // Duration between Temperature measurements (msec)
const unsigned int debounceMovement = 10000;        // Ignore PIR triggers during this period
const unsigned int debounceSwitch = 50;             // Ignore switch changes during this period (msec) 

#define PIN_PIR 13                                  // PIR Sensor
#define PIN_TEMP 2                                  // Temperature Sensor
#define PIN_SWITCH 12                               // Switch (e.g. Limit, Reed, ..)   1, 3  (16 gives error with cam)
#define PIN_FLASH_LED 4                             // Flash LED (big bright)
#define PIN_BOARD_LED 33                            // Board LED (small red)
// Define I2C Pins for I2C (TSL2561). Remap from default pins 21 and 22.
#define I2C_SDA 14
#define I2C_SCL 15
const int pinSwitch = GPIO_NUM_12;
const int pinPIR = GPIO_NUM_13;

//CAMERA_MODEL_AI_THINKER
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

#define PART_BOUNDARY "123456789000000000000987654321"
static const char* _STREAM_CONTENT_TYPE = "multipart/x-mixed-replace;boundary=" PART_BOUNDARY;
static const char* _STREAM_BOUNDARY = "\r\n--" PART_BOUNDARY "\r\n";
static const char* _STREAM_PART = "Content-Type: image/jpeg\r\nContent-Length: %u\r\n\r\n";
