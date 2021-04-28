# ESP32Cam_I2C_OneWire_Switch
ESP32-Cam based project to test the use of multiple types of sensors in combination with streaming video on the ESP32-Cam. This sketch has no other purpose than to make sure that all sensors can be read correctly, while at the same time streaming video from the web server hosted on the ESP32-Cam.    
    
The sketch can be used to verify your hardware is functioning ok before you develop/run your own sketch, or use it as a base to add your own functionality, or just use the relevant code to support whatever sensors you are going to use.  

## Description
When the sketch is successfully compiled and all sensors are connected, the following will be shown as debug output:    
   - periodic temperature and luminosity sensor readings.    
   - PIR movement detection.    
   - switch status changes when it is opened or closed.    
    
Using a browser, the video stream can be viewed by opening the URL of the ESP32-Cam.  ( _`http://<IP adress>`_ )    
    
#### Setup
   1. Copy the code (_`main.cpp`_ and also _`configuration.h`_) to your local project.
   2. Set your own SSID and password.
   3. Update the Adafruit libraries as described below.
   4. Connect the sensors as shown in the Fritzing diagram and table below.
   5. Compile the sketch.    
    
<br>       
Sensors used:   
    
Type | Sensor | Function | GPIO
--- | --- | --- | ---
I2C | [TSL2561](https://learn.adafruit.com/tsl2561) | Luminosity (Lux) | 14 (SDA)  <br>  15 (SCL)
Wire | [DS18B20](https://datasheets.maximintegrated.com/en/ds/DS18B20.pdf) | Temperature | 2
Binary | [AM312](https://unusualelectronics.co.uk/as312-am312-mini-pir-module-review/) | PIR | 13
Switch |  |  |  12
    
## Issues
Although not all that difficult, there are a couple of problems that must be kept in mind to make this work.
   1. **I2C**    
      The default I2C pins in a ESP32 are GPIO 22 (SCL) and GPIO 21 (SDA). Problem with the ESP32-Cam is that these pins are already used by the camera, so you have to remap SDA and SCL to other pins.
   3. **Attach Interrupts**    
      The [GPIO interrupt service can only be initialized once](https://github.com/espressif/esp-who/issues/90#issuecomment-518142982), and that already happens when the camera is initialized. This means that the normal _`attachInterrupt()`_ command can't be used. Use _`gpio_set_intr_type()`_ and _`gpio_isr_handler_add()`_ instead.
   5. **Interrupt Pins**    
      Not just any of the available GPIO pins can be used to attach a hardware interrupt. E.g. pin 16 is constantly triggered when video is streaming.
   7. **Adafruit Libraries**    
      When (some of?) the Adafruit Sensor libraries are used together with the ESP camera libraries, there are errors raised during compilation because different structures in these libraries share the same name. See below.
   9. **Setup Sequence**    
       The order in which the functionality is initialized in _`setup()`_ is important to avoid errors and make things work.
    
#### Resolve Struct Conflict
A struct with the same name, **_`sensor_t`_** but with a different definition exists in both the _`esp_camera`_ library as well as some of the Adafruit Sensor libraries. When these libraries are used together in the same project, errors are raised as shown below. These errors can be solved by renaming the struct in the Arduino libraries using the steps below:
   1. Include the _`esp_camera.h`_ library before the _`Adafruit_Sensor.h`_ and _`Adafruit_TSL2561_U.h`_ libraries. This will help to identify the occurrences of the struct name in the Adafruit libraries (else the errors are reported in the ESP library).
   2. Compile the sketch. There may be errors as shown below. Click on each of the errors (in [`Platform IO`](https://platformio.org/), under `Problems`) to open the relevant file and put the cursor on the offending line.
   3. Rename the _`sensor_t`_ struct in the Adafruit libraries to something else, e.g. _`sensor_tas`_ (for Adafruit Sensor).    
      Below are the files I had to change in my case:   
		     - _`Adafruit_Sensor.h`_	(x2)   
		     - _`Adafruit_Sensor.cpp`_	(x1)   
		     - _`Adafruit_TSL2561_U.h`_	(x1)   
		     - _`Adafruit_TSL2561_U.cpp`_	(x2)   

##### Wire Diagram

![Fritzing Diagram](https://github.com/JJFourie/ESP32Cam_I2C_OneWire_Switch/blob/main/Images/ESP32Cam_I2C_OneWire_Switch.jpg)
- The white wires are only necessary when flashing the board with a new sketch, or when debugging in order to see the output of debug statements in the IDE.    
  **Note**: *Ensure your FTDI card pins match the layout in the diagram! My (fake?) card pin layout was the opposite, an exact mirror image of the most common cards.*
    
-----
.    
    
#### Compile Errors caused by struct conflict
```
In file included from src\main.cpp:26:0:
.pio\libdeps\esp32cam\Adafruit Unified Sensor/Adafruit_Sensor.h:155:3: error: conflicting declaration 'typedef struct sensor_t sensor_t'
 } sensor_t;
   ^
In file included from C:\Users\johan\.platformio\packages\framework-arduinoespressif32\tools\sdk\include\esp32-camera/esp_camera.h:70:0,
                 from src\main.cpp:17:
C:\Users\johan\.platformio\packages\framework-arduinoespressif32\tools\sdk\include\esp32-camera/sensor.h:191:3: note: previous declaration as 'typedef struct _sensor sensor_t'
 } sensor_t;
   ^
src\main.cpp: In function 'esp_err_t cam_Setup()':
src\main.cpp:172:57: error: 'err' was not declared in this scope
     Serial.printf("Camera init failed with error 0x%x", err);
                                                         ^
src\main.cpp:173:5: error: return-statement with no value, in function returning 'esp_err_t {aka int}' [-fpermissive]
     return;
     ^
Archiving .pio\build\esp32cam\libba4\libAdafruit Unified Sensor.a
*** [.pio\build\esp32cam\src\main.cpp.o] Error 1


```
