# ESP32Cam_I2C_OneWire_Switch
ESP32-Cam based project to test the use of multiple types of sensors in combination with streaming video on the ESP32-Cam.   
This sketch has no other purpose than to make sure that all sensors can be read correctly, while at the same time streaming video from the camera on a locally hosted web server.  

## Description
When the sketch is successfully compiled and all sensors are connected, the following will be shown as debug output:    
   - periodic temperature and illumenence sensor readings.    
   - PIR movement detection.    
   - switch status changes when it is opened or closed.    
    
Using a browser, watch the video stream by opening the URL of the ESP32-Cam.  ( _`http://<IP adress>`_ )    
    
##### Setup
   1. Copy the code (_`main.cpp`_ and also _`configuration.h`_) to your local project.
   2. Set your own SSID and password.
   3. Update the Adafruit libraries as described below.
   4. Connect the sensors following the Fritzing diagram and table below.
   5. Compile the sketch.    
   
Below are the sensors used:   
    
Type | Sensor | Function | GPIO
--- | --- | --- | ---
I2C | TSL2561 | Illumination (Lux) | 14 (SDA)  <br>  15 (SCL)
Wire | DS18B20 | Temperature | 2
Binary | AM312 | PIR | 13
Switch |  |  |  12
    
## Issues
Although not all that difficult, there are a couple of problems that need to be addressed in order to make this work.
   1. **I2C**    
      The default I2C pins (21, 22) are already used by the camera, so you have to remap SDA and SLC to other pins.
   3. **Attach Interrupts**    
      The [interrupts can only initialized once](https://github.com/espressif/esp-who/issues/90), and that already happens when the camera is initialized. This means that the normal _`attachInterrupt()`_ command can't be used. Use _`gpio_set_intr_type()`_ and _`gpio_isr_handler_add()`_ instead.
   5. **Interrupt Pins**    
      Not just any GPIO pin can be used to attach a hardware interrupt, e.g. pin 16 is constantly triggered when video is streaming. Use e.g. pin 12 instead.
   7. **Adafruit Libraries**    
      When (some of?) the Adafruit Sensor libraries are used together with the ESP camera libraries, there is a conflict with a different struct but with the same name. See below.
   9. **Setup sequence**    
       The order in which the functionality is initialized in _`setup()`_ is important to avoid errors and make things work.
    
#### Resolve Struct Conflict
A struct *sensor_t* with a different definition exists in the _`esp_camera.h`_ library as well as some of the Adafruit Sensor libraries. When these libraries are used together in the same project, errors are raised as shown below. These errors can be solvedby renaming the struct in the Arduino libraries following the steps below:
   1. Include the _`esp_camera.h`_ library before the _`Adafruit_Sensor`_ and _`Adafruit_TSL2561_U.h`_ libraries. This will help to identify the occurrences of the struct in the Adafruit libraries.
   2. Compile the sketch. There will be errors as shown below. Click on the errors (in Platform IO) to open the library and put the cursor on the offending line.
   3. Rename the _`sensor_t`_ struct in the Adafruit libraries to something else, e.g. _`sensor_tas`_ (for Adafruit Sensor).    
      Below are the files I needed to change in my case:   
		     - Adafruit_Sensor.h	(x2)   
		     - Adafruit_Sensor.cpp	(x1)   
		     - Adafruit_TSL2561_U.cpp	(x2)   
		     - Adafruit_TSL2561_U.h	(x1)   

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
