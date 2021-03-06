# ESPDrone
Library for an Embedded Flight Controller based on an ESP32 with Radio Reception.

This library is written for easily configuring and deploying a Quadcopter object on an ESP32 Developer Board. 

### Installation:
1. Download the repository as a zip or clone it.
2. Move the **lib_dep.zip** file out of the cloned directory.
3. Extract the **lib_dep.zip** file. The extracted folder should contain all the dependencies.
4. Open Arduino IDE and add all the libraries in the extracted folder by going to *Sketch* >> *Include Library* >> *Add .ZIP library*.
      - It does not matter if the libraries are not in .ZIP format as long as they are folders.
      - You will have to add the libraries one by one, you cannot select multiple folders.
5. In the same way, add the cloned directory (**ESPDrone/**) as a library.
6. In the Arduino IDE, go to *File* >> *Examples* >> *ESPDrone* >> *ESPDrone_test*.
      - This is the Arduino sketch to flash on the ESP32 module.

### Description
The sketch implements an ESPDrone object consisting of an RF24 object, an SH1106 object, 4 Servo objects and variables for storing Gyro readings. The IMU configuration is not internal to the ESPDrone class due to the use of an (External) Interrupt Service Routine. The dual core ESP32 MCU has been exploited for running an RTOS with a dedicated core for running the embedded Digital Motion Processor (DMP) and a dedicated core for running the ESPDrone object methods. Data from the DMP is stored in global variables, accessible by the ESPDrone class methods.

### Further Scope
I hope to inherit the DMP Implementation of the IMU into the ESPDrone class by configuring the ISR as a static method.


# Dependencies
Library | Author
:-----: | :----:
<ESP32Servo.h> | John K. Bennett
<Adafruit_SH1106.h> | Limor Fried, Adafruit Industries
<SPI.h> | Internal Arduino Lib
<RF24.h> | J. Coliz, 2011
<MPU6050_6Axis_MotionApps20.h> | Jeff Rowberg, 2012


- The object uses the custom ESP32Servo library for sending PPM signals to the ESCs of the propellers. Default pin configuration for the ESCs is {16,17,18,19}. 

- The ESPDrone class houses an implementation for displaying Gyro Values and Throttle on an SH1106 OLED display (128x64) connected to the default I2C bus (21,22). 

- The class also houses a radio receiver implementation making use of the NRF24L01+ module connected to the default SPI bus (12,14,26,25,27). The RF24 class can configure the Radio module on the SPI bus for the ESP32 by calling the parametrized Constructor - RF24(CE,CS,SCK,MISO,MOSI). 

- The ESPDrone class uses the MPU6050 IMU module for reading Yaw-Pitch-Roll values using the on-board Digital Motion Processor (DMP). The IMU needs to be connected to the I2C bus (21,22). An extra connection needs to be made between the IMU Interrupt pin and pin 33 of the ESP32.

**All dependencies have been provided in the *"lib_dep.zip"* file.**

# Hardware
The list of elementary hardware is tabulated below.

Hardware | Description
:------: | -----------
ESP32 Wroom-32 NodeMCU | Microcontroller Board for running FC Firmware
MPU6050 | Inertial Measurement Unit (IMU) with embedded Gyroscope, Accelerometer & Temperature Sensor
NRF24L01+ | 2.4 GHz Wireless Transceiver Module
0.96" OLED Display | Embedded OLED Display module using the SH1106 driver chip


<p align="center">
  <img src="/img/nrf.jpeg" height="200" />
  <img src="/img/imu.jpg" height="200" /> 
  <img src="/img/oled.jpeg" height="200" />
</p>


### MPU6050 Connection to ESP32
MPU6050 | ESP32
:-----: | :---:
VCC | 3.3V
GND | GND
SCL | 22
SDA | 21
INT | 33


### NRF24L01+ Connection to ESP32
NRF24L01+ | ESP32
:-----: | :---:
CE | 12
CSN | 14
SCK | 26
MISO | 25
MOSI | 27
Vcc | 3.3V
GND | GND


### OLED Display Connection to ESP32
SH1106 | ESP32
:-----: | :---:
VCC | 3.3V
GND | GND
SCL | 22
SDA | 21

# Prototype
The following image shows the Hardware that was used for testing the Flight Controller Firmware.

<p align="center">
  <img src="/img/fc.jpg" width="500" /> 
</p>

# Wireless Comms.
Use the ESPDroneTransmitter library for implementing an RC Transmitter to drive this ESPDrone Flight Controller. ESPDroneTransmitter can be found on the following Github Link: https://github.com/robokrishan/ESPDroneTransmitter

The current implementation of the Flight Controller is programmed to receive raw, digital throttle input from a potentiometer in the range [0, 1023]. This is mapped to the range [1000, 2000] for generating the accepted PPM signal. 
