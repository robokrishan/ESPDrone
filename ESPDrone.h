/*
  ESPDrone.h - Library for ESP32-based Quadcopter.
  Created by Harit Krishan, August 20, 2021.
  Released into the public domain.
*/

#ifndef ESPDrone_h
#define ESPDrone_h

#include "Arduino.h"
#include <Adafruit_SH1106.h>
#include <ESP32Servo.h>
#include <SPI.h>
#include <RF24.h>

class ESPDrone {
  public:

    // Constructor
    ESPDrone();

    // Setters
    void setThrottle(int newDuty);
    void setRoll(float newRoll);
    void setPitch(float newPitch);
    void setYaw(float newYaw);
    void setIMU(bool stat);
    void setGyro(float* vect);

    // ESC Configuration
    void attachESCPins();
    void attachESCPins(int fl, int fr, int bl, int br);

    // Getters
    int getThrottle();
    int getRoll();
    int getPitch();
    int getYaw();

    // Radio
    void initRadio();
    void receive();
    void initBuffer();

    // Debugging
    void printStep(char* msg);
    int getStep();
    void incrementStep();

    // OLED Display
    void initOLED();
    void printHbar(int x, int y);
    void printVbar(int x, int y);
    void showOLED();
    String imuConnected();

    // Kinematics
    void Fly();

  private:
    // Radio Variables
    RF24 *radio;
    char address[6] = "00001";
    char buffer[32];
    int bufferInt;

    // OLED Display Variables
    Adafruit_SH1106 *oled;
    uint16_t OLED_ADDR = 0x3C;
    bool imu;
    String sThrottle, sRoll, sPitch, sYaw;
    int Throttle, Roll, Pitch, Yaw;

    // Debug Step Counter
    int stp;

    // Motor Configuration
    Servo fl_esc;
    Servo fr_esc;
		Servo bl_esc;
		Servo br_esc;
    uint16_t duty;

};

#endif
