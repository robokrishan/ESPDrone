/*
  ESPDrone.cpp - Library for ESP32-based Quadcopter.
  Created by Harit Krishan, August 20, 2021.
  Released into the public domain.
*/



#include "ESPDrone.h"
#include "Arduino.h"

// Constructors

ESPDrone::ESPDrone() {
  stp = 0;
  imu = false;
  printStep("Drone Object Created");
  setThrottle(0);
  printStep("Throttle Initialized");
  setRoll(0.0);
  setPitch(0.0);
  setYaw(0.0);
  printStep("RPY Initialized");
  initBuffer();
  delay(1000);
  printStep("RC Buffer Initialized");
}


// Setter Methods

void ESPDrone::setThrottle(int newDuty) {
  Throttle = map(newDuty, 0,1023, 1000, 2000);
  newDuty = map(newDuty, 0, 1023, 0, 100);
  sThrottle = String(newDuty);
  //Serial.println(Throttle);
}

void ESPDrone::setRoll(float newRoll) {
  Roll = (float)newRoll*180/PI;
  sRoll = String(Roll);
}

void ESPDrone::setPitch(float newPitch) {
  Pitch = (float)newPitch*180/PI;
  sPitch = String(Pitch);
}

void ESPDrone::setYaw(float newYaw) {
  Yaw = (float)newYaw*180/PI;
  sYaw = String(Yaw);
}

void ESPDrone::setIMU(bool stat) {
  imu = stat;
}

void ESPDrone::setGyro(float* vect) {
  setYaw(vect[0]);
  setPitch(vect[1]);
  setRoll(vect[2]);
}


// ESC Configuration

void ESPDrone::attachESCPins() {
  fl_esc.attach(16);
	fr_esc.attach(17);
	bl_esc.attach(18);
	br_esc.attach(19);
  printStep("ESCs Configured on Defaults Pins");
}

void ESPDrone::attachESCPins(int fl, int fr, int bl, int br) {
  fl_esc.attach(fl);
	fr_esc.attach(fr);
	bl_esc.attach(bl);
	br_esc.attach(br);
  printStep("ESCs Configured");
}

// Getter Methods

int ESPDrone::getThrottle() {
  return(Throttle);
}

int ESPDrone::getRoll() {
  return(Roll);
}

int ESPDrone::getPitch() {
  return(Pitch);
}

int ESPDrone::getYaw() {
  return(Yaw);
}


// Radio

void ESPDrone::initRadio() {
  radio = new RF24(12,14,26,25,27);
  radio->begin();
  radio->openReadingPipe(0,atoi(address));
  radio->setPALevel(RF24_PA_MIN);
  radio->startListening();
  printStep("Radio Configured on SPI Bus");
}


void ESPDrone::receive() {
  initBuffer();
  if(radio->available()) {
    radio->read(&buffer, sizeof(buffer));
    bufferInt = atoi(buffer);
    //Serial.print(bufferInt);
    //Serial.print("\t");
    setThrottle(bufferInt);

  }
}

void ESPDrone::initBuffer() {
  bufferInt = 0;
}


// Debugging

void ESPDrone::printStep(char* msg) {
  if(stp == 0) {
    Serial.begin(115200);
  }
  Serial.print("\t[");
  Serial.print(stp);
  Serial.print("]\t");
  Serial.println(msg);
  stp++;
}

int ESPDrone::getStep() {
  return stp;
}

void ESPDrone::incrementStep() {
  stp++;
}


// OLED Display

void ESPDrone::initOLED() {
  oled = new Adafruit_SH1106;
  oled->begin(SH1106_SWITCHCAPVCC, OLED_ADDR);
  oled->clearDisplay();
  oled->setTextSize(1);
  oled->setTextColor(WHITE);
  oled->setCursor(10,20);
  oled->println("Initializing...");
  oled->setCursor(10,40);
  oled->println("Configured!");
  oled->display();
  delay(2000);
  oled->clearDisplay();
  printStep("OLED Configured on I2C Bus");
}

void ESPDrone::printHbar(int x, int y) {
  oled->setCursor(x,y);
  oled->print("---------------------");
}

void ESPDrone::printVbar(int x, int y) {
  oled->setCursor(x,y);
  for(int i = y; i <= 61-y; i+=4) {
    oled->setCursor(x, i);
    oled->print("|");
  }
}

void ESPDrone::showOLED() {

  oled->setCursor(22,0);
  oled->print("FC Dashboard");
  oled->clearDisplay();
  printHbar(0,7);
  printVbar(60,15);
  oled->setCursor(70,20);
  oled->print("Thr: ");
  oled->print(sThrottle);
  oled->print("%");
  oled->setCursor(70, 32);
  oled->print("PPM: ");
  oled->print(Throttle);
  oled->setCursor(15, 57);
  oled->print("IMU ");
  oled->print(imuConnected());
  oled->setCursor(0, 20);
  oled->print("R:");
  oled->setCursor(0, 32);
  oled->print("P:");
  oled->setCursor(0, 44);
  oled->print("Y:");
  oled->setCursor(25, 20);
  oled->print(sRoll);
  oled->setCursor(25, 32);
  oled->print(sPitch);
  oled->setCursor(25, 44);
  oled->print(sYaw);

  oled->display();

}

String ESPDrone::imuConnected() {
  if(imu == false) {
    return("Disconnected");
  } else {
    return("Connected");
  }
}



// Kinematics

void ESPDrone::Fly() {
  fl_esc.writeMicroseconds(Throttle);
	fr_esc.writeMicroseconds(Throttle);
	bl_esc.writeMicroseconds(Throttle);
	br_esc.writeMicroseconds(Throttle);
}
