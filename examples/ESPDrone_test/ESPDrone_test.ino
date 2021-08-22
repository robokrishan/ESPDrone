#include <ESPDrone.h>
#include <MPU6050_6Axis_MotionApps20.h>
#include <Wire.h>

// Create Drone object
ESPDrone quad;

// IMU Variables
uint16_t MPU_ADDR = 0x68;
int MPU_INTERRUPT = 33;
int LED_INTERNAL = 2;
MPU6050 mpu;
bool blinkState = false;
volatile bool mpuInterrupt = false;

// MPU control/status vars
bool dmpReady = false;  // set true if DMP init was successful
uint8_t mpuIntStatus;   // holds actual interrupt status byte from MPU
uint8_t devStatus;      // return status after each device operation (0 = success, !0 = error)
uint16_t packetSize;    // expected DMP packet size (default is 42 bytes)
uint16_t fifoCount;     // count of all bytes currently in FIFO
uint8_t fifoBuffer[64]; // FIFO storage buffer

// orientation/motion vars
Quaternion q;           // [w, x, y, z]         quaternion container
VectorFloat gravity;    // [x, y, z]            gravity vector
float ypr[3];           // [yaw, pitch, roll]   yaw/pitch/roll container and gravity vector


void setup() {
  // 1. Initialize Radio
  quad.initRadio();
  delay(1000);
  printStep("Radio Initialized");

  // 2. Initialize OLED
  quad.initOLED();
  delay(1000);
  printStep("OLED Initialized");

  // 3. Initialize BLDCs
  quad.attachESCPins();
  delay(1000);
  printStep("Motors Initialized");

  // 4. Begin Displaying FC Dashboard on OLED
  quad.showOLED();
  printStep("Setup complete.");
  delay(1000);

  // 5. Initialize the IMU
  initIMU();
  quad.setIMU(true);
  delay(500);
}



void loop() {
  // put your main code here, to run repeatedly:
  readIMU();
  quad.receive();
  quad.Fly();
  quad.showOLED();
}


// Interrupt Service Routine
void IRAM_ATTR dmpDataReady() {
    mpuInterrupt = true;
}

// Debug Print Info
void printStep(char* msg) {
  Serial.print("\t[");
  Serial.print(quad.getStep());
  Serial.print("]\t");
  Serial.println(msg);
  quad.incrementStep();
}

// Initialize DMP
void initIMU() {
  mpu.initialize();
  if(mpu.testConnection()) {
    printStep("IMU Initialized");
  } else {
    printStep("IMU failed to initialize");
    while(true){}
  }

  // Configure Interrup Pin and corresponding LED indicator
  pinMode(MPU_INTERRUPT, INPUT);
  pinMode(LED_INTERNAL, OUTPUT);
  printStep("MPU Interrupt Pin Configured");

  devStatus = mpu.dmpInitialize();
  // Set Gyro Offsets
  mpu.setXGyroOffset(220);
  mpu.setYGyroOffset(76);
  mpu.setZGyroOffset(-85);
  mpu.setZAccelOffset(1788); // 1688 factory default for my test chip
  printStep("Configured Gyro Offsets");

  if (devStatus == 0) {
        mpu.setDMPEnabled(true);
        printStep("DMP enabled");

        // enable interrupt detection

        attachInterrupt(MPU_INTERRUPT, dmpDataReady, RISING);
        mpuIntStatus = mpu.getIntStatus();
        printStep("ISR enabled");

        // set our DMP Ready flag so the main loop() function knows it's okay to use it
        printStep("DMP Ready!");
        dmpReady = true;

        // get expected DMP packet size for later comparison
        packetSize = mpu.dmpGetFIFOPacketSize();
    } else {
        printStep("DMP failed to initialize\t");
        while(true){}
    }
}

// Read Y-P-R from DMP
void readIMU() {
  // if programming failed, don't try to do anything
    if (!dmpReady) return;
    mpu.resetFIFO();
    // wait for MPU interrupt or extra packet(s) available
    while (!mpuInterrupt && fifoCount < packetSize) {}

    // reset interrupt flag and get INT_STATUS byte
    mpuInterrupt = false;
    mpuIntStatus = mpu.getIntStatus();

    // get current FIFO count
    fifoCount = mpu.getFIFOCount();

    // check for overflow (this should never happen unless our code is too inefficient)
    if ((mpuIntStatus & 0x10) || fifoCount == 1024) {
        // reset so we can continue cleanly
        mpu.resetFIFO();
        Serial.println(F("FIFO overflow!"));

    // otherwise, check for DMP data ready interrupt (this should happen frequently)
    } else if (mpuIntStatus & 0x02) {
        // wait for correct available data length, should be a VERY short wait
        while (fifoCount < packetSize) {
          fifoCount = mpu.getFIFOCount();
        }

        // read a packet from FIFO
        mpu.getFIFOBytes(fifoBuffer, packetSize);

        // track FIFO count here in case there is > 1 packet available
        // (this lets us immediately read more without waiting for an interrupt)
        fifoCount -= packetSize;

        // display Euler angles in degrees
        mpu.dmpGetQuaternion(&q, fifoBuffer);
        mpu.dmpGetGravity(&gravity, &q);
        mpu.dmpGetYawPitchRoll(ypr, &q, &gravity);
        quad.setYaw(ypr[0]);
        quad.setPitch(ypr[1]);
        quad.setRoll(ypr[2]);


        // blink LED to indicate activity
        blinkState = !blinkState;
        digitalWrite(LED_INTERNAL, blinkState);
    }
}
