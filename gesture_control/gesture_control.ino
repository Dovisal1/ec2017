
#include "MPU6050_6Axis_MotionApps20.h"

#include <SPI.h>
#include "RF24.h"

// set DEBUG to 1 for to get status ouputs when testing
#define DEBUG 0
#define Dprint(s) if (DEBUG) Serial.print(s)
#define Dprintln(s) if (DEBUG) Serial.println(s)

// radio object
RF24 radio(7,8);
byte addresses[][7] = {"3drive", "3cntrl"};

// gyroscope object
MPU6050 mpu;

// external interrupt on pin 2 for gyroscope
volatile bool mpuInterrupt = false;
ISR(INT0_vect) {
  mpuInterrupt = true;
}

// gyro status and data variables
uint8_t mpuIntStatus;
uint16_t packet_size;
uint16_t fifo_count;
uint8_t fifo[64];
Quaternion q;
VectorFloat gravity;
float ypr[3]; // pitch, roll

unsigned long lastread = 0;

void setup()
{  
  // Begin the I2C
  Wire.begin();
  TWBR = 24; //400KHz SCL

  // initialize the external interrupt on pin2
  EICRA = 0x03;
  EIMSK = 0x01;

  #if DEBUG == 1
    Serial.begin(9600);
    while(!Serial)
      ;
  #endif

  mpu.initialize();
 
  Dprint(F("Testing device connections..."));

  while (!mpu.testConnection()) {
    Dprintln(F("MPU6050 connection failed."));
    Dprint(F("Trying again..."));
    mpu.initialize();
    delay(250);
  }

  Dprintln(F("MPU6050 connection successful."));

  mpu.setXGyroOffset(49);
  mpu.setYGyroOffset(-15);
  mpu.setZGyroOffset(49);
  mpu.setZAccelOffset(1546);

  while (mpu.dmpInitialize())
    Dprintln(F("DMP init failed!"));
  
  mpu.setDMPEnabled(true);
  packet_size = mpu.dmpGetFIFOPacketSize();

  radio.begin();
  radio.openWritingPipe(addresses[1]);
  radio.openReadingPipe(1, addresses[0]);
}

void loop()
{

  if (mpuInterrupt || fifo_count >= packet_size)
  {
    mpuInterrupt = false;
    mpuIntStatus = mpu.getIntStatus();
    fifo_count = mpu.getFIFOCount();

    // check for FIFO overflow
    if ((mpuIntStatus & 0x10) || fifo_count == 1024) {
      mpu.resetFIFO();
    }
    // otherwise, check for DMP ready
    else if (mpuIntStatus & 0x02) {
      while(fifo_count < packet_size)
        fifo_count = mpu.getFIFOCount();

      mpu.getFIFOBytes(fifo, packet_size);
      fifo_count -= packet_size;

      // get values
      mpu.dmpGetQuaternion(&q, fifo);
      mpu.dmpGetGravity(&gravity, &q);
      mpu.dmpGetYawPitchRoll(ypr, &q, &gravity);

      // convert pitch and roll to degrees
      ypr[1] = ypr[1] * 180/M_PI;
      ypr[2] = ypr[2] * 180/M_PI;

      // transmit the orientation angles
      radio.write(ypr+1, 2 * sizeof(float));

      Dprint(ypr[1]);
      Dprint(F(" "));
      Dprintln(ypr[2]);
    }
   
   // record the last gyro read
   lastread = millis();
  }
  // if no gyro read in last 250ms restart it
  else if (millis() - lastread > 250)
  {
    Dprintln(F("timeout!"));
    while (mpu.dmpInitialize())
      Dprintln(F("init"));
    mpu.setDMPEnabled(true);
    lastread = millis();
  }
}
