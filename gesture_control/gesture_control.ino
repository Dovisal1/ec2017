
#include "I2Cdev.h"
#include "MPU6050_6Axis_MotionApps20.h"

#if I2CDEV_IMPLEMENTATION == I2CDEV_ARDUINO_WIRE
    #include "Wire.h"
#endif

#include <SPI.h>
#include "RF24.h"

#include <avr/wdt.h>

#define DEBUG 0
#define Dprint(s) if (DEBUG) Serial.print(s)
#define Dprintln(s) if (DEBUG) Serial.println(s)

byte addresses[][7] = {"3drive", "3cntrl"};

RF24 radio(7,8);

MPU6050 mpu;

volatile bool mpuInterrupt = false;
ISR(INT0_vect) {
	mpuInterrupt = true;
}

uint8_t mpuIntStatus;
uint16_t packet_size;
uint16_t fifo_count;
uint8_t fifo[64];

Quaternion q;
VectorFloat gravity;
float pr[2]; // pitch, roll
unsigned long lastread = 0;

void setup() {
  
  // Begin the I2C for MPU6050 (gyroscope input)
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

  /* enable the watchdog
   *  The gyroscope sensor will arbitrarily hang, and it seems
   *  that others online are reporting the same problem without
   *  any good solutions.
   *  
   *  Hopefully this watchdog timer will solve the issue
   *  albeit in an imperfect way.
   */
  cli();
  WDTCSR = 0x18; // enter config
  WDTCSR = 0x0b; // reset enable
  sei();
}

void loop() {

  wdt_reset(); // update the watchdog
  
	if (mpuInterrupt || fifo_count >= packet_size) {
		mpuInterrupt = false;
		mpuIntStatus = mpu.getIntStatus();
		fifo_count = mpu.getFIFOCount();

		// check for FIFO overflow
		if ((mpuIntStatus & 0x10) || fifo_count == 1024) {
			mpu.resetFIFO();
			// otherwise, check for DMP ready
		} else if (mpuIntStatus & 0x02) {
			while(fifo_count < packet_size)
				fifo_count = mpu.getFIFOCount();

			mpu.getFIFOBytes(fifo, packet_size);
			fifo_count -= packet_size;

			// get values
			mpu.dmpGetQuaternion(&q, fifo);
			mpu.dmpGetGravity(&gravity, &q);
      mpu.dmpGetPitchRoll(pr, &q, &gravity);

			// convert pitch and roll to degrees
			pr[0] = pr[0] * 180/M_PI;
			pr[1] = pr[1] * 180/M_PI;

      radio.write(pr, sizeof(pr));

      Dprint(pr[0]);
      Dprint(F(" "));
      Dprintln(pr[1]);
		}
   
   lastread = millis();
	} else if (millis() - lastread > 250) {
    Dprintln(F("timeout!"));
    while (mpu.dmpInitialize())
      Dprintln(F("init"));
    mpu.setDMPEnabled(true);
    lastread = millis();
	}
}
