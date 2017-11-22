
#include "I2Cdev.h"
#include "MPU6050_6Axis_MotionApps20.h"

#if I2CDEV_IMPLEMENTATION == I2CDEV_ARDUINO_WIRE
    #include "Wire.h"
#endif

#include <SPI.h>
#include "RF24.h"

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
float ypr[3]; // yaw, pitch, roll

void setup() {
	Wire.begin();
	TWBR = 24; //400KHz SCL

  EICRA = 0x03;
  EIMSK = 0x01;

	Serial.begin(115200);

	mpu.initialize();
	//mpu.testConnection();

	uint8_t dmpstatus = mpu.dmpInitialize();

	mpu.setXGyroOffset(49);
	mpu.setYGyroOffset(-15);
	mpu.setZGyroOffset(49);
	mpu.setZAccelOffset(1546);

	if (!dmpstatus) { //success
		mpu.setDMPEnabled(true);
		mpu.getIntStatus();

		mpuIntStatus = mpu.getIntStatus();

		packet_size = mpu.dmpGetFIFOPacketSize();
	} // else error

  radio.begin();
  radio.openWritingPipe(addresses[1]);
  radio.openReadingPipe(1, addresses[0]);
  
}

void loop() {

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
			mpu.dmpGetYawPitchRoll(ypr, &q, &gravity);

			// convert pitch and roll to degrees
			ypr[1] = ypr[1] * 180/M_PI;
			ypr[2] = ypr[2] * 180/M_PI;

      if (!radio.write(ypr+1, 2*sizeof(float))) {
        Serial.println("tx error");
      }

      Serial.print(ypr[1]);
      Serial.print(" ");
      Serial.println(ypr[2]);
		}
	}
}
