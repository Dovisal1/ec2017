
#include "I2Cdev.h"
#include "MPU6050_6Axis_MotionApps20.h"

#if I2CDEV_IMPLEMENTATION == I2CDEV_ARDUINO_WIRE
    #include "Wire.h"
#endif

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

  pinMode(13, OUTPUT);
}

float pitch = 0;
float roll = 0;
uint8_t ledstatus = 0;

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
			pitch = ypr[1] * 180/M_PI;
			roll  = ypr[2] * 180/M_PI;

      char level;
      if (pitch > 40)
        level = 'A';
      else if (pitch > 30)
        level = 'B';
			else if (pitch > 20)
				level = 'C';
			else if (pitch > 10)
				level = 'D';
			else if (pitch > -10)
				level = 'E';
			else if (pitch > -20)
				level = 'F';
      else if (pitch > -30)
        level = 'G';
      else if (pitch > -40)
        level = 'H';
      else
        level = 'I';

      Serial.println(pitch);
		}
	}
}
