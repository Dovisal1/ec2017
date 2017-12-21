# Embedded Challenge 2017
An arduino implementation of a Robot Rover remotely controlled by a gyroscope.

## Hardware Needed
This project requires 2 arduinos. The first one uses an MPU-6050 as an orientation sensor to receive the user control input.
The second car controlls the motors on the rover. The arduinos communicate using 2 NRF24L wireless modules.

## Dependancies
The MPU6050 library is required from [i2cdevlib](https://github.com/jrowberg/i2cdevlib).
Install `Arduino/MPU6050` and `Arduino/I2Cdev` from the repo into your arduno library or sketch folder.
