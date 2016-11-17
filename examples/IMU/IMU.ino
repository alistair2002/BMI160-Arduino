/*
 * Copyright (c) 2016 Intel Corporation.  All rights reserved.
 * See the bottom of this file for the license terms.
 */

/*
   This sketch example demonstrates how the BMI160 on the
   Intel(R) Curie(TM) module can be used to read gyroscope data
*/


#include <BMI160Gen.h>
#include <limits.h>
#include <MadgwickAHRS.h>

unsigned long microsPerReading, microsPrevious;
Madgwick filter;

const float ACCEL_RANGE = 4;
const float GYRO_RANGE = 250;

void setup() {
  Serial.begin(115200); // initialize Serial communication
  while (!Serial);    // wait for the serial port to open

  // initialize device
  BMI160.begin(10);
  uint8_t dev_id = BMI160.getDeviceID();
  Serial.print("DEVICE ID: ");
  Serial.println(dev_id, HEX);

  BMI160.setGyroRate(25);
  BMI160.setAccelerometerRate(25);
  filter.begin(25);
  
  // AR calibrate gyro
  BMI160.autoCalibrateGyroOffset();

   // Set the accelerometer range to 250 degrees/second
  BMI160.setGyroRange(GYRO_RANGE);
  BMI160.setAccelerometerRange(ACCEL_RANGE);

  BMI160.autoCalibrateAccelerometerOffset( X_AXIS, 0 );//level no G
  BMI160.autoCalibrateAccelerometerOffset( Y_AXIS, 0 );//level no G
  BMI160.autoCalibrateAccelerometerOffset( Z_AXIS, 1 );//flat 1g

  // initialize variables to pace updates to correct rate
  microsPerReading = 1000000 / 25;
  microsPrevious = micros();

}

void loop() {
	int gxRaw, gyRaw, gzRaw;         // raw gyro values
	float gx, gy, gz;
	int axRaw, ayRaw, azRaw;         // raw gyro values
	float ax, ay, az;
	int temp = 0;
	float read_temp = 0;
	float roll, pitch, heading;
	unsigned long microsNow;

	// check if it's time to read data and update the filter
	microsNow = micros();
	if (microsNow - microsPrevious >= microsPerReading) {

		// read raw gyro measurements from device
		BMI160.readGyro(gxRaw, gyRaw, gzRaw);

		// temp range over 128 values (64+ and 64- offset by 23)
		read_temp = BMI160.getTemperature();

		// convert the short read to a float otherwise we get rounding errors
		temp = 23 + (64 * read_temp)/SHRT_MAX;

		// convert the raw gyro data to degrees/second
		gx = convertRawGyro(gxRaw);
		gy = convertRawGyro(gyRaw);
		gz = convertRawGyro(gzRaw);

		//read the raw accelerometer values;
		BMI160.readAccelerometer( axRaw, ayRaw, azRaw);

		// convert the raw gyro data to degrees/second
		ax = convertRawAccel(axRaw);
		ay = convertRawAccel(ayRaw);
		az = convertRawAccel(azRaw);

		// update the filter, which computes orientation
		filter.updateIMU(gx, gy, gz, ax, ay, az);

		// print the heading, pitch and roll
		roll = filter.getRoll();
		pitch = filter.getPitch();
		heading = filter.getYaw();
		Serial.print("Orientation: ");
		Serial.print(heading);
		Serial.print(" ");
		Serial.print(pitch);
		Serial.print(" ");
		Serial.println(roll);

		// increment previous time, so we keep proper pace
		microsPrevious = microsPrevious + microsPerReading;
	}
}

float convertRawGyro(int gRaw) {
	// if we are using 250 degrees/seconds range
	// -250 maps to a raw value of -32768
	// +250 maps to a raw value of 32767

	float g = ((float)gRaw * GYRO_RANGE) / SHRT_MAX;//32768.0;

	return g;
}

float convertRawAccel(int aRaw) {

	/* nb this value is a ratio of the range.  I.e. 
	   if set to 4g 25% of SHRT_MAX is 1g.  2g will be
	   reflected as about 50%.  If we set the range to
	   2G then 2g will reflect a 100% value */
	float a = (((float)aRaw * ACCEL_RANGE) / SHRT_MAX) * 9.8;

	return a;
}

/*
  Copyright (c) 2016 Intel Corporation.  All rights reserved.
  This library is free software; you can redistribute it and/or
  modify it under the terms of the GNU Lesser General Public
  License as published by the Free Software Foundation; either
  version 2.1 of the License, or (at your option) any later version.
  This library is distributed in the hope that it will be useful,
  but WITHOUT ANY WARRANTY; without even the implied warranty of
  MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the GNU
  Lesser General Public License for more details.
  You should have received a copy of the GNU Lesser General Public
  License along with this library; if not, write to the Free Software
  Foundation, Inc., 51 Franklin Street, Fifth Floor, Boston, MA  02110-1301  USA
*/
