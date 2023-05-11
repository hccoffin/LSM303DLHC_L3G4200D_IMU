extern "C" {
	#include <inttypes.h>
}

#include <Wire.h>
#include <math.h>
#include "IMU.h"

float reading_to_float(uint8_t lower_byte, uint8_t higher_byte, float factor=1) {
	uint16_t temp = ((uint16_t)higher_byte << 8) | lower_byte;
	int16_t intval;
	memcpy(&intval, &temp, 2);
	return ((float)intval) * factor;
}

void IMU::init_magnetometer() {
	Wire.beginTransmission(MAG_I2C_ADD);
	Wire.write(MAG_CRA_REG_M);
	Wire.write(MAG_OUTPUT_220);
	Wire.endTransmission();

	Wire.beginTransmission(MAG_I2C_ADD);
	Wire.write(MAG_CRB_REG_M);
	Wire.write(MAG_RANGE_1_3);
	Wire.endTransmission();

	Wire.beginTransmission(MAG_I2C_ADD);
	Wire.write(MAG_MR_REG_M);
	Wire.write(MAG_CONTINUOUS_MODE);
	Wire.endTransmission();


	mag_factor_xy = 1 / (pow(2.0, 15.0) / (1100.0));
	mag_factor_z = 1 / (pow(2.0, 15.0) / (980.0));

	read_magnetometer();
}

void IMU::init_accelerometer(float local_g_acc, uint8_t mode) {
	Wire.beginTransmission(ACC_I2C_ADD);
	Wire.write(ACC_CTRL_REG1_A);
	Wire.write(ACC_DATA_RATE_1344_5376HZ | mode | ACC_ENABLE_XYZ);
	Wire.endTransmission();

	Wire.beginTransmission(ACC_I2C_ADD);
	Wire.write(ACC_CTRL_REG3_A);
	Wire.write(ACC_I1_DRDY1);
	Wire.endTransmission();

	Wire.beginTransmission(ACC_I2C_ADD);
	Wire.write(ACC_CTRL_REG4_A);
	Wire.write(ACC_SCALE_2g | (mode ^ 0b00001000)); // mode ^ 0b00001000 is used to invert mode
	Wire.endTransmission();

	acc_factor = 1 / (pow(2.0, 15.0) / (2.0 * local_g_acc));

	acc_status = 0;
	read_accelerometer();
}

void IMU::init_gyro() {
	Wire.beginTransmission(GYR_I2C_ADD);
	Wire.write(GYR_CTRL_REG1);
	Wire.write(GYR_DATA_RATE_800HZ | GYR_LPF2_CUTOFF_110HZ | GYR_ENABLE_XYZ);
	Wire.endTransmission();

	Wire.beginTransmission(GYR_I2C_ADD);
	Wire.write(GYR_CTRL_REG3);
	Wire.write(GYR_I2_DRDY);
	Wire.endTransmission();

	Wire.beginTransmission(GYR_I2C_ADD);
	Wire.write(GYR_CTRL_REG4);
	Wire.write(GYR_SCALE_250DPS);
	Wire.endTransmission();

	gyro_factor = 1 / (pow(2.0, 15.0) / (250.0));

	gyr_status = 0;
	read_gyro();
}

IMU::IMU(float local_g_acc) {
	Wire.begin();
	Wire.setClock(400000);
	init_accelerometer(local_g_acc, ACC_LOW_POWER_MODE);
	init_gyro();
	// init_magnetometer();
}

bool IMU::read_accelerometer() {
	// this delay seems to be necessary to prevent inconsistent readings
	// without it, can get a status of 1011, which implies that there is new data on z, y, and x but not on z somehow
	// 9 microseconds was minimum I found to work consistently
	// However, minimum period of the accelerometer is ~186 microseconds so I delay a little more since it won't matter
	delayMicroseconds(20); 
	Wire.beginTransmission(ACC_I2C_ADD);
	Wire.write(ACC_STATUS_REG_A | (1 << 7));
	Wire.endTransmission(false);
	Wire.requestFrom(ACC_I2C_ADD, 7);

	uint8_t status = Wire.read();
	acc_status = acc_status | status;
	if ((uint8_t)(acc_status << 5) >> 5 == 0b111) {
		// new data available for accelerometer
		uint8_t xl = Wire.read();
		uint8_t xh = Wire.read();
		uint8_t yl = Wire.read();
		uint8_t yh = Wire.read();
		uint8_t zl = Wire.read();
		uint8_t zh = Wire.read();
			
		xa = -reading_to_float(xl, xh, acc_factor);
		ya = -reading_to_float(yl, yh, acc_factor);
		za = -reading_to_float(zl, zh, acc_factor);

		acc_status = 0;
		return true;
	} else {
		return false;
	}
}

bool IMU::read_gyro() {
	// uint8_t bytes_read = Wire.requestFrom(GYR_I2C_ADD, 7, GYR_STATUS_REG | (1 << 7), 1, true);
	Wire.beginTransmission(GYR_I2C_ADD);
	Wire.write(GYR_STATUS_REG | (1 << 7));
	Wire.endTransmission(false);
	Wire.requestFrom(GYR_I2C_ADD, 7);

	gyr_status = gyr_status | Wire.read();
	if ((uint8_t)(gyr_status << 5) == 0b11100000) {
		// new data available for gyro
		uint8_t xl = Wire.read();
		uint8_t xh = Wire.read();
		uint8_t yl = Wire.read();
		uint8_t yh = Wire.read();
		uint8_t zl = Wire.read();
		uint8_t zh = Wire.read();
			
		xw = reading_to_float(xl, xh, gyro_factor);
		yw = reading_to_float(yl, yh, gyro_factor);
		zw = reading_to_float(zl, zh, gyro_factor);

		gyr_status = 0;
		return true;
	} else {
		return false;
	}
}

void IMU::read_magnetometer() {
	Wire.beginTransmission(MAG_I2C_ADD);
	Wire.write(MAG_XH);
	Wire.endTransmission();
	Wire.requestFrom(MAG_I2C_ADD, 6);

	uint8_t xh = Wire.read();
	uint8_t xl = Wire.read();
	uint8_t zh = Wire.read();
	uint8_t zl = Wire.read();
	uint8_t yh = Wire.read();
	uint8_t yl = Wire.read();

	Wire.beginTransmission(MAG_I2C_ADD);
	Wire.write(0x09);
	Wire.endTransmission();
	Wire.requestFrom(MAG_I2C_ADD, 1);
	Serial.println(Wire.read(), BIN);

	xm = reading_to_float(xl, xh, mag_factor_xy);
	ym = reading_to_float(yl, yh, mag_factor_xy);
	zm = reading_to_float(zl, zh, mag_factor_z);
	Serial.println();
}