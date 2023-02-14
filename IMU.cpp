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

void IMU::init_accelerometer(float local_g_acc, uint8_t mode) {
	Wire.beginTransmission(ACC_I2C_ADD);
	Wire.write(ACC_CTRL_REG1_A);
	Wire.write((ACC_DATA_RATE_1344_5376HZ << 4) | (mode << 3) | 0b111); // 0b111 enables xyz axes
	Wire.endTransmission();

	Wire.beginTransmission(ACC_I2C_ADD);
	Wire.write(ACC_CTRL_REG4_A);
	Wire.write((ACC_SCALE_2g << 4) | ((1-mode) << 3)); // 1 - mode is used to invert mode
	Wire.endTransmission();
	acc_factor = 1 / (pow(2.0, 15.0) / (2.0 * local_g_acc));

	acc_status = 0;
	read_accelerometer();
}

void IMU::init_gyro() {
	Wire.beginTransmission(GYR_I2C_ADD);
	Wire.write(GYR_CTRL_REG1);
	Wire.write((GYR_DATA_RATE_800HZ << 6) | (GYR_LPF2_CUTOFF_110HZ << 4) | 0b1111); // 0b1111 turns on and enables xyz axes
	Wire.endTransmission();

	Wire.beginTransmission(GYR_I2C_ADD);
	Wire.write(GYR_CTRL_REG4);
	Wire.write(GYR_SCALE_250DPS << 4);
	Wire.endTransmission();
	gyro_factor = 1 / (pow(2.0, 15.0) / (250.0));

	gyr_status = 0;
	read_gyro();
}

IMU::IMU(float local_g_acc) {
	Wire.begin();
	Wire.setClock(400000);
	init_accelerometer(local_g_acc, ACC_NORMAL_POWER_MODE);
	init_gyro();
}

bool IMU::read_accelerometer() {
	uint8_t bytes_read = Wire.requestFrom(ACC_I2C_ADD, 7, ACC_STATUS_REG_A | (1 << 7), 1, true);
	acc_status = acc_status | Wire.read();
	if ((uint8_t)(acc_status << 5) == 0b11100000) {
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
	uint8_t bytes_read = Wire.requestFrom(GYR_I2C_ADD, 7, GYR_STATUS_REG | (1 << 7), 1, true);
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
		Serial.println(gyr_status, BIN);
		return false;
	}
}