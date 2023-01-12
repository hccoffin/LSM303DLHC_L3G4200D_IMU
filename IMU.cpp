extern "C" {
	#include <inttypes.h>
}

#include <math.h>
#include "IMU.h"

float reading_to_float(uint8_t lower_byte, uint8_t higher_byte, float factor=1) {
	uint16_t temp = ((uint16_t)higher_byte << 8) | lower_byte;
	int16_t intval;
	memcpy(&intval, &temp, 2);
	return ((float)intval) * factor;
}

void IMU::init_sensors(float local_g_acc) {
	// accelerometer init
	Wire.beginTransmission(ACC_I2C_ADD);
	Wire.write(ACC_CTRL_REG1_A);
	Wire.write((ACC_DATA_RATE_1344_5376HZ << 4) | (ACC_LOW_POWER_MODE << 3) | 0b111); // 0b111 enables xyz axes
	Wire.endTransmission();

	Wire.beginTransmission(ACC_I2C_ADD);
	Wire.write(ACC_CTRL_REG4_A);
	Wire.write((ACC_SCALE_2g << 4) | ((1-ACC_LOW_POWER_MODE) << 3)); // 1 - mode is used to invert mode
	Wire.endTransmission();
	acc_factor = 1 / (pow(2.0, 15.0) / (2.0 * local_g_acc));


	// gyro init
	Wire.beginTransmission(GYR_I2C_ADD);
	Wire.write(GYR_CTRL_REG1);
	Wire.write((GYR_DATA_RATE_800HZ << 6) | (GYR_LPF2_CUTOFF_110HZ << 4) | 0b1111); // 0b1111 turns on and enables xyz axes
	Wire.endTransmission();

	Wire.beginTransmission(GYR_I2C_ADD);
	Wire.write(GYR_CTRL_REG4);
	Wire.write(GYR_SCALE_250DPS << 4);
	Wire.endTransmission();
	gyro_factor = 1 / (pow(2.0, 15.0) / (250.0));
}

bool IMU::read_accelerometer() {
	uint8_t bytes_read = Wire.requestFrom(ACC_I2C_ADD, 7, ACC_STATUS_REG_A | (1 << 7), 1, true);
	uint8_t acc_status = Wire.read();
	if (acc_status & (1 << 3)) {
		// new data available for accelerometer
		uint8_t xl = Wire.read();
		uint8_t xh = Wire.read();
		uint8_t yl = Wire.read();
		uint8_t yh = Wire.read();
		uint8_t zl = Wire.read();
		uint8_t zh = Wire.read();
			
		xa = reading_to_float(xl, xh, acc_factor);
		ya = reading_to_float(yl, yh, acc_factor);
		za = reading_to_float(zl, zh, acc_factor);
		return true;
	} else {
		return false;
	}
}

bool IMU::read_gyro() {
	long t_start = micros();
	uint8_t bytes_read = Wire.requestFrom(GYR_I2C_ADD, 7, GYR_STATUS_REG | (1 << 7), 1, true);
	uint8_t gyr_status = Wire.read();
	if (gyr_status & (1 << 3)) {
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
		return true;
	} else {
		return false;
	}
}