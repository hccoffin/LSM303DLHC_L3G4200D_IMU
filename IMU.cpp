#include <Wire.h>
#include "IMU.h"
#include <assert.h>

RingBuffer::RingBuffer(size_t max_size) {
	buffer = (xyz_reading_raw*)malloc(max_size * sizeof(xyz_reading_raw));
	this->max_size = max_size;
}
size_t RingBuffer::size() {
	return (push_index - pop_index) % this->max_size;
}
void RingBuffer::push(xyz_reading_raw reading) {
	assert(this->size() < (this->max_size - 1));
	buffer[this->push_index] = reading;
	this->push_index = (this->push_index + 1) % this->max_size;
}
xyz_reading_raw RingBuffer::pop() {
	assert(this->size() > 0);
	xyz_reading_raw reading = this->buffer[this->pop_index];
	this->pop_index = (this->pop_index + 1) % this->max_size;
	return reading;
}

IMU::IMU(size_t acc_buffer_size, size_t gyr_buffer_size) {
	acc_buffer = RingBuffer(acc_buffer_size);
	gyr_buffer = RingBuffer(gyr_buffer_size);
}

int16_t IMU::data_to_int16(uint8_t lower_byte, uint8_t higher_byte) {
	uint16_t temp = ((uint16_t)higher_byte << 8) | lower_byte;
	int16_t intval;
	memcpy(&intval, &temp, 2);
	return intval;
}

xyz_reading IMU::raw_reading_to_float(xyz_reading_raw raw_reading, float factor) {
	xyz_reading reading = {
		((float)raw_reading.x) * factor,
		((float)raw_reading.y) * factor,
		((float)raw_reading.z) * factor
	};
	return reading;
}

void IMU::write_byte(uint8_t addr, uint8_t reg, uint8_t data) {
	Wire.beginTransmission(addr);
	Wire.write(reg);
	Wire.write(data);
	Wire.endTransmission();
}

uint8_t IMU::read_byte(uint8_t addr, uint8_t reg) {
	Wire.beginTransmission(addr);
	Wire.write(reg);
	Wire.endTransmission(false);
	assert(Wire.requestFrom(addr, (uint8_t)1) == 1);
	return Wire.read();
}

void IMU::read_bytes(uint8_t addr, uint8_t reg, uint8_t* data, uint8_t n) {
	Wire.beginTransmission(addr);
	Wire.write(reg);
	Wire.endTransmission(false);
	assert(Wire.requestFrom(addr, n) == n);
	for (int i = 0; i < n; i++) {
		data[i] = Wire.read();
	}
}

// void IMU::init_magnetometer() {
// 	Wire.beginTransmission(MAG_I2C_ADD);
// 	Wire.write(MAG_CRA_REG_M);
// 	Wire.write(MAG_OUTPUT_220);
// 	Wire.endTransmission();

// 	Wire.beginTransmission(MAG_I2C_ADD);
// 	Wire.write(MAG_CRB_REG_M);
// 	Wire.write(MAG_RANGE_1_3);
// 	Wire.endTransmission();

// 	Wire.beginTransmission(MAG_I2C_ADD);
// 	Wire.write(MAG_MR_REG_M);
// 	Wire.write(MAG_CONTINUOUS_MODE);
// 	Wire.endTransmission();


// 	mag_factor_xy = 1 / (pow(2.0, 15.0) / (1100.0));
// 	mag_factor_z = 1 / (pow(2.0, 15.0) / (980.0));

// 	read_magnetometer();
// }

void IMU::apply_acc_settings(AccSetting settings) {
	acc_settings = settings;
	IMU::write_byte(ACC_I2C_ADD, ACC_CTRL_REG1_A, settings.data_rate | settings.power_mode | ACC_ENABLE_XYZ);

	uint8_t interrupt1_special_config_int = (
		(settings.int1_special_config.click << 7) + 
		(settings.int1_special_config.and_or_interrupt1 << 6) + 
		(settings.int1_special_config.and_or_interrupt2 << 5) + 
		(settings.int1_special_config.data_ready1 << 4) + 
		(settings.int1_special_config.data_ready2 << 3) + 
		(settings.int1_special_config.fifo_watermark << 2) + 
		(settings.int1_special_config.fifo_overrun << 1)
	);
	IMU::write_byte(ACC_I2C_ADD, ACC_CTRL_REG3_A, interrupt1_special_config_int);

	IMU::write_byte(ACC_I2C_ADD, ACC_CTRL_REG4_A, settings.scale | settings.resolution_mode);

	IMU::write_byte(ACC_I2C_ADD, ACC_CTRL_REG5_A, settings.fifo_enabled);
	IMU::write_byte(ACC_I2C_ADD, ACC_FIFO_CTRL_REG_A, settings.fifo_mode | settings.fifo_watermark);

	if (settings.scale == AccSetting::ACC_2g) {
		acc_factor = GRAVITY_CONST * .001 / 16; // divide by 16 because readings are actually 12 bits not 16
	} else if (settings.scale == AccSetting::ACC_4g) {
		acc_factor = GRAVITY_CONST * .002 / 16;
	} else if (settings.scale == AccSetting::ACC_8g) {
		acc_factor = GRAVITY_CONST * .004 / 16;
	} else if (settings.scale == AccSetting::ACC_16g) {
		acc_factor = GRAVITY_CONST * .012 / 16;
	}
}

void IMU::apply_gyr_settings(GyrSetting settings) {
	gyr_settings = settings;
	IMU::write_byte(GYR_I2C_ADD, GYR_CTRL_REG1, settings.data_rate | GYR_ENABLE_XYZ);
	IMU::write_byte(GYR_I2C_ADD, GYR_CTRL_REG4, settings.scale);
	IMU::write_byte(GYR_I2C_ADD, GYR_CTRL_REG5, settings.fifo_enabled | settings.output_selection);
	IMU::write_byte(GYR_I2C_ADD, GYR_FIFO_CTRL_REG, settings.fifo_mode | settings.fifo_watermark);

	if (settings.scale == GyrSetting::GYR_250DPS) {
		gyr_factor = .00875;
	} else if (settings.scale == GyrSetting::GYR_500DPS) {
		gyr_factor = .0175;
	} else if (settings.scale == GyrSetting::GYR_2000DPS) {
		gyr_factor = .070;
	}
}


size_t IMU::read_accelerometer() {
	uint8_t fifo_status = IMU::read_byte(ACC_I2C_ADD, ACC_FIFO_SRC_REG_A);
	uint8_t buffered = fifo_status & 0b11111;
	if (buffered == 0) {
		return 0;
	}
	size_t bytes_per_reading = 6;
	size_t n_bytes = buffered * bytes_per_reading;
	uint8_t data[n_bytes];
	IMU::read_bytes(ACC_I2C_ADD, ACC_DATA_REG, data, n_bytes);
	for (size_t i = 0; i < buffered; i++) {
		uint8_t xl = data[bytes_per_reading*i + 0];
		uint8_t xh = data[bytes_per_reading*i + 1];
		uint8_t yl = data[bytes_per_reading*i + 2];
		uint8_t yh = data[bytes_per_reading*i + 3];
		uint8_t zl = data[bytes_per_reading*i + 4];
		uint8_t zh = data[bytes_per_reading*i + 5];
		xyz_reading_raw reading = {
			(int16_t)-IMU::data_to_int16(xl, xh),
			(int16_t)-IMU::data_to_int16(yl, yh),
			(int16_t)-IMU::data_to_int16(zl, zh)
		};
		this->acc_buffer.push(reading);
	}
	return buffered;
}

size_t IMU::read_gyro() {
	uint8_t fifo_status = IMU::read_byte(GYR_I2C_ADD, GYR_FIFO_SRC_REG);
	uint8_t buffered = fifo_status & 0b11111;
	if (buffered == 0) {
		return 0;
	}
	size_t bytes_per_reading = 6;
	size_t n_bytes = buffered * bytes_per_reading;
	uint8_t data[n_bytes];
	IMU::read_bytes(GYR_I2C_ADD, GYR_DATA_REG, data, n_bytes);
	for (size_t i = 0; i < buffered; i++) {
		uint8_t xl = data[bytes_per_reading*i + 0];
		uint8_t xh = data[bytes_per_reading*i + 1];
		uint8_t yl = data[bytes_per_reading*i + 2];
		uint8_t yh = data[bytes_per_reading*i + 3];
		uint8_t zl = data[bytes_per_reading*i + 4];
		uint8_t zh = data[bytes_per_reading*i + 5];
		xyz_reading_raw reading = {
			IMU::data_to_int16(xl, xh),
			IMU::data_to_int16(yl, yh),
			IMU::data_to_int16(zl, zh)
		};
		this->gyr_buffer.push(reading);
	}
	return buffered;

	// gyr_status = gyr_status | Wire.read();
	// if ((uint8_t)(gyr_status << 5) == 0b11100000) {
	// 	// new data available for gyro
	// 	uint8_t xl = Wire.read();
	// 	uint8_t xh = Wire.read();
	// 	uint8_t yl = Wire.read();
	// 	uint8_t yh = Wire.read();
	// 	uint8_t zl = Wire.read();
	// 	uint8_t zh = Wire.read();
			
	// 	xw = reading_to_float(xl, xh, gyr_factor);
	// 	yw = reading_to_float(yl, yh, gyr_factor);
	// 	zw = reading_to_float(zl, zh, gyr_factor);

	// 	gyr_status = 0;
	// 	return true;
	// } else {
	// 	return false;
	// }
}

// void IMU::read_magnetometer() {
// 	Wire.beginTransmission(MAG_I2C_ADD);
// 	Wire.write(MAG_XH);
// 	Wire.endTransmission();
// 	Wire.requestFrom(MAG_I2C_ADD, 6);

// 	uint8_t xh = Wire.read();
// 	uint8_t xl = Wire.read();
// 	uint8_t zh = Wire.read();
// 	uint8_t zl = Wire.read();
// 	uint8_t yh = Wire.read();
// 	uint8_t yl = Wire.read();

// 	Wire.beginTransmission(MAG_I2C_ADD);
// 	Wire.write(0x09);
// 	Wire.endTransmission();
// 	Wire.requestFrom(MAG_I2C_ADD, 1);
// 	Serial.println(Wire.read(), BIN);

// 	xm = reading_to_float(xl, xh, mag_factor_xy);
// 	ym = reading_to_float(yl, yh, mag_factor_xy);
// 	zm = reading_to_float(zl, zh, mag_factor_z);
// 	Serial.println();
// }