// for board 9-Axis IMU L3GD20 LSM303D Module 9DOF Compass Acceleration Gyroscope For Arduino (lies about the gyro being used)
// with LSM303DLHC accelerometer/magnetometer and L3G4200D gyro

#include <Arduino.h>

// i2c addresses for accelerometer, magnetometer, and gyro
const uint8_t ACC_I2C_ADD =  0b0011001;
const uint8_t MAG_I2C_ADD = 0b0011110;
const uint8_t GYR_I2C_ADD = 0b1101001;

// msb set to 1 indicates to device to auto increment the register address on each read call
const uint8_t read_multiple_bytes = 1 << 7;

// i2c registers and settings for accelerometer
const uint8_t ACC_CTRL_REG1_A = 0X20;
const uint8_t ACC_ENABLE_XYZ = 0b111;
const uint8_t ACC_CTRL_REG3_A = 0X22;
const uint8_t ACC_CTRL_REG4_A = 0X23;
const uint8_t ACC_CTRL_REG5_A = 0x24;
const uint8_t ACC_STATUS_REG_A = 0x27;
const uint8_t ACC_DATA_REG = 0x28 | read_multiple_bytes;
const uint8_t ACC_FIFO_CTRL_REG_A = 0x2e;
const uint8_t ACC_FIFO_SRC_REG_A = 0x2f;

struct AccSetting {
	enum {
		ACC_2g = 0b00 << 4,
		ACC_4g = 0b01 << 4,
		ACC_8g = 0b10 << 4,
		ACC_16g = 0b11 << 4
	} scale = AccSetting::ACC_2g;
	enum {
		POWER_DOWN = 0b0000 << 4,
		ACC_1HZ = 0b0001 << 4,
		ACC_10HZ = 0b0010 << 4,
		ACC_25HZ = 0b0011 << 4,
		ACC_50HZ = 0b0100 << 4,
		ACC_100HZ = 0b0101 << 4,
		ACC_200HZ = 0b0110 << 4,
		ACC_400HZ = 0b0111 << 4,
		ACC_1620HZ = 0b1000 << 4,
		ACC_1344_5376HZ = 0b1001 << 4 // 1344 Hz in NORMAL power mode and 5376 Hz in LOW_POWER mode
	} data_rate = ACC_1620HZ;
	enum {
		NORMAL = 0b0 << 3,
		LOW_POWER = 0b1 << 3
	} power_mode = AccSetting::NORMAL;
	enum {
		HIGH_RESOLUTION = 1 << 3,
		LOW_RESOLUTION = 0 << 3,
	} resolution_mode = AccSetting::HIGH_RESOLUTION;
	enum {
		DISABLED = 0 << 6,
		ENABLED = 1 << 6
	} fifo_enabled = AccSetting::DISABLED;
	enum AccFIFOMode {
		BYPASS = 0b00 << 6,
		FIFO = 0b01 << 6,
		STREAM = 0b10 << 6,
		TRIGGER = 0b11 << 6
	} fifo_mode = AccSetting::FIFO;
	struct {
		bool click = false;
		bool and_or_interrupt1 = false;
		bool and_or_interrupt2 = false;
		bool data_ready1 = false;
		bool data_ready2 = false;
		bool fifo_watermark = false;
		bool fifo_overrun = false;
	} int1_special_config;
	uint8_t fifo_watermark = 0; // between 0 and 32 inclusive
};

// // i2c registers and settings for magnetometer
// #define MAG_CRA_REG_M 0x00
// #define MAG_OUTPUT_220 0b111 << 2

// #define MAG_CRB_REG_M 0x01
// #define MAG_RANGE_1_3 (0b001 << 5)

// #define MAG_MR_REG_M 0x02
// #define MAG_CONTINUOUS_MODE 0b00

// #define MAG_XH 0x03 // next 6 registers are xl, zh, zl, yh, yl

// i2c registers and settings for gyro
const uint8_t GYR_CTRL_REG1 = 0x20;
const uint8_t GYR_ENABLE_XYZ = 0b1111;
const uint8_t GYR_CTRL_REG4 = 0x23;
const uint8_t GYR_CTRL_REG5 = 0x24;
const uint8_t GYR_STATUS_REG = 0x27;
const uint8_t GYR_DATA_REG = 0x28 | read_multiple_bytes;
const uint8_t GYR_FIFO_CTRL_REG = 0x2e;
const uint8_t GYR_FIFO_SRC_REG = 0x2f;

struct GyrSetting {
	enum {
		GYR_250DPS = 0b00 << 4,
		GYR_500DPS = 0b01 << 4,
		GYR_2000DPS = 0b10 << 4
	} scale = GyrSetting::GYR_250DPS;
	enum {
		GYR_100HZ = 0b00 << 6,
		GYR_200HZ = 0b01 << 6,
		GYR_400HZ = 0b10 << 6,
		GYR_800HZ = 0b11 << 6
	} data_rate = GyrSetting::GYR_800HZ;
	enum {
		NO_EXTRA_FILTER = 0b00000,
		HIGH_PASS = 0b10001,
		LOW_PASS2 = 0b00010,
		HIGH_PASS_AND_LOWPASS2 = 0b10011
	} output_selection = GyrSetting::NO_EXTRA_FILTER;
	enum {
		DISABLED = 0 << 6,
		ENABLED = 1 << 6
	} fifo_enabled = GyrSetting::DISABLED;
	enum {
		BYPASS = 0b000 << 5,
		FIFO = 0b001 << 5,
		STREAM = 0b010 << 5,
		STREAM_TO_FIFO = 0b011 << 5,
		BYPASS_TO_STREAM = 0b100 << 5
	} fifo_mode = GyrSetting::FIFO;
	uint8_t fifo_watermark = 0; // between 0 and 32 inclusive
};

const float GRAVITY_CONST = 9.80665;

struct xyz_reading_raw {
	int16_t x;
	int16_t y;
	int16_t z;
};
struct xyz_reading {
	float x;
	float y;
	float z;
};
class RingBuffer
{
	private:
		size_t push_index = 0;
		size_t pop_index = 0;
		xyz_reading_raw* buffer;
		size_t max_size;
	public:
		RingBuffer(size_t max_size = 0);
		size_t size();
		void push(xyz_reading_raw reading);
		xyz_reading_raw pop();
};

class IMU
{
	private:
		AccSetting acc_settings;
		GyrSetting gyr_settings;

		static void write_byte(uint8_t addr, uint8_t reg, uint8_t data);
		static uint8_t read_byte(uint8_t addr, uint8_t reg);
		static void read_bytes(uint8_t addr, uint8_t reg, uint8_t* data, uint8_t n);
		static int16_t data_to_int16(uint8_t lower_byte, uint8_t higher_byte);
	public:
		static xyz_reading raw_reading_to_float(xyz_reading_raw raw_reading, float factor=1);
		float acc_factor; // conversion factor to get meters per second^2 from raw reading
		float gyr_factor; // conversion factor to get degrees per second from raw reading
		RingBuffer acc_buffer;
		RingBuffer gyr_buffer;

		IMU(size_t acc_buffer_size = 1000, size_t gyr_buffer_size = 1000);
		void apply_acc_settings(AccSetting settings);
		void apply_gyr_settings(GyrSetting settings);

		size_t read_accelerometer(); // returns number of items read
		size_t read_gyro(); // returns number of items read
		// void read_magnetometer(); // no return value because the magnetometer outputs a data ready line
};