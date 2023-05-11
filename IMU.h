// for board 9-Axis IMU L3GD20 LSM303D Module 9DOF Compass Acceleration Gyroscope For Arduino (lies about the gyro being used)
// with LSM303DLHC accelerometer/magnetometer and L3G4200D gyro

#include <Arduino.h>

// i2c addresses for accelerometer, magnetometer, and gyro
#define ACC_I2C_ADD 0b0011001
#define MAG_I2C_ADD 0b0011110
#define GYR_I2C_ADD 0b1101001

// i2c registers and settings for accelerometer
#define ACC_CTRL_REG1_A 0X20
#define ACC_DATA_RATE_1344_5376HZ (0b1001 << 4)
#define ACC_NORMAL_POWER_MODE (0b0 << 3)
#define ACC_LOW_POWER_MODE (0b1 << 3)
#define ACC_ENABLE_XYZ 0b111 // bit 6-8

#define ACC_CTRL_REG3_A 0X22
#define ACC_I1_DRDY1 (0b1 << 4)

#define ACC_CTRL_REG4_A 0X23
#define ACC_SCALE_2g (0b00 << 4)

#define ACC_STATUS_REG_A 0x27 // next 6 registers are xl, xh, yl, yh, zl, zh

// i2c registers and settings for magnetometer
#define MAG_CRA_REG_M 0x00
#define MAG_OUTPUT_220 0b111 << 2

#define MAG_CRB_REG_M 0x01
#define MAG_RANGE_1_3 (0b001 << 5)

#define MAG_MR_REG_M 0x02
#define MAG_CONTINUOUS_MODE 0b00

#define MAG_XH 0x03 // next 6 registers are xl, zh, zl, yh, yl

// i2c registers for gyro
#define GYR_CTRL_REG1 0x20
#define GYR_DATA_RATE_800HZ (0b11 << 6) // also determines the cutoff frequency for the 1st low pass filter
#define GYR_LPF2_CUTOFF_110HZ (0b11 << 4) // << 4 2nd low pass filter cutoff frequency for 800Hz data rate (called bandwidth in datasheet)
#define GYR_ENABLE_XYZ 0b1111

#define GYR_CTRL_REG3 0x22
#define GYR_I2_DRDY (0b1 << 3)

#define GYR_CTRL_REG4 0x23
#define GYR_SCALE_250DPS (0b00 << 4) // degrees per second

#define GYR_STATUS_REG 0x27 // next 6 registers are xl, xh, yl, yh, zl, zh

class IMU
{
	private:
		// factors for converting readings to 
		float acc_factor;
		float gyro_factor;
		float mag_factor_xy;
		float mag_factor_z;
		uint8_t gyr_status;
		uint8_t acc_status;
		void init_accelerometer(float local_g_acc, uint8_t mode);
		void init_gyro();
		void init_magnetometer();
	public:
		IMU(float local_g_acc = 9.8);
		float xa, ya, za; // xyz accelerometer readings (in meters per second^2)
		float xw, yw, zw; // xyz gyroscope angular velocity readings (in degrees per second)
		float xm, ym, zm; // xyz magnetometer readings
		bool read_accelerometer(); // returns whether there was a new reading
		bool read_gyro(); // returns whether there was as new reading
		void read_magnetometer(); // no return value because the magnetometer outputs a data ready line
};