// for board 9-Axis IMU L3GD20 LSM303D Module 9DOF Compass Acceleration Gyroscope For Arduino (lies about the gyro being used)
// with LSM303DLHC accelerometer/magnetometer and L3G4200D gyro

#include <Wire.h>

// i2c addresses for accelerometer, magnetometer, and gyro
#define ACC_I2C_ADD 0b0011001
#define MAG_I2C_ADD 0b0011110
#define GYR_I2C_ADD 0b1101001

// i2c registers for accelerometer
#define ACC_CTRL_REG1_A 0X20
#define ACC_CTRL_REG4_A 0X23
#define ACC_STATUS_REG_A 0x27 // next 6 registers are xl, xh, yl, yh, zl, zh

// configuration for accelerometer
#define ACC_DATA_RATE_1344_5376HZ 0b1001
#define ACC_LOW_POWER_MODE 0b1
#define ACC_SCALE_2g 0b00 // g is gravitational acceleration

// i2c registers for gyro
#define GYR_CTRL_REG1 0x20
#define GYR_CTRL_REG4 0x23
#define GYR_STATUS_REG 0x27 // next 6 registers are xl, xh, yl, yh, zl, zh

// configuration for gyro
#define GYR_DATA_RATE_800HZ 0b11 // also determines the cutoff frequency for the 1st low pass filter
#define GYR_LPF2_CUTOFF_110HZ 0b11 // 2nd low pass filter cutoff frequency for 800Hz data rate (called bandwidth in datasheet)
#define GYR_SCALE_250DPS 0b00 // degrees per second

class IMU
{
	private:
		// factors for converting readings to 
		float acc_factor;
		float gyro_factor;
	public:
		IMU();
		float xa, ya, za; // xyz accelerations (in meters per second^2)
		float xw, yw, zw; // xyz angular velocities (in degrees per second)
		void init_sensors(float local_g_acc = 9.8);
		bool read_accelerometer(); // returns whether reading is new or a repeat of previous reading
		bool read_gyro(); // returns whether reading is new or a repeat of previous reading
}