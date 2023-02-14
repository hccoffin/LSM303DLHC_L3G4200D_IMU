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
#define ACC_DATA_RATE_1HZ 0b0001
#define ACC_DATA_RATE_400HZ 0b0111
#define ACC_DATA_RATE_1344_5376HZ 0b1001
#define ACC_LOW_POWER_MODE 0b1
#define ACC_NORMAL_MODE 0b0
#define ACC_SCALE_2g 0b00 // g is gravitational acceleration

// i2c registers for gyro
#define WHO_AM_I 0x0F
#define GYR_CTRL_REG1 0x20
#define GYR_CTRL_REG4 0x23
#define GYR_STATUS_REG 0x27 // next 6 registers are xl, xh, yl, yh, zl, zh


// configuration for gyro
#define GYR_DATA_RATE_800HZ 0b11 // also determines the cutoff frequency for the 1st low pass filter
#define GYR_LPF2_CUTOFF_110HZ 0b11 // 2nd low pass filter cutoff frequency for 800Hz data rate (called bandwidth in datasheet)
#define GYR_SCALE_250DPS 0b00 // degrees per second

void init_accelerometer(uint8_t data_rate, uint8_t mode, uint8_t scale) {
  Wire.beginTransmission(ACC_I2C_ADD);
  Wire.write(ACC_CTRL_REG1_A);
  Wire.write((data_rate << 4) | (mode << 3) | 0b111); // 0b111 enables xyz axes
  Wire.endTransmission();

  Wire.beginTransmission(ACC_I2C_ADD);
  Wire.write(ACC_CTRL_REG4_A);
  Wire.write((scale << 4) | ((1-mode) << 3)); // 1 - mode is just to invert mode
  Wire.endTransmission();
}

void init_gyro(uint8_t data_rate, uint8_t lpf2_cutoff, uint8_t scale) {
  Wire.beginTransmission(GYR_I2C_ADD);
  Wire.write(GYR_CTRL_REG1);
  Wire.write((data_rate << 6) | (lpf2_cutoff << 4) | 0b1111); // 0b1111 turns on and enables xyz axes
  Wire.endTransmission();

  Wire.beginTransmission(GYR_I2C_ADD);
  Wire.write(GYR_CTRL_REG4);
  Wire.write(scale << 4);
  Wire.endTransmission();
}

void setup() {
  Serial.begin(115200);
  while(!Serial);

  Wire.begin();
  Wire.setClock(400000);
  init_accelerometer(ACC_DATA_RATE_1344_5376HZ, ACC_LOW_POWER_MODE, ACC_SCALE_2g);
  init_gyro(GYR_DATA_RATE_800HZ, GYR_LPF2_CUTOFF_110HZ, GYR_SCALE_250DPS);
}

float reading_to_float(uint8_t lower_byte, uint8_t higher_byte) {
  uint16_t temp = ((uint16_t)higher_byte << 8) | lower_byte;
  int16_t intval;
  memcpy(&intval, &temp, 2);
  return ((float)intval);
}

void read_accelerometer() {
  long t_start = micros();
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
      
    float x = reading_to_float(xl, xh) / 1671.961; // divide by 2^15 / (2g * 9.79927 m/gs^2) to get in units in m/s^2
    float y = reading_to_float(yl, yh) / 1671.961;
    float z = reading_to_float(zl, zh) / 1671.961;
  
    long micros_taken = micros() - t_start;
    Serial.print("Accelerometer ");
    Serial.print(1000000.0 / (float)micros_taken);
    Serial.print(" ");

    Serial.print(t_start / 1000);
    Serial.print(" ");
    Serial.print(micros_taken);
    Serial.print(" ");
    Serial.print(acc_status, BIN);
    Serial.print(" ");
    Serial.print("xyz ");
    Serial.print(x, 4);
    Serial.print(" ");
    Serial.print(y, 4);
    Serial.print(" ");
    Serial.println(z, 4);
  } else {
    Serial.println("No new data from accelerometer");
  }
}

void read_gyro() {
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
      
    float x = reading_to_float(xl, xh) / 131.072; // divide by 2^15 / 250 dps to get in units in dps since scale is 250 dps
    float y = reading_to_float(yl, yh) / 131.072;
    float z = reading_to_float(zl, zh) / 131.072;
  
    long micros_taken = micros() - t_start;
    Serial.print("Gyro ");
    Serial.print(1000000.0 / (float)micros_taken);
    Serial.print(" ");
    Serial.print(t_start / 1000);
    Serial.print(" ");
    Serial.print(micros_taken);
    Serial.print(" ");
    Serial.print(gyr_status, BIN);
    Serial.print(" ");
    Serial.print("xyz ");
    Serial.print(x, 4);
    Serial.print(" ");
    Serial.print(y, 4);
    Serial.print(" ");
    Serial.println(z, 4);
  } else {
    Serial.println("No new data from gyro");
  }
}

void loop() {
//  read_accelerometer();
  read_gyro();
}
