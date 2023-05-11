#include <IMU.h>
#include <Wire.h>

#define LIN1 34 // interrupt 1 configured to give a data ready signal for accelerometer
#define LRDY 32 // lrdy tells when magnetometer has new data
#define GRDY 39 // gyro data ready signal

IMU* imu;

int n = 0;

void setup() {
  Serial.begin(115200);
  while (!Serial);
  imu = new IMU();

  // interrupt pins shouldn't actually be used as interrupts
  // Wire library used for i2c messes with interrupts
  pinMode(LIN1, INPUT_PULLDOWN);
  pinMode(LRDY, INPUT_PULLDOWN);
  pinMode(GRDY, INPUT_PULLDOWN);
}



void loop() {
  // int new_acc_val = digitalRead(LIN1);
  
  // if (new_acc_val) {
  //   imu->read_accelerometer();
  //   Serial.print("Accelerometer xyz: ");
  //   Serial.print(imu->xa, 3);
  //   Serial.print(" ");
  //   Serial.print(imu->ya, 3);
  //   Serial.print(" ");
  //   Serial.print(imu->za, 3);
  //   Serial.println("");
  // }

  // int new_mag_val = digitalRead(LRDY);

  // if (new_mag_val) {
  //   imu->read_magnetometer();
  //   // Serial.print("Magnetometer xyz: ");
  //   // Serial.print(imu->xm, 3);
  //   // Serial.print(" ");
  //   // Serial.print(imu->ym, 3);
  //   // Serial.print(" ");
  //   // Serial.print(imu->zm, 3);
  //   // Serial.println("");
  // } else {
  //   n += 1;
  //   Serial.print("--------------------------- ");
  //   Serial.println(n);
  // }

  // Wire.beginTransmission(ACC_I2C_ADD);
	// Wire.write(INT1_SRC_A);
	// Wire.endTransmission();
  // Wire.requestFrom(ACC_I2C_ADD, 1);

  // uint8_t val = Wire.read();
  // // Serial.print(micros());
  // // Serial.print(" ");
  // Serial.println(0b100000000 | val, BIN);

  // delay(500);


  bool new_mag_reading = digitalRead(GRDY);
  if (new_mag_reading) {
    imu->read_gyro();
    Serial.print(imu->xw, 3);
    Serial.print(" ");
    Serial.print(imu->yw, 3);
    Serial.print(" ");
    Serial.print(imu->zw, 3);
    Serial.println();
  } else {
    Serial.println();
  }
  
  // // accelerometer
  // 
  // // gyro
  // 

  // delay(500);
}
