#import <IMU.h>

const float xw_offset = -1.47698;
const float yw_offset = 0.60528;
const float zw_offset = 0.62949;

IMU* imu;
float count = 0;
double xw_sum, yw_sum, zw_sum = 0;

void setup() {
  Serial.begin(115200);
  while (!Serial);
  imu = new IMU();
}

void loop() {
  imu->read_accelerometer();
  imu->read_gyro();
  // accelerometer
//  Serial.print(imu->xa, 3);
//  Serial.print(" ");
//  Serial.print(imu->ya, 3);
//  Serial.print(" ");
//  Serial.print(imu->za, 3);
  // gyro
  count += 1.0;
  xw_sum += imu->xw;
  yw_sum += imu->yw;
  zw_sum += imu->zw;
  Serial.print((xw_sum / count) - xw_offset, 5);
  Serial.print(" ");
  Serial.print((yw_sum / count) - yw_offset, 5);
  Serial.print(" ");
  Serial.print((zw_sum / count) - zw_offset, 5);

  Serial.println(" ");
}
