#include <IMU.h>
#include <Wire.h>
#include <L298n.h>

#define pwma 2
#define fora 3
#define reva 4
#define pwmb 5
#define forb 6
#define revb 7

L298n* motor_driver;

IMU* imu;

void setup() {
    motor_driver = new L298n(pwma, fora, reva, pwmb, forb, revb);
    motor_driver->set(0, 0);

    Serial.begin(921600);
    while (!Serial);

    Wire.begin();
	Wire.setClock(400000);
    
    imu = new IMU();
    GyrSetting settings;
    settings.scale = GyrSetting::GYR_250DPS;
    settings.data_rate = GyrSetting::GYR_800HZ;
    settings.fifo_enabled = GyrSetting::ENABLED;
    settings.fifo_mode = GyrSetting::BYPASS; // must enter bypass mode first to reset the FIFO buffer
    imu->apply_gyr_settings(settings);

    settings.fifo_mode = GyrSetting::FIFO;
    imu->apply_gyr_settings(settings);
}

void loop() {
    size_t n_read = imu->read_gyro();
    while (imu->gyr_buffer.size() > 0) {
        xyz_reading_raw reading = imu->gyr_buffer.pop();
        Serial.print(reading.x); 
        Serial.print(" ");
        Serial.print(reading.y); 
        Serial.print(" ");
        Serial.print(reading.z); 
        Serial.println(" ");
    }
}