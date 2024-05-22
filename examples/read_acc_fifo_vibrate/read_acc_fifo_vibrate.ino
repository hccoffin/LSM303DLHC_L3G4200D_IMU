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
    motor_driver->set(255, 255);

    Serial.begin(921600);
    while (!Serial);

    Wire.begin();
	Wire.setClock(400000);
    
    imu = new IMU();
    AccSetting settings;
    settings.scale = AccSetting::ACC_2g;
    settings.data_rate = AccSetting::ACC_1344_5376HZ;
    settings.power_mode = AccSetting::NORMAL;
    settings.fifo_enabled = AccSetting::ENABLED;
    settings.fifo_mode = AccSetting::BYPASS; // must enter bypass mode to reset the FIFO buffer
    imu->apply_acc_settings(settings);

    settings.fifo_mode = AccSetting::FIFO;
    imu->apply_acc_settings(settings);
}

void loop() {
    size_t n_read = imu->read_accelerometer();
    while (imu->acc_buffer.size() > 0) {
        xyz_reading_raw reading = imu->acc_buffer.pop();
        Serial.print(reading.x); 
        Serial.print(" ");
        Serial.print(reading.y); 
        Serial.print(" ");
        Serial.print(reading.z); 
        Serial.println(" ");
    }
}