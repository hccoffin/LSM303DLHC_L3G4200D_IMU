#include <IMU.h>
#include <Wire.h>

IMU* imu;

bool firstReading = true;
uint64_t total_read = 0;
uint32_t t_start = 0;

void setup() {
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
    if (n_read > 0 && firstReading) {
        firstReading = false;
        t_start = micros();
    } else {
        total_read += n_read;
        Serial.println((double)(total_read * 1000000) / (double)(micros() - t_start));
    }
    while (imu->acc_buffer.size() > 0) {
        xyz_reading_raw reading = imu->acc_buffer.pop();
        // Serial.print(reading.x); 
        // Serial.print(" ");
        // Serial.print(reading.y); 
        // Serial.print(" ");
        // Serial.print(reading.z); 
        // Serial.println(" ");
    }

}