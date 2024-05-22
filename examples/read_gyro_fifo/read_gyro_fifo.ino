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
    if (n_read > 0 && firstReading) {
        firstReading = false;
        t_start = micros();
    } else {
        total_read += n_read;
        Serial.println((double)(total_read * 1000000) / (double)(micros() - t_start));
    }
    while (imu->gyr_buffer.size() > 0) {
        xyz_reading_raw reading = imu->gyr_buffer.pop();
        // Serial.print(reading.x); 
        // Serial.print(" ");
        // Serial.print(reading.y); 
        // Serial.print(" ");
        // Serial.print(reading.z); 
        // Serial.println(" ");
    }
}