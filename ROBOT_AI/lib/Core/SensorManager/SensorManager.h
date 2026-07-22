#ifndef SENSOR_MANAGER_H
#define SENSOR_MANAGER_H

#include <Arduino.h>

struct SensorData {
    float roll = 0.0f, pitch = 0.0f, yaw = 0.0f;
    float accel_x = 0.0f, accel_y = 0.0f, accel_z = 0.0f;
    float gyro_x = 0.0f, gyro_y = 0.0f, gyro_z = 0.0f;
    float front_distance = 450.0f, rear_distance = 450.0f;
    long encoder_fl = 0, encoder_fr = 0, encoder_rl = 0, encoder_rr = 0;
    float battery_voltage = 12.0f;
};

class SensorManager {
private:
    SensorData _currentData;
    SensorManager() = default;

public:
    static SensorManager& getInstance() {
        static SensorManager instance;
        return instance;
    }

    void publishIMU(float r, float p, float y, float ax, float ay, float az, float gx, float gy, float gz) {
        _currentData.roll = r;
        _currentData.pitch = p;
        _currentData.yaw = y;
        _currentData.accel_x = ax;
        _currentData.accel_y = ay;
        _currentData.accel_z = az;
        _currentData.gyro_x = gx;
        _currentData.gyro_y = gy;
        _currentData.gyro_z = gz;
    }

    void publishUltrasonic(float front, float rear) {
        _currentData.front_distance = front;
        _currentData.rear_distance = rear;
    }

    void publishEncoders(long fl, long fr, long rl, long rr) {
        _currentData.encoder_fl = fl;
        _currentData.encoder_fr = fr;
        _currentData.encoder_rl = rl;
        _currentData.encoder_rr = rr;
    }

    void publishBattery(float voltage) {
        _currentData.battery_voltage = voltage;
    }

    const SensorData& getSensorData() const {
        return _currentData;
    }
};

#endif // SENSOR_MANAGER_H
