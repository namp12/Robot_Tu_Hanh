/**
 * @file SensorManager.h
 * @brief Class Quản lý Cảm biến Tập trung (SensorManager).
 * Tự động đọc dữ liệu IMU, Encoder, Cảm biến Siêu âm Phía trước/Sau & Pin 100% liên tục ở mọi mode.
 * Định kỳ phát tín hiệu Telemetry 20Hz lên Raspberry Pi / ROS2 / Web.
 */

#ifndef SENSOR_MANAGER_H
#define SENSOR_MANAGER_H

#include <Arduino.h>
#include "encoder.h"
#include "imu.h"
#include "distance.h"
#include "battery.h"

struct SensorData {
    float yaw;
    float roll;
    float pitch;
    float totalDistance;
    float frontDistance;
    float rearDistance;
    float batteryVoltage;
    uint8_t batteryPercentage;
    bool imuOnline;
    bool frontOnline;
    bool rearOnline;
};

class SensorManager {
public:
    static SensorManager& getInstance();

    void begin();
    void update();
    void sendData();

    const SensorData& getData() const { return _data; }

    void setTelemetryEnabled(bool enable) { _telemetryEnabled = enable; }
    bool isTelemetryEnabled() const { return _telemetryEnabled; }

private:
    SensorManager();

    SensorData _data;
    bool _telemetryEnabled;
    unsigned long _lastReadMs;
    unsigned long _lastSendMs;
};

#endif // SENSOR_MANAGER_H
