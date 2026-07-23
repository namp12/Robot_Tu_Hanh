#ifndef TELEMETRY_MANAGER_H
#define TELEMETRY_MANAGER_H

#include <Arduino.h>

struct UnifiedTelemetry {
    unsigned long timestamp;
    const char* robotState;
    long encoderPulse[4];
    float encoderRPM[4];
    float odomX;
    float odomY;
    float odomTheta;
    float yaw;
    float pitch;
    float roll;
    float gyroX;
    float gyroY;
    float gyroZ;
    float accelX;
    float accelY;
    float accelZ;
    float frontDistance;
    float rearDistance;
    float batteryVoltage;
    float vx;
    float vy;
    float wz;
};

class TelemetryManager {
private:
    UnifiedTelemetry _telemetry;

    TelemetryManager();

public:
    static TelemetryManager& getInstance() {
        static TelemetryManager instance;
        return instance;
    }

    void update();
    const UnifiedTelemetry& getTelemetry() const { return _telemetry; }
};

#endif // TELEMETRY_MANAGER_H
