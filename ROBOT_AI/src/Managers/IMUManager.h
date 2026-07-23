#ifndef IMU_MANAGER_H
#define IMU_MANAGER_H

#include <Arduino.h>
#include "Mpu6050.h"

struct Vector3 {
    float x;
    float y;
    float z;
};

class IMUManager {
private:
    MPU6050Sensor _mpu;
    bool _isOnline;

    IMUManager();

public:
    static IMUManager& getInstance() {
        static IMUManager instance;
        return instance;
    }

    bool begin(uint8_t sda = 18, uint8_t scl = 19);
    void update();

    bool isOnline() const { return _isOnline; }
    float getYaw() { return _mpu.getYaw(); }
    float getPitch() { return _mpu.getPitch(); }
    float getRoll() { return _mpu.getRoll(); }
    
    Vector3 getGyro() {
        return {_mpu.getGyroX(), _mpu.getGyroY(), _mpu.getGyroZ()};
    }

    Vector3 getAccel() {
        return {_mpu.getAccelX(), _mpu.getAccelY(), _mpu.getAccelZ()};
    }

    void resetYaw() { _mpu.resetAngle(); }
};

#endif // IMU_MANAGER_H
