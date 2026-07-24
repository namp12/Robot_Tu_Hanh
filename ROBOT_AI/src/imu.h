/**
 * @file imu.h
 * @brief Phân hệ đọc Cảm biến Góc nghiêng / Gia tốc MPU6050 IMU.
 */

#ifndef IMU_MODULE_H
#define IMU_MODULE_H

#include <Arduino.h>

class ImuModule {
public:
    static ImuModule& getInstance();

    bool begin(uint8_t sda = 18, uint8_t scl = 19);
    void update();

    bool isOnline() const;
    float getYaw();
    float getRoll();
    float getPitch();
    float getGyroZ();

    void resetYaw();

private:
    ImuModule();
};

#endif // IMU_MODULE_H
