/**
 * @file imu.cpp
 * @brief Implementations cho ImuModule.
 */

#include "imu.h"
#include "robot_global.h"

ImuModule& ImuModule::getInstance() {
    static ImuModule instance;
    return instance;
}

ImuModule::ImuModule() {}

bool ImuModule::begin(uint8_t sda, uint8_t scl) {
    bool ok = mpu.begin(sda, scl);
    mpuOk = ok;
    return ok;
}

void ImuModule::update() {
    if (mpuOk) {
        mpu.update();
    }
}

bool ImuModule::isOnline() const {
    return mpuOk;
}

float ImuModule::getYaw() {
    return mpuOk ? mpu.getYaw() : 0.0f;
}

float ImuModule::getRoll() {
    return mpuOk ? mpu.getRoll() : 0.0f;
}

float ImuModule::getPitch() {
    return mpuOk ? mpu.getPitch() : 0.0f;
}

float ImuModule::getGyroZ() {
    return mpuOk ? mpu.getGyroZ() : 0.0f;
}

void ImuModule::resetYaw() {
    if (mpuOk) {
        mpu.resetAngle();
    }
}
