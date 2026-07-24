#include "IMUManager.h"
#include "robot_global.h"

IMUManager::IMUManager() : _isOnline(false) {}

bool IMUManager::begin(uint8_t sda, uint8_t scl) {
    _isOnline = mpu.begin(sda, scl);
    mpuOk = _isOnline;
    return _isOnline;
}

void IMUManager::update() {
    if (_isOnline) {
        mpu.update();
        mpuOk = true;
    }
}
