#include "IMUManager.h"

IMUManager::IMUManager() : _isOnline(false) {}

bool IMUManager::begin(uint8_t sda, uint8_t scl) {
    _isOnline = _mpu.begin(sda, scl);
    return _isOnline;
}

void IMUManager::update() {
    if (_isOnline) {
        _mpu.update();
    }
}
