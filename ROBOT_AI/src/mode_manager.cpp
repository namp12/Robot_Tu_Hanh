/**
 * @file mode_manager.cpp
 * @brief Implementations cho phân hệ Quản lý Chế độ ModeManager.
 */

#include "mode_manager.h"
#include "robot_global.h"
#include "safety.h"

ModeManager& ModeManager::getInstance() {
    static ModeManager instance;
    return instance;
}

ModeManager::ModeManager() : _currentMode(MODE_MANUAL) {}

void ModeManager::init(OperatingMode defaultMode) {
    setMode(defaultMode);
}

bool ModeManager::setMode(OperatingMode mode) {
    if (_currentMode == mode) return true;

    _currentMode = mode;
    currentMode = mode; // Đồng bộ biến toàn cục

    Serial.printf("📢 [ModeManager] Đã chuyển sang Chế độ: %s\n", getModeString());

    if (mode == MODE_AUTO) {
        autoModeStartTime = millis();
        auto_run_ResetState();
    } else if (mode == MODE_MANUAL || mode == MODE_ROS) {
        car.stop();
    }
    SafetyMonitor::getInstance().clearEmergencyStop();
    return true;
}

bool ModeManager::setModeFromString(const String& modeStr) {
    String str = modeStr;
    str.trim();
    str.toLowerCase();

    if (str == "manual" || str == "man" || str == "m" || str == "1") {
        return setMode(MODE_MANUAL);
    } else if (str == "auto" || str == "run" || str == "a" || str == "2") {
        return setMode(MODE_AUTO);
    } else if (str == "ros" || str == "ros2" || str == "r" || str == "3") {
        return setMode(MODE_ROS);
    }
    return false;
}

const char* ModeManager::getModeString() const {
    switch (_currentMode) {
        case MODE_MANUAL: return "MODE_MANUAL";
        case MODE_AUTO:   return "MODE_AUTO";
        case MODE_ROS:    return "MODE_ROS";
        default:          return "UNKNOWN";
    }
}
