/**
 * @file mode_manager.h
 * @brief Phân hệ quản lý Chế độ vận hành xe (MODE_MANUAL, MODE_AUTO, MODE_ROS).
 */

#ifndef MODE_MANAGER_H
#define MODE_MANAGER_H

#include <Arduino.h>
#include "robot_global.h"

class ModeManager {
public:
    static ModeManager& getInstance();

    void init(OperatingMode defaultMode = MODE_MANUAL);
    
    bool setMode(OperatingMode mode);
    bool setModeFromString(const String& modeStr);

    OperatingMode getMode() const { return _currentMode; }
    const char* getModeString() const;

private:
    ModeManager();
    OperatingMode _currentMode;
};

#endif // MODE_MANAGER_H
