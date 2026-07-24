/**
 * @file auto_run.cpp
 * @brief Phân hệ chạy tự động (Auto Mode) nâng cao cho Robot Mecanum.
 *        Được tách lớp theo kiến trúc Decoupling (SOLID) thông qua AutoNavigator & ObstacleAvoidance.
 */

#include "robot_global.h"
#include "Config.h"
#include "AutoNavigator.h"

const char* auto_run_GetStateName(AutoState state) {
    switch (state) {
        case AUTO_IDLE:          return "AUTO_IDLE";
        case AUTO_FORWARD:       return "AUTO_FORWARD";
        case AUTO_SLOW_FORWARD:  return "AUTO_SLOW_FORWARD";
        case AUTO_STOP:          return "AUTO_STOP";
        case AUTO_BACKWARD:      return "AUTO_BACKWARD";
        case AUTO_SCAN:          return "AUTO_SCAN";
        case AUTO_ROTATE_LEFT:   return "AUTO_ROTATE_LEFT";
        case AUTO_ROTATE_RIGHT:  return "AUTO_ROTATE_RIGHT";
        case AUTO_RECOVER:       return "AUTO_RECOVER";
        default:                 return "UNKNOWN";
    }
}

void auto_run_Init() {
    AutoNavigator::getInstance().begin();
}

void auto_run_ResetState() {
    AutoNavigator::getInstance().reset();
}

void auto_run_ProcessPiCommand(const String& cmd) {
    // Xử lý mở rộng cho Raspberry Pi nếu có
}

void auto_run_Update() {
    AutoNavigator::getInstance().update();
}