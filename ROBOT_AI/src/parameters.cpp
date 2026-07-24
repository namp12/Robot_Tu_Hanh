/**
 * @file parameters.cpp
 * @brief Định nghĩa và khởi tạo giá trị mặc định cho ParameterManager.
 */

#include "parameters.h"
#include "Config.h"

ParameterManager& ParameterManager::getInstance() {
    static ParameterManager instance;
    return instance;
}

ParameterManager::ParameterManager() {
    initDefaults();
}

void ParameterManager::initDefaults() {
    _params.wheelDiameter      = 0.08f;    // 80mm
    _params.wheelBaseX         = 0.15f;    // 150mm
    _params.wheelBaseY         = 0.15f;    // 150mm
    _params.gearRatio          = 30.0f;
    _params.ppr                = 11.0f;

    _params.maxLinearVelocityX  = 1.0f;     // m/s
    _params.maxLinearVelocityY  = 1.0f;     // m/s
    _params.maxAngularVelocityZ = 3.14f;    // rad/s (~180 deg/s)
    _params.maxPwmSpeed         = 255;
    _params.minPwmSpeed         = 65;
    _params.accelRampStep       = 10;

    _params.wheelKp            = 1.5f;
    _params.wheelKi            = 0.05f;
    _params.wheelKd            = 0.1f;

    _params.yawKp              = 1.2f;
    _params.yawKi              = 0.02f;
    _params.yawKd              = 0.4f;
    _params.yawPidMaxCorrection= 25;

    _params.safeDistanceCm     = 5.0f;     // Mặc định 5cm theo chuẩn ROS2 mới
    _params.slowDistanceCm     = 70.0f;
    _params.backDistanceCm     = 25.0f;
    _params.turnAngleDeg       = 90.0f;    // Quay 90 độ theo yêu cầu chuẩn ROS2
    _params.scanDelayMs        = 400;
    _params.recoveryLimit      = 3;

    _params.cmdTimeoutMs       = 500;      // 500ms ngắt kết nối cmd_vel
    _params.lowBatteryVoltage  = 10.5f;    // V (Cho pin 3S LiPo)
    _params.minBatteryVoltage  = 9.6f;     // V
}
