/**
 * @file AutoNavigator.h
 * @brief Điều phối chính luồng Chạy Tự Động (Auto Navigation Controller & State Machine).
 * Quản lý trạng thái, Ramp PWM, MPU Yaw PID, Encoder Feedback, Structured Logging và Telemetry Data.
 */

#ifndef AUTO_NAVIGATOR_H
#define AUTO_NAVIGATOR_H

#include <Arduino.h>
#include "Config.h"
#include "robot_global.h"
#include "ObstacleAvoidance.h"

/**
 * @brief Cấu trúc dữ liệu Telemetry cung cấp thông tin cho Raspberry Pi / WebSocket
 */
struct AutoTelemetryData {
    AutoState autoState;
    float frontDistance;
    float rearDistance;
    float leftScanDistance;
    float rightScanDistance;
    float currentYaw;
    float encoderDistance;
    AvoidanceDirection selectedDirection;
    bool obstacleDetected;
    uint8_t recoveryCount;
};

class AutoNavigator {
public:
    static AutoNavigator& getInstance();

    void begin();
    void update();
    void reset();

    void setState(AutoState newState);
    AutoState getState() const { return _currentState; }

    AutoTelemetryData getTelemetryData() const;

    ObstacleAvoidance& getAvoidanceEngine() { return _avoidanceEngine; }

    void printStatus();

private:
    AutoNavigator();
    ~AutoNavigator() = default;

    AutoNavigator(const AutoNavigator&) = delete;
    AutoNavigator& operator=(const AutoNavigator&) = delete;

    // Helper thực thi State Machine
    void handleStateIdle();
    void handleStateForward();
    void handleStateScan();
    void handleStateBackward();
    void handleStateRotateLeft();
    void handleStateRotateRight();
    void handleStateRecover();

    // Ramping & Control Helpers
    int updatePwmRamp(int targetSpeed);
    int calculateYawPidCorrection(float targetYaw, float currentYaw);
    float normalizeAngle(float angle);

    // Structured Logger & Telemetry Builder
    void logStructured(const char* eventMsg = nullptr);

    // Thành phần quản lý
    ObstacleAvoidance _avoidanceEngine;
    AutoState _currentState;

    // Các biến thời gian & theo dõi trạng thái non-blocking
    unsigned long _stateTimer;
    unsigned long _lastRampTime;
    unsigned long _lastLogTime;
    unsigned long _lastProgressTime;

    // Quản lý Ramp PWM & Tốc độ
    int _currentRampedSpeed;
    int _targetSpeed;

    // Các biến phục vụ quét góc MPU & Encoder
    uint8_t _scanStep;          ///< Bước thực thi trong AUTO_SCAN (0..7)
    float _scanStartHeading;    ///< Góc Yaw ban đầu trước khi quét
    float _scanLeftDistance;    ///< Khoảng cách đo ở góc 45° Trái
    float _scanRightDistance;   ///< Khoảng cách đo ở góc 45° Phải
    float _scanFrontDistance;   ///< Khoảng cách đo ở hướng Trước

    float _turnTargetHeading;   ///< Góc Yaw mục tiêu khi quay rẽ
    float _startEncoderPos;     ///< Vị trí Encoder ban đầu trước khi lùi

    // PID Variables cho Yaw Heading Hold
    float _pidIntegral;
    float _pidPreviousError;
    unsigned long _lastPidTime;

    AvoidanceDirection _selectedDirection;
};

#endif // AUTO_NAVIGATOR_H
