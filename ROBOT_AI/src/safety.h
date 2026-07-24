/**
 * @file safety.h
 * @brief Phân hệ Giám sát An toàn Hệ thống Cao cấp (System Safety Module).
 * Kiểm tra quyền di chuyển (Forward, Backward, Strafe, Rotate) trước khi Motion Controller xuất PWM.
 * Luôn hoạt động ở 100% các mode (MANUAL, AUTO, ROS) với độ ưu tiên cao nhất.
 */

#ifndef SAFETY_H
#define SAFETY_H

#include <Arduino.h>

class SafetyMonitor {
public:
    static SafetyMonitor& getInstance();

    void init();
    void update();

    /**
     * @brief Reset Serial Watchdog timer khi nhận được bất kỳ lệnh hợp lệ nào từ Host/Serial/ROS.
     */
    void feedWatchdog();

    void emergencyStop(const char* reason = "Emergency Stop Triggered");
    void clearEmergencyStop();

    bool isEmergencyStop() const { return _isEmergency; }
    const char* getSafetyReason() const { return _reason; }

    // API Kiểm tra quyền di chuyển cho Motion Controller trước khi xuất PWM:
    bool canMoveForward() const;
    bool canMoveBackward() const;
    bool canStrafe() const;
    bool canRotate() const;

private:
    SafetyMonitor();

    unsigned long _lastCmdTime;
    bool _isEmergency;
    const char* _reason;

    bool _allowForward;
    bool _allowBackward;
    bool _allowStrafe;
    bool _allowRotate;
};

#endif // SAFETY_H
