/**
 * @file safety.cpp
 * @brief Implementations cho Phân hệ Giám sát An toàn Hệ thống Cao cấp (SafetyMonitor).
 */

#include "safety.h"
#include "parameters.h"
#include "mode_manager.h"
#include "distance.h"
#include "battery.h"
#include "imu.h"
#include "robot_global.h"
#include "MovementController.h"

SafetyMonitor& SafetyMonitor::getInstance() {
    static SafetyMonitor instance;
    return instance;
}

SafetyMonitor::SafetyMonitor()
    : _lastCmdTime(0),
      _isEmergency(false),
      _reason("None"),
      _allowForward(true),
      _allowBackward(true),
      _allowStrafe(true),
      _allowRotate(true) {
}

void SafetyMonitor::init() {
    _lastCmdTime = millis();
    _isEmergency = false;
    _reason = "None";
    _allowForward = true;
    _allowBackward = true;
    _allowStrafe = true;
    _allowRotate = true;
}

void SafetyMonitor::feedWatchdog() {
    _lastCmdTime = millis();
    if (_isEmergency && strcmp(_reason, "Low Battery Cutoff") != 0) {
        _isEmergency = false;
        _reason = "None";
        _allowForward = true;
        _allowBackward = true;
        _allowStrafe = true;
        _allowRotate = true;
    }
}

void SafetyMonitor::emergencyStop(const char* reason) {
    if (!_isEmergency) {
        _isEmergency = true;
        _reason = reason;
        _allowForward = false;
        _allowBackward = false;
        _allowStrafe = false;
        _allowRotate = false;
        car.stop();
        moveControl.stop();
        Serial.printf("🛑 [SafetyMonitor] PHANH KHẨN CẤP! Lý do: %s\n", reason);
    }
}

void SafetyMonitor::clearEmergencyStop() {
    _isEmergency = false;
    _reason = "None";
    _lastCmdTime = millis();
    _allowForward = true;
    _allowBackward = true;
    _allowStrafe = true;
    _allowRotate = true;
    Serial.println(F("✅ [SafetyMonitor] Đã giải phóng Phanh Khẩn Cấp."));
}

bool SafetyMonitor::canMoveForward() const {
    if (_isEmergency) return false;
    return _allowForward;
}

bool SafetyMonitor::canMoveBackward() const {
    if (_isEmergency) return false;
    return _allowBackward;
}

bool SafetyMonitor::canStrafe() const {
    if (_isEmergency) return false;
    return _allowStrafe;
}

bool SafetyMonitor::canRotate() const {
    if (_isEmergency) return false;
    return _allowRotate;
}

void SafetyMonitor::update() {
    unsigned long now = millis();
    const SystemParameters& params = ParameterManager::getInstance().getParams();

    // 1. Kiểm tra ngắt khẩn cấp khi Pin quá yếu
    float batVolts = BatteryModule::getInstance().getVoltage();
    if (batVolts < params.minBatteryVoltage && batVolts > 5.0f) {
        emergencyStop("Low Battery Cutoff");
        return;
    }

    // 2. Kiểm tra lỗi phần cứng IMU - cảnh báo nhưng không khóa ngắt phanh khẩn cấp để đảm bảo điều khiển bánh xe
    bool imuOnline = ImuModule::getInstance().isOnline();
    if (!imuOnline) {
        static unsigned long lastImuWarn = 0;
        if (now - lastImuWarn > 3000) {
            lastImuWarn = now;
            Serial.println(F("⚠️ [SafetyMonitor] IMU offline - Tiếp tục cho phép di chuyển"));
        }
    }

    // 3. Serial Watchdog Timeout ở MODE_ROS: Dừng xe tạm thời khi ngắt lệnh, tự động di chuyển lại khi có cmd_vel mới
    if (ModeManager::getInstance().getMode() == MODE_ROS) {
        if (now - _lastCmdTime > params.cmdTimeoutMs) {
            car.stop();
            moveControl.stop();
            return;
        }
    }

    // Nếu đang ở trạng thái Phanh khẩn cấp kích hoạt thủ công -> cấm toàn bộ
    if (_isEmergency) {
        _allowForward = false;
        _allowBackward = false;
        _allowStrafe = false;
        _allowRotate = false;
        car.stop();
        return;
    }

    // 4. Phân tích khoảng cách Cảm biến Siêu âm né vật cản theo vùng quy định
    if (bypassSensorCheck || ModeManager::getInstance().getMode() == MODE_MANUAL) {
        // Trong chế độ MANUAL thủ công hoặc khi bật bypass, cho phép di chuyển 100% không bị kẹt bởi cảm biến
        _allowForward  = true;
        _allowBackward = true;
        _allowStrafe   = true;
        _allowRotate   = true;
        return;
    }

    float frontDist = DistanceModule::getInstance().getFrontDistance();
    float rearDist = DistanceModule::getInstance().getRearDistance();
    float safeDist = params.safeDistanceCm;

    bool frontBlocked = (frontDist > 0.0f && frontDist < safeDist);
    bool rearBlocked  = (rearDist > 0.0f && rearDist < safeDist);

    if (frontBlocked && rearBlocked) {
        // Nếu CẢ HAI HƯỚNG TRƯỚC VÀ SAU ĐỀU < SafeDistance -> Dừng hoàn toàn (STOP)
        _allowForward  = false;
        _allowBackward = false;
        _allowStrafe   = false;
        _allowRotate   = false;
        car.stop();
    } 
    else if (frontBlocked) {
        // Nếu Front < SafeDistance -> Cấm tiến, cho phép Lùi, Xoay, Đi ngang
        _allowForward  = false;
        _allowBackward = true;
        _allowStrafe   = true;
        _allowRotate   = true;
    } 
    else if (rearBlocked) {
        // Nếu Rear < SafeDistance -> Cấm lùi, cho phép Tiến, Xoay, Đi ngang
        _allowForward  = true;
        _allowBackward = false;
        _allowStrafe   = true;
        _allowRotate   = true;
    } 
    else {
        // An toàn 100% -> Cho phép tất cả các hướng
        _allowForward  = true;
        _allowBackward = true;
        _allowStrafe   = true;
        _allowRotate   = true;
    }
}
