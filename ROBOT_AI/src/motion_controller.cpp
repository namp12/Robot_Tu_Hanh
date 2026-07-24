/**
 * @file motion_controller.cpp
 * @brief Implementations cho MotionController.
 */

#include "motion_controller.h"
#include "parameters.h"
#include "safety.h"

MotionController& MotionController::getInstance() {
    static MotionController instance;
    return instance;
}

MotionController::MotionController()
    : _motorDriver(nullptr), _kinematics(0.08f, 0.15f, 0.15f), _lastUpdateMs(0) {
    _targetVel = {0.0f, 0.0f, 0.0f};
    _currentVel = {0.0f, 0.0f, 0.0f};
}

void MotionController::begin(Motor* motorDriver) {
    _motorDriver = motorDriver;
    const SystemParameters& p = ParameterManager::getInstance().getParams();
    _kinematics.setGeometry(p.wheelDiameter, p.wheelBaseX, p.wheelBaseY);
    stop();
}

void MotionController::setTargetVelocity(float vx, float vy, float wz) {
    const SystemParameters& p = ParameterManager::getInstance().getParams();

    _targetVel.vx = constrain(vx, -p.maxLinearVelocityX, p.maxLinearVelocityX);
    _targetVel.vy = constrain(vy, -p.maxLinearVelocityY, p.maxLinearVelocityY);
    _targetVel.wz = constrain(wz, -p.maxAngularVelocityZ, p.maxAngularVelocityZ);
}

void MotionController::setManualCommand(const String& dir, int speed) {
    if (_motorDriver == nullptr) return;

    String d = dir;
    d.toUpperCase();
    d.trim();

    float speedRatio = (speed > 0) ? ((float)speed / 255.0f) : 0.6f;
    const SystemParameters& p = ParameterManager::getInstance().getParams();

    if (d == "FORWARD" || d == "TIEN" || d == "W") {
        setTargetVelocity(p.maxLinearVelocityX * speedRatio, 0.0f, 0.0f);
    } else if (d == "BACKWARD" || d == "LUI" || d == "S") {
        setTargetVelocity(-p.maxLinearVelocityX * speedRatio, 0.0f, 0.0f);
    } else if (d == "STRAFE_LEFT" || d == "TRAI" || d == "A") {
        setTargetVelocity(0.0f, -p.maxLinearVelocityY * speedRatio, 0.0f);
    } else if (d == "STRAFE_RIGHT" || d == "PHAI" || d == "D") {
        setTargetVelocity(0.0f, p.maxLinearVelocityY * speedRatio, 0.0f);
    } else if (d == "ROTATE_LEFT" || d == "XOAY_TRAI" || d == "Q") {
        setTargetVelocity(0.0f, 0.0f, -p.maxAngularVelocityZ * speedRatio);
    } else if (d == "ROTATE_RIGHT" || d == "XOAY_PHAI" || d == "E") {
        setTargetVelocity(0.0f, 0.0f, p.maxAngularVelocityZ * speedRatio);
    } else if (d == "DIAGONAL_FRONT_LEFT" || d == "CHEO_TT" || d == "Z") {
        setTargetVelocity(p.maxLinearVelocityX * 0.707f * speedRatio, -p.maxLinearVelocityY * 0.707f * speedRatio, 0.0f);
    } else if (d == "DIAGONAL_FRONT_RIGHT" || d == "CHEO_TP" || d == "C") {
        setTargetVelocity(p.maxLinearVelocityX * 0.707f * speedRatio, p.maxLinearVelocityY * 0.707f * speedRatio, 0.0f);
    } else if (d == "DIAGONAL_REAR_LEFT" || d == "CHEO_ST") {
        setTargetVelocity(-p.maxLinearVelocityX * 0.707f * speedRatio, -p.maxLinearVelocityY * 0.707f * speedRatio, 0.0f);
    } else if (d == "DIAGONAL_REAR_RIGHT" || d == "CHEO_SP") {
        setTargetVelocity(-p.maxLinearVelocityX * 0.707f * speedRatio, p.maxLinearVelocityY * 0.707f * speedRatio, 0.0f);
    } else {
        stop();
    }
}

void MotionController::stop() {
    _targetVel = {0.0f, 0.0f, 0.0f};
    _currentVel = {0.0f, 0.0f, 0.0f};
    if (_motorDriver != nullptr) {
        _motorDriver->stop();
    }
}

void MotionController::update() {
    if (_motorDriver == nullptr) return;

    unsigned long now = millis();
    float dt = (now - _lastUpdateMs) / 1000.0f;
    if (dt <= 0.0f) dt = 0.01f;
    _lastUpdateMs = now;

    // Acceleration Ramp (Tăng/giảm tốc mềm mượt)
    float accel = 2.0f * dt; // 2 m/s^2 gia tốc mượt
    
    if (_currentVel.vx < _targetVel.vx) _currentVel.vx = min(_currentVel.vx + accel, _targetVel.vx);
    else if (_currentVel.vx > _targetVel.vx) _currentVel.vx = max(_currentVel.vx - accel, _targetVel.vx);

    if (_currentVel.vy < _targetVel.vy) _currentVel.vy = min(_currentVel.vy + accel, _targetVel.vy);
    else if (_currentVel.vy > _targetVel.vy) _currentVel.vy = max(_currentVel.vy - accel, _targetVel.vy);

    if (_currentVel.wz < _targetVel.wz) _currentVel.wz = min(_currentVel.wz + 4.0f * dt, _targetVel.wz);
    else if (_currentVel.wz > _targetVel.wz) _currentVel.wz = max(_currentVel.wz - 4.0f * dt, _targetVel.wz);

    // KIỂM TRA PHÂN HỆ AN TOÀN (SAFETY MONITOR) TRƯỚC KHU XUẤT PWM (ƯU TIÊN CAO NHẤT)
    SafetyMonitor& safety = SafetyMonitor::getInstance();
    
    if (_currentVel.vx > 0.0f && !safety.canMoveForward()) {
        _currentVel.vx = 0.0f;
    }
    if (_currentVel.vx < 0.0f && !safety.canMoveBackward()) {
        _currentVel.vx = 0.0f;
    }
    if (_currentVel.vy != 0.0f && !safety.canStrafe()) {
        _currentVel.vy = 0.0f;
    }
    if (_currentVel.wz != 0.0f && !safety.canRotate()) {
        _currentVel.wz = 0.0f;
    }

    // Tính toán PWM điều khiển động cơ từ Vận tốc Động học Ngược
    const SystemParameters& p = ParameterManager::getInstance().getParams();
    WheelPwm pwm = _kinematics.velocityToPwm(_currentVel, p.maxLinearVelocityX);

    _motorDriver->setAllMotor(pwm.fl, pwm.fr, pwm.rl, pwm.rr);
}
