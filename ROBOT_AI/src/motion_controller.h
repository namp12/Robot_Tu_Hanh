/**
 * @file motion_controller.h
 * @brief Điều khiển chuyển động trung tâm (Motion Controller).
 * Nhận lệnh vận hành từ ROS2 (vx, vy, wz), Chế độ Tự Động hoặc Thủ Công,
 * xử lý Động học Mecanum, PID Vận tốc & Tăng/Giảm tốc Ramp PWM.
 */

#ifndef MOTION_CONTROLLER_H
#define MOTION_CONTROLLER_H

#include <Arduino.h>
#include "mecanum.h"
#include "pid.h"
#include "Motor.h"

class MotionController {
public:
    static MotionController& getInstance();

    void begin(Motor* motorDriver);
    void update();

    /**
     * @brief Đặt vận tốc mục tiêu chuẩn ROS2 (cmd_vel: vx m/s, vy m/s, wz rad/s).
     */
    void setTargetVelocity(float vx, float vy, float wz);

    /**
     * @brief Điều khiển thủ công theo mã hướng và tốc độ PWM (0-255).
     */
    void setManualCommand(const String& dir, int speed);

    RobotVelocity getTargetVelocity() const { return _targetVel; }
    RobotVelocity getCurrentVelocity() const { return _currentVel; }

    void stop();

private:
    MotionController();

    Motor* _motorDriver;
    MecanumKinematics _kinematics;

    RobotVelocity _targetVel;
    RobotVelocity _currentVel;

    unsigned long _lastUpdateMs;
};

#endif // MOTION_CONTROLLER_H
