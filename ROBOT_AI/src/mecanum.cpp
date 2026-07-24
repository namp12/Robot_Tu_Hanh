/**
 * @file mecanum.cpp
 * @brief Thuật toán Động học Thuận và Động học Ngược cho Robot Mecanum 4 bánh.
 */

#include "mecanum.h"

MecanumKinematics::MecanumKinematics(float wheelDiameter, float lx, float ly) {
    setGeometry(wheelDiameter, lx, ly);
}

void MecanumKinematics::setGeometry(float wheelDiameter, float lx, float ly) {
    _wheelRadius = wheelDiameter / 2.0f;
    _lx = lx;
    _ly = ly;
    _lSum = _lx + _ly;
}

RobotVelocity MecanumKinematics::forwardKinematics(const MecanumWheelSpeeds& w) const {
    RobotVelocity v;
    v.vx = (w.fl + w.fr + w.rl + w.rr) / 4.0f;
    v.vy = (-w.fl + w.fr + w.rl - w.rr) / 4.0f;
    v.wz = (-w.fl + w.fr - w.rl + w.rr) / (4.0f * _lSum);
    return v;
}

MecanumWheelSpeeds MecanumKinematics::inverseKinematics(const RobotVelocity& v) const {
    MecanumWheelSpeeds w;
    w.fl = v.vx - v.vy - (_lSum * v.wz);
    w.fr = v.vx + v.vy + (_lSum * v.wz);
    w.rl = v.vx + v.vy - (_lSum * v.wz);
    w.rr = v.vx - v.vy + (_lSum * v.wz);
    return w;
}

WheelPwm MecanumKinematics::velocityToPwm(const RobotVelocity& robotVel, float maxLinearSpeed) const {
    MecanumWheelSpeeds w = inverseKinematics(robotVel);
    WheelPwm pwm;

    if (maxLinearSpeed <= 0.0f) maxLinearSpeed = 1.0f;

    // Chuyển đổi từ m/s sang dải PWM (-255 đến 255)
    pwm.fl = constrain((int)((w.fl / maxLinearSpeed) * 255.0f), -255, 255);
    pwm.fr = constrain((int)((w.fr / maxLinearSpeed) * 255.0f), -255, 255);
    pwm.rl = constrain((int)((w.rl / maxLinearSpeed) * 255.0f), -255, 255);
    pwm.rr = constrain((int)((w.rr / maxLinearSpeed) * 255.0f), -255, 255);

    return pwm;
}
