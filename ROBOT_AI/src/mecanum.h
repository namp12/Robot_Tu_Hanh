/**
 * @file mecanum.h
 * @brief Động học xe Mecanum (Forward & Inverse Kinematics).
 * Chuyển đổi giữa vận tốc Robot (vx, vy, wz) và tốc độ 4 bánh xe.
 */

#ifndef MECANUM_H
#define MECANUM_H

#include <Arduino.h>

struct RobotVelocity {
    float vx;  ///< Vận tốc tiến/lùi (m/s)
    float vy;  ///< Vận tốc ngang trái/phải (m/s)
    float wz;  ///< Vận tốc góc xoay (rad/s)
};

struct MecanumWheelSpeeds {
    float fl;  ///< Bánh Trước Trái (m/s hoặc rad/s)
    float fr;  ///< Bánh Trước Phải (m/s hoặc rad/s)
    float rl;  ///< Bánh Sau Trái (m/s hoặc rad/s)
    float rr;  ///< Bánh Sau Phải (m/s hoặc rad/s)
};

struct WheelPwm {
    int fl;    ///< PWM Trước Trái (-255 đến 255)
    int fr;    ///< PWM Trước Phải (-255 đến 255)
    int rl;    ///< PWM Sau Trái (-255 đến 255)
    int rr;    ///< PWM Sau Phải (-255 đến 255)
};

class MecanumKinematics {
public:
    MecanumKinematics(float wheelDiameter = 0.08f, float lx = 0.15f, float ly = 0.15f);

    void setGeometry(float wheelDiameter, float lx, float ly);

    /**
     * @brief Động học Thuận (Forward Kinematics): Từ tốc độ 4 bánh xe tính ra vận tốc Robot (vx, vy, wz).
     */
    RobotVelocity forwardKinematics(const MecanumWheelSpeeds& wheelSpeeds) const;

    /**
     * @brief Động học Ngược (Inverse Kinematics): Từ vận tốc Robot (vx, vy, wz) tính ra tốc độ 4 bánh xe.
     */
    MecanumWheelSpeeds inverseKinematics(const RobotVelocity& robotVel) const;

    /**
     * @brief Chuyển đổi vận tốc xe (m/s) trực tiếp sang PWM điều khiển động cơ.
     */
    WheelPwm velocityToPwm(const RobotVelocity& robotVel, float maxLinearSpeed = 1.0f) const;

private:
    float _wheelRadius;
    float _lx;
    float _ly;
    float _lSum; ///< _lx + _ly
};

#endif // MECANUM_H
