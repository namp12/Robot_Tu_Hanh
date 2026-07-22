#ifndef MOTION_COMMAND_H
#define MOTION_COMMAND_H

#include <Arduino.h>

enum MotionType {
    MOTION_STOP,
    MOTION_FORWARD,
    MOTION_BACKWARD,
    MOTION_LEFT,
    MOTION_RIGHT,
    MOTION_ROTATE_LEFT,
    MOTION_ROTATE_RIGHT,
    MOTION_DIAGONAL_FL,
    MOTION_DIAGONAL_FR,
    MOTION_DIAGONAL_BL,
    MOTION_DIAGONAL_BR,
    MOTION_STRAFE,      // Di chuyển Mecanum đa hướng tự do
    MOTION_ARC,         // Chạy vòng cung
    MOTION_CUSTOM       // Lệnh đặc thù
};

struct MotionCommand {
    MotionType type;
    float vx;           // Vận tốc tiến/lùi (m/s)
    float vy;           // Vận tốc dịch ngang (m/s)
    float wz;           // Vận tốc quay góc (rad/s)
    int speed;          // Tốc độ PWM mục tiêu [0, 255]
    int acceleration;   // Gia tốc ramping
    bool brake;         // Dừng cứng chủ động
    bool emergency_stop;// Dừng khẩn cấp
    unsigned long timestamp;
};

#endif // MOTION_COMMAND_H
