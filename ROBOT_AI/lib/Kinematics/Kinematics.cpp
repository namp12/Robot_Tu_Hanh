#include "Kinematics.h"

Kinematics::Kinematics() {}

WheelSpeeds Kinematics::getWheelSpeeds(float vx, float vy, float omega) {
    WheelSpeeds speeds;
    
    // Áp dụng công thức động học Mecanum ngược (Inverse Kinematics)
    float fl_speed = vx - vy - omega;
    float fr_speed = vx + vy + omega;
    float rl_speed = vx + vy - omega;
    float rr_speed = vx - vy + omega;

    // Chuẩn hóa nếu vận tốc tính toán vượt ngoài dải PWM [-255, 255]
    float maxSpeed = max(abs(fl_speed), max(abs(fr_speed), max(abs(rl_speed), abs(rr_speed))));
    if (maxSpeed > 255.0) {
        fl_speed = (fl_speed / maxSpeed) * 255.0;
        fr_speed = (fr_speed / maxSpeed) * 255.0;
        rl_speed = (rl_speed / maxSpeed) * 255.0;
        rr_speed = (rr_speed / maxSpeed) * 255.0;
    }

    speeds.fl = (int16_t)fl_speed;
    speeds.fr = (int16_t)fr_speed;
    speeds.rl = (int16_t)rl_speed;
    speeds.rr = (int16_t)rr_speed;

    return speeds;
}
