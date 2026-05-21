#ifndef KINEMATICS_H
#define KINEMATICS_H

#include <Arduino.h>
#include "Config.h"

/**
 * ⚙️ ĐỘNG HỌC ROBOT MECANUM (KINEMATICS)
 * Cung cấp cấu trúc WheelSpeeds và hàm quy đổi động học Mecanum ngược.
 */
struct WheelSpeeds {
    int16_t fl;
    int16_t fr;
    int16_t rl;
    int16_t rr;
};

class Kinematics {
public:
    Kinematics();
    
    // Động học nghịch: Từ tốc độ robot (vx, vy, w) -> Tốc độ bánh xe PWM [-255, 255]
    WheelSpeeds getWheelSpeeds(float vx, float vy, float omega);
};

#endif // KINEMATICS_H
