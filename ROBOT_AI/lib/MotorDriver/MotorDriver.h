#ifndef MOTOR_DRIVER_H
#define MOTOR_DRIVER_H

#include <Arduino.h>
#include "PinMap.h"
#include "Config.h"

/**
 * 🚗 THƯ VIỆN ĐIỀU KHIỂN ĐỘNG CƠ (BTS7960 DRIVER)
 * Quản lý khởi tạo PWM LEDC và điều khiển tốc độ từng bánh xe.
 */
class MotorDriver {
public:
    MotorDriver();
    void begin();
    void setSpeed(int16_t fl, int16_t fr, int16_t rl, int16_t rr);
    void stop();

private:
    void setSingleMotorSpeed(uint8_t ch_r, uint8_t ch_l, int16_t speed);
};

#endif // MOTOR_DRIVER_H
