/**
 * @file encoder.h
 * @brief Phân hệ đọc Encoder 4 bánh xe và tính toán Odometry.
 */

#ifndef ENCODER_MODULE_H
#define ENCODER_MODULE_H

#include <Arduino.h>
#include "EncoderManager.h"
#include "mecanum.h"

class EncoderModule {
public:
    static EncoderModule& getInstance();

    void begin();
    void update();

    MecanumWheelSpeeds getWheelSpeeds() const;
    float getTotalDistance() const;
    void reset();

private:
    EncoderModule();
};

#endif // ENCODER_MODULE_H
