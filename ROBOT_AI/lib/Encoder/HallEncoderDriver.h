#ifndef HALL_ENCODER_DRIVER_H
#define HALL_ENCODER_DRIVER_H

#include "IEncoderDriver.h"
#include <Arduino.h>

class HallEncoderDriver : public IEncoderDriver {
private:
    uint8_t _pinA;
    uint8_t _pinB;
    uint8_t _index;
    volatile long _pulseCount;
    volatile int _direction;

public:
    HallEncoderDriver(uint8_t pinA, uint8_t pinB, uint8_t index);
    void begin() override;
    long getPulseCount() override;
    int getDirection() override;
    void reset() override;

    void handleInterrupt();
};

#endif // HALL_ENCODER_DRIVER_H
