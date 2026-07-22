#ifndef I_ENCODER_DRIVER_H
#define I_ENCODER_DRIVER_H

class IEncoderDriver {
public:
    virtual ~IEncoderDriver() = default;
    virtual void begin() = 0;
    virtual long getPulseCount() = 0;
    virtual int getDirection() = 0; // 1 for forward, -1 for backward
    virtual void reset() = 0;
};

#endif // I_ENCODER_DRIVER_H
