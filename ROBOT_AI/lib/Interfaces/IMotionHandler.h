#ifndef I_MOTION_HANDLER_H
#define I_MOTION_HANDLER_H

#include "MotionCommand.h"
#include "Motor.h"

class IMotionHandler {
public:
    virtual ~IMotionHandler() = default;
    virtual void execute(const MotionCommand& cmd, Motor& car, float yawCorrection) = 0;
};

#endif // I_MOTION_HANDLER_H
