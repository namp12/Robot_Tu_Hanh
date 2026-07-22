#ifndef I_COMM_INTERFACE_H
#define I_COMM_INTERFACE_H

#include "MotionCommand.h"
#include "TelemetryData.h"

class ICommInterface {
public:
    virtual ~ICommInterface() = default;
    virtual void begin() = 0;
    virtual void update() = 0;
    virtual bool sendTelemetry(const TelemetryData& data) = 0;
    virtual bool receiveCommand(MotionCommand& cmd) = 0;
    virtual void publishStatus() = 0;
};

#endif // I_COMM_INTERFACE_H
