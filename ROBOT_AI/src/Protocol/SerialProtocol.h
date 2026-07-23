#ifndef SERIAL_PROTOCOL_H
#define SERIAL_PROTOCOL_H

#include <Arduino.h>
#include "Managers/CommandManager.h"
#include "Managers/TelemetryManager.h"

class SerialProtocol {
private:
    Stream* _stream;
    bool _isTelemetryEnabled;

    SerialProtocol();

public:
    static SerialProtocol& getInstance() {
        static SerialProtocol instance;
        return instance;
    }

    void begin(Stream* stream = &Serial);
    void update();
    bool parseCommand(const String& rawInput, RobotCommand& outCmd);
    void sendTelemetry();

    void setTelemetryEnabled(bool enable) { _isTelemetryEnabled = enable; }
    bool isTelemetryEnabled() const { return _isTelemetryEnabled; }
};

#endif // SERIAL_PROTOCOL_H
