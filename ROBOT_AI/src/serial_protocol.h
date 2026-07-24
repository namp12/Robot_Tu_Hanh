/**
 * @file serial_protocol.h
 * @brief Giao thức Serial Chuyên Nghiệp kết nối ROS2 (Serial Protocol HAL).
 * Xử lý các lệnh: MODE, CMD_VEL, STOP, PING.
 * Xuất dữ liệu phản hồi Telemetry: MODE, Encoder, IMU, Front/Rear Distance, Battery, Status.
 */

#ifndef SERIAL_PROTOCOL_MODULE_H
#define SERIAL_PROTOCOL_MODULE_H

#include <Arduino.h>

class SerialProtocolModule {
public:
    static SerialProtocolModule& getInstance();

    void begin(Stream* stream = &Serial);
    void update();

    void sendTelemetry();
    void sendPingResponse();

private:
    SerialProtocolModule();

    void parseLine(const String& line);

    Stream* _stream;
    unsigned long _lastTelemetryMs;
};

#endif // SERIAL_PROTOCOL_MODULE_H
