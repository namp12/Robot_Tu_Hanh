/**
 * @file SerialProtocol.h
 * @brief Module Giao thức Serial Chuẩn ROS2 (SerialProtocol).
 * Tự động Reconnect, Nhận/Giải mã Packet từ CommandParser, Phát Heartbeat 1000ms & Reset Watchdog.
 */

#ifndef SERIAL_PROTOCOL_H
#define SERIAL_PROTOCOL_H

#include <Arduino.h>

class SerialProtocol {
public:
    static SerialProtocol& getInstance();

    void begin(Stream* stream = &Serial);
    void update();
    void sendHeartbeat();

    bool isConnected() const { return _isConnected; }

private:
    SerialProtocol();

    Stream* _stream;
    bool _isConnected;
    unsigned long _lastHeartbeatMs;
    unsigned long _lastRxMs;
};

#endif // SERIAL_PROTOCOL_H
