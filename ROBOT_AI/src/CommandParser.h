/**
 * @file CommandParser.h
 * @brief Module phân tích cú pháp lệnh nhận từ Raspberry Pi / ROS2 / Web / BLE (CommandParser).
 */

#ifndef COMMAND_PARSER_H
#define COMMAND_PARSER_H

#include <Arduino.h>

enum ParsedCommandType {
    CMD_TYPE_UNKNOWN,
    CMD_TYPE_MOVE,
    CMD_TYPE_CMD_VEL,
    CMD_TYPE_MODE,
    CMD_TYPE_SET_SAFE_DIST,
    CMD_TYPE_SET_TIMEOUT,
    CMD_TYPE_STOP,
    CMD_TYPE_PING
};

struct CommandPacket {
    ParsedCommandType type;
    String moveDirection;
    int moveSpeed;
    float vx;
    float vy;
    float wz;
    String modeString;
    float floatValue;
    unsigned long ulongValue;
};

class CommandParser {
public:
    static CommandParser& getInstance();

    CommandPacket parse(const String& line);

private:
    CommandParser();
};

#endif // COMMAND_PARSER_H
