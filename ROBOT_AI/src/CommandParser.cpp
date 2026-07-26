/**
 * @file CommandParser.cpp
 * @brief Implementations cho CommandParser.
 */

#include "CommandParser.h"

CommandParser& CommandParser::getInstance() {
    static CommandParser instance;
    return instance;
}

CommandParser::CommandParser() {}

CommandPacket CommandParser::parse(const String& line) {
    CommandPacket packet;
    packet.type = CMD_TYPE_UNKNOWN;
    packet.moveDirection = "STOP";
    packet.moveSpeed = 0;
    packet.vx = 0.0f;
    packet.vy = 0.0f;
    packet.wz = 0.0f;
    packet.floatValue = 0.0f;
    packet.ulongValue = 0;

    String input = line;
    input.trim();
    if (input.length() == 0) return packet;

    // 1. Lệnh PING
    if (input.equalsIgnoreCase("PING")) {
        packet.type = CMD_TYPE_PING;
        return packet;
    }

    // 2. Lệnh STOP
    if (input.equalsIgnoreCase("STOP") || input.equalsIgnoreCase("dung") || input.equalsIgnoreCase("x")) {
        packet.type = CMD_TYPE_STOP;
        return packet;
    }

    // 3. Lệnh MODE <chế độ> (MANUAL, AUTO, ROS)
    if (input.startsWith("MODE ") || input.startsWith("mode ")) {
        packet.type = CMD_TYPE_MODE;
        packet.modeString = input.substring(5);
        packet.modeString.trim();
        return packet;
    }

    // 4. Lệnh SET_SAFE_DISTANCE <cm>
    if (input.startsWith("SET_SAFE_DISTANCE ") || input.startsWith("set_safe_distance ")) {
        packet.type = CMD_TYPE_SET_SAFE_DIST;
        packet.floatValue = input.substring(18).toFloat();
        return packet;
    }

    // 5. Lệnh SET_TIMEOUT <ms>
    if (input.startsWith("SET_TIMEOUT ") || input.startsWith("set_timeout ")) {
        packet.type = CMD_TYPE_SET_TIMEOUT;
        packet.ulongValue = (unsigned long)input.substring(12).toInt();
        return packet;
    }

    // 6. Lệnh 11 Hướng di chuyển: MOVE <DIRECTION> <SPEED>
    if (input.startsWith("MOVE ") || input.startsWith("move ")) {
        packet.type = CMD_TYPE_MOVE;
        String moveArgs = input.substring(5);
        moveArgs.trim();

        char dirBuf[32] = {0};
        int speed = 150;
        int parsed = sscanf(moveArgs.c_str(), "%31s %d", dirBuf, &speed);
        if (parsed >= 1) {
            packet.moveDirection = String(dirBuf);
            packet.moveDirection.toUpperCase();
            packet.moveSpeed = (parsed >= 2) ? speed : 150;
        }
        return packet;
    }

    // 7. Lệnh ROS2 Kinematic: CMD_VEL <vx> <vy> <wz>
    if (input.startsWith("CMD_VEL ") || input.startsWith("cmd_vel ")) {
        packet.type = CMD_TYPE_CMD_VEL;
        String velArgs = input.substring(8);
        velArgs.trim();

        float vx = 0.0f, vy = 0.0f, wz = 0.0f;
        int parsedCount = sscanf(velArgs.c_str(), "%f %f %f", &vx, &vy, &wz);
        if (parsedCount >= 1) {
            packet.vx = vx;
            packet.vy = vy;
            packet.wz = wz;
        }
        return packet;
    }

    // 8. Phân tích trực tiếp các từ khóa lệnh di chuyển tiếng Việt (tien 150, lui 120, trai, phai, ...)
    String lowerInput = input;
    lowerInput.toLowerCase();
    int spaceIndex = lowerInput.indexOf(' ');
    String firstWord = (spaceIndex == -1) ? lowerInput : lowerInput.substring(0, spaceIndex);
    int argSpeed = (spaceIndex == -1) ? 150 : lowerInput.substring(spaceIndex + 1).toInt();
    if (argSpeed <= 0) argSpeed = 150;

    if (firstWord == "tien" || firstWord == "w" ||
        firstWord == "lui" || firstWord == "s" ||
        firstWord == "trai" || firstWord == "a" ||
        firstWord == "phai" || firstWord == "d" ||
        firstWord == "xoay_trai" || firstWord == "q" ||
        firstWord == "xoay_phai" || firstWord == "e" ||
        firstWord == "cheo_tt" || firstWord == "z" ||
        firstWord == "cheo_tp" || firstWord == "c" ||
        firstWord == "cheo_st" || firstWord == "cheo_sp") {
        
        packet.type = CMD_TYPE_MOVE;
        if (firstWord == "w") packet.moveDirection = "TIEN";
        else if (firstWord == "s") packet.moveDirection = "LUI";
        else if (firstWord == "a") packet.moveDirection = "TRAI";
        else if (firstWord == "d") packet.moveDirection = "PHAI";
        else if (firstWord == "q") packet.moveDirection = "XOAY_TRAI";
        else if (firstWord == "e") packet.moveDirection = "XOAY_PHAI";
        else if (firstWord == "z") packet.moveDirection = "CHEO_TT";
        else if (firstWord == "c") packet.moveDirection = "CHEO_TP";
        else {
            packet.moveDirection = firstWord;
            packet.moveDirection.toUpperCase();
        }
        packet.moveSpeed = argSpeed;
        return packet;
    }

    return packet;
}

