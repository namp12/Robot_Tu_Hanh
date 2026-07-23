#include "SerialProtocol.h"
#include "robot_global.h"

SerialProtocol::SerialProtocol() 
    : _stream(&Serial), _isTelemetryEnabled(false) {}

void SerialProtocol::begin(Stream* stream) {
    _stream = stream;
    _isTelemetryEnabled = false; // Mặc định TẮT để không làm rối màn hình Serial Monitor ở chế độ Manual
}

void SerialProtocol::update() {
    if (_stream == nullptr || !_stream->available()) return;

    String input = _stream->readStringUntil('\n');
    input.trim();
    if (input.length() == 0) return;

    RobotCommand cmd;
    if (parseCommand(input, cmd)) {
        CommandManager::getInstance().executeCommand(cmd);
    } else {
        // Chuyển toàn bộ các phím bấm và lệnh lái thủ công (w, s, a, d, z, c, q, e, x, tien, etc.) sang clien_dieukhien
        processCommand(input);
    }
}

bool SerialProtocol::parseCommand(const String& rawInput, RobotCommand& outCmd) {
    String input = rawInput;
    input.toLowerCase();

    if (input == "telemetry on" || input == "t on") {
        _isTelemetryEnabled = true;
        Serial.println(F("📢 [SerialProtocol] BẬT xuất thông số Telemetry liên tục (50Hz)."));
        return false;
    }
    if (input == "telemetry off" || input == "t off") {
        _isTelemetryEnabled = false;
        Serial.println(F("📢 [SerialProtocol] TẮT xuất thông số Telemetry liên tục."));
        return false;
    }

    if (input.startsWith("move ") || input.startsWith("twist ")) {
        outCmd.type = CMD_MOVE;
        int s1 = input.indexOf(' ');
        int s2 = input.indexOf(' ', s1 + 1);
        int s3 = input.indexOf(' ', s2 + 1);
        outCmd.linear_x = (s1 != -1 && s2 != -1) ? input.substring(s1 + 1, s2).toFloat() : 0.0f;
        outCmd.linear_y = (s2 != -1 && s3 != -1) ? input.substring(s2 + 1, s3).toFloat() : 
                          ((s2 != -1) ? input.substring(s2 + 1).toFloat() : 0.0f);
        outCmd.angular_z = (s3 != -1) ? input.substring(s3 + 1).toFloat() : 0.0f;
        return true;
    } 
    else if (input == "stop") {
        outCmd.type = CMD_STOP;
        return true;
    }
    else if (input == "ping") {
        outCmd.type = CMD_PING;
        return true;
    }
    else if (input == "reset_odom") {
        outCmd.type = CMD_RESET_ODOMETRY;
        return true;
    }
    else if (input == "mode_ros" || input == "ros") {
        outCmd.type = CMD_SET_MODE;
        outCmd.mode = STATE_ROS_CONTROL;
        _isTelemetryEnabled = true; // Tự động bật telemetry khi sang chế độ ROS
        return true;
    }
    return false;
}

void SerialProtocol::sendTelemetry() {
    if (_stream == nullptr || !_isTelemetryEnabled) return;

    TelemetryManager::getInstance().update();
    const UnifiedTelemetry& t = TelemetryManager::getInstance().getTelemetry();

    _stream->printf(
        "TELEMETRY ts=%lu state=%s front=%.2f rear=%.2f roll=%.2f pitch=%.2f yaw=%.2f vx=%.2f vy=%.2f wz=%.2f batt=%.1f\n",
        t.timestamp, t.robotState, t.frontDistance, t.rearDistance,
        t.roll, t.pitch, t.yaw, t.vx, t.vy, t.wz, t.batteryVoltage
    );
}
