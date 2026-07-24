/**
 * @file SerialProtocol.cpp
 * @brief Implementations cho SerialProtocol với Reconnect & Heartbeat.
 */

#include "SerialProtocol.h"
#include "CommandParser.h"
#include "SensorManager.h"
#include "mode_manager.h"
#include "motion_controller.h"
#include "safety.h"
#include "parameters.h"
#include "test_module.h"

SerialProtocol& SerialProtocol::getInstance() {
    static SerialProtocol instance;
    return instance;
}

SerialProtocol::SerialProtocol()
    : _stream(&Serial), _isConnected(true), _lastHeartbeatMs(0), _lastRxMs(0) {
}

void SerialProtocol::begin(Stream* stream) {
    _stream = stream;
    _isConnected = true;
    _lastHeartbeatMs = millis();
    _lastRxMs = millis();
}

void SerialProtocol::sendHeartbeat() {
    if (_stream == nullptr) return;
    unsigned long now = millis();
    if (now - _lastHeartbeatMs >= 1000) { // Realtime Heartbeat 1s/lần
        _lastHeartbeatMs = now;
        _stream->printf("[HEARTBEAT] timestamp=%lu\n", now);
    }
}

void SerialProtocol::update() {
    sendHeartbeat();

    if (_stream == nullptr) return;

    // Kiểm tra tự động Reconnect Serial khi có dữ liệu mới
    if (_stream->available()) {
        if (!_isConnected) {
            _isConnected = true;
            Serial.println(F("🔄 [SerialProtocol] Đã kết nối lại cổng Serial!"));
        }

        String line = _stream->readStringUntil('\n');
        line.trim();
        if (line.length() == 0) return;

        // 0. Phân tích lệnh Tắt/Bật in Telemetry
        if (line.equalsIgnoreCase("t off") || line.equalsIgnoreCase("telemetry off") || line.equalsIgnoreCase("t 0")) {
            SensorManager::getInstance().setTelemetryEnabled(false);
            Serial.println(F("🔇 [System] Đã TẮT in Telemetry. Cảm biến vẫn đọc ngầm 100% liên tục!"));
            return;
        }
        if (line.equalsIgnoreCase("t on") || line.equalsIgnoreCase("telemetry on") || line.equalsIgnoreCase("t 1")) {
            SensorManager::getInstance().setTelemetryEnabled(true);
            Serial.println(F("🔊 [System] Đã BẬT lại in Telemetry Realtime."));
            return;
        }

        // 1. Phân tích lệnh thông qua CommandParser
        CommandPacket cmd = CommandParser::getInstance().parse(line);

        switch (cmd.type) {
            case CMD_TYPE_PING:
                _stream->println(F("PONG"));
                SafetyMonitor::getInstance().feedWatchdog();
                break;

            case CMD_TYPE_STOP:
                MotionController::getInstance().stop();
                SafetyMonitor::getInstance().feedWatchdog();
                Serial.println(F("🤖 [SerialProtocol] Đã nhận lệnh STOP!"));
                break;

            case CMD_TYPE_MODE:
                ModeManager::getInstance().setModeFromString(cmd.modeString);
                SafetyMonitor::getInstance().feedWatchdog();
                break;

            case CMD_TYPE_SET_SAFE_DIST:
                if (cmd.floatValue > 0.0f) {
                    ParameterManager::getInstance().setSafeDistance(cmd.floatValue);
                    SafetyMonitor::getInstance().feedWatchdog();
                    Serial.printf("⚙️ [Param] Đã cập nhật SafeDistance = %.1f cm\n", cmd.floatValue);
                }
                break;

            case CMD_TYPE_SET_TIMEOUT:
                if (cmd.ulongValue > 0) {
                    ParameterManager::getInstance().setCmdTimeout(cmd.ulongValue);
                    SafetyMonitor::getInstance().feedWatchdog();
                    Serial.printf("⚙️ [Param] Đã cập nhật Timeout = %lu ms\n", cmd.ulongValue);
                }
                break;

            case CMD_TYPE_MOVE:
                SafetyMonitor::getInstance().feedWatchdog();
                if (ModeManager::getInstance().getMode() == MODE_MANUAL || ModeManager::getInstance().getMode() == MODE_ROS) {
                    MotionController::getInstance().setManualCommand(cmd.moveDirection, cmd.moveSpeed);
                }
                break;

            case CMD_TYPE_CMD_VEL:
                SafetyMonitor::getInstance().feedWatchdog();
                if (ModeManager::getInstance().getMode() == MODE_MANUAL) {
                    ModeManager::getInstance().setMode(MODE_ROS);
                }
                if (ModeManager::getInstance().getMode() == MODE_ROS) {
                    MotionController::getInstance().setTargetVelocity(cmd.vx, cmd.vy, cmd.wz);
                }
                break;

            default:
                // Hỗ trợ lùi CLI cũ
                processMainCommand(line);
                break;
        }
    }
}
