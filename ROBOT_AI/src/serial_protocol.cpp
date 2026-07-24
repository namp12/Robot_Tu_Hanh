/**
 * @file serial_protocol.cpp
 * @brief Implementations cho Giao thức Serial Chuyên Nghiệp ROS2 Humble (Serial Protocol HAL).
 */

#include "serial_protocol.h"
#include "mode_manager.h"
#include "motion_controller.h"
#include "safety.h"
#include "encoder.h"
#include "imu.h"
#include "distance.h"
#include "battery.h"
#include "parameters.h"
#include "test_module.h"

SerialProtocolModule& SerialProtocolModule::getInstance() {
    static SerialProtocolModule instance;
    return instance;
}

SerialProtocolModule::SerialProtocolModule()
    : _stream(&Serial), _lastTelemetryMs(0) {
}

void SerialProtocolModule::begin(Stream* stream) {
    _stream = stream;
    _lastTelemetryMs = millis();
}

void SerialProtocolModule::update() {
    if (_stream == nullptr || !_stream->available()) return;

    String line = _stream->readStringUntil('\n');
    line.trim();
    if (line.length() == 0) return;

    parseLine(line);
}

void SerialProtocolModule::parseLine(const String& line) {
    String input = line;
    input.trim();

    // 1. Phân tích lệnh PING -> Phản hồi PONG
    if (input.equalsIgnoreCase("PING")) {
        sendPingResponse();
        SafetyMonitor::getInstance().feedWatchdog();
        return;
    }

    // 2. Phân tích lệnh STOP
    if (input.equalsIgnoreCase("STOP") || input.equalsIgnoreCase("dung") || input.equalsIgnoreCase("x")) {
        MotionController::getInstance().stop();
        SafetyMonitor::getInstance().feedWatchdog();
        Serial.println(F("🤖 [ROS2 Protocol] Đã nhận lệnh STOP!"));
        return;
    }

    // 3. Phân tích lệnh MODE <chế độ> (MODE MANUAL, MODE AUTO, MODE ROS)
    if (input.startsWith("MODE ") || input.startsWith("mode ")) {
        String modeName = input.substring(5);
        modeName.trim();
        ModeManager::getInstance().setModeFromString(modeName);
        SafetyMonitor::getInstance().feedWatchdog();
        return;
    }

    // 4. Phân tích lệnh thay đổi SafeDistance động: SET_SAFE_DISTANCE <cm>
    if (input.startsWith("SET_SAFE_DISTANCE ") || input.startsWith("set_safe_distance ")) {
        float val = input.substring(18).toFloat();
        if (val > 0.0f) {
            ParameterManager::getInstance().setSafeDistance(val);
            SafetyMonitor::getInstance().feedWatchdog();
            Serial.printf("⚙️ [Param] Đã cập nhật SafeDistance mới = %.1f cm\n", val);
        }
        return;
    }

    // 5. Phân tích lệnh thay đổi Timeout động: SET_TIMEOUT <ms>
    if (input.startsWith("SET_TIMEOUT ") || input.startsWith("set_timeout ")) {
        unsigned long ms = (unsigned long)input.substring(12).toInt();
        if (ms > 0) {
            ParameterManager::getInstance().setCmdTimeout(ms);
            SafetyMonitor::getInstance().feedWatchdog();
            Serial.printf("⚙️ [Param] Đã cập nhật Serial Timeout mới = %lu ms\n", ms);
        }
        return;
    }

    // 6. Phân tích 11 Lệnh di chuyển thống nhất: MOVE <DIRECTION> <SPEED>
    if (input.startsWith("MOVE ") || input.startsWith("move ")) {
        String moveArgs = input.substring(5);
        moveArgs.trim();

        char dirBuf[32] = {0};
        int speed = 150;
        int parsed = sscanf(moveArgs.c_str(), "%31s %d", dirBuf, &speed);

        if (parsed >= 1) {
            SafetyMonitor::getInstance().feedWatchdog();
            String dirStr = String(dirBuf);
            dirStr.toUpperCase();

            // Nếu nhận lệnh từ Web/Bluetooth ở MODE_MANUAL hoặc ROS2
            OperatingMode curMode = ModeManager::getInstance().getMode();
            if (curMode == MODE_MANUAL || curMode == MODE_ROS) {
                MotionController::getInstance().setManualCommand(dirStr, speed);
            }
        }
        return;
    }

    // 7. Phân tích lệnh CMD_VEL <vx> <vy> <wz> từ ROS2 robot_serial
    if (input.startsWith("CMD_VEL ") || input.startsWith("cmd_vel ")) {
        String velArgs = input.substring(8);
        velArgs.trim();

        float vx = 0.0f, vy = 0.0f, wz = 0.0f;
        int parsedCount = sscanf(velArgs.c_str(), "%f %f %f", &vx, &vy, &wz);

        if (parsedCount >= 1) {
            SafetyMonitor::getInstance().feedWatchdog();
            
            // Tự động chuyển sang MODE_ROS nếu đang nhận lệnh ROS2
            if (ModeManager::getInstance().getMode() == MODE_MANUAL) {
                ModeManager::getInstance().setMode(MODE_ROS);
            }

            if (ModeManager::getInstance().getMode() == MODE_ROS) {
                MotionController::getInstance().setTargetVelocity(vx, vy, wz);
            }
        }
        return;
    }

    // 8. Tương thích lùi với Serial CLI cũ
    processMainCommand(input);
}

void SerialProtocolModule::sendPingResponse() {
    if (_stream == nullptr) return;
    _stream->println(F("PONG"));
}

void SerialProtocolModule::sendTelemetry() {
    if (_stream == nullptr) return;

    OperatingMode currentMode = ModeManager::getInstance().getMode();
    const char* modeStr = ModeManager::getInstance().getModeString();

    float totalDist = EncoderModule::getInstance().getTotalDistance();
    float yaw = ImuModule::getInstance().getYaw();
    float roll = ImuModule::getInstance().getRoll();
    float pitch = ImuModule::getInstance().getPitch();
    float frontDist = DistanceModule::getInstance().getFrontDistance();
    float rearDist = DistanceModule::getInstance().getRearDistance();
    float batteryVolts = BatteryModule::getInstance().getVoltage();
    uint8_t batteryPct = BatteryModule::getInstance().getPercentage();
    bool isEmergency = SafetyMonitor::getInstance().isEmergencyStop();

    const char* statusStr = isEmergency ? "EMERGENCY_STOP" : "READY";

    // Xuất dữ liệu cảm biến thống nhất 20Hz lên Raspberry Pi / ROS2 / Web
    _stream->printf("[TELEMETRY] MODE: %s | STATUS: %s | BATTERY: %.2fV (%d%%) | FRONT_DISTANCE: %.1fcm | REAR_DISTANCE: %.1fcm | IMU: Yaw=%.1f° Roll=%.1f° Pitch=%.1f° | ENCODER: Dist=%.2fm\n",
                    modeStr,
                    statusStr,
                    batteryVolts,
                    batteryPct,
                    frontDist,
                    rearDist,
                    yaw,
                    roll,
                    pitch,
                    totalDist);
}
