#include "ROS2BridgeManager.h"
#include "robot_global.h"
#include "SensorManager/SensorManager.h"
#include "MovementController.h"
#include "test_module.h"

ROS2BridgeManager::ROS2BridgeManager() {
    _serial = &Serial;
    _lastTelemetryTime = 0;
    _telemetryIntervalMs = 20; // 50Hz
    _lastCmdVelTime = 0;
    _watchdogTimeoutMs = 500;  // 500ms timeout
    _isEStopActive = false;
    _cmdVx = 0.0f;
    _cmdVy = 0.0f;
    _cmdW = 0.0f;
    _hasNewCmd = false;
    _isTelemetryEnabled = false;
}

void ROS2BridgeManager::begin(HardwareSerial* serialPointer, uint16_t telemetryRateHz) {
    if (serialPointer != nullptr) {
        _serial = serialPointer;
    }
    if (telemetryRateHz > 0) {
        _telemetryIntervalMs = 1000 / telemetryRateHz;
    }
    _lastTelemetryTime = millis();
    _lastCmdVelTime = millis();
    _parser.reset();
}

void ROS2BridgeManager::update() {
    unsigned long now = millis();

    // 1. Read & decode Pi Serial
    while (_serial->available()) {
        uint8_t byteIn = (uint8_t)_serial->read();

        // Tích lũy bộ đệm CLI nếu không trong chế độ nhận gói tin nhị phân
        static String cliBuffer = "";
        if (!_parser.isInPacket() && byteIn != ROS2_HEADER1) {
            if (byteIn == '\n' || byteIn == '\r') {
                if (cliBuffer.length() > 0) {
                    processMainCommand(cliBuffer);
                    cliBuffer = "";
                }
            } else if (byteIn >= 32 && byteIn <= 126) {
                cliBuffer += (char)byteIn;
                if (cliBuffer.length() > 60) {
                    cliBuffer = "";
                }
            }
        } else {
            // Giải phóng bộ đệm nếu nhận được header của gói tin nhị phân
            cliBuffer = "";
        }

        uint8_t msgId = 0;
        uint8_t payloadLen = 0;

        if (_parser.parseByte(byteIn, msgId, _rxPayloadBuffer, payloadLen)) {
            // Tự động bật phát dữ liệu phản hồi (telemetry) khi có thiết bị kết nối truyền lệnh nhị phân
            _isTelemetryEnabled = true;

            switch (msgId) {
                case MSG_ID_CMD_VEL: {
                    if (payloadLen == sizeof(CmdVelPayload)) {
                        CmdVelPayload cmd;
                        memcpy(&cmd, _rxPayloadBuffer, sizeof(CmdVelPayload));
                        _cmdVx = cmd.linear_x;
                        _cmdVy = cmd.linear_y;
                        _cmdW  = cmd.angular_z;
                        _lastCmdVelTime = now;

                        // Convert geometry_msgs/Twist to standard MotionCommand
                        _latestCmd.timestamp = now;
                        _latestCmd.vx = _cmdVx;
                        _latestCmd.vy = _cmdVy;
                        _latestCmd.wz = _cmdW;
                        _latestCmd.brake = false;
                        _latestCmd.emergency_stop = _isEStopActive;
                        
                        if (abs(_cmdVx) < 0.001f && abs(_cmdVy) < 0.001f && abs(_cmdW) < 0.001f) {
                            _latestCmd.type = MOTION_STOP;
                            _latestCmd.speed = 0;
                        } else {
                            _latestCmd.type = MOTION_STRAFE;
                            float velocity = sqrt(_cmdVx*_cmdVx + _cmdVy*_cmdVy);
                            _latestCmd.speed = constrain((int)(velocity * 510.0f + abs(_cmdW) * 150.0f), 0, 255);
                        }
                        _latestCmd.acceleration = 1000;
                        _hasNewCmd = true;

                        // Execute cmd via MovementController if in MODE_ROS2
                        if (currentMode == MODE_ROS2 && !_isEStopActive) {
                            moveControl.handleCommand(_latestCmd);
                            currentMoveDir = "ROS2 cmd_vel";
                            currentSpeed = _latestCmd.speed;
                        }
                    }
                    break;
                }

                case MSG_ID_SET_MODE: {
                    if (payloadLen == sizeof(SetModePayload)) {
                        SetModePayload modePayload;
                        memcpy(&modePayload, _rxPayloadBuffer, sizeof(SetModePayload));

                        if (modePayload.target_mode == 0) {
                            currentMode = MODE_MANUAL;
                            moveControl.stop();
                            currentMoveDir = "dung (Manual via ROS2)";
                            currentSpeed = 0;
                            Serial.println(F("📢 [ROS2 Protocol] Raspberry Pi yeu cau chuyen sang MODE_MANUAL"));
                        } else if (modePayload.target_mode == 1) {
                            currentMode = MODE_AUTO;
                            autoModeStartTime = millis();
                            Serial.println(F("📢 [ROS2 Protocol] Raspberry Pi yeu cau chuyen sang MODE_AUTO"));
                        } else if (modePayload.target_mode == 2) {
                            currentMode = MODE_ROS2;
                            _lastCmdVelTime = now;
                            Serial.println(F("📢 [ROS2 Protocol] Raspberry Pi yeu cau chuyen sang MODE_ROS2 (MAY TINH LAI)"));
                        }

                        if (modePayload.e_stop == 1) {
                            setEmergencyStop(true);
                        } else if (modePayload.e_stop == 0 && _isEStopActive) {
                            setEmergencyStop(false);
                        }
                    }
                    break;
                }

                case MSG_ID_RESET_GOC: {
                    mpu.resetAngle();
                    Serial.println(F("📢 [ROS2 Protocol] Raspberry Pi yeu cau reset goc Yaw MPU6050 ve 0"));
                    break;
                }

                case MSG_ID_TRIGGER_BEEP: {
                    MH_FMD_Beep(500);
                    Serial.println(F("📢 [ROS2 Protocol] Raspberry Pi yeu cau coi keu (Beep)"));
                    break;
                }
            }
        }
    }

    // 2. Watchdog Safety Stop
    if (currentMode == MODE_ROS2 && !_isEStopActive) {
        if (now - _lastCmdVelTime > _watchdogTimeoutMs) {
            moveControl.stop();
            currentMoveDir = "DUNG KHAN (SERIAL TIMEOUT)";
            currentSpeed = 0;
            static unsigned long lastWarnTime = 0;
            if (now - lastWarnTime > 2000) {
                lastWarnTime = now;
                Serial.println(F("⚠️ [ROS2 WATCHDOG] Mat tin hieu cmd_vel qua 500ms! Da tu dong DUNG XE khan cap."));
            }
        }
    }

    // 3. Publish Status / Send Telemetry at 50Hz
    if (now - _lastTelemetryTime >= _telemetryIntervalMs) {
        _lastTelemetryTime = now;
        if (_isTelemetryEnabled) {
            sendTelemetry();
        }
    }
}

bool ROS2BridgeManager::sendTelemetry(const TelemetryData& data) {
    TelemetryPayload payload;
    payload.timestamp_ms = data.timestamp_ms;
    payload.accel_x = data.accel_x;
    payload.accel_y = data.accel_y;
    payload.accel_z = data.accel_z;
    payload.gyro_x = data.gyro_x;
    payload.gyro_y = data.gyro_y;
    payload.gyro_z = data.gyro_z;
    payload.roll = data.roll;
    payload.pitch = data.pitch;
    payload.yaw = data.yaw;
    payload.front_distance = data.front_distance;
    payload.rear_distance = data.rear_distance;
    payload.current_mode = data.current_mode;
    payload.auto_state = data.auto_state;
    payload.motor_fl_speed = data.motor_fl_speed;
    payload.motor_fr_speed = data.motor_fr_speed;
    payload.motor_rl_speed = data.motor_rl_speed;
    payload.motor_rr_speed = data.motor_rr_speed;
    payload.flags = data.flags;

    size_t packetLen = PacketBuilder::buildTelemetryPacket(payload, _txBuffer, sizeof(_txBuffer));
    if (packetLen > 0) {
        return _serial->write(_txBuffer, packetLen) == packetLen;
    }
    return false;
}

void ROS2BridgeManager::sendTelemetry() {
    // Construct TelemetryData from SensorManager
    const SensorData& sensor = SensorManager::getInstance().getSensorData();
    TelemetryData data;
    data.timestamp_ms = millis();
    data.accel_x = sensor.accel_x;
    data.accel_y = sensor.accel_y;
    data.accel_z = sensor.accel_z;
    data.gyro_x = sensor.gyro_x;
    data.gyro_y = sensor.gyro_y;
    data.gyro_z = sensor.gyro_z;
    data.roll = sensor.roll;
    data.pitch = sensor.pitch;
    data.yaw = sensor.yaw;
    data.front_distance = sensor.front_distance;
    data.rear_distance = sensor.rear_distance;
    data.current_mode = (int)currentMode;
    data.auto_state = (int)currentAutoState;
    data.motor_fl_speed = (int16_t)motorFL.getSpeed();
    data.motor_fr_speed = (int16_t)motorFR.getSpeed();
    data.motor_rl_speed = (int16_t)motorRL.getSpeed();
    data.motor_rr_speed = (int16_t)motorRR.getSpeed();

    data.flags = 0;
    if (mpuOk) data.flags |= (1 << 0);
    if (HC_SR04_FrontOnline()) data.flags |= (1 << 1);
    if (HC_SR04_RearOnline()) data.flags |= (1 << 2);
    if (_isEStopActive) data.flags |= (1 << 3);

    sendTelemetry(data);
}

bool ROS2BridgeManager::receiveCommand(MotionCommand& cmd) {
    if (_hasNewCmd) {
        cmd = _latestCmd;
        _hasNewCmd = false;
        return true;
    }
    return false;
}

void ROS2BridgeManager::publishStatus() {
    // In status or send diagnostic heartbeat
}

void ROS2BridgeManager::setEmergencyStop(bool enable) {
    _isEStopActive = enable;
    if (_isEStopActive) {
        moveControl.stop();
        currentMoveDir = "E-STOP ACTIVATED";
        currentSpeed = 0;
        MH_FMD_Beep(500);
        Serial.println(F("🚨 [ROS2 E-STOP] Da KICH HOAT dung khan cap!"));
    } else {
        Serial.println(F("✅ [ROS2 E-STOP] Da HUY BO dung khan cap. System Ready."));
    }
}

bool ROS2BridgeManager::isEmergencyStop() const {
    return _isEStopActive;
}

unsigned long ROS2BridgeManager::getLastCmdTime() const {
    return _lastCmdVelTime;
}

void ROS2BridgeManager::getCmdVel(float& vx, float& vy, float& w) const {
    vx = _cmdVx;
    vy = _cmdVy;
    w  = _cmdW;
}
