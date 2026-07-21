#include "ROS2BridgeManager.h"
#include "robot_global.h"

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

    // 1. Đọc và giải mã dữ liệu Serial từ Raspberry Pi
    while (_serial->available()) {
        uint8_t byteIn = (uint8_t)_serial->read();
        uint8_t msgId = 0;
        uint8_t payloadLen = 0;

        if (_parser.parseByte(byteIn, msgId, _rxPayloadBuffer, payloadLen)) {
            // Giải mã gói tin thành công
            switch (msgId) {
                case MSG_ID_CMD_VEL: {
                    if (payloadLen == sizeof(CmdVelPayload)) {
                        CmdVelPayload cmd;
                        memcpy(&cmd, _rxPayloadBuffer, sizeof(CmdVelPayload));
                        _cmdVx = cmd.linear_x;
                        _cmdVy = cmd.linear_y;
                        _cmdW  = cmd.angular_z;
                        _lastCmdVelTime = now;

                        // Nếu xe đang ở MODE_ROS2, thực thi lệnh vận tốc ra bánh
                        if (currentMode == MODE_ROS2 && !_isEStopActive) {
                            // Quy đổi v_x, v_y (m/s) và omega (rad/s) sang PWM [-255, 255]
                            // Giả định vận tốc tối đa 0.5 m/s tương ứng PWM 255
                            float scaleVx = _cmdVx * 510.0f; // 0.5 m/s -> 255
                            float scaleVy = _cmdVy * 510.0f;
                            float scaleW  = _cmdW  * 150.0f; // 1.7 rad/s (~100 deg/s) -> 255

                            WheelSpeeds speeds = _kinematics.getWheelSpeeds(scaleVx, scaleVy, scaleW);
                            car.setAllMotor(speeds.fl, speeds.fr, speeds.rl, speeds.rr);

                            currentMoveDir = "ROS2 cmd_vel";
                            currentSpeed = max(abs(speeds.fl), max(abs(speeds.fr), max(abs(speeds.rl), abs(speeds.rr))));
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
                            car.stop();
                            currentMoveDir = "dung (Manual via ROS2)";
                            currentSpeed = 0;
                            Serial.println(F("📢 [ROS2 Protocol] Raspberry Pi yêu cầu chuyển sang MODE_MANUAL"));
                        } else if (modePayload.target_mode == 1) {
                            currentMode = MODE_AUTO;
                            autoModeStartTime = millis();
                            Serial.println(F("📢 [ROS2 Protocol] Raspberry Pi yêu cầu chuyển sang MODE_AUTO"));
                        } else if (modePayload.target_mode == 2) {
                            currentMode = MODE_ROS2;
                            _lastCmdVelTime = now;
                            Serial.println(F("📢 [ROS2 Protocol] Raspberry Pi yêu cầu chuyển sang MODE_ROS2 (MÁY TÍNH LÁI)"));
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
                    Serial.println(F("📢 [ROS2 Protocol] Raspberry Pi yêu cầu reset góc Yaw MPU6050 về 0"));
                    break;
                }
            }
        }
    }

    // 2. Kiểm tra Watchdog an toàn: Nếu đang ở MODE_ROS2 mà mất kết nối RPi > 500ms
    if (currentMode == MODE_ROS2 && !_isEStopActive) {
        if (now - _lastCmdVelTime > _watchdogTimeoutMs) {
            car.stop();
            currentMoveDir = "DỪNG KHẨN (SERIAL TIMEOUT)";
            currentSpeed = 0;
            static unsigned long lastWarnTime = 0;
            if (now - lastWarnTime > 2000) {
                lastWarnTime = now;
                Serial.println(F("⚠️ [ROS2 WATCHDOG] Mất tín hiệu cmd_vel quá 500ms! Đã tự động DỪNG XE khẩn cấp."));
            }
        }
    }

    // 3. Gửi gói tin Telemetry định kỳ về Raspberry Pi (50Hz = 20ms)
    if (now - _lastTelemetryTime >= _telemetryIntervalMs) {
        _lastTelemetryTime = now;
        sendTelemetry();
    }
}

void ROS2BridgeManager::sendTelemetry() {
    TelemetryPayload payload;
    payload.timestamp_ms = millis();
    
    payload.accel_x = mpu.getAccelX();
    payload.accel_y = mpu.getAccelY();
    payload.accel_z = mpu.getAccelZ();

    payload.gyro_x  = mpu.getGyroX();
    payload.gyro_y  = mpu.getGyroY();
    payload.gyro_z  = mpu.getGyroZ();

    payload.roll    = mpu.getRoll();
    payload.pitch   = mpu.getPitch();
    payload.yaw     = mpu.getYaw();

    payload.front_distance = HC_SR04_GetFrontDistance();
    payload.rear_distance  = HC_SR04_GetRearDistance();

    payload.current_mode   = (uint8_t)currentMode;
    payload.auto_state     = (uint8_t)currentAutoState;

    payload.motor_fl_speed = (int16_t)motorFL.getSpeed();
    payload.motor_fr_speed = (int16_t)motorFR.getSpeed();
    payload.motor_rl_speed = (int16_t)motorRL.getSpeed();
    payload.motor_rr_speed = (int16_t)motorRR.getSpeed();

    payload.flags = 0;
    if (mpuOk) payload.flags |= (1 << 0);
    if (HC_SR04_FrontOnline()) payload.flags |= (1 << 1);
    if (HC_SR04_RearOnline()) payload.flags |= (1 << 2);
    if (_isEStopActive) payload.flags |= (1 << 3);

    size_t packetLen = PacketBuilder::buildTelemetryPacket(payload, _txBuffer, sizeof(_txBuffer));
    if (packetLen > 0) {
        _serial->write(_txBuffer, packetLen);
    }
}

void ROS2BridgeManager::setEmergencyStop(bool enable) {
    _isEStopActive = enable;
    if (_isEStopActive) {
        car.stop();
        currentMoveDir = "E-STOP ACTIVATED";
        currentSpeed = 0;
        MH_FMD_Beep(500);
        Serial.println(F("🚨 [ROS2 E-STOP] Đã KÍCH HOẠT dừng khẩn cấp!"));
    } else {
        Serial.println(F("✅ [ROS2 E-STOP] Đã HỦY BỎ dừng khẩn cấp. System Ready."));
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
