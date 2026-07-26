#include "ROS2BridgeManager.h"
#include "robot_global.h"
#include "SensorManager/SensorManager.h"
#include "motion_controller.h"
#include "mode_manager.h"
#include "safety.h"
#include "test_module.h"

ROS2BridgeManager::ROS2BridgeManager()
    : _serial(&Serial), _lastTelemetryTime(0), _telemetryIntervalMs(20),
      _lastCmdVelTime(0), _watchdogTimeoutMs(500), _cmdVx(0.0f), _cmdVy(0.0f),
      _cmdW(0.0f), _hasNewCmd(false), _isTelemetryEnabled(true) {
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
    // Tất cả truyền thông Serial (ASCII Text Protocol) được xử lý tập trung thống nhất bởi SerialProtocol
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
    if (SafetyMonitor::getInstance().isEmergencyStop()) data.flags |= (1 << 3);

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
    if (enable) {
        SafetyMonitor::getInstance().emergencyStop("ROS2 E-Stop");
    } else {
        SafetyMonitor::getInstance().clearEmergencyStop();
    }
}

bool ROS2BridgeManager::isEmergencyStop() const {
    return SafetyMonitor::getInstance().isEmergencyStop();
}

unsigned long ROS2BridgeManager::getLastCmdTime() const {
    return _lastCmdVelTime;
}

void ROS2BridgeManager::getCmdVel(float& vx, float& vy, float& w) const {
    vx = _cmdVx;
    vy = _cmdVy;
    w  = _cmdW;
}
