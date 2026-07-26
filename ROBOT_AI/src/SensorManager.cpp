/**
 * @file SensorManager.cpp
 * @brief Implementations cho SensorManager.
 */

#include "SensorManager.h"
#include "mode_manager.h"
#include "safety.h"
#include "PinMap.h"

SensorManager& SensorManager::getInstance() {
    static SensorManager instance;
    return instance;
}

SensorManager::SensorManager()
    : _telemetryEnabled(true), _lastReadMs(0), _lastSendMs(0) {
    _data = {0};
}

void SensorManager::begin() {
    EncoderModule::getInstance().begin();
    ImuModule::getInstance().begin(18, 19);
    DistanceModule::getInstance().begin();
    BatteryModule::getInstance().begin(BATTERY_ADC_PIN);

    _lastReadMs = millis();
    _lastSendMs = millis();
}

void SensorManager::update() {
    // Cập nhật tất cả các module cảm biến liên tục ở mọi chu kỳ loop() không điều kiện
    EncoderModule::getInstance().update();
    ImuModule::getInstance().update();
    DistanceModule::getInstance().update(true, true);
    BatteryModule::getInstance().update();

    // 2. Thu thập dữ liệu vào cấu trúc tập trung
    _data.yaw              = ImuModule::getInstance().getYaw();
    _data.roll             = ImuModule::getInstance().getRoll();
    _data.pitch            = ImuModule::getInstance().getPitch();
    _data.totalDistance    = EncoderModule::getInstance().getTotalDistance();
    _data.frontDistance    = DistanceModule::getInstance().getFrontDistance();
    _data.rearDistance     = DistanceModule::getInstance().getRearDistance();
    _data.batteryVoltage   = BatteryModule::getInstance().getVoltage();
    _data.batteryPercentage= BatteryModule::getInstance().getPercentage();
    _data.imuOnline        = ImuModule::getInstance().isOnline();
    _data.frontOnline      = DistanceModule::getInstance().isFrontOnline();
    _data.rearOnline       = DistanceModule::getInstance().isRearOnline();
}

void SensorManager::sendData() {
    if (!_telemetryEnabled) return;

    unsigned long now = millis();
    if (now - _lastSendMs < 10000) return; // In Telemetry 10 giây / 1 lần (10000ms)
    _lastSendMs = now;

    const char* modeStr = ModeManager::getInstance().getModeString();
    bool isEmergency = SafetyMonitor::getInstance().isEmergencyStop();
    const char* statusStr = isEmergency ? "EMERGENCY_STOP" : "READY";

    // Xuất dữ liệu cảm biến thống nhất lên Serial
    Serial.printf("[TELEMETRY] MODE: %s | STATUS: %s | BATTERY: %.2fV (%d%%) | FRONT_DISTANCE: %.1fcm | REAR_DISTANCE: %.1fcm | IMU: Yaw=%.1f° Roll=%.1f° Pitch=%.1f° | ENCODER: Dist=%.2fm\n",
                  modeStr,
                  statusStr,
                  _data.batteryVoltage,
                  _data.batteryPercentage,
                  _data.frontDistance,
                  _data.rearDistance,
                  _data.yaw,
                  _data.roll,
                  _data.pitch,
                  _data.totalDistance);
}
