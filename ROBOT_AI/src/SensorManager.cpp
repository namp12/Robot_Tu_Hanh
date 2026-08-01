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

    // 1. Gửi dữ liệu ROS2 tần suất cao (20Hz = 50ms/lần) cho Lidar/Odom/IMU
    static unsigned long lastRos2SendMs = 0;
    if (now - lastRos2SendMs >= 50) {
        lastRos2SendMs = now;

        // Gửi khoảng cách siêu âm (đơn vị: cm)
        Serial.printf("RANGE %.1f %.1f\n", _data.frontDistance, _data.rearDistance);

        // Gửi Encoder 4 bánh độc lập (đơn vị: mét)
        float d_fl = EncoderManager::getInstance().getDistance(0);
        float d_fr = EncoderManager::getInstance().getDistance(1);
        float d_rl = EncoderManager::getInstance().getDistance(2);
        float d_rr = EncoderManager::getInstance().getDistance(3);
        Serial.printf("ENCODER %.4f %.4f %.4f %.4f\n", d_fl, d_fr, d_rl, d_rr);

        // Chuyển đổi Roll, Pitch, Yaw (độ) sang Quaternion để gửi IMU qx qy qz qw
        float r = _data.roll * 0.0174532925f;
        float p = _data.pitch * 0.0174532925f;
        float y = _data.yaw * 0.0174532925f;

        float cy = cos(y * 0.5f);
        float sy = sin(y * 0.5f);
        float cp = cos(p * 0.5f);
        float sp = sin(p * 0.5f);
        float cr = cos(r * 0.5f);
        float sr = sin(r * 0.5f);

        float qw = cr * cp * cy + sr * sp * sy;
        float qx = sr * cp * cy - cr * sp * sy;
        float qy = cr * sp * cy + sr * cp * sy;
        float qz = cr * cp * sy - sr * sp * cy;

        Serial.printf("IMU %.4f %.4f %.4f %.4f\n", qx, qy, qz, qw);
    }

    // 2. Gửi dữ liệu Telemetry debug cũ (10 giây / 1 lần)
    if (now - _lastSendMs < 10000) return;
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
