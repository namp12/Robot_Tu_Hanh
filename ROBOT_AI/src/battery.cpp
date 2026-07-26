/**
 * @file battery.cpp
 * @brief Implementations cho BatteryModule.
 */

#include "battery.h"
#include "parameters.h"
#include "PinMap.h"

BatteryModule& BatteryModule::getInstance() {
    static BatteryModule instance;
    return instance;
}

BatteryModule::BatteryModule()
    : _pin(BATTERY_ADC_PIN), _voltage(12.6f), _percentage(100), _lastReadTime(0) {
}

void BatteryModule::begin(uint8_t adcPin) {
    _pin = adcPin;
    pinMode(_pin, INPUT);
    _voltage = 12.6f;
    _percentage = 100;
    _lastReadTime = millis();
}

void BatteryModule::update() {
    unsigned long now = millis();
    if (now - _lastReadTime < 1000) return; // Đọc 1s/lần
    _lastReadTime = now;

    // Giả lập/đọc ADC (Phần cứng dùng cầu phân áp 1:4)
    int rawAdc = analogRead(_pin);
    float measuredVolts = (rawAdc / 4095.0f) * 3.3f * 4.15f;

    // Lọc mượt trung bình động
    if (measuredVolts > 5.0f) {
        _voltage = (_voltage * 0.8f) + (measuredVolts * 0.2f);
    } else {
        _voltage = 12.6f; // Giá trị mặc định an toàn khi chưa cắm dây ADC
    }

    // Tính phần trăm pin 3S LiPo (9.6V -> 12.6V)
    float pct = ((_voltage - 9.6f) / (12.6f - 9.6f)) * 100.0f;
    _percentage = (uint8_t)constrain(pct, 0.0f, 100.0f);
}

bool BatteryModule::isLowBattery() const {
    const SystemParameters& params = ParameterManager::getInstance().getParams();
    return (_voltage < params.lowBatteryVoltage);
}
