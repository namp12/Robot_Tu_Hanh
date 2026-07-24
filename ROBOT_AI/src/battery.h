/**
 * @file battery.h
 * @brief Phân hệ Giám sát Điện áp Pin (Battery Voltage Monitor).
 */

#ifndef BATTERY_MODULE_H
#define BATTERY_MODULE_H

#include <Arduino.h>

class BatteryModule {
public:
    static BatteryModule& getInstance();

    void begin(uint8_t adcPin = 4);
    void update();

    float getVoltage() const { return _voltage; }
    uint8_t getPercentage() const { return _percentage; }
    bool isLowBattery() const;

private:
    BatteryModule();
    uint8_t _pin;
    float _voltage;
    uint8_t _percentage;
    unsigned long _lastReadTime;
};

#endif // BATTERY_MODULE_H
