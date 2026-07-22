#ifndef ENCODER_H
#define ENCODER_H

#include "IEncoderDriver.h"
#include <Arduino.h>

struct EncoderConfig {
    float ppr = 11.0f;
    float gearRatio = 30.0f;
    float wheelDiameter = 0.08f; // in meters
    bool enableMinPulseFilter = true;
    bool enableMedianFilter = true;
    bool enableMovingAverage = true;
    long minPulseThreshold = 1;
    unsigned long timeoutMs = 2000;
    float maxSpeedThreshold = 5.0f; // m/s limit for failure
};

struct EncoderTelemetry {
    unsigned long timestamp;
    long pulse;
    float rpm;
    float speed;
    float distance;
    int direction;
    bool status; // true for healthy
};

class Encoder {
private:
    IEncoderDriver* _driver;
    EncoderConfig _config;
    uint8_t _id; // For event reporting (0: FL, 1: FR, 2: RL, 3: RR)

    long _pulseCount;
    long _prevPulseCount;
    long _deltaPulse;
    int _direction;
    float _rpm;
    float _wheelSpeed;
    float _distance;
    unsigned long _lastUpdateTime;
    
    // Health status variables
    bool _isHealthy;
    bool _signalLost;
    bool _overflow;
    unsigned long _lastPulseChangeTime;
    
    // Filter Buffers
    static const int MEDIAN_WINDOW = 5;
    static const int MA_WINDOW = 5;
    float _rpmMedianHistory[MEDIAN_WINDOW];
    int _medianIdx;
    float _rpmMaHistory[MA_WINDOW];
    int _maIdx;

    void updateHealthMonitor(unsigned long now);
    float applyFilters(float rawRpm);

public:
    Encoder(IEncoderDriver* driver, uint8_t id, const EncoderConfig& config = EncoderConfig());
    void begin();
    void update();
    void reset();
    void configure(const EncoderConfig& config);

    // Getters
    long getPulse() const { return _pulseCount; }
    long getDeltaPulse() const { return _deltaPulse; }
    float getRPM() const { return _rpm; }
    float getSpeed() const { return _wheelSpeed; }
    float getDistance() const { return _distance; }
    int getDirection() const { return _direction; }
    bool isHealthy() const { return _isHealthy; }
    bool isSignalLost() const { return _signalLost; }
    bool isOverflow() const { return _overflow; }
    
    EncoderTelemetry getTelemetry() const;
};

#endif // ENCODER_H
