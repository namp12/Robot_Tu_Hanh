#include "Encoder.h"
#include "EventBus/EventBus.h"

Encoder::Encoder(IEncoderDriver* driver, uint8_t id, const EncoderConfig& config)
    : _driver(driver), _config(config), _id(id),
      _pulseCount(0), _prevPulseCount(0), _deltaPulse(0), _direction(1),
      _rpm(0.0f), _wheelSpeed(0.0f), _distance(0.0f), _lastUpdateTime(0),
      _isHealthy(true), _signalLost(false), _overflow(false), _lastPulseChangeTime(0),
      _medianIdx(0), _maIdx(0) {
    
    for (int i = 0; i < MEDIAN_WINDOW; i++) _rpmMedianHistory[i] = 0.0f;
    for (int i = 0; i < MA_WINDOW; i++) _rpmMaHistory[i] = 0.0f;
}

void Encoder::begin() {
    if (_driver != nullptr) {
        _driver->begin();
    }
    _lastUpdateTime = millis();
    _lastPulseChangeTime = millis();
}

void Encoder::configure(const EncoderConfig& config) {
    _config = config;
}

float Encoder::applyFilters(float rawRpm) {
    float filteredRpm = rawRpm;

    // 1. Median Filter
    if (_config.enableMedianFilter) {
        _rpmMedianHistory[_medianIdx] = rawRpm;
        _medianIdx = (_medianIdx + 1) % MEDIAN_WINDOW;

        // Sort copy to find median
        float temp[MEDIAN_WINDOW];
        memcpy(temp, _rpmMedianHistory, sizeof(temp));
        for (int i = 0; i < MEDIAN_WINDOW - 1; i++) {
            for (int j = i + 1; j < MEDIAN_WINDOW; j++) {
                if (temp[i] > temp[j]) {
                    float t = temp[i];
                    temp[i] = temp[j];
                    temp[j] = t;
                }
            }
        }
        filteredRpm = temp[MEDIAN_WINDOW / 2];
    }

    // 2. Moving Average Filter
    if (_config.enableMovingAverage) {
        _rpmMaHistory[_maIdx] = filteredRpm;
        _maIdx = (_maIdx + 1) % MA_WINDOW;

        float sum = 0;
        for (int i = 0; i < MA_WINDOW; i++) {
            sum += _rpmMaHistory[i];
        }
        filteredRpm = sum / MA_WINDOW;
    }

    return filteredRpm;
}

void Encoder::updateHealthMonitor(unsigned long now) {
    // 1. Check for Overflow
    if (_pulseCount > 2147483600L || _pulseCount < -2147483600L) {
        _overflow = true;
        _isHealthy = false;
        
        Event ev;
        ev.type = EVENT_ENCODER_FAILURE;
        ev.timestamp = now;
        ev.data.motor_id = _id;
        EventBus::getInstance().publish(ev);
        
        Serial.printf("🚨 [Encoder] ID=%d OVERFLOW detection!\n", _id);
    }

    // 2. Check for Extreme/Abnormal Pulses (Velocity limit exceeded)
    if (abs(_wheelSpeed) > _config.maxSpeedThreshold) {
        _isHealthy = false;
        
        Event ev;
        ev.type = EVENT_ENCODER_FAILURE;
        ev.timestamp = now;
        ev.data.motor_id = _id;
        EventBus::getInstance().publish(ev);
        
        Serial.printf("🚨 [Encoder] ID=%d EXTREME speed failure detected: %.2f m/s!\n", _id, _wheelSpeed);
    }

    // 3. Timeout Check (Signal Lost when delta remains 0 under expectation)
    // If pulse count hasn't changed for timeoutMs
    if (now - _lastPulseChangeTime > _config.timeoutMs) {
        if (!_signalLost) {
            _signalLost = true;
            
            Event ev;
            ev.type = EVENT_ENCODER_TIMEOUT;
            ev.timestamp = now;
            ev.data.motor_id = _id;
            EventBus::getInstance().publish(ev);
            
            Serial.printf("🚨 [Encoder] ID=%d TIMEOUT! Mat tin hieu encoder.\n", _id);
        }
    } else {
        _signalLost = false;
    }
}

void Encoder::update() {
    if (_driver == nullptr) return;

    unsigned long now = millis();
    unsigned long dt_ms = now - _lastUpdateTime;
    if (dt_ms <= 0) return;
    float dt = dt_ms / 1000.0f;

    _pulseCount = _driver->getPulseCount();
    _direction = _driver->getDirection();
    
    long rawDelta = _pulseCount - _prevPulseCount;

    // Apply Minimum Pulse Noise Filter
    if (_config.enableMinPulseFilter && abs(rawDelta) < _config.minPulseThreshold) {
        _deltaPulse = 0;
    } else {
        _deltaPulse = rawDelta;
        _lastPulseChangeTime = now;
    }

    _prevPulseCount = _pulseCount;
    _lastUpdateTime = now;

    // Calculate RPM
    float divider = _config.ppr * _config.gearRatio;
    float rawRpm = 0.0f;
    if (divider > 0.0f) {
        rawRpm = ((float)_deltaPulse / divider) * (60.0f / dt);
    }

    // Apply Filter Pipeline
    _rpm = applyFilters(rawRpm);

    // Calculate Wheel Velocity (m/s)
    float wheelCircumference = 3.14159265f * _config.wheelDiameter;
    _wheelSpeed = (_rpm / 60.0f) * wheelCircumference;

    // Calculate drift-free exact cumulative distance
    if (divider > 0.0f) {
        _distance = ((float)_pulseCount / divider) * wheelCircumference;
    }

    // Health checks
    updateHealthMonitor(now);
}

void Encoder::reset() {
    if (_driver != nullptr) {
        _driver->reset();
    }
    _pulseCount = 0;
    _prevPulseCount = 0;
    _deltaPulse = 0;
    _rpm = 0.0f;
    _wheelSpeed = 0.0f;
    _distance = 0.0f;
    _lastUpdateTime = millis();
    _lastPulseChangeTime = millis();
    _isHealthy = true;
    _signalLost = false;
    _overflow = false;
    
    for (int i = 0; i < MEDIAN_WINDOW; i++) _rpmMedianHistory[i] = 0.0f;
    for (int i = 0; i < MA_WINDOW; i++) _rpmMaHistory[i] = 0.0f;
}

EncoderTelemetry Encoder::getTelemetry() const {
    EncoderTelemetry t;
    t.timestamp = _lastUpdateTime;
    t.pulse = _pulseCount;
    t.rpm = _rpm;
    t.speed = _wheelSpeed;
    t.distance = _distance;
    t.direction = _direction;
    t.status = _isHealthy && !_signalLost;
    return t;
}
