#ifndef GENERIC_PID_H
#define GENERIC_PID_H

#include "IPIDController.h"
#include <Arduino.h>

class GenericPID : public IPIDController {
private:
    float _kp, _ki, _kd;
    float _integral;
    float _lastError;
    float _minOutput, _maxOutput;
    bool _isAngle;

public:
    GenericPID(float kp = 0.0f, float ki = 0.0f, float kd = 0.0f, float minOut = -255.0f, float maxOut = 255.0f, bool isAngle = false)
        : _kp(kp), _ki(ki), _kd(kd), _integral(0.0f), _lastError(0.0f), _minOutput(minOut), _maxOutput(maxOut), _isAngle(isAngle) {}

    void setGains(float kp, float ki, float kd) override {
        _kp = kp;
        _ki = ki;
        _kd = kd;
    }

    void setAngleMode(bool enable) {
        _isAngle = enable;
    }

    void setOutputLimits(float minOut, float maxOut) {
        _minOutput = minOut;
        _maxOutput = maxOut;
    }

    float calculate(float setpoint, float measurement, float dt) override {
        if (dt <= 0.0f) return 0.0f;
        
        float error = setpoint - measurement;
        if (_isAngle) {
            while (error > 180.0f) error -= 360.0f;
            while (error < -180.0f) error += 360.0f;
        }
        
        _integral += error * dt;
        _integral = constrain(_integral, _minOutput, _maxOutput);
        
        float derivative = (error - _lastError) / dt;
        _lastError = error;
        
        float output = (_kp * error) + (_ki * _integral) + (_kd * derivative);
        return constrain(output, _minOutput, _maxOutput);
    }

    void reset() override {
        _integral = 0.0f;
        _lastError = 0.0f;
    }
};

#endif // GENERIC_PID_H
