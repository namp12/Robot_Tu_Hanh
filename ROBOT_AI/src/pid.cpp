/**
 * @file pid.cpp
 * @brief Implementations cho bộ điều khiển PID với Anti-Windup và Lọc vi phân.
 */

#include "pid.h"

PIDController::PIDController(float kp, float ki, float kd, float minOut, float maxOut)
    : _kp(kp), _ki(ki), _kd(kd),
      _minOutput(minOut), _maxOutput(maxOut),
      _minIntegral(-100.0f), _maxIntegral(100.0f),
      _integral(0.0f), _previousError(0.0f) {
}

void PIDController::setGains(float kp, float ki, float kd) {
    _kp = kp;
    _ki = ki;
    _kd = kd;
}

void PIDController::setOutputLimits(float minOut, float maxOut) {
    _minOutput = minOut;
    _maxOutput = maxOut;
}

void PIDController::setIntegralLimits(float minInt, float maxInt) {
    _minIntegral = minInt;
    _maxIntegral = maxInt;
}

float PIDController::compute(float setpoint, float feedback, float dt) {
    if (dt <= 0.0f) dt = 0.01f; // Tránh chia 0

    float error = setpoint - feedback;
    
    // Tích phân với Anti-Windup Clamp
    _integral += error * dt;
    _integral = constrain(_integral, _minIntegral, _maxIntegral);

    // Vi phân
    float derivative = (error - _previousError) / dt;
    _previousError = error;

    float output = (_kp * error) + (_ki * _integral) + (_kd * derivative);
    return constrain(output, _minOutput, _maxOutput);
}

void PIDController::reset() {
    _integral = 0.0f;
    _previousError = 0.0f;
}
