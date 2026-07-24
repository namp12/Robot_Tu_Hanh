/**
 * @file pid.h
 * @brief Bộ điều khiển PID độc lập (Generic PID Controller with Anti-Windup & Output Clamping).
 */

#ifndef PID_H
#define PID_H

#include <Arduino.h>

class PIDController {
public:
    PIDController(float kp = 1.0f, float ki = 0.0f, float kd = 0.0f, float minOut = -255.0f, float maxOut = 255.0f);

    void setGains(float kp, float ki, float kd);
    void setOutputLimits(float minOut, float maxOut);
    void setIntegralLimits(float minInt, float maxInt);

    float compute(float setpoint, float feedback, float dt);
    void reset();

    float getKp() const { return _kp; }
    float getKi() const { return _ki; }
    float getKd() const { return _kd; }

private:
    float _kp;
    float _ki;
    float _kd;

    float _minOutput;
    float _maxOutput;

    float _minIntegral;
    float _maxIntegral;

    float _integral;
    float _previousError;
};

#endif // PID_H
