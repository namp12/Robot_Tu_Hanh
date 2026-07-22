#ifndef I_PID_CONTROLLER_H
#define I_PID_CONTROLLER_H

class IPIDController {
public:
    virtual ~IPIDController() = default;
    virtual void setGains(float kp, float ki, float kd) = 0;
    virtual float calculate(float setpoint, float measurement, float dt) = 0;
    virtual void reset() = 0;
};

#endif // I_PID_CONTROLLER_H
