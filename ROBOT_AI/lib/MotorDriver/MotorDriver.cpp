#include "MotorDriver.h"

MotorDriver::MotorDriver() {}

void MotorDriver::begin() {
    // Cấu hình kênh PWM, tần số và độ phân giải
    ledcSetup(CH_FL_R, PWM_FREQ, PWM_RES);
    ledcSetup(CH_FL_L, PWM_FREQ, PWM_RES);
    ledcSetup(CH_FR_R, PWM_FREQ, PWM_RES);
    ledcSetup(CH_FR_L, PWM_FREQ, PWM_RES);
    ledcSetup(CH_RL_R, PWM_FREQ, PWM_RES);
    ledcSetup(CH_RL_L, PWM_FREQ, PWM_RES);
    ledcSetup(CH_RR_R, PWM_FREQ, PWM_RES);
    ledcSetup(CH_RR_L, PWM_FREQ, PWM_RES);

    // Liên kết chân GPIO vật lý với các kênh PWM
    ledcAttachPin(MOTOR_FL_RPWM, CH_FL_R);
    ledcAttachPin(MOTOR_FL_LPWM, CH_FL_L);
    ledcAttachPin(MOTOR_FR_RPWM, CH_FR_R);
    ledcAttachPin(MOTOR_FR_LPWM, CH_FR_L);
    ledcAttachPin(MOTOR_RL_RPWM, CH_RL_R);
    ledcAttachPin(MOTOR_RL_LPWM, CH_RL_L);
    ledcAttachPin(MOTOR_RR_RPWM, CH_RR_R);
    ledcAttachPin(MOTOR_RR_LPWM, CH_RR_L);

    stop();
}

void MotorDriver::setSingleMotorSpeed(uint8_t ch_r, uint8_t ch_l, int16_t speed) {
    speed = constrain(speed, -255, 255);

    if (speed > 0) {
        ledcWrite(ch_r, speed);
        ledcWrite(ch_l, 0);
    } else if (speed < 0) {
        ledcWrite(ch_r, 0);
        ledcWrite(ch_l, abs(speed));
    } else {
        ledcWrite(ch_r, 0);
        ledcWrite(ch_l, 0);
    }
}

void MotorDriver::setSpeed(int16_t fl, int16_t fr, int16_t rl, int16_t rr) {
    setSingleMotorSpeed(CH_FL_R, CH_FL_L, fl);
    setSingleMotorSpeed(CH_FR_R, CH_FR_L, fr);
    setSingleMotorSpeed(CH_RL_R, CH_RL_L, rl);
    setSingleMotorSpeed(CH_RR_R, CH_RR_L, rr);
}

void MotorDriver::stop() {
    setSpeed(0, 0, 0, 0);
}
