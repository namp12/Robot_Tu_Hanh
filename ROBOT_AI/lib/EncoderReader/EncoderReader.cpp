#include "EncoderReader.h"

// Sử dụng anonymous namespace để ẩn biến và hàm ISR bên trong file này,
// tránh xung đột liên kết (link-time conflicts) với các file khác.
namespace {
    volatile long enc_fl_ticks = 0;
    volatile long enc_fr_ticks = 0;
    volatile long enc_rl_ticks = 0;
    volatile long enc_rr_ticks = 0;

    void IRAM_ATTR ISR_FL() {
        if (digitalRead(ENC_FL_B) == HIGH) {
            enc_fl_ticks++;
        } else {
            enc_fl_ticks--;
        }
    }

    void IRAM_ATTR ISR_FR() {
        if (digitalRead(ENC_FR_B) == HIGH) {
            enc_fr_ticks++;
        } else {
            enc_fr_ticks--;
        }
    }

    void IRAM_ATTR ISR_RL() {
        if (digitalRead(ENC_RL_B) == HIGH) {
            enc_rl_ticks++;
        } else {
            enc_rl_ticks--;
        }
    }

    void IRAM_ATTR ISR_RR() {
        if (digitalRead(ENC_RR_B) == HIGH) {
            enc_rr_ticks++;
        } else {
            enc_rr_ticks--;
        }
    }
}

EncoderReader::EncoderReader() {}

// Lưu các biến trạng thái vận tốc tĩnh
static unsigned long last_calc = 0;
static long prev_fl = 0, prev_fr = 0, prev_rl = 0, prev_rr = 0;
static float rpm_fl = 0.0, rpm_fr = 0.0, rpm_rl = 0.0, rpm_rr = 0.0;
static float speed_fl = 0.0, speed_fr = 0.0, speed_rl = 0.0, speed_rr = 0.0;

void EncoderReader::begin() {
    pinMode(ENC_FL_A, INPUT_PULLUP);
    pinMode(ENC_FL_B, INPUT_PULLUP);
    pinMode(ENC_FR_A, INPUT_PULLUP);
    pinMode(ENC_FR_B, INPUT_PULLUP);
    pinMode(ENC_RL_A, INPUT_PULLUP);
    pinMode(ENC_RL_B, INPUT_PULLUP);
    pinMode(ENC_RR_A, INPUT_PULLUP);
    pinMode(ENC_RR_B, INPUT_PULLUP);

    attachInterrupt(digitalPinToInterrupt(ENC_FL_A), ISR_FL, RISING);
    attachInterrupt(digitalPinToInterrupt(ENC_FR_A), ISR_FR, RISING);
    attachInterrupt(digitalPinToInterrupt(ENC_RL_A), ISR_RL, RISING);
    attachInterrupt(digitalPinToInterrupt(ENC_RR_A), ISR_RR, RISING);

    last_calc = millis();
}

void EncoderReader::update() {
    unsigned long now = millis();
    unsigned long dt = now - last_calc;
    
    if (dt >= 100) { // Cập nhật vận tốc mỗi 100ms
        noInterrupts();
        long curr_fl = enc_fl_ticks;
        long curr_fr = enc_fr_ticks;
        long curr_rl = enc_rl_ticks;
        long curr_rr = enc_rr_ticks;
        interrupts();

        long d_fl = curr_fl - prev_fl;
        long d_fr = curr_fr - prev_fr;
        long d_rl = curr_rl - prev_rl;
        long d_rr = curr_rr - prev_rr;

        prev_fl = curr_fl;
        prev_fr = curr_fr;
        prev_rl = curr_rl;
        prev_rr = curr_rr;
        last_calc = now;

        float secs = dt / 1000.0;
        float freq_fl = d_fl / secs;
        float freq_fr = d_fr / secs;
        float freq_rl = d_rl / secs;
        float freq_rr = d_rr / secs;

        float divider = PPR * GEAR_RATIO;
        rpm_fl = (freq_fl * 60.0) / divider;
        rpm_fr = (freq_fr * 60.0) / divider;
        rpm_rl = (freq_rl * 60.0) / divider;
        rpm_rr = (freq_rr * 60.0) / divider;

        speed_fl = (rpm_fl / 60.0) * WHEEL_CIRCUMFERENCE;
        speed_fr = (rpm_fr / 60.0) * WHEEL_CIRCUMFERENCE;
        speed_rl = (rpm_rl / 60.0) * WHEEL_CIRCUMFERENCE;
        speed_rr = (rpm_rr / 60.0) * WHEEL_CIRCUMFERENCE;
    }
}

void EncoderReader::getTicks(long &fl, long &fr, long &rl, long &rr) {
    noInterrupts();
    fl = enc_fl_ticks;
    fr = enc_fr_ticks;
    rl = enc_rl_ticks;
    rr = enc_rr_ticks;
    interrupts();
}

void EncoderReader::getRPM(float &fl, float &fr, float &rl, float &rr) {
    fl = rpm_fl;
    fr = rpm_fr;
    rl = rpm_rl;
    rr = rpm_rr;
}

void EncoderReader::getSpeeds(float &fl, float &fr, float &rl, float &rr) {
    fl = speed_fl;
    fr = speed_fr;
    rl = speed_rl;
    rr = speed_rr;
}

void EncoderReader::resetTicks() {
    noInterrupts();
    enc_fl_ticks = 0;
    enc_fr_ticks = 0;
    enc_rl_ticks = 0;
    enc_rr_ticks = 0;
    interrupts();
    
    prev_fl = prev_fr = prev_rl = prev_rr = 0;
    rpm_fl = rpm_fr = rpm_rl = rpm_rr = 0.0;
    speed_fl = speed_fr = speed_rl = speed_rr = 0.0;
}
