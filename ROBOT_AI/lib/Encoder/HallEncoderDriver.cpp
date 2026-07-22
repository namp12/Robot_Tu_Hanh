#include "HallEncoderDriver.h"
#include "driver/gpio.h"

static HallEncoderDriver* _instances[4] = {nullptr, nullptr, nullptr, nullptr};

void IRAM_ATTR ISR_0() {
    if (_instances[0] != nullptr) _instances[0]->handleInterrupt();
}
void IRAM_ATTR ISR_1() {
    if (_instances[1] != nullptr) _instances[1]->handleInterrupt();
}
void IRAM_ATTR ISR_2() {
    if (_instances[2] != nullptr) _instances[2]->handleInterrupt();
}
void IRAM_ATTR ISR_3() {
    if (_instances[3] != nullptr) _instances[3]->handleInterrupt();
}

HallEncoderDriver::HallEncoderDriver(uint8_t pinA, uint8_t pinB, uint8_t index)
    : _pinA(pinA), _pinB(pinB), _index(index), _pulseCount(0), _direction(1) {
    if (index < 4) {
        _instances[index] = this;
    }
}

void HallEncoderDriver::begin() {
    // Giải phóng chức năng JTAG trên các chân GPIO tương ứng để có thể sử dụng như GPIO thường
    gpio_reset_pin((gpio_num_t)_pinA);
    gpio_reset_pin((gpio_num_t)_pinB);

    pinMode(_pinA, INPUT_PULLUP);
    pinMode(_pinB, INPUT_PULLUP);

    void (*isr_func)() = nullptr;
    if (_index == 0) isr_func = ISR_0;
    else if (_index == 1) isr_func = ISR_1;
    else if (_index == 2) isr_func = ISR_2;
    else if (_index == 3) isr_func = ISR_3;

    if (isr_func != nullptr) {
        attachInterrupt(digitalPinToInterrupt(_pinA), isr_func, RISING);
    }
}

void IRAM_ATTR HallEncoderDriver::handleInterrupt() {
    if (digitalRead(_pinB) == HIGH) {
        _pulseCount++;
        _direction = 1;
    } else {
        _pulseCount--;
        _direction = -1;
    }
}

long HallEncoderDriver::getPulseCount() {
    noInterrupts();
    long val = _pulseCount;
    interrupts();
    return val;
}

int HallEncoderDriver::getDirection() {
    return _direction;
}

void HallEncoderDriver::reset() {
    noInterrupts();
    _pulseCount = 0;
    interrupts();
    _direction = 1;
}
