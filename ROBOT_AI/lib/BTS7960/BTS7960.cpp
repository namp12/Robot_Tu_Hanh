#include "BTS7960.h"

#if defined(ESP32)
#include "driver/gpio.h"
#endif

// =============================================
// Core 2.x: khởi tạo bộ đếm kênh tĩnh
// =============================================
#if !defined(ESP_ARDUINO_VERSION_MAJOR) || ESP_ARDUINO_VERSION_MAJOR < 3
uint8_t BTS7960::_nextChannel = 0;
#endif

// =============================================
// Constructor
// =============================================
BTS7960::BTS7960(uint8_t rpwm, uint8_t lpwm, uint8_t ren, uint8_t len,
                 uint32_t freq, uint8_t resolution) {
  _rpwmPin    = rpwm;
  _lpwmPin    = lpwm;
  _renPin     = ren;
  _lenPin     = len;
  _freq       = freq;
  _resolution = resolution;
  _speed      = 0;
  _enabled    = false;

#if !defined(ESP_ARDUINO_VERSION_MAJOR) || ESP_ARDUINO_VERSION_MAJOR < 3
  // Core 2.x: tự động gán kênh LEDC
  _rpwmChannel = _nextChannel++;
  _lpwmChannel = _nextChannel++;
#endif
}

// =============================================
// Helper: ghi PWM theo API của từng Core
// =============================================
void BTS7960::writePWM(uint8_t pin, uint8_t channel, uint8_t duty) {
#if defined(ESP_ARDUINO_VERSION_MAJOR) && ESP_ARDUINO_VERSION_MAJOR >= 3
  ledcWrite(pin, duty);
#else
  ledcWrite(channel, duty);
#endif
}

// =============================================
// begin(): khởi tạo PWM và chân Enable
// =============================================
void BTS7960::begin() {
#if defined(ESP32)
  gpio_reset_pin((gpio_num_t)_rpwmPin);
  gpio_reset_pin((gpio_num_t)_lpwmPin);
#endif
  pinMode(_rpwmPin, OUTPUT);
  pinMode(_lpwmPin, OUTPUT);

  // Cấu hình chân Enable nếu được khai báo
  if (_renPin != 255) { pinMode(_renPin, OUTPUT); digitalWrite(_renPin, HIGH); }
  if (_lenPin != 255) { pinMode(_lenPin, OUTPUT); digitalWrite(_lenPin, HIGH); }

#if defined(ESP_ARDUINO_VERSION_MAJOR) && ESP_ARDUINO_VERSION_MAJOR >= 3
  // Core 3.x: gắn pin PWM, tự quản lý kênh nội bộ
  bool rpwmAttached = ledcAttach(_rpwmPin, _freq, _resolution);
  bool lpwmAttached = ledcAttach(_lpwmPin, _freq, _resolution);
  Serial.printf("[LEDC Debug] Pin %d attach: %s | Pin %d attach: %s\n", 
                _rpwmPin, rpwmAttached ? "SUCCESS" : "FAILED", 
                _lpwmPin, lpwmAttached ? "SUCCESS" : "FAILED");
#else
  // Core 2.x: đặt kênh, tần số, độ phân giải rồi gắn pin
  ledcSetup(_rpwmChannel, _freq, _resolution);
  ledcAttachPin(_rpwmPin, _rpwmChannel);
  ledcSetup(_lpwmChannel, _freq, _resolution);
  ledcAttachPin(_lpwmPin, _lpwmChannel);
#endif

  // Khởi tạo trạng thái dừng
#if defined(ESP_ARDUINO_VERSION_MAJOR) && ESP_ARDUINO_VERSION_MAJOR >= 3
  // Core 3.x: chỉ cần pin, channel bỏ qua
  writePWM(_rpwmPin, 0, 0);
  writePWM(_lpwmPin, 0, 0);
#else
  // Core 2.x: cần số kênh
  writePWM(_rpwmPin, _rpwmChannel, 0);
  writePWM(_lpwmPin, _lpwmChannel, 0);
#endif

  _enabled = true;
}

// =============================================
// Điều khiển tốc độ và hướng
// =============================================
void BTS7960::forward(uint8_t pwm) {
  if (!_enabled) return;
  writePWM(_rpwmPin, _rpwmChannel, pwm);
  writePWM(_lpwmPin, _lpwmChannel, 0);
  _speed = pwm;
}

void BTS7960::backward(uint8_t pwm) {
  if (!_enabled) return;
  writePWM(_rpwmPin, _rpwmChannel, 0);
  writePWM(_lpwmPin, _lpwmChannel, pwm);
  _speed = -pwm;
}

void BTS7960::setSpeed(int speed) {
  if (!_enabled) return;
  speed = constrain(speed, -255, 255);
  if      (speed > 0) forward(speed);
  else if (speed < 0) backward(-speed);
  else                stop();
}

void BTS7960::stop() {
  writePWM(_rpwmPin, _rpwmChannel, 0);
  writePWM(_lpwmPin, _lpwmChannel, 0);
  _speed = 0;
}

void BTS7960::brake() {
  if (!_enabled) return;
  uint8_t maxDuty = (1 << _resolution) - 1;
  writePWM(_rpwmPin, _rpwmChannel, maxDuty);
  writePWM(_lpwmPin, _lpwmChannel, maxDuty);
  _speed = 0;
}

void BTS7960::enable() {
  if (_renPin != 255) digitalWrite(_renPin, HIGH);
  if (_lenPin != 255) digitalWrite(_lenPin, HIGH);
  _enabled = true;
}

void BTS7960::disable() {
  stop();
  if (_renPin != 255) digitalWrite(_renPin, LOW);
  if (_lenPin != 255) digitalWrite(_lenPin, LOW);
  _enabled = false;
}

int  BTS7960::getSpeed()  const { return _speed;   }
bool BTS7960::isEnabled() const { return _enabled;  }
