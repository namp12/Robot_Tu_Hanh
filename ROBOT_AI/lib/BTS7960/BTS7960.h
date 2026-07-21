#ifndef BTS7960_H
#define BTS7960_H

#include <Arduino.h>

/**
 * 🏎️ THƯ VIỆN ĐIỀU KHIỂN ĐỘNG CƠ BTS7960
 * Tương thích với cả Arduino ESP32 Core 2.x và Core 3.x
 *  - Core 2.x: dùng ledcSetup + ledcAttachPin + ledcWrite(channel, duty)
 *  - Core 3.x: dùng ledcAttach + ledcWrite(pin, duty)
 */
class BTS7960 {
private:
  uint8_t  _rpwmPin;
  uint8_t  _lpwmPin;
  uint8_t  _renPin;
  uint8_t  _lenPin;
  uint32_t _freq;
  uint8_t  _resolution;
  int      _speed;
  bool     _enabled;

#if !defined(ESP_ARDUINO_VERSION_MAJOR) || ESP_ARDUINO_VERSION_MAJOR < 3
  // Core 2.x: cần quản lý kênh LEDC thủ công
  uint8_t _rpwmChannel;
  uint8_t _lpwmChannel;
  static uint8_t _nextChannel;
#endif

  void writePWM(uint8_t pin, uint8_t channel, uint8_t duty);

public:
  // ren, len = 255 nếu chân Enable nối thẳng VCC
  BTS7960(uint8_t rpwm, uint8_t lpwm,
          uint8_t ren = 255, uint8_t len = 255,
          uint32_t freq = 15000, uint8_t resolution = 8);

  void begin();
  void setSpeed(int speed);
  void forward(uint8_t pwm);
  void backward(uint8_t pwm);
  void stop();
  void brake();
  void enable();
  void disable();

  int  getSpeed()    const;
  bool isEnabled()   const;
  uint8_t getRpwmChannel() const {
#if !defined(ESP_ARDUINO_VERSION_MAJOR) || ESP_ARDUINO_VERSION_MAJOR < 3
    return _rpwmChannel;
#else
    return _rpwmPin;
#endif
  }
  uint8_t getLpwmChannel() const {
#if !defined(ESP_ARDUINO_VERSION_MAJOR) || ESP_ARDUINO_VERSION_MAJOR < 3
    return _lpwmChannel;
#else
    return _lpwmPin;
#endif
  }
};

#endif // BTS7960_H
