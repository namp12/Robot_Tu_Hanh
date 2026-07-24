/**
 * @file parameters.h
 * @brief Quản lý toàn bộ tham số cấu hình hệ thống Robot Mecanum (Parameter Manager).
 * Cho phép đọc/ghi và vi điều chỉnh linh hoạt các thông số PID, Kích thước, Tốc độ, Timeout.
 */

#ifndef PARAMETERS_H
#define PARAMETERS_H

#include <Arduino.h>

struct SystemParameters {
    // --- Thông số vật lý Robot ---
    float wheelDiameter;      ///< Đường kính bánh xe (m)
    float wheelBaseX;         ///< Khoảng cách từ tâm đến trục bánh xe X (m)
    float wheelBaseY;         ///< Khoảng cách từ tâm đến trục bánh xe Y (m)
    float gearRatio;          ///< Tỷ số truyền động cơ
    float ppr;                ///< Xung trên mỗi vòng quay (Pulse Per Revolution)

    // --- Giới hạn Tốc độ & Gia tốc ---
    float maxLinearVelocityX; ///< Tốc độ tiến/lùi tối đa (m/s)
    float maxLinearVelocityY; ///< Tốc độ sang trái/phải tối đa (m/s)
    float maxAngularVelocityZ;///< Tốc độ xoay tối đa (rad/s)
    int maxPwmSpeed;          ///< Tốc độ PWM tối đa (0 - 255)
    int minPwmSpeed;          ///< Tốc độ PWM tối thiểu thắng ma sát (0 - 255)
    int accelRampStep;        ///< Bước tăng PWM mỗi chu kỳ ramp

    // --- Thông số PID Bánh xe & Yaw ---
    float wheelKp;
    float wheelKi;
    float wheelKd;
    
    float yawKp;
    float yawKi;
    float yawKd;
    int yawPidMaxCorrection;

    // --- Thông số Cảm biến & Né vật cản (AUTO Mode) ---
    float safeDistanceCm;     ///< Khoảng cách dừng an toàn (cm)
    float slowDistanceCm;     ///< Khoảng cách bắt đầu giảm tốc (cm)
    float backDistanceCm;     ///< Quãng đường lùi tránh vật cản (cm)
    float turnAngleDeg;       ///< Góc quay đổi hướng (độ)
    unsigned long scanDelayMs;///< Thời gian chờ cảm biến quét (ms)
    uint8_t recoveryLimit;    ///< Số lần lùi thử tối đa

    // --- An toàn & Timeout ---
    unsigned long cmdTimeoutMs; ///< Timeout ngắt kết nối lệnh ROS2 cmd_vel (ms)
    float lowBatteryVoltage;    ///< Ngưỡng cảnh báo pin yếu (V)
    float minBatteryVoltage;    ///< Ngưỡng ngắt khẩn cấp pin yếu (V)
};

class ParameterManager {
public:
    static ParameterManager& getInstance();

    void initDefaults();
    
    const SystemParameters& getParams() const { return _params; }
    SystemParameters& getParamsMutable() { return _params; }

    void setParams(const SystemParameters& params) { _params = params; }
    void setSafeDistance(float cm) { _params.safeDistanceCm = cm; }
    void setCmdTimeout(unsigned long ms) { _params.cmdTimeoutMs = ms; }

private:
    ParameterManager();
    SystemParameters _params;
};

#endif // PARAMETERS_H
