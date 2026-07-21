/**
 * @file ROS2BridgeManager.h
 * @brief Manager quản lý truyền thông 2 chiều giữa ESP32 và Raspberry Pi (ROS2).
 *        Điều khiển nhận/gửi gói nhị phân, quy đổi cmd_vel sang PWM 4 bánh,
 *        và tích hợp Hardware Safety Watchdog chống trôi xe.
 */

#ifndef ROS2_BRIDGE_MANAGER_H
#define ROS2_BRIDGE_MANAGER_H

#include "ROS2Protocol.h"
#include "PacketBuilder.h"
#include "PacketParser.h"
#include "Kinematics.h"

class ROS2BridgeManager {
private:
    HardwareSerial* _serial;
    PacketParser _parser;
    Kinematics _kinematics;

    unsigned long _lastTelemetryTime;
    unsigned long _telemetryIntervalMs; // Mặc định 20ms (50Hz)

    unsigned long _lastCmdVelTime;
    unsigned long _watchdogTimeoutMs;  // Mặc định 500ms
    bool _isEStopActive;

    uint8_t _txBuffer[128];
    uint8_t _rxPayloadBuffer[64];

    // Lệnh vận tốc ROS2 hiện tại
    float _cmdVx;
    float _cmdVy;
    float _cmdW;

public:
    ROS2BridgeManager();

    /**
     * @brief Khởi tạo Bridge Manager và tốc độ baudrate
     * @param serialPointer Con trỏ tới Serial kết nối Raspberry Pi (Mặc định &Serial)
     * @param telemetryRateHz Tần số gửi telemetry về Pi (Hz), mặc định 50Hz (20ms)
     */
    void begin(HardwareSerial* serialPointer = &Serial, uint16_t telemetryRateHz = 50);

    /**
     * @brief Cập nhật luồng xử lý không block CPU (Gọi liên tục trong loop chính)
     */
    void update();

    /**
     * @brief Gửi gói tin Telemetry ngay lập tức về Raspberry Pi
     */
    void sendTelemetry();

    /**
     * @brief Kích hoạt hoặc hủy bỏ dừng khẩn cấp (Emergency Stop)
     */
    void setEmergencyStop(bool enable);
    bool isEmergencyStop() const;

    /**
     * @brief Lấy thời điểm nhận lệnh cmd_vel gần nhất (ms)
     */
    unsigned long getLastCmdTime() const;

    /**
     * @brief Lấy các vận tốc lệnh ROS2 hiện tại
     */
    void getCmdVel(float& vx, float& vy, float& w) const;
};

#endif // ROS2_BRIDGE_MANAGER_H
