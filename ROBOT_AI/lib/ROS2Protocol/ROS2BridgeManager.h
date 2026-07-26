#ifndef ROS2_BRIDGE_MANAGER_H
#define ROS2_BRIDGE_MANAGER_H

#include "ROS2Protocol.h"
#include "PacketBuilder.h"
#include "PacketParser.h"
#include "Kinematics.h"
#include "ICommInterface.h"

class ROS2BridgeManager : public ICommInterface {
private:
    HardwareSerial* _serial;
    PacketParser _parser;
    Kinematics _kinematics;

    unsigned long _lastTelemetryTime;
    unsigned long _telemetryIntervalMs; // Mặc định 20ms (50Hz)

    unsigned long _lastCmdVelTime;
    unsigned long _watchdogTimeoutMs;  // Mặc định 500ms

    uint8_t _txBuffer[128];
    uint8_t _rxPayloadBuffer[64];

    // Lệnh vận tốc ROS2 hiện tại
    float _cmdVx;
    float _cmdVy;
    float _cmdW;

    // Command cache for interface query
    MotionCommand _latestCmd;
    bool _hasNewCmd;
    bool _isTelemetryEnabled;

public:
    ROS2BridgeManager();

    // Implement ICommInterface
    void begin() override { begin(&Serial, 50); }
    void update() override;
    bool sendTelemetry(const TelemetryData& data) override;
    bool receiveCommand(MotionCommand& cmd) override;
    void publishStatus() override;

    // Overloaded begin for backward compatibility
    void begin(HardwareSerial* serialPointer, uint16_t telemetryRateHz = 50);

    /**
     * @brief Gửi gói tin Telemetry ngay lập tức về Raspberry Pi (Tương thích ngược)
     */
    void sendTelemetry();

    /**
     * @brief Kích hoạt hoặc hủy bỏ dừng khẩn cấp (Emergency Stop)
     */
    void setEmergencyStop(bool enable);
    bool isEmergencyStop() const;

    void setTelemetryEnabled(bool enable) { _isTelemetryEnabled = enable; }
    bool isTelemetryEnabled() const { return _isTelemetryEnabled; }

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
