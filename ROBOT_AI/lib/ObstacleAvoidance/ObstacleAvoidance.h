/**
 * @file ObstacleAvoidance.h
 * @brief Module phân tích cảm biến và đưa ra quyết định né vật cản độc lập (Decoupled Perception & Decision Engine).
 * Tuân thủ nguyên lý SOLID - Đơn trách nhiệm (Single Responsibility Principle).
 */

#ifndef OBSTACLE_AVOIDANCE_H
#define OBSTACLE_AVOIDANCE_H

#include <Arduino.h>
#include "Config.h"

enum AvoidanceDirection {
    AVOID_DIR_FORWARD,   ///< Tiến thẳng
    AVOID_DIR_LEFT,      ///< Rẽ trái
    AVOID_DIR_RIGHT,     ///< Rẽ phải
    AVOID_DIR_BACKWARD,  ///< Lùi lại
    AVOID_DIR_RECOVER,   ///< Phục hồi (Quay 180 độ)
    AVOID_DIR_STOP,      ///< Dừng khẩn cấp
    AVOID_DIR_IDLE       ///< Nghỉ / Chờ
};

class ObstacleAvoidance {
public:
    ObstacleAvoidance();

    void init();
    
    /**
     * @brief Kiểm tra xem khoảng cách phía trước có vi phạm ngưỡng an toàn SAFE_DISTANCE không.
     */
    bool isObstacleDetected(float frontDist) const;

    /**
     * @brief So sánh khoảng cách các hướng thu thập được từ AUTO_SCAN và đưa ra quyết định chuyển hướng.
     * @param front Khoảng cách trước (cm)
     * @param left Khoảng cách hướng Trái (cm)
     * @param right Khoảng cách hướng Phải (cm)
     * @return AvoidanceDirection Hướng di chuyển tối ưu được chọn
     */
    AvoidanceDirection evaluateScanDecision(float front, float left, float right);

    // Getters & Setters thông số cấu hình
    float getSafeDistance() const { return _safeDistance; }
    void setSafeDistance(float val) { _safeDistance = val; }

    float getScanAngle() const { return _scanAngle; }
    void setScanAngle(float val) { _scanAngle = val; }

    float getBackDistance() const { return _backDistance; }
    void setBackDistance(float val) { _backDistance = val; }

    float getTurnAngle() const { return _turnAngle; }
    void setTurnAngle(float val) { _turnAngle = val; }

    uint8_t getRecoveryLimit() const { return _recoveryLimit; }
    void setRecoveryLimit(uint8_t val) { _recoveryLimit = val; }

    uint8_t getRecoveryCount() const { return _recoveryCount; }
    void incrementRecoveryCount() { _recoveryCount++; }
    void resetRecoveryCount() { _recoveryCount = 0; }

    static const char* directionToString(AvoidanceDirection dir);

private:
    float _safeDistance;
    float _scanAngle;
    float _backDistance;
    float _turnAngle;
    uint8_t _recoveryLimit;
    uint8_t _recoveryCount;
};

#endif // OBSTACLE_AVOIDANCE_H
