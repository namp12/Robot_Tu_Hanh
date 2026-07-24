/**
 * @file ObstacleAvoidance.cpp
 * @brief Logic phân tích khoảng cách và ra quyết định hướng đi cho Robot.
 */

#include "ObstacleAvoidance.h"

ObstacleAvoidance::ObstacleAvoidance()
    : _safeDistance(SAFE_DISTANCE),
      _scanAngle(SCAN_ANGLE),
      _backDistance(BACK_DISTANCE),
      _turnAngle(TURN_ANGLE),
      _recoveryLimit(RECOVERY_LIMIT),
      _recoveryCount(0) {
}

void ObstacleAvoidance::init() {
    _safeDistance = SAFE_DISTANCE;
    _scanAngle = SCAN_ANGLE;
    _backDistance = BACK_DISTANCE;
    _turnAngle = TURN_ANGLE;
    _recoveryLimit = RECOVERY_LIMIT;
    _recoveryCount = 0;
}

bool ObstacleAvoidance::isObstacleDetected(float frontDist) const {
    if (frontDist <= 0.0f || frontDist >= 400.0f) {
        return false; // Coi là thoáng / khoảng cách lỗi out-of-range
    }
    return (frontDist < _safeDistance);
}

AvoidanceDirection ObstacleAvoidance::evaluateScanDecision(float front, float left, float right) {
    // 1. Nếu một trong 2 hướng lớn hơn ngưỡng an toàn, chọn hướng thoáng hơn
    if (left > right && left > _safeDistance) {
        resetRecoveryCount();
        return AVOID_DIR_LEFT;
    }
    
    if (right > left && right > _safeDistance) {
        resetRecoveryCount();
        return AVOID_DIR_RIGHT;
    }

    // 2. Trường hợp đặc biệt: Cả 2 bên đều có khoảng cách xấp xỉ nhau nhưng vẫn > safeDistance
    if (left > _safeDistance || right > _safeDistance) {
        resetRecoveryCount();
        return (left >= right) ? AVOID_DIR_LEFT : AVOID_DIR_RIGHT;
    }

    // 3. Nếu cả 2 bên đều <= SAFE_DISTANCE (Bị chặn cả 2 hướng)
    incrementRecoveryCount();
    if (_recoveryCount >= _recoveryLimit) {
        return AVOID_DIR_RECOVER; // Quá số lần thử -> Kích hoạt Recovery 180°
    }

    return AVOID_DIR_BACKWARD; // Lùi lại và quét lại
}

const char* ObstacleAvoidance::directionToString(AvoidanceDirection dir) {
    switch (dir) {
        case AVOID_DIR_FORWARD:  return "FORWARD";
        case AVOID_DIR_LEFT:     return "LEFT";
        case AVOID_DIR_RIGHT:    return "RIGHT";
        case AVOID_DIR_BACKWARD: return "BACKWARD";
        case AVOID_DIR_RECOVER:  return "RECOVER";
        case AVOID_DIR_STOP:     return "STOP";
        case AVOID_DIR_IDLE:     return "IDLE";
        default:                 return "UNKNOWN";
    }
}
