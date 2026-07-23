#include "RobotStateManager.h"

RobotStateManager::RobotStateManager() 
    : _currentState(STATE_BOOT), _lastErrorReason("") {}

void RobotStateManager::begin() {
    _currentState = STATE_READY;
}

void RobotStateManager::setState(RobotState newState) {
    _currentState = newState;
}

void RobotStateManager::setError(const String& reason) {
    _currentState = STATE_ERROR;
    _lastErrorReason = reason;
}

const char* RobotStateManager::getStateName() const {
    switch (_currentState) {
        case STATE_BOOT:        return "BOOT";
        case STATE_READY:       return "READY";
        case STATE_MANUAL:      return "MANUAL";
        case STATE_ROS_CONTROL: return "ROS_CONTROL";
        case STATE_TEST:        return "TEST";
        case STATE_ERROR:       return "ERROR";
        case STATE_LOW_BATTERY: return "LOW_BATTERY";
        default:                return "UNKNOWN";
    }
}
