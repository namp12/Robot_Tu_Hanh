#ifndef ROBOT_STATE_MANAGER_H
#define ROBOT_STATE_MANAGER_H

#include <Arduino.h>

enum RobotState {
    STATE_BOOT,
    STATE_READY,
    STATE_MANUAL,
    STATE_ROS_CONTROL,
    STATE_TEST,
    STATE_ERROR,
    STATE_LOW_BATTERY
};

class RobotStateManager {
private:
    RobotState _currentState;
    String _lastErrorReason;

    RobotStateManager();

public:
    static RobotStateManager& getInstance() {
        static RobotStateManager instance;
        return instance;
    }

    void begin();
    void setState(RobotState newState);
    RobotState getState() const { return _currentState; }
    const char* getStateName() const;
    void setError(const String& reason);
    String getLastError() const { return _lastErrorReason; }
};

#endif // ROBOT_STATE_MANAGER_H
