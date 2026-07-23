#ifndef MOVEMENT_CONTROLLER_H
#define MOVEMENT_CONTROLLER_H

#include <Arduino.h>
#include "Motor.h"
#include "EventBus/EventTypes.h"
#include "MotionCommand.h"
#include "IMotionHandler.h"
#include "PID/GenericPID.h"
#include <unordered_map>

// Keep compatibility with old code
enum MovementMode {
    MOVE_IDLE,
    MOVE_FORWARD,
    MOVE_BACKWARD,
    MOVE_STRAFE_LEFT,
    MOVE_STRAFE_RIGHT,
    MOVE_ROTATE_LEFT,
    MOVE_ROTATE_RIGHT,
    MOVE_DIAGONAL_FL,
    MOVE_DIAGONAL_FR,
    MOVE_DIAGONAL_BL,
    MOVE_DIAGONAL_BR,
    MOVE_STOP
};

class MovementController : public IEventSubscriber {
private:
    Motor& _car;
    MovementMode _currentMode;
    MotionCommand _currentCmd;
    
    int _targetSpeed;
    int _currentRampedSpeed;
    unsigned long _lastRampTime;
    
    // PID Yaw
    float _targetYaw;
    GenericPID _yawPID;
    unsigned long _lastPidTime;
    
    // Command Registry for handlers
    std::unordered_map<int, IMotionHandler*> _commandRegistry;
    
    // Safety flag
    bool _isEStopActive;
    
    float normalizeAngle(float angle);
    int updatePwmRamp(int target);

public:
    MovementController(Motor& car);
    ~MovementController();
    
    // Implement IEventSubscriber
    void onEvent(const Event& event) override;
    
    // Primary ROS2 Twist compatible API
    void move(float linear_x, float linear_y, float angular_z);

    // Compatibility APIs
    void forward(int speed);
    void backward(int speed);
    void strafeLeft(int speed);
    void strafeRight(int speed);
    void rotateLeft(int speed);
    void rotateRight(int speed);
    void diagonalFrontLeft(int speed);
    void diagonalFrontRight(int speed);
    void diagonalBackLeft(int speed);
    void diagonalBackRight(int speed);
    void stop();
    
    void setSpeed(int speed);
    void setTargetAngle(float angle);
    void resetPID();
    
    // Accept standard MotionCommand
    void handleCommand(const MotionCommand& cmd);
    
    void update();
    
    MovementMode getMode() const { return _currentMode; }
    int getSpeed() const { return _currentRampedSpeed; }
    float getTargetYaw() const { return _targetYaw; }
};

#endif // MOVEMENT_CONTROLLER_H
