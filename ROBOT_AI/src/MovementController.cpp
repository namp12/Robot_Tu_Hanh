#include "MovementController.h"
#include "robot_global.h"
#include "Config.h"
#include "EventBus/EventBus.h"
#include "SensorManager/SensorManager.h"
#include "MotionHandlers/MotionHandlers.h"
#include "Kinematics.h"
#include "safety.h"

void MovementController::move(float linear_x, float linear_y, float angular_z) {
    if (_isEStopActive) {
        _car.stop();
        return;
    }
    Kinematics kinematics;
    WheelSpeeds speeds = kinematics.getWheelSpeeds(linear_x, linear_y, angular_z);
    _car.setAllMotor(speeds.fl, speeds.fr, speeds.rl, speeds.rr);
}

MovementController::MovementController(Motor& car) 
    : _car(car), 
      _currentMode(MOVE_IDLE), 
      _targetSpeed(0), 
      _currentRampedSpeed(0), 
      _lastRampTime(0),
      _targetYaw(0.0f),
      _yawPID(AUTO_KP, AUTO_KI, AUTO_KD, -AUTO_PID_OUTPUT_CLAMP, AUTO_PID_OUTPUT_CLAMP, true),
      _lastPidTime(0),
      _isEStopActive(false) {
    
    // Register handlers to lookup map
    _commandRegistry[MOTION_STOP]         = new StopMotionHandler();
    _commandRegistry[MOTION_FORWARD]      = new ForwardMotionHandler();
    _commandRegistry[MOTION_BACKWARD]     = new BackwardMotionHandler();
    _commandRegistry[MOTION_LEFT]         = new StrafeLeftMotionHandler();
    _commandRegistry[MOTION_RIGHT]        = new StrafeRightMotionHandler();
    _commandRegistry[MOTION_ROTATE_LEFT]  = new RotateLeftMotionHandler();
    _commandRegistry[MOTION_ROTATE_RIGHT] = new RotateRightMotionHandler();
    _commandRegistry[MOTION_DIAGONAL_FL]  = new DiagonalFLMotionHandler();
    _commandRegistry[MOTION_DIAGONAL_FR]  = new DiagonalFRMotionHandler();
    _commandRegistry[MOTION_DIAGONAL_BL]  = new DiagonalBLMotionHandler();
    _commandRegistry[MOTION_DIAGONAL_BR]  = new DiagonalBRMotionHandler();
    _commandRegistry[MOTION_STRAFE]       = new StrafeMotionHandler();
    _commandRegistry[MOTION_ARC]          = new ArcMotionHandler();
    _commandRegistry[MOTION_CUSTOM]       = new StopMotionHandler();

    // Set initial command to STOP
    _currentCmd.type = MOTION_STOP;
    _currentCmd.speed = 0;
    _currentCmd.vx = 0.0f;
    _currentCmd.vy = 0.0f;
    _currentCmd.wz = 0.0f;
    _currentCmd.brake = false;
    _currentCmd.emergency_stop = false;
    _currentCmd.timestamp = millis();

    // Subscribe to EventBus events
    EventBus::getInstance().subscribe(EVENT_EMERGENCY_STOP, this);
    EventBus::getInstance().subscribe(EVENT_OBSTACLE_DETECTED, this);
}

MovementController::~MovementController() {
    for (auto& pair : _commandRegistry) {
        delete pair.second;
    }
}

float MovementController::normalizeAngle(float angle) {
    while (angle > 180.0f) angle -= 360.0f;
    while (angle < -180.0f) angle += 360.0f;
    return angle;
}

int MovementController::updatePwmRamp(int target) {
    if (target <= 0) {
        _currentRampedSpeed = 0;
        return 0;
    }
    // Khi nhận lệnh chạy, lập tức nhảy tới ngưỡng PWM tối thiểu 120 để thắng ma sát bánh xe ngay tức thì
    if (_currentRampedSpeed < 120 && target >= 120) {
        _currentRampedSpeed = 120;
    }
    unsigned long now = millis();
    if (now - _lastRampTime >= AUTO_RAMP_INTERVAL_MS) {
        _lastRampTime = now;
        if (_currentRampedSpeed < target) {
            _currentRampedSpeed = min(_currentRampedSpeed + AUTO_RAMP_STEP, target);
        } else if (_currentRampedSpeed > target) {
            _currentRampedSpeed = max(_currentRampedSpeed - AUTO_RAMP_STEP, target);
        }
    }
    return _currentRampedSpeed;
}

void MovementController::onEvent(const Event& event) {
    if (event.type == EVENT_EMERGENCY_STOP) {
        _isEStopActive = true;
        _targetSpeed = 0;
        _currentRampedSpeed = 0;
        _car.stop();
        Serial.println(F("🚨 [EventBus] MovementController nhan su kien EMERGENCY STOP!"));
    } else if (event.type == EVENT_OBSTACLE_DETECTED) {
        if (_currentCmd.type == MOTION_FORWARD || _currentCmd.type == MOTION_STRAFE) {
            Serial.println(F("⚠️ [EventBus] MovementController nhan su kien OBSTACLE DETECTED! Dung khan."));
            _targetSpeed = 0;
            _currentRampedSpeed = 0;
            _car.stop();
        }
    }
}

void MovementController::handleCommand(const MotionCommand& cmd) {
    if (_isEStopActive || cmd.emergency_stop) {
        _isEStopActive = true;
        _car.stop();
        return;
    }

    _currentCmd = cmd;
    _targetSpeed = cmd.speed;

    // Mapping back to MovementMode for diagnostics print compatibility
    switch (cmd.type) {
        case MOTION_STOP:         _currentMode = MOVE_STOP; break;
        case MOTION_FORWARD:      _currentMode = MOVE_FORWARD; break;
        case MOTION_BACKWARD:     _currentMode = MOVE_BACKWARD; break;
        case MOTION_LEFT:         _currentMode = MOVE_STRAFE_LEFT; break;
        case MOTION_RIGHT:        _currentMode = MOVE_STRAFE_RIGHT; break;
        case MOTION_ROTATE_LEFT:  _currentMode = MOVE_ROTATE_LEFT; break;
        case MOTION_ROTATE_RIGHT: _currentMode = MOVE_ROTATE_RIGHT; break;
        case MOTION_DIAGONAL_FL:  _currentMode = MOVE_DIAGONAL_FL; break;
        case MOTION_DIAGONAL_FR:  _currentMode = MOVE_DIAGONAL_FR; break;
        case MOTION_DIAGONAL_BL:  _currentMode = MOVE_DIAGONAL_BL; break;
        case MOTION_DIAGONAL_BR:  _currentMode = MOVE_DIAGONAL_BR; break;
        default:                  _currentMode = MOVE_IDLE; break;
    }

    if (cmd.type == MOTION_FORWARD && _lastPidTime == 0) {
        _targetYaw = SensorManager::getInstance().getSensorData().yaw;
        resetPID();
    }
}

void MovementController::forward(int speed) {
    MotionCommand cmd;
    cmd.type = MOTION_FORWARD;
    cmd.speed = speed;
    cmd.vx = 0.2f;
    cmd.vy = 0.0f;
    cmd.wz = 0.0f;
    cmd.brake = false;
    cmd.emergency_stop = false;
    cmd.timestamp = millis();
    handleCommand(cmd);
}

void MovementController::backward(int speed) {
    MotionCommand cmd;
    cmd.type = MOTION_BACKWARD;
    cmd.speed = speed;
    cmd.vx = -0.2f;
    cmd.vy = 0.0f;
    cmd.wz = 0.0f;
    cmd.brake = false;
    cmd.emergency_stop = false;
    cmd.timestamp = millis();
    handleCommand(cmd);
}

void MovementController::strafeLeft(int speed) {
    MotionCommand cmd;
    cmd.type = MOTION_LEFT;
    cmd.speed = speed;
    cmd.vx = 0.0f;
    cmd.vy = -0.2f;
    cmd.wz = 0.0f;
    cmd.brake = false;
    cmd.emergency_stop = false;
    cmd.timestamp = millis();
    handleCommand(cmd);
}

void MovementController::strafeRight(int speed) {
    MotionCommand cmd;
    cmd.type = MOTION_RIGHT;
    cmd.speed = speed;
    cmd.vx = 0.0f;
    cmd.vy = 0.2f;
    cmd.wz = 0.0f;
    cmd.brake = false;
    cmd.emergency_stop = false;
    cmd.timestamp = millis();
    handleCommand(cmd);
}

void MovementController::rotateLeft(int speed) {
    MotionCommand cmd;
    cmd.type = MOTION_ROTATE_LEFT;
    cmd.speed = speed;
    cmd.vx = 0.0f;
    cmd.vy = 0.0f;
    cmd.wz = -0.5f;
    cmd.brake = false;
    cmd.emergency_stop = false;
    cmd.timestamp = millis();
    handleCommand(cmd);
}

void MovementController::rotateRight(int speed) {
    MotionCommand cmd;
    cmd.type = MOTION_ROTATE_RIGHT;
    cmd.speed = speed;
    cmd.vx = 0.0f;
    cmd.vy = 0.0f;
    cmd.wz = 0.5f;
    cmd.brake = false;
    cmd.emergency_stop = false;
    cmd.timestamp = millis();
    handleCommand(cmd);
}

void MovementController::diagonalFrontLeft(int speed) {
    MotionCommand cmd;
    cmd.type = MOTION_DIAGONAL_FL;
    cmd.speed = speed;
    cmd.vx = 0.15f;
    cmd.vy = -0.15f;
    cmd.wz = 0.0f;
    cmd.brake = false;
    cmd.emergency_stop = false;
    cmd.timestamp = millis();
    handleCommand(cmd);
}

void MovementController::diagonalFrontRight(int speed) {
    MotionCommand cmd;
    cmd.type = MOTION_DIAGONAL_FR;
    cmd.speed = speed;
    cmd.vx = 0.15f;
    cmd.vy = 0.15f;
    cmd.wz = 0.0f;
    cmd.brake = false;
    cmd.emergency_stop = false;
    cmd.timestamp = millis();
    handleCommand(cmd);
}

void MovementController::diagonalBackLeft(int speed) {
    MotionCommand cmd;
    cmd.type = MOTION_DIAGONAL_BL;
    cmd.speed = speed;
    cmd.vx = -0.15f;
    cmd.vy = -0.15f;
    cmd.wz = 0.0f;
    cmd.brake = false;
    cmd.emergency_stop = false;
    cmd.timestamp = millis();
    handleCommand(cmd);
}

void MovementController::diagonalBackRight(int speed) {
    MotionCommand cmd;
    cmd.type = MOTION_DIAGONAL_BR;
    cmd.speed = speed;
    cmd.vx = -0.15f;
    cmd.vy = 0.15f;
    cmd.wz = 0.0f;
    cmd.brake = false;
    cmd.emergency_stop = false;
    cmd.timestamp = millis();
    handleCommand(cmd);
}

void MovementController::stop() {
    MotionCommand cmd;
    cmd.type = MOTION_STOP;
    cmd.speed = 0;
    cmd.vx = 0.0f;
    cmd.vy = 0.0f;
    cmd.wz = 0.0f;
    cmd.brake = true;
    cmd.emergency_stop = false;
    cmd.timestamp = millis();
    handleCommand(cmd);
}

void MovementController::setSpeed(int speed) {
    _targetSpeed = speed;
    _currentCmd.speed = speed;
}

void MovementController::setTargetAngle(float angle) {
    _targetYaw = normalizeAngle(angle);
    resetPID();
}

void MovementController::resetPID() {
    _yawPID.reset();
    _lastPidTime = millis();
}

void MovementController::update() {
    SafetyMonitor& safety = SafetyMonitor::getInstance();
    if (_isEStopActive || safety.isEmergencyStop()) {
        _car.stop();
        _currentRampedSpeed = 0;
        return;
    }

    // 1. Ramp PWM
    int speed = updatePwmRamp(_targetSpeed);
    currentSpeed = speed; // compatibility global variable
    _currentCmd.speed = speed;

    // 2. Query sensor manager
    const SensorData& sensorData = SensorManager::getInstance().getSensorData();

    // 3. PID correction (Yaw stabilization)
    float yawCorrection = 0.0f;
    if (AUTO_PID_ENABLED && (_currentCmd.type == MOTION_FORWARD || _currentCmd.type == MOTION_STRAFE)) {
        unsigned long now = millis();
        float dt = (now - _lastPidTime) / 1000.0f;
        if (dt <= 0.0f || dt > 0.5f) dt = 0.02f;
        _lastPidTime = now;

        yawCorrection = _yawPID.calculate(_targetYaw, sensorData.yaw, dt);
    }

    // 4. KIỂM TRA AN TOÀN TRƯỚC KHI XUẤT PWM
    if (_currentCmd.type == MOTION_FORWARD && !safety.canMoveForward()) {
        _currentCmd.type = MOTION_STOP;
        _currentCmd.speed = 0;
        _targetSpeed = 0;
        _currentRampedSpeed = 0;
    }
    if (_currentCmd.type == MOTION_BACKWARD && !safety.canMoveBackward()) {
        _currentCmd.type = MOTION_STOP;
        _currentCmd.speed = 0;
        _targetSpeed = 0;
        _currentRampedSpeed = 0;
    }
    if ((_currentCmd.type == MOTION_LEFT || _currentCmd.type == MOTION_RIGHT) && !safety.canStrafe()) {
        _currentCmd.type = MOTION_STOP;
        _currentCmd.speed = 0;
        _targetSpeed = 0;
        _currentRampedSpeed = 0;
    }
    if ((_currentCmd.type == MOTION_ROTATE_LEFT || _currentCmd.type == MOTION_ROTATE_RIGHT) && !safety.canRotate()) {
        _currentCmd.type = MOTION_STOP;
        _currentCmd.speed = 0;
        _targetSpeed = 0;
        _currentRampedSpeed = 0;
    }

    // 5. Dispatch using Lookup Map
    auto it = _commandRegistry.find((int)_currentCmd.type);
    if (it != _commandRegistry.end() && it->second != nullptr) {
        it->second->execute(_currentCmd, _car, yawCorrection);
    } else {
        _car.stop();
    }
}
