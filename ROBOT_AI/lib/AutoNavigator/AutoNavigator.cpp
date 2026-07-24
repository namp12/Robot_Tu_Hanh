/**
 * @file AutoNavigator.cpp
 * @brief Implementations cho phân hệ Chạy Tự Động (AutoNavigator) nâng cao.
 */

#include "AutoNavigator.h"
#include "MovementController.h"
#include "EncoderManager.h"
#include "Managers/RobotStateManager.h"

AutoNavigator& AutoNavigator::getInstance() {
    static AutoNavigator instance;
    return instance;
}

AutoNavigator::AutoNavigator()
    : _currentState(AUTO_IDLE),
      _stateTimer(0),
      _lastRampTime(0),
      _lastLogTime(0),
      _lastProgressTime(0),
      _currentRampedSpeed(0),
      _targetSpeed(0),
      _scanStep(0),
      _scanStartHeading(0.0f),
      _scanLeftDistance(0.0f),
      _scanRightDistance(0.0f),
      _scanFrontDistance(0.0f),
      _turnTargetHeading(0.0f),
      _startEncoderPos(0.0f),
      _pidIntegral(0.0f),
      _pidPreviousError(0.0f),
      _lastPidTime(0),
      _selectedDirection(AVOID_DIR_IDLE) {
}

void AutoNavigator::begin() {
    _avoidanceEngine.init();
    reset();
    Serial.println(F("🤖 [AutoNavigator] Hệ thống Chạy Tự Động (Decoupled State Machine) Khởi Động!"));
}

void AutoNavigator::reset() {
    _currentState = AUTO_IDLE;
    _stateTimer = millis();
    _lastRampTime = millis();
    _lastLogTime = millis();
    _lastProgressTime = millis();
    _currentRampedSpeed = 0;
    _targetSpeed = 0;
    _scanStep = 0;
    _scanStartHeading = mpuOk ? mpu.getYaw() : 0.0f;
    _scanLeftDistance = 0.0f;
    _scanRightDistance = 0.0f;
    _scanFrontDistance = 0.0f;
    _turnTargetHeading = _scanStartHeading;
    _startEncoderPos = 0.0f;
    _pidIntegral = 0.0f;
    _pidPreviousError = 0.0f;
    _lastPidTime = millis();
    _selectedDirection = AVOID_DIR_IDLE;
    _avoidanceEngine.resetRecoveryCount();
    car.stop();
}

void AutoNavigator::setState(AutoState newState) {
    if (_currentState == newState) return;

    Serial.printf("🤖 [AUTO STATE] %s -> %s\n",
                  auto_run_GetStateName(_currentState),
                  auto_run_GetStateName(newState));

    _currentState = newState;
    currentAutoState = newState;
    _stateTimer = millis();

    // Reset PID khi chuyển trạng thái
    _pidIntegral = 0.0f;
    _pidPreviousError = 0.0f;
    _lastPidTime = millis();

    switch (newState) {
        case AUTO_IDLE:
            car.stop();
            _currentRampedSpeed = 0;
            _targetSpeed = 0;
            _selectedDirection = AVOID_DIR_IDLE;
            break;

        case AUTO_FORWARD:
            _scanStartHeading = mpuOk ? mpu.getYaw() : 0.0f;
            _selectedDirection = AVOID_DIR_FORWARD;
            _lastProgressTime = millis();
            break;

        case AUTO_SCAN:
            car.stop();
            _currentRampedSpeed = 0;
            _scanStep = 0;
            _scanStartHeading = mpuOk ? mpu.getYaw() : 0.0f;
            break;

        case AUTO_BACKWARD:
            _scanStartHeading = mpuOk ? mpu.getYaw() : 0.0f;
            _startEncoderPos = ENCODER_ENABLED ? encoderManager.getWheelDistance() : 0.0f;
            _selectedDirection = AVOID_DIR_BACKWARD;
            break;

        case AUTO_ROTATE_LEFT:
            _scanStartHeading = mpuOk ? mpu.getYaw() : 0.0f;
            _turnTargetHeading = normalizeAngle(_scanStartHeading + _avoidanceEngine.getTurnAngle());
            _selectedDirection = AVOID_DIR_LEFT;
            break;

        case AUTO_ROTATE_RIGHT:
            _scanStartHeading = mpuOk ? mpu.getYaw() : 0.0f;
            _turnTargetHeading = normalizeAngle(_scanStartHeading - _avoidanceEngine.getTurnAngle());
            _selectedDirection = AVOID_DIR_RIGHT;
            break;

        case AUTO_RECOVER:
            _scanStartHeading = mpuOk ? mpu.getYaw() : 0.0f;
            _turnTargetHeading = normalizeAngle(_scanStartHeading + 180.0f);
            _selectedDirection = AVOID_DIR_RECOVER;
            break;
    }

    logStructured("State Changed");
}

void AutoNavigator::update() {
    if (currentMode != MODE_AUTO) return;

    // 1. Cập nhật dữ liệu cảm biến siêu âm
    HC_SR04_Update(true, false);

    // 2. Chạy State Machine chính
    switch (_currentState) {
        case AUTO_IDLE:
            handleStateIdle();
            break;
        case AUTO_FORWARD:
            handleStateForward();
            break;
        case AUTO_SCAN:
            handleStateScan();
            break;
        case AUTO_BACKWARD:
            handleStateBackward();
            break;
        case AUTO_ROTATE_LEFT:
            handleStateRotateLeft();
            break;
        case AUTO_ROTATE_RIGHT:
            handleStateRotateRight();
            break;
        case AUTO_RECOVER:
            handleStateRecover();
            break;
        default:
            setState(AUTO_IDLE);
            break;
    }

    // 3. In log định kỳ mỗi 1 giây
    if (millis() - _lastLogTime >= 1000) {
        _lastLogTime = millis();
        logStructured("Periodic Update");
    }
}

void AutoNavigator::handleStateIdle() {
    car.stop();
    _currentRampedSpeed = 0;
    if (currentMode == MODE_AUTO && (millis() - autoModeStartTime >= 1500) && (millis() - _stateTimer >= 500)) {
        setState(AUTO_FORWARD);
    }
}

void AutoNavigator::handleStateForward() {
    float frontDist = HC_SR04_GetFrontDistance();

    // Phát hiện vật cản < SAFE_DISTANCE
    if (_avoidanceEngine.isObstacleDetected(frontDist)) {
        Serial.printf("🤖 [AUTO] Phát hiện vật cản (%.1fcm < %.1fcm)! Giảm tốc mềm & Chuyển AUTO_SCAN...\n",
                      frontDist, _avoidanceEngine.getSafeDistance());
        
        // Giảm tốc mượt trước khi dừng hẳn
        _targetSpeed = 0;
        updatePwmRamp(0);
        car.stop();
        setState(AUTO_SCAN);
        return;
    }

    // Tính toán tốc độ PWM ramp mượt
    _targetSpeed = AUTO_MAX_SPEED; // PWM 100-150 mượt
    int rampedSpeed = updatePwmRamp(_targetSpeed);

    // Tính toán PID bù góc nghiêng Yaw
    int pidCorrection = 0;
    if (mpuOk) {
        float currentYaw = mpu.getYaw();
        pidCorrection = calculateYawPidCorrection(_scanStartHeading, currentYaw);
    }

    int leftSpeed = constrain(rampedSpeed - pidCorrection, 0, 255);
    int rightSpeed = constrain(rampedSpeed + pidCorrection, 0, 255);

    // Điều khiển động cơ qua car
    car.setAllMotor(leftSpeed, rightSpeed, leftSpeed, rightSpeed);
}

void AutoNavigator::handleStateScan() {
    car.stop();
    _currentRampedSpeed = 0;
    unsigned long now = millis();
    float frontDist = HC_SR04_GetFrontDistance();

    switch (_scanStep) {
        case 0: // Ổn định dừng 300-500ms
            if (now - _stateTimer >= SCAN_DELAY) {
                _scanFrontDistance = frontDist;
                _scanStartHeading = mpuOk ? mpu.getYaw() : 0.0f;
                _turnTargetHeading = normalizeAngle(_scanStartHeading + _avoidanceEngine.getScanAngle());
                _stateTimer = now;
                _scanStep = 1;
                Serial.printf("🤖 [AUTO SCAN] Bước 1: Xoay TRÁI %.0f° để đo...\n", _avoidanceEngine.getScanAngle());
            }
            break;

        case 1: { // Xoay Trái 45°
            int rampSpeed = updatePwmRamp(100);
            car.rotateLeft(rampSpeed);

            bool turnDone = false;
            if (mpuOk) {
                float currentYaw = mpu.getYaw();
                float diff = abs(normalizeAngle(currentYaw - _scanStartHeading));
                if (diff >= _avoidanceEngine.getScanAngle()) turnDone = true;
            }
            if (!mpuOk || (now - _stateTimer >= 800)) turnDone = true;

            if (turnDone) {
                car.stop();
                _stateTimer = now;
                _scanStep = 2;
            }
            break;
        }

        case 2: // Đợi 300-500ms đo khoảng cách Trái
            if (now - _stateTimer >= SCAN_DELAY) {
                _scanLeftDistance = frontDist;
                _scanStartHeading = mpuOk ? mpu.getYaw() : 0.0f;
                _turnTargetHeading = normalizeAngle(_scanStartHeading - (_avoidanceEngine.getScanAngle() * 2.0f));
                _stateTimer = now;
                _scanStep = 3;
                Serial.printf("🤖 [AUTO SCAN] Khoảng cách TRÁI: %.1f cm. Bước 2: Xoay PHẢI %.0f°...\n",
                              _scanLeftDistance, _avoidanceEngine.getScanAngle() * 2.0f);
            }
            break;

        case 3: { // Xoay Phải 90° (sang 45° Phải)
            int rampSpeed = updatePwmRamp(100);
            car.rotateRight(rampSpeed);

            bool turnDone = false;
            if (mpuOk) {
                float currentYaw = mpu.getYaw();
                float diff = abs(normalizeAngle(_scanStartHeading - currentYaw));
                if (diff >= (_avoidanceEngine.getScanAngle() * 2.0f)) turnDone = true;
            }
            if (!mpuOk || (now - _stateTimer >= 1400)) turnDone = true;

            if (turnDone) {
                car.stop();
                _stateTimer = now;
                _scanStep = 4;
            }
            break;
        }

        case 4: // Đợi 300-500ms đo khoảng cách Phải
            if (now - _stateTimer >= SCAN_DELAY) {
                _scanRightDistance = frontDist;
                _scanStartHeading = mpuOk ? mpu.getYaw() : 0.0f;
                _turnTargetHeading = normalizeAngle(_scanStartHeading + _avoidanceEngine.getScanAngle());
                _stateTimer = now;
                _scanStep = 5;
                Serial.printf("🤖 [AUTO SCAN] Khoảng cách PHẢI: %.1f cm. Bước 3: Xoay về giữa...\n", _scanRightDistance);
            }
            break;

        case 5: { // Xoay về lại vị trí giữa ban đầu
            int rampSpeed = updatePwmRamp(100);
            car.rotateLeft(rampSpeed);

            bool turnDone = false;
            if (mpuOk) {
                float currentYaw = mpu.getYaw();
                float diff = abs(normalizeAngle(currentYaw - _scanStartHeading));
                if (diff >= _avoidanceEngine.getScanAngle()) turnDone = true;
            }
            if (!mpuOk || (now - _stateTimer >= 800)) turnDone = true;

            if (turnDone) {
                car.stop();
                _stateTimer = now;
                _scanStep = 6;
            }
            break;
        }

        case 6: { // Đánh giá & Ra Quyết Định
            AvoidanceDirection decision = _avoidanceEngine.evaluateScanDecision(
                _scanFrontDistance, _scanLeftDistance, _scanRightDistance);

            Serial.printf("🤖 [AUTO SCAN] Đánh giá: Front=%.1fcm, Left=%.1fcm, Right=%.1fcm -> Quyết định: %s\n",
                          _scanFrontDistance, _scanLeftDistance, _scanRightDistance,
                          ObstacleAvoidance::directionToString(decision));

            switch (decision) {
                case AVOID_DIR_LEFT:
                    setState(AUTO_ROTATE_LEFT);
                    break;
                case AVOID_DIR_RIGHT:
                    setState(AUTO_ROTATE_RIGHT);
                    break;
                case AVOID_DIR_BACKWARD:
                    setState(AUTO_BACKWARD);
                    break;
                case AVOID_DIR_RECOVER:
                    setState(AUTO_RECOVER);
                    break;
                default:
                    setState(AUTO_FORWARD);
                    break;
            }
            break;
        }
    }
}

void AutoNavigator::handleStateBackward() {
    int rampSpeed = updatePwmRamp(90);
    car.backward(rampSpeed);

    unsigned long now = millis();
    bool targetReached = false;

    // Đo quãng đường lùi bằng Encoder
    if (ENCODER_ENABLED) {
        float traveledCm = fabs(encoderManager.getWheelDistance() - _startEncoderPos) * 100.0f;
        if (traveledCm >= _avoidanceEngine.getBackDistance()) {
            targetReached = true;
        }
    }

    // Fallback theo thời gian an toàn nếu Encoder không bật
    if (!ENCODER_ENABLED || (now - _stateTimer >= 1000)) {
        if (now - _stateTimer >= 800) {
            targetReached = true;
        }
    }

    if (targetReached) {
        car.stop();
        Serial.printf("🤖 [AUTO BACKWARD] Đã lùi xong %.1fcm. Chuyển sang AUTO_SCAN...\n",
                      _avoidanceEngine.getBackDistance());
        setState(AUTO_SCAN);
    }
}

void AutoNavigator::handleStateRotateLeft() {
    int rampSpeed = updatePwmRamp(95);
    car.rotateLeft(rampSpeed);

    unsigned long now = millis();
    bool turnDone = false;

    if (mpuOk) {
        float currentYaw = mpu.getYaw();
        float diff = abs(normalizeAngle(currentYaw - _scanStartHeading));
        if (diff >= _avoidanceEngine.getTurnAngle()) {
            turnDone = true;
        }
    }

    if (!mpuOk || (now - _stateTimer >= 1200)) {
        turnDone = true;
    }

    if (turnDone) {
        car.stop();
        Serial.println(F("🤖 [AUTO ROTATE LEFT] Hoàn tất xoay Trái! Chuyển sang AUTO_FORWARD..."));
        setState(AUTO_FORWARD);
    }
}

void AutoNavigator::handleStateRotateRight() {
    int rampSpeed = updatePwmRamp(95);
    car.rotateRight(rampSpeed);

    unsigned long now = millis();
    bool turnDone = false;

    if (mpuOk) {
        float currentYaw = mpu.getYaw();
        float diff = abs(normalizeAngle(_scanStartHeading - currentYaw));
        if (diff >= _avoidanceEngine.getTurnAngle()) {
            turnDone = true;
        }
    }

    if (!mpuOk || (now - _stateTimer >= 1200)) {
        turnDone = true;
    }

    if (turnDone) {
        car.stop();
        Serial.println(F("🤖 [AUTO ROTATE RIGHT] Hoàn tất xoay Phải! Chuyển sang AUTO_FORWARD..."));
        setState(AUTO_FORWARD);
    }
}

void AutoNavigator::handleStateRecover() {
    unsigned long now = millis();
    int rampSpeed = updatePwmRamp(95);

    // Xoay 180 độ phục hồi
    car.rotateLeft(rampSpeed);

    bool recoverDone = false;
    if (mpuOk) {
        float currentYaw = mpu.getYaw();
        float diff = abs(normalizeAngle(currentYaw - _scanStartHeading));
        if (diff >= 170.0f) {
            recoverDone = true;
        }
    }

    if (!mpuOk || (now - _stateTimer >= 2200)) {
        recoverDone = true;
    }

    if (recoverDone) {
        car.stop();
        _avoidanceEngine.resetRecoveryCount();
        Serial.println(F("🤖 [AUTO RECOVER] Hoàn tất quay 180°! Tiếp tục tìm đường..."));
        setState(AUTO_FORWARD);
    }
}

int AutoNavigator::updatePwmRamp(int targetSpeed) {
    _targetSpeed = targetSpeed;
    unsigned long now = millis();

    if (now - _lastRampTime >= AUTO_RAMP_INTERVAL_MS) {
        _lastRampTime = now;
        if (_currentRampedSpeed < _targetSpeed) {
            _currentRampedSpeed = min(_currentRampedSpeed + AUTO_RAMP_STEP, _targetSpeed);
        } else if (_currentRampedSpeed > _targetSpeed) {
            _currentRampedSpeed = max(_currentRampedSpeed - AUTO_RAMP_STEP, _targetSpeed);
        }
    }
    return _currentRampedSpeed;
}

int AutoNavigator::calculateYawPidCorrection(float targetYaw, float currentYaw) {
    if (!AUTO_PID_ENABLED) return 0;

    unsigned long now = millis();
    float dt = (now - _lastPidTime) / 1000.0f;
    if (dt <= 0.0f) dt = 0.02f;
    _lastPidTime = now;

    float error = normalizeAngle(targetYaw - currentYaw);
    _pidIntegral += error * dt;
    _pidIntegral = constrain(_pidIntegral, -50.0f, 50.0f);

    float derivative = (error - _pidPreviousError) / dt;
    _pidPreviousError = error;

    float output = (AUTO_KP * error) + (AUTO_KI * _pidIntegral) + (AUTO_KD * derivative);
    return constrain((int)output, -AUTO_PID_OUTPUT_CLAMP, AUTO_PID_OUTPUT_CLAMP);
}

float AutoNavigator::normalizeAngle(float angle) {
    while (angle > 180.0f) angle -= 360.0f;
    while (angle < -180.0f) angle += 360.0f;
    return angle;
}

void AutoNavigator::logStructured(const char* eventMsg) {
    float frontDist = HC_SR04_GetFrontDistance();
    float currentYaw = mpuOk ? mpu.getYaw() : 0.0f;
    float encDist = ENCODER_ENABLED ? encoderManager.getWheelDistance() : 0.0f;

    Serial.printf("[AUTO] State: %s | Front: %.1fcm | Left: %.1fcm | Right: %.1fcm | Yaw: %.1f° | Enc: %.2fm | Dir: %s | RecCount: %d | Event: %s\n",
                  auto_run_GetStateName(_currentState),
                  frontDist,
                  _scanLeftDistance,
                  _scanRightDistance,
                  currentYaw,
                  encDist,
                  ObstacleAvoidance::directionToString(_selectedDirection),
                  _avoidanceEngine.getRecoveryCount(),
                  eventMsg ? eventMsg : "None");
}

AutoTelemetryData AutoNavigator::getTelemetryData() const {
    AutoTelemetryData data;
    data.autoState = _currentState;
    data.frontDistance = HC_SR04_GetFrontDistance();
    data.rearDistance = HC_SR04_GetRearDistance();
    data.leftScanDistance = _scanLeftDistance;
    data.rightScanDistance = _scanRightDistance;
    data.currentYaw = mpuOk ? const_cast<MPU6050Sensor&>(mpu).getYaw() : 0.0f;
    data.encoderDistance = ENCODER_ENABLED ? encoderManager.getWheelDistance() : 0.0f;
    data.selectedDirection = _selectedDirection;
    data.obstacleDetected = _avoidanceEngine.isObstacleDetected(data.frontDistance);
    data.recoveryCount = _avoidanceEngine.getRecoveryCount();
    return data;
}

void AutoNavigator::printStatus() {
    logStructured("Status Request");
}
