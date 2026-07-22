/**
 * @file auto_run.cpp
 * @brief Phân hệ chạy tự động nâng cao tích hợp State Machine, PID Giữ hướng Yaw MPU6050,
 *        Clamp PID Output, PWM Ramp mềm, bộ lọc va chạm nhiều mẫu, AUTO_IDLE,
 *        quy trình AUTO_SCAN & Recovery cải tiến, phản hồi Encoder và giao diện Raspberry Pi mở rộng.
 */

#include "robot_global.h"
#include "Config.h"
#include <unordered_map>
#include <functional>
#include "MovementController.h"

// =============================================================================
// BIẾN NỘI BỘ QUẢN LÝ STATE MACHINE VÀ NÂNG CAO
// =============================================================================

// Trạng thái State Machine nội bộ
static unsigned long stateTimer = 0;           // Bộ đếm thời gian non-blocking cho các trạng thái
static unsigned long lastProgressTime = 0;     // Thời điểm cuối cùng robot tiến thẳng thành công (chống mắc kẹt)
static uint8_t obstacleDebounceCounter = 0;   // Bộ đếm xác nhận vật cản liên tiếp
static uint8_t clearDebounceCounter = 0;      // Bộ đếm xác nhận đường thoáng liên tiếp
static uint8_t collisionDebounceCounter = 0;  // Bộ đếm xác nhận va chạm nhiều mẫu liên tiếp
static uint8_t recoveryStep = 0;              // Bước hiện tại trong chu trình Recovery
static uint8_t recoveryAttempts = 0;          // Số lần thử Recovery liên tiếp

// Biến điều khiển PWM Ramp (Tăng/giảm tốc mượt non-blocking)
static int currentRampedSpeed = 0;            // Tốc độ PWM thực tế đang áp dụng
static int targetRampedSpeed = 0;             // Tốc độ PWM mục tiêu muốn đạt tới
static unsigned long lastRampTime = 0;        // Thời điểm cập nhật Ramp gần nhất

// Biến điều khiển PID giữ hướng Yaw bằng MPU6050
static float targetYaw = 0.0f;                // Góc Yaw mục tiêu khi chạy tiến thẳng
static float pidIntegral = 0.0f;              // Tích lũy sai số I
static float pidLastErr = 0.0f;               // Sai số lần trước D
static unsigned long lastPidTime = 0;         // Thời gian tính dt cho PID

// Biến phục vụ quay theo góc MPU6050
static float turnStartYaw = 0.0f;             // Góc Yaw ban đầu trước khi quay
static float desiredTurnAngle = 0.0f;         // Góc cần quay (ví dụ: 90, 180)
static float targetTurnAngle = 0.0f;          // Góc Yaw mục tiêu sau khi quay

// Tốc độ tùy chỉnh qua Raspberry Pi Interface
static int piCustomSpeed = AUTO_MAX_SPEED;

// =============================================================================
// CÁC HÀM BỔ TRỢ NỘI BỘ (INTERNAL HELPERS)
// =============================================================================

/**
 * @brief Chuẩn hóa góc về khoảng [-180.0, 180.0] độ.
 */
static float normalizeAngle(float angle) {
    while (angle > 180.0f) angle -= 360.0f;
    while (angle < -180.0f) angle += 360.0f;
    return angle;
}

/**
 * @brief Lấy tên văn bản đại diện cho trạng thái AutoState để in Debug.
 */
const char* auto_run_GetStateName(AutoState state) {
    switch (state) {
        case AUTO_IDLE:       return "AUTO_IDLE";
        case AUTO_FORWARD:    return "AUTO_FORWARD";
        case AUTO_STOP:       return "AUTO_STOP";
        case AUTO_BACKWARD:   return "AUTO_BACKWARD";
        case AUTO_SCAN:       return "AUTO_SCAN";
        case AUTO_TURN_LEFT:  return "AUTO_TURN_LEFT";
        case AUTO_TURN_RIGHT: return "AUTO_TURN_RIGHT";
        case AUTO_RECOVER:    return "AUTO_RECOVER";
        default:              return "UNKNOWN";
    }
}

/**
 * @brief Bộ tăng/giảm tốc mượt PWM (PWM Ramp) non-blocking.
 */
static int updatePwmRamp(int target) {
    targetRampedSpeed = target;
    unsigned long now = millis();

    if (now - lastRampTime >= AUTO_RAMP_INTERVAL_MS) {
        lastRampTime = now;

        if (currentRampedSpeed < targetRampedSpeed) {
            currentRampedSpeed = min(currentRampedSpeed + AUTO_RAMP_STEP, targetRampedSpeed);
        } else if (currentRampedSpeed > targetRampedSpeed) {
            currentRampedSpeed = max(currentRampedSpeed - AUTO_RAMP_STEP, targetRampedSpeed);
        }
    }
    return currentRampedSpeed;
}

/**
 * @brief Tính toán tốc độ PWM theo khoảng cách mượt mà (Linear scaling).
 */
static int calculateSpeedFromDistance(float dist) {
    int maxSpd = (piCustomSpeed > 0) ? piCustomSpeed : AUTO_MAX_SPEED;
    if (dist >= AUTO_DIST_SLOW_CM) return maxSpd;
    if (dist <= AUTO_DIST_STOP_CM) return 0;

    float ratio = (dist - AUTO_DIST_STOP_CM) / (AUTO_DIST_SLOW_CM - AUTO_DIST_STOP_CM);
    int speed = AUTO_MIN_SPEED + (int)(ratio * (maxSpd - AUTO_MIN_SPEED));
    return constrain(speed, AUTO_MIN_SPEED, maxSpd);
}

/**
 * @brief Bộ điều khiển PID giữ hướng Yaw có áp dụng PID Output Clamp.
 */
static int calculatePIDCorrection(float target, float current) {
    if (!AUTO_PID_ENABLED || !mpuOk) return 0;

    unsigned long now = millis();
    float dt = (now - lastPidTime) / 1000.0f;
    if (dt <= 0.0f || dt > 0.5f) dt = 0.02f;
    lastPidTime = now;

    float err = normalizeAngle(target - current);
    pidIntegral += err * dt;
    pidIntegral = constrain(pidIntegral, -30.0f, 30.0f); // Anti-windup

    float derivative = (err - pidLastErr) / dt;
    pidLastErr = err;

    float output = (AUTO_KP * err) + (AUTO_KI * pidIntegral) + (AUTO_KD * derivative);

    // Áp dụng PID Output Clamp để tránh rung lắc động cơ
    return (int)constrain(output, -AUTO_PID_OUTPUT_CLAMP, AUTO_PID_OUTPUT_CLAMP);
}

/**
 * @brief Kiểm tra va chạm mạnh có lọc xác nhận nhiều mẫu liên tiếp (Debounce).
 */
static bool checkCollisionDetected() {
    if (!AUTO_COLLISION_ENABLED || !mpuOk) return false;

    float ax = mpu.getAccelX();
    float ay = mpu.getAccelY();
    float az = mpu.getAccelZ();

    float accelMagnitude = sqrt(ax * ax + ay * ay + az * az);

    if (accelMagnitude >= AUTO_COLLISION_THRESHOLD_MS2) {
        collisionDebounceCounter++;
        if (collisionDebounceCounter >= AUTO_COLLISION_DEBOUNCE_COUNT) {
            return true;
        }
    } else {
        collisionDebounceCounter = 0;
    }
    return false;
}

/**
 * @brief Chuyển đổi trạng thái Auto State Machine non-blocking.
 */
static void switchAutoState(AutoState newState) {
    currentAutoState = newState;
    stateTimer = millis();

    switch (newState) {
        case AUTO_IDLE:
            car.stop();
            currentRampedSpeed = 0;
            currentMoveDir = "chờ (AUTO_IDLE)";
            currentSpeed = 0;
            break;

        case AUTO_FORWARD:
            currentMoveDir = "tiến thẳng tự động (PID)";
            if (mpuOk) {
                targetYaw = mpu.getYaw();
                pidIntegral = 0.0f;
                pidLastErr = 0.0f;
                lastPidTime = millis();
            }
            break;

        case AUTO_STOP:
            car.stop();
            currentRampedSpeed = 0;
            currentMoveDir = "dừng (AUTO)";
            currentSpeed = 0;
            break;

        case AUTO_BACKWARD:
            currentMoveDir = "lùi lại (Recovery)";
            currentSpeed = AUTO_TURN_DEFAULT_SPEED;
            break;

        case AUTO_SCAN:
            car.stop();
            currentRampedSpeed = 0;
            currentMoveDir = "dừng quét & đo cảm biến";
            currentSpeed = 0;
            clearDebounceCounter = 0;
            break;

        case AUTO_TURN_LEFT:
            if (mpuOk) {
                turnStartYaw = mpu.getYaw();
                targetTurnAngle = normalizeAngle(turnStartYaw + desiredTurnAngle);
            }
            currentMoveDir = "xoay trái theo góc";
            currentSpeed = AUTO_TURN_DEFAULT_SPEED;
            break;

        case AUTO_TURN_RIGHT:
            if (mpuOk) {
                turnStartYaw = mpu.getYaw();
                targetTurnAngle = normalizeAngle(turnStartYaw - desiredTurnAngle);
            }
            currentMoveDir = "xoay phải theo góc";
            currentSpeed = AUTO_TURN_DEFAULT_SPEED;
            break;

        case AUTO_RECOVER:
            car.stop();
            currentRampedSpeed = 0;
            currentMoveDir = "quy trình Recovery";
            currentSpeed = 0;
            break;
    }
}

// =============================================================================
// APIS IMPLEMENTATION
// =============================================================================

void auto_run_Init() {
    auto_run_ResetState();
}

void auto_run_ResetState() {
    currentAutoState = AUTO_IDLE;
    stateTimer = millis();
    lastProgressTime = millis();
    obstacleDebounceCounter = 0;
    clearDebounceCounter = 0;
    collisionDebounceCounter = 0;
    recoveryStep = 0;
    recoveryAttempts = 0;
    currentRampedSpeed = 0;
    targetRampedSpeed = 0;
    pidIntegral = 0.0f;
    pidLastErr = 0.0f;
    piCustomSpeed = AUTO_MAX_SPEED;
    if (mpuOk) {
        targetYaw = mpu.getYaw();
    } else {
        targetYaw = 0.0f;
    }
    isAvoidanceActive = false;
}

/**
 * @brief Mở rộng hỗ trợ bộ lệnh đầy đủ cho Raspberry Pi.
 */
struct PiCommand {
    String actionName;
    std::function<void(const String&)> handler;
};

static std::unordered_map<std::string, PiCommand> piCommandRegistry;

static void initPiCommandRegistry() {
    if (!piCommandRegistry.empty()) return;

    piCommandRegistry["forward"] = {"Forward", [](const String&) {
        switchAutoState(AUTO_FORWARD);
    }};
    piCommandRegistry["run"] = {"Forward", [](const String&) {
        switchAutoState(AUTO_FORWARD);
    }};
    piCommandRegistry["backward"] = {"Backward", [](const String&) {
        switchAutoState(AUTO_BACKWARD);
    }};
    piCommandRegistry["left"] = {"Rotate Left", [](const String&) {
        desiredTurnAngle = 90.0f;
        switchAutoState(AUTO_TURN_LEFT);
    }};
    piCommandRegistry["rotate_left"] = {"Rotate Left", [](const String&) {
        desiredTurnAngle = 90.0f;
        switchAutoState(AUTO_TURN_LEFT);
    }};
    piCommandRegistry["right"] = {"Rotate Right", [](const String&) {
        desiredTurnAngle = 90.0f;
        switchAutoState(AUTO_TURN_RIGHT);
    }};
    piCommandRegistry["rotate_right"] = {"Rotate Right", [](const String&) {
        desiredTurnAngle = 90.0f;
        switchAutoState(AUTO_TURN_RIGHT);
    }};
    piCommandRegistry["stop"] = {"Stop", [](const String&) {
        switchAutoState(AUTO_IDLE);
    }};
    piCommandRegistry["idle"] = {"Stop", [](const String&) {
        switchAutoState(AUTO_IDLE);
    }};
    piCommandRegistry["pause"] = {"Stop", [](const String&) {
        switchAutoState(AUTO_IDLE);
    }};
    piCommandRegistry["recover"] = {"Recover", [](const String&) {
        recoveryStep = 0;
        switchAutoState(AUTO_RECOVER);
    }};
    piCommandRegistry["set_speed"] = {"Set Speed", [](const String& param) {
        piCustomSpeed = constrain(param.toInt(), 0, 255);
        moveControl.setSpeed(piCustomSpeed);
        Serial.printf("   [PI INTERFACE] Đặt tốc độ mặc định Auto = %d PWM\n", piCustomSpeed);
    }};
    piCommandRegistry["set_target_angle"] = {"Set Target Angle", [](const String& param) {
        targetYaw = param.toFloat();
        moveControl.setTargetAngle(targetYaw);
        Serial.printf("   [PI INTERFACE] Đặt góc Yaw mục tiêu = %.1f deg\n", targetYaw);
    }};
    piCommandRegistry["turn_left"] = {"Rotate Left By Angle", [](const String& param) {
        desiredTurnAngle = param.toFloat();
        if (desiredTurnAngle <= 0) desiredTurnAngle = 90.0f;
        switchAutoState(AUTO_TURN_LEFT);
    }};
    piCommandRegistry["turn_right"] = {"Rotate Right By Angle", [](const String& param) {
        desiredTurnAngle = param.toFloat();
        if (desiredTurnAngle <= 0) desiredTurnAngle = 90.0f;
        switchAutoState(AUTO_TURN_RIGHT);
    }};
}

void auto_run_ProcessPiCommand(const String& cmd) {
    String origCmd = cmd;
    origCmd.trim();
    String c = origCmd;
    c.toLowerCase();

    int spaceIdx = c.indexOf(' ');
    String action = (spaceIdx == -1) ? c : c.substring(0, spaceIdx);
    String param = (spaceIdx == -1) ? "" : origCmd.substring(spaceIdx + 1);
    param.trim();

    initPiCommandRegistry();

    std::string actionStr = std::string(action.c_str());
    auto it = piCommandRegistry.find(actionStr);
    if (it != piCommandRegistry.end()) {
        // Output Serial Log in required format
        Serial.println(F("=========================="));
        Serial.print(F("RX: "));
        Serial.println(origCmd);
        Serial.print(F("ACTION: "));
        Serial.println(it->second.actionName);
        Serial.println(F("=========================="));

        it->second.handler(param);
    } else {
        Serial.println(F("⚠️ [PI INTERFACE] Lệnh không hợp lệ! Hỗ trợ: forward, backward, left, right, rotate_left, rotate_right, stop, recover, set_speed <val>, set_target_angle <val>"));
    }
}

void auto_run_Update() {
    // Chỉ hoạt động khi xe ở chế độ tự động (MODE_AUTO)
    if (currentMode != MODE_AUTO) return;

    // 1. Đợi 1.5 giây sau khi bật AUTO để cảm biến siêu âm thu thập đủ mẫu và ổn định bộ lọc
    if (millis() - autoModeStartTime < 1500) {
        car.stop();
        currentMoveDir = "đồng bộ cảm biến (AUTO)";
        currentSpeed = 0;
        return;
    }

    // 2. Kiểm tra an toàn: Cảm biến trước OFFLINE
    if (!HC_SR04_FrontOnline()) {
        if (!bypassSensorCheck) {
            car.stop();
            currentMoveDir = "dừng (lỗi cảm biến)";
            currentSpeed = 0;
            isAvoidanceActive = false;
            currentMode = MODE_MANUAL;
            Serial.println(F("⚠️ [AUTO LỖI] Cảm biến siêu âm trước OFFLINE! Dừng khẩn cấp và chuyển về MANUAL."));
            return;
        } else {
            static unsigned long lastBypassWarnTime = 0;
            if (millis() - lastBypassWarnTime > 1000) {
                lastBypassWarnTime = millis();
                Serial.println(F("⚠️ [AUTO CẢNH BÁO] Cảm biến trước OFFLINE nhưng Bypass đang BẬT. Xe tiếp tục chạy!"));
            }
        }
    }

    // 3. Kiểm tra va chạm mạnh có lọc xác nhận nhiều mẫu liên tiếp
    if (checkCollisionDetected()) {
        car.stop();
        currentMoveDir = "DỪNG (VA CHẠM MẠNH)";
        currentSpeed = 0;
        isAvoidanceActive = false;
        currentMode = MODE_MANUAL;
        MH_FMD_Beep(1500);
        Serial.println(F("🚨 [CẢNH BÁO VA CHẠM] Phát hiện va chạm mạnh liên tiếp! Dừng động cơ và chuyển về chế độ MANUAL."));
        return;
    }

    // 4. Lấy khoảng cách cảm biến trước đã qua lọc Median
    float frontDist = HC_SR04_GetFrontDistance();
    unsigned long now = millis();

    // =========================================================================
    // STATE MACHINE DÙNG MILLIS()
    // =========================================================================
    switch (currentAutoState) {

        // ---------------------------------------------------------------------
        // STATE 0: AUTO_IDLE - Trạng thái chờ / tạm dừng
        // ---------------------------------------------------------------------
        case AUTO_IDLE: {
            car.stop();
            currentRampedSpeed = 0;
            currentMoveDir = "chờ (AUTO_IDLE)";
            currentSpeed = 0;

            // Tự động chuyển sang AUTO_FORWARD sau 1.5s nếu vừa khởi tạo
            if (now - autoModeStartTime >= 1500 && now - stateTimer >= 500) {
                switchAutoState(AUTO_FORWARD);
            }
            break;
        }

        // ---------------------------------------------------------------------
        // STATE 1: AUTO_FORWARD - Tiến thẳng tự động kèm PWM Ramp & PID Yaw
        // ---------------------------------------------------------------------
        case AUTO_FORWARD: {
            // A. Kiểm tra lọc xác nhận vật cản
            if (frontDist <= AUTO_DIST_STOP_CM) {
                obstacleDebounceCounter++;
                clearDebounceCounter = 0;
            } else if (frontDist > AUTO_DIST_CLEAR_CM) {
                clearDebounceCounter++;
                if (clearDebounceCounter >= AUTO_OBSTACLE_DEBOUNCE_COUNT) {
                    obstacleDebounceCounter = 0;
                }
            } else {
                obstacleDebounceCounter = 0;
            }

            // B. Khi phát hiện vật cản đủ số lần quy định
            if (obstacleDebounceCounter >= AUTO_OBSTACLE_DEBOUNCE_COUNT) {
                isAvoidanceActive = true;
                Serial.printf("🤖 [AUTO] Phát hiện vật cản ở %.1fcm! Kích hoạt Recovery...\n", frontDist);
                recoveryStep = 0;
                switchAutoState(AUTO_RECOVER);
                break;
            }

            // C. Tính tốc độ mục tiêu theo khoảng cách và áp dụng PWM Ramp mượt
            int rawTargetSpeed = calculateSpeedFromDistance(frontDist);
            int rampedSpeed = updatePwmRamp(rawTargetSpeed);
            currentSpeed = rampedSpeed;

            // D. Điều khiển tiến thẳng kèm PID giữ hướng Yaw (Clamp) & Encoder
            if (rampedSpeed > 0) {
                if (AUTO_PID_ENABLED && mpuOk) {
                    int correction = calculatePIDCorrection(targetYaw, mpu.getYaw());
                    int leftSpeed  = constrain(rampedSpeed - correction, 0, 255);
                    int rightSpeed = constrain(rampedSpeed + correction, 0, 255);
                    car.setAllMotor(leftSpeed, rightSpeed, leftSpeed, rightSpeed);
                } else {
                    car.forward(rampedSpeed);
                }
                lastProgressTime = now;
            } else {
                car.stop();
            }

            // E. Kiểm tra chống mắc kẹt (Stuck Prevention)
            if (now - lastProgressTime > AUTO_STUCK_TIMEOUT_MS) {
                Serial.println(F("⚠️ [AUTO STUCK] Xe không thể tiến thẳng trong thời gian dài! Kích hoạt Recovery..."));
                recoveryStep = 0;
                switchAutoState(AUTO_RECOVER);
            }
            break;
        }

        // ---------------------------------------------------------------------
        // STATE 2: AUTO_STOP - Dừng tạm thời
        // ---------------------------------------------------------------------
        case AUTO_STOP: {
            car.stop();
            if (now - stateTimer >= 200) {
                switchAutoState(AUTO_SCAN);
            }
            break;
        }

        // ---------------------------------------------------------------------
        // STATE 3: AUTO_BACKWARD - Lùi lại ngắn khi gặp vật cản
        // ---------------------------------------------------------------------
        case AUTO_BACKWARD: {
            int rampedSpeed = updatePwmRamp(AUTO_TURN_DEFAULT_SPEED);
            car.backward(rampedSpeed);
            if (now - stateTimer >= AUTO_BACKWARD_TIME_MS) {
                switchAutoState(AUTO_SCAN);
            }
            break;
        }

        // ---------------------------------------------------------------------
        // STATE 4: AUTO_SCAN - Dừng xe, chờ cảm biến ổn định & xác nhận an toàn
        // ---------------------------------------------------------------------
        case AUTO_SCAN: {
            car.stop();
            currentRampedSpeed = 0;

            // Đợi ổn định cảm biến trong AUTO_SCAN_TIME_MS
            if (now - stateTimer >= AUTO_SCAN_TIME_MS) {
                if (frontDist > AUTO_DIST_CLEAR_CM) {
                    clearDebounceCounter++;
                    if (clearDebounceCounter >= AUTO_OBSTACLE_DEBOUNCE_COUNT) {
                        isAvoidanceActive = false;
                        recoveryAttempts = 0; // Reset đếm recovery khi đường đã thoáng
                        Serial.println(F("🤖 [AUTO_SCAN] Đường phía trước đã thoáng! Tiếp tục tiến thẳng."));
                        switchAutoState(AUTO_FORWARD);
                    }
                } else {
                    clearDebounceCounter = 0;
                    // Đường vẫn bị chặn -> Chuyển sang bước Recovery tiếp theo
                    Serial.printf("🤖 [AUTO_SCAN] Đường vẫn bị chặn (%.1fcm). Chuyển sang bước né tiếp theo...\n", frontDist);
                    switchAutoState(AUTO_RECOVER);
                }
            }
            break;
        }

        // ---------------------------------------------------------------------
        // STATE 5: AUTO_TURN_LEFT - Quay trái theo góc MPU6050
        // ---------------------------------------------------------------------
        case AUTO_TURN_LEFT: {
            int rampedSpeed = updatePwmRamp(AUTO_TURN_DEFAULT_SPEED);
            car.rotateLeft(rampedSpeed);
            bool turnComplete = false;

            if (mpuOk) {
                float currentYaw = mpu.getYaw();
                float diff = abs(normalizeAngle(currentYaw - targetTurnAngle));
                if (diff <= AUTO_TURN_TOLERANCE_DEG) {
                    turnComplete = true;
                }
            }

            if (!mpuOk || (now - stateTimer >= AUTO_TURN_TIMEOUT_MS)) {
                turnComplete = true;
            }

            if (turnComplete) {
                car.stop();
                switchAutoState(AUTO_SCAN); // Luôn chuyển sang AUTO_SCAN sau khi quay
            }
            break;
        }

        // ---------------------------------------------------------------------
        // STATE 6: AUTO_TURN_RIGHT - Quay phải theo góc MPU6050
        // ---------------------------------------------------------------------
        case AUTO_TURN_RIGHT: {
            int rampedSpeed = updatePwmRamp(AUTO_TURN_DEFAULT_SPEED);
            car.rotateRight(rampedSpeed);
            bool turnComplete = false;

            if (mpuOk) {
                float currentYaw = mpu.getYaw();
                float diff = abs(normalizeAngle(currentYaw - targetTurnAngle));
                if (diff <= AUTO_TURN_TOLERANCE_DEG) {
                    turnComplete = true;
                }
            }

            if (!mpuOk || (now - stateTimer >= AUTO_TURN_TIMEOUT_MS)) {
                turnComplete = true;
            }

            if (turnComplete) {
                car.stop();
                switchAutoState(AUTO_SCAN); // Luôn chuyển sang AUTO_SCAN sau khi quay
            }
            break;
        }

        // ---------------------------------------------------------------------
        // STATE 7: AUTO_RECOVER - Quy trình Recovery đa bước nghiêm ngặt
        // Mỗi bước quay đều bắt buộc qua AUTO_SCAN kiểm tra khoảng cách
        // ---------------------------------------------------------------------
        case AUTO_RECOVER: {
            switch (recoveryStep) {
                case 0:
                    // Bước 0: Dừng -> Lùi ngắn -> SCAN
                    Serial.println(F("🤖 [RECOVERY 1/4] Lùi lại ngắn..."));
                    recoveryStep = 1;
                    switchAutoState(AUTO_BACKWARD);
                    break;

                case 1:
                    // Bước 1: Quay trái 90° -> SCAN
                    Serial.println(F("🤖 [RECOVERY 2/4] Thử xoay trái 90°..."));
                    desiredTurnAngle = 90.0f;
                    recoveryStep = 2;
                    switchAutoState(AUTO_TURN_LEFT);
                    break;

                case 2:
                    // Bước 2: Quay phải 180° (đổi sang bên phải) -> SCAN
                    Serial.println(F("🤖 [RECOVERY 3/4] Phía trái vẫn bị chặn! Xoay sang phải 180°..."));
                    desiredTurnAngle = 180.0f;
                    recoveryStep = 3;
                    switchAutoState(AUTO_TURN_RIGHT);
                    break;

                case 3:
                    // Bước 3: Xoay 180° quay đầu -> SCAN
                    Serial.println(F("🤖 [RECOVERY 4/4] Cả hai phía bị chặn! Xoay 180° quay đầu..."));
                    desiredTurnAngle = 180.0f;
                    recoveryStep = 4;
                    switchAutoState(AUTO_TURN_RIGHT);
                    break;

                default: {
                    recoveryAttempts++;
                    Serial.printf("⚠️ [RECOVERY THẤT BẠI] Đã thử chu trình Recovery %d/%d lần.\n",
                                  recoveryAttempts, AUTO_RECOVERY_RETRY_LIMIT);

                    if (recoveryAttempts >= AUTO_RECOVERY_RETRY_LIMIT) {
                        car.stop();
                        currentMoveDir = "DỪNG (MẮC KẸT HOÀN TOÀN)";
                        currentSpeed = 0;
                        isAvoidanceActive = false;
                        currentMode = MODE_MANUAL;
                        MH_FMD_Beep(1000);
                        Serial.println(F("🚨 [AUTO LỖI] Không thể tìm đường thoát sau nhiều lần Recovery! Chuyển về MANUAL."));
                    } else {
                        recoveryStep = 0;
                        switchAutoState(AUTO_BACKWARD);
                    }
                    break;
                }
            }
            break;
        }
    }
}
