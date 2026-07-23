/**
 * @file auto_run.cpp
 * @brief Phân hệ chạy tự động (Auto Mode) nâng cao cho Robot Mecanum.
 *        Vận hành theo State Machine thuần túy, non-blocking bằng millis().
 *        Điều khiển tốc độ theo khoảng cách mượt mà (85 - 140 PWM),
 *        không bị khựng bánh, lùi xe bằng Encoder (~15cm, PWM 90), 
 *        tự động quét hướng né vật cản chính xác bằng MPU6050 & HC-SR04.
 */

#include "robot_global.h"
#include "Config.h"
#include <unordered_map>
#include <functional>
#include "MovementController.h"
#include "EncoderManager.h"
#include "Managers/RobotStateManager.h"

// =============================================================================
// BIẾN NỘI BỘ QUẢN LÝ STATE MACHINE VÀ NÂNG CAO
// =============================================================================

static unsigned long stateTimer = 0;           // Bộ đếm thời gian non-blocking cho các trạng thái
static unsigned long lastProgressTime = 0;     // Thời điểm cuối cùng robot tiến thẳng thành công
static uint8_t collisionDebounceCounter = 0;  // Bộ đếm xác nhận va chạm nhiều mẫu liên tiếp

// Biến điều khiển PWM Ramp (Tăng/giảm tốc mượt non-blocking)
static int currentRampedSpeed = 0;            // Tốc độ PWM thực tế đang áp dụng
static int targetRampedSpeed = 0;             // Tốc độ PWM mục tiêu muốn đạt tới
static unsigned long lastRampTime = 0;        // Thời điểm cập nhật Ramp gần nhất

// Biến phục vụ quy trình Lùi Encoder & Quét góc MPU6050
static float startBackwardDist = 0.0f;        // Vị trí Encoder ban đầu trước khi lùi
static float turnStartYaw = 0.0f;             // Góc Yaw ban đầu trước khi quay
static float desiredTurnAngle = 0.0f;         // Góc cần quay (ví dụ: 25, 50 độ)
static float scanLeftDist = 0.0f;             // Khoảng cách đo được ở hướng Trái 25°
static float scanRightDist = 0.0f;            // Khoảng cách đo được ở hướng Phải 25°
static uint8_t scanPhase = 0;                 // Bước hiện tại trong quy trình AUTO_SCAN

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
        case AUTO_IDLE:          return "AUTO_IDLE";
        case AUTO_FORWARD:       return "AUTO_FORWARD";
        case AUTO_SLOW_FORWARD:  return "AUTO_SLOW_FORWARD";
        case AUTO_STOP:          return "AUTO_STOP";
        case AUTO_BACKWARD:      return "AUTO_BACKWARD";
        case AUTO_SCAN:          return "AUTO_SCAN";
        case AUTO_ROTATE_LEFT:   return "AUTO_ROTATE_LEFT";
        case AUTO_ROTATE_RIGHT:  return "AUTO_ROTATE_RIGHT";
        case AUTO_RECOVER:       return "AUTO_RECOVER";
        default:                 return "UNKNOWN";
    }
}

/**
 * @brief Bộ tăng/giảm tốc mượt PWM (PWM Ramp) non-blocking tránh phanh đột ngột.
 */
static int updatePwmRamp(int target) {
    targetRampedSpeed = target;
    unsigned long now = millis();

    if (now - lastRampTime >= 15) {
        lastRampTime = now;

        if (currentRampedSpeed < targetRampedSpeed) {
            currentRampedSpeed = min(currentRampedSpeed + 15, targetRampedSpeed);
        } else if (currentRampedSpeed > targetRampedSpeed) {
            currentRampedSpeed = max(currentRampedSpeed - 15, targetRampedSpeed);
        }
    }
    return currentRampedSpeed;
}

/**
 * @brief Tính toán tốc độ PWM theo khoảng cách cảm biến trước.
 *        Đảm bảo PWM tối thiểu >= 85 để vượt qua ma sát cơ học bánh Mecanum:
 *        - Distance > 100 cm  => PWM = 140 (Chạy mượt nhanh)
 *        - 80 ~ 100 cm       => PWM = 120
 *        - 60 ~ 80 cm        => PWM = 105
 *        - 40 ~ 60 cm        => PWM = 90
 *        - 25 ~ 40 cm        => PWM = 85
 *        - <= 25 cm          => STOP (0)
 */
static int calculateSpeedFromDistance(float dist) {
    if (dist <= 0.0f || dist >= 400.0f) return 140; // Bỏ qua nhiễu / thoáng hoàn toàn
    if (dist <= 25.0f) return 0;
    if (dist <= 40.0f) return 85;
    if (dist <= 60.0f) return 90;
    if (dist <= 80.0f) return 105;
    if (dist <= 100.0f) return 120;
    return 140; // > 100 cm
}

/**
 * @brief Chuyển đổi trạng thái Auto State Machine non-blocking kèm Log chi tiết.
 */
static void switchAutoState(AutoState newState) {
    Serial.printf("🤖 [AUTO STATE] %s -> %s\n", 
                  auto_run_GetStateName(currentAutoState), 
                  auto_run_GetStateName(newState));
    currentAutoState = newState;
    stateTimer = millis();

    if (newState == AUTO_SCAN || newState == AUTO_BACKWARD || 
        newState == AUTO_ROTATE_LEFT || newState == AUTO_ROTATE_RIGHT || 
        newState == AUTO_RECOVER) {
        HC_SR04_ResetBuffer();
    }

    switch (newState) {
        case AUTO_IDLE:
            car.stop();
            currentRampedSpeed = 0;
            currentMoveDir = "chờ (AUTO_IDLE)";
            currentSpeed = 0;
            break;

        case AUTO_FORWARD:
            currentMoveDir = "tiến thẳng tự động";
            break;

        case AUTO_SLOW_FORWARD:
            currentMoveDir = "tiến chậm (tiếp cận vật cản)";
            break;

        case AUTO_STOP:
            car.stop();
            currentRampedSpeed = 0;
            currentMoveDir = "dừng xe hoàn toàn (AUTO_STOP)";
            currentSpeed = 0;
            break;

        case AUTO_BACKWARD:
            currentMoveDir = "lùi xe Encoder (~15cm, PWM 90)";
            currentSpeed = 90;
            startBackwardDist = encoderManager.getWheelDistance();
            break;

        case AUTO_SCAN:
            car.stop();
            currentRampedSpeed = 0;
            currentMoveDir = "đọc & quét cảm biến tìm hướng";
            currentSpeed = 0;
            break;

        case AUTO_ROTATE_LEFT:
            currentMoveDir = "quay trái MPU6050 (PWM 90)";
            currentSpeed = 90;
            if (mpuOk) {
                turnStartYaw = mpu.getYaw();
            }
            break;

        case AUTO_ROTATE_RIGHT:
            currentMoveDir = "quay phải MPU6050 (PWM 90)";
            currentSpeed = 90;
            if (mpuOk) {
                turnStartYaw = mpu.getYaw();
            }
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
    collisionDebounceCounter = 0;
    currentRampedSpeed = 0;
    targetRampedSpeed = 0;
    piCustomSpeed = AUTO_MAX_SPEED;
    scanPhase = 0;
    isAvoidanceActive = false;
}

void auto_run_ProcessPiCommand(const String& cmd) {
    // Process commands when needed
}

/**
 * @brief Vòng lặp chính xử lý State Machine Auto Mode (Non-blocking).
 */
void auto_run_Update() {
    if (currentMode != MODE_AUTO) return;

    // 1. Đợi 1.5 giây sau khi bật AUTO để cảm biến siêu âm thu thập đủ mẫu và ổn định bộ lọc
    if (millis() - autoModeStartTime < 1500) {
        car.stop();
        currentMoveDir = "đồng bộ cảm biến (AUTO)";
        currentSpeed = 0;
        return;
    }

    // 2. Lấy khoảng cách cảm biến trước đã qua lọc Median
    float frontDist = HC_SR04_GetFrontDistance();
    unsigned long now = millis();

    // =========================================================================
    // STATE MACHINE THUẦN TÚY DÙNG MILLIS()
    // =========================================================================
    switch (currentAutoState) {

        case AUTO_IDLE: {
            car.stop();
            currentRampedSpeed = 0;
            if (now - autoModeStartTime >= 1500 && now - stateTimer >= 500) {
                switchAutoState(AUTO_FORWARD);
            }
            break;
        }

        case AUTO_FORWARD: {
            // Gặp vật cản <= 25cm -> Dừng xe
            if (frontDist > 0.0f && frontDist <= 25.0f) {
                Serial.printf("🤖 [AUTO FORWARD] Phát hiện vật cản (%.1fcm <= 25cm)! Dừng xe...\n", frontDist);
                switchAutoState(AUTO_STOP);
                break;
            } 
            else if (frontDist > 0.0f && frontDist <= 60.0f) {
                switchAutoState(AUTO_SLOW_FORWARD);
                break;
            }

            int targetSpd = calculateSpeedFromDistance(frontDist);
            int rampedSpeed = updatePwmRamp(targetSpd);
            currentSpeed = rampedSpeed;

            if (rampedSpeed > 0) {
                car.forward(rampedSpeed);
                lastProgressTime = now;
            } else {
                car.stop();
            }

            if (now - lastProgressTime > AUTO_STUCK_TIMEOUT_MS) {
                Serial.println(F("⚠️ [AUTO STUCK] Xe không thể tiến thẳng trong thời gian dài! Kích hoạt Recovery..."));
                switchAutoState(AUTO_RECOVER);
            }
            break;
        }

        case AUTO_SLOW_FORWARD: {
            if (frontDist > 0.0f && frontDist <= 25.0f) {
                Serial.printf("🤖 [AUTO SLOW FORWARD] Phát hiện vật cản (%.1fcm <= 25cm)! Dừng xe...\n", frontDist);
                switchAutoState(AUTO_STOP);
                break;
            } 
            else if (frontDist > 60.0f) {
                switchAutoState(AUTO_FORWARD);
                break;
            }

            int targetSpd = calculateSpeedFromDistance(frontDist);
            int rampedSpeed = updatePwmRamp(targetSpd);
            currentSpeed = rampedSpeed;

            if (rampedSpeed > 0) {
                car.forward(rampedSpeed);
                lastProgressTime = now;
            } else {
                car.stop();
            }
            break;
        }

        case AUTO_STOP: {
            car.stop();
            currentRampedSpeed = 0;
            if (now - stateTimer >= 200) {
                scanPhase = 0;
                switchAutoState(AUTO_BACKWARD);
            }
            break;
        }

        case AUTO_BACKWARD: {
            int rampedSpeed = updatePwmRamp(90); // PWM lùi = 90 (đủ lực không bị giật khựng)
            car.backward(rampedSpeed);

            bool reached15cm = false;
            if (ENCODER_ENABLED) {
                float traveled = fabs(encoderManager.getWheelDistance() - startBackwardDist);
                if (traveled >= 0.15f) { // 15 cm
                    reached15cm = true;
                }
            }
            
            if (!ENCODER_ENABLED || (now - stateTimer >= 600)) {
                if (now - stateTimer >= 600) {
                    reached15cm = true;
                }
            }

            if (reached15cm) {
                car.stop();
                Serial.println(F("🤖 [AUTO BACKWARD] Đã lùi xong 15cm bằng Encoder (PWM 90)! Kiểm tra lại cảm biến..."));
                switchAutoState(AUTO_SCAN);
            }
            break;
        }

        case AUTO_SCAN: {
            car.stop();
            currentRampedSpeed = 0;

            if (now - stateTimer >= 300) { // Chờ 300ms thu thập mẫu siêu âm mới ở vị trí mới
                if (scanPhase == 0) {
                    // Đọc cảm biến phía trước sau khi lùi 15cm
                    if (frontDist > 80.0f) {
                        Serial.printf("🤖 [AUTO SCAN] Phía trước đã thoáng (%.1fcm > 80cm)! Tiếp tục tiến thẳng...\n", frontDist);
                        switchAutoState(AUTO_FORWARD);
                    } else {
                        Serial.printf("🤖 [AUTO SCAN] Phía trước vẫn vướng (%.1fcm <= 80cm). Quay TRÁI 25° để quét...\n", frontDist);
                        desiredTurnAngle = 25.0f;
                        scanPhase = 1;
                        switchAutoState(AUTO_ROTATE_LEFT);
                    }
                } else if (scanPhase == 1) {
                    // Sau khi quay Trái 25° -> Đọc sensor hướng Trái
                    scanLeftDist = frontDist;
                    Serial.printf("🤖 [AUTO SCAN] Khoảng cách hướng TRÁI 25°: %.1f cm\n", scanLeftDist);

                    if (scanLeftDist > 80.0f) {
                        Serial.println(F("🤖 [AUTO SCAN] Hướng TRÁI thoáng (>80cm)! Chọn hướng TRÁI."));
                        switchAutoState(AUTO_FORWARD);
                    } else {
                        Serial.println(F("🤖 [AUTO SCAN] Hướng TRÁI vướng. Quay PHẢI 50° tổng cộng (từ Trái sang Phải)..."));
                        desiredTurnAngle = 50.0f;
                        scanPhase = 2;
                        switchAutoState(AUTO_ROTATE_RIGHT);
                    }
                } else if (scanPhase == 2) {
                    // Sau khi quay Phải 50° -> Đọc sensor hướng Phải
                    scanRightDist = frontDist;
                    Serial.printf("🤖 [AUTO SCAN] Khoảng cách hướng PHẢI 25°: %.1f cm\n", scanRightDist);

                    if (scanRightDist > 80.0f || (scanRightDist > scanLeftDist && scanRightDist > 50.0f)) {
                        Serial.println(F("🤖 [AUTO SCAN] Hướng PHẢI tốt hơn! Chọn hướng PHẢI."));
                        switchAutoState(AUTO_FORWARD);
                    } else {
                        // Cả 2 hướng đều bị chặn (<= 50cm) -> Quay về giữa và lùi thêm
                        Serial.printf("🤖 [AUTO SCAN] Cả 2 hướng đều bị chặn (Trái: %.1fcm, Phải: %.1fcm)! Quay về giữa & Lùi thêm...\n", scanLeftDist, scanRightDist);
                        desiredTurnAngle = 25.0f;
                        scanPhase = 3;
                        switchAutoState(AUTO_ROTATE_LEFT);
                    }
                } else if (scanPhase == 3) {
                    // Đã quay về giữa -> Lùi thêm 15cm & quét lại
                    scanPhase = 0;
                    switchAutoState(AUTO_BACKWARD);
                }
            }
            break;
        }

        case AUTO_ROTATE_LEFT: {
            int rampedSpeed = updatePwmRamp(90); // PWM quay = 90 (quay mượt không bị khựng bánh)
            car.rotateLeft(rampedSpeed);

            bool turnComplete = false;
            if (mpuOk) {
                float currentYaw = mpu.getYaw();
                float diff = abs(normalizeAngle(currentYaw - turnStartYaw));
                if (diff >= desiredTurnAngle) {
                    turnComplete = true;
                }
            }

            if (!mpuOk || (now - stateTimer >= 2000)) { // Timeout an toàn 2.0s
                turnComplete = true;
            }

            if (turnComplete) {
                car.stop();
                switchAutoState(AUTO_SCAN);
            }
            break;
        }

        case AUTO_ROTATE_RIGHT: {
            int rampedSpeed = updatePwmRamp(90); // PWM quay = 90 (quay mượt không bị khựng bánh)
            car.rotateRight(rampedSpeed);

            bool turnComplete = false;
            if (mpuOk) {
                float currentYaw = mpu.getYaw();
                float diff = abs(normalizeAngle(turnStartYaw - currentYaw));
                if (diff >= desiredTurnAngle) {
                    turnComplete = true;
                }
            }

            if (!mpuOk || (now - stateTimer >= 2000)) { // Timeout an toàn 2.0s
                turnComplete = true;
            }

            if (turnComplete) {
                car.stop();
                switchAutoState(AUTO_SCAN);
            }
            break;
        }

        case AUTO_RECOVER: {
            scanPhase = 0;
            switchAutoState(AUTO_BACKWARD);
            break;
        }
    }
}
