/**
 * @file auto_run.cpp
 * @brief Phân hệ chạy tự động và tránh vật cản tự động dựa trên cảm biến siêu âm.
 */

#include "robot_global.h"

// =============================================================================
// APIS IMPLEMENTATION
// =============================================================================

void auto_run_Init() {
    // Dự phòng thiết lập cấu hình tự động nếu có sau này
}

void auto_run_Update() {
    // Chỉ hoạt động khi xe ở chế độ tự động (MODE_AUTO)
    if (currentMode != MODE_AUTO) return;

    // 1. Đợi 1.5 giây sau khi bật AUTO để cảm biến siêu âm thu thập đủ 5 mẫu và ổn định bộ lọc
    if (millis() - autoModeStartTime < 1500) {
        car.stop();
        currentMoveDir = "đồng bộ cảm biến (AUTO)";
        currentSpeed = 0;
        return;
    }

    // 2. Kiểm tra an toàn: Nếu cảm biến trước bị hỏng hoặc mất kết nối (OFFLINE)
    //    thì dừng xe ngay lập tức để tránh đâm mù, chuyển sang MANUAL để bảo vệ robot.
    //    (Trừ khi cờ bypassSensorCheck đang được BẬT để phục vụ việc test).
    if (!HC_SR04_FrontOnline()) {
        if (!bypassSensorCheck) {
            car.stop();
            currentMoveDir = "dừng (lỗi cảm biến)";
            currentSpeed = 0;
            isAvoidanceActive = false;
            currentMode = MODE_MANUAL;
            Serial.println(F("⚠️ [AUTO LỖI] Cảm biến siêu âm trước OFFLINE! Dừng xe khẩn cấp và chuyển về chế độ MANUAL."));
            return;
        } else {
            // Cảnh báo định kỳ mỗi giây khi chạy ở chế độ bypass
            static unsigned long lastBypassWarnTime = 0;
            if (millis() - lastBypassWarnTime > 1000) {
                lastBypassWarnTime = millis();
                Serial.println(F("⚠️ [AUTO CẢNH BÁO] Cảm biến trước OFFLINE nhưng Bypass đang BẬT. Xe tiếp tục chạy!"));
            }
        }
    }

    // 3. Thực hiện thuật toán tránh vật cản khi cảm biến đã online
    bool frontObstacle = HC_SR04_FrontObstacle(OBSTACLE_TRIGGER_CM);

    if (frontObstacle) {
        if (!isAvoidanceActive) {
            isAvoidanceActive = true;
            Serial.println(F("🤖 [AUTO] Gặp vật cản phía trước! Đang xoay trái để tránh..."));
        }
    }

    if (isAvoidanceActive) {
        // Xoay trái tại chỗ để tìm hướng thoáng
        car.rotateLeft(140);
        currentMoveDir = "xoay_trai (tránh vật cản)";
        currentSpeed = 140;

        // Đọc khoảng cách cảm biến trước
        float checkFront = HC_SR04_GetFrontDistance();
        
        // Hysteresis: Chỉ tiến lên lại khi vật cản đã xa hơn mức an toàn (OBSTACLE_CLEAR_CM = 70cm)
        bool frontClear = (checkFront > OBSTACLE_CLEAR_CM);

        if (frontClear) {
            isAvoidanceActive = false;
            Serial.println(F("🤖 [AUTO] Đường đã thoáng! Tiếp tục tiến thẳng."));
        }
    } else {
        // Nếu không có vật cản, tự động chạy tiến thẳng
        car.forward(150);
        currentMoveDir = "tiến thẳng tự động";
        currentSpeed = 150;
    }
}
