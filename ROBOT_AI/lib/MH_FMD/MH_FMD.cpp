/**
 * @file MH_FMD.cpp
 * @brief Implementations for the MH_FMD Active Buzzer module.
 * 
 * Chi tiết cấu hình:
 * 1. Tích hợp Active Low: Kéo chân lên HIGH để tắt còi, kéo xuống LOW để bật còi (mặc định cho MH-FMD).
 * 2. Cảnh báo <= 50cm: Ngoài ngưỡng 50cm, còi tắt hoàn toàn.
 * 3. Chỉ phát xung bíp (Pulsing): Không kêu liên tục gây ồn, chỉ bíp bíp theo nhịp độ.
 * 4. Ghi log sự kiện: Bắn log Serial khi chuyển chế độ kêu chẩn đoán rõ ràng lý do.
 */

#include "MH_FMD.h"
#include "driver/gpio.h"

// =============================================================================
// CẤU HÌNH MỨC LOGIC CHO ACTIVE LOW VÀ ACTIVE HIGH
// =============================================================================
#if MH_FMD_ACTIVE_LEVEL == 1
    #define BUZZER_ON_LEVEL  HIGH
    #define BUZZER_OFF_LEVEL LOW
#else
    #define BUZZER_ON_LEVEL  LOW
    #define BUZZER_OFF_LEVEL HIGH
#endif

// =============================================================================
// BIẾN NỘI BỘ (INTERNAL VARIABLES)
// =============================================================================
static bool is_enabled = true;              ///< Trạng thái kích hoạt hệ thống còi cảnh báo
static float warning_threshold = 50.0f;     ///< Ngưỡng khoảng cách bắt đầu kêu (mặc định 50cm)
static float current_distance = 400.0f;     ///< Khoảng cách hiện tại được ghi nhận (cm)
static BuzzerMode active_mode = BUZZER_OFF; ///< Chế độ hoạt động hiện tại của còi
static BuzzerMode last_logged_mode = BUZZER_OFF;

// Các biến phục vụ chế độ override (ghi đè) thủ công
static bool manual_force_on = false;        ///< Bắt buộc kêu liên tục bằng MH_FMD_On()
static bool is_beeping = false;             ///< Đang chạy lệnh bíp thời gian ngắn
static unsigned long beep_end_time = 0;     ///< Thời điểm kết thúc lệnh bíp (ms)

// Các biến phục vụ chế độ tự động Test còi
static bool is_testing = false;             ///< Đang trong chế độ tự động Test
static unsigned long test_start_time = 0;   ///< Thời điểm bắt đầu chạy Test (ms)

// Trạng thái vật lý hiện tại của chân pin còi (true đại diện cho còi đang kêu vật lý)
static bool physical_pin_state = false;

// =============================================================================
// CÁC HÀM TIỆN ÍCH NỘI BỘ (HELPER FUNCTIONS)
// =============================================================================

/**
 * @brief Điều khiển còi chớp tắt dựa trên millis() mà không dùng delay.
 */
static void executeBuzzerMode(unsigned long now) {
    unsigned long period = 0;
    unsigned long active_time = 0;

    switch (active_mode) {
        case BUZZER_OFF:
            if (physical_pin_state) {
                digitalWrite(MH_FMD_PIN, BUZZER_OFF_LEVEL);
                physical_pin_state = false;
            }
            return;

        case BUZZER_SLOW:
            period = 1000;    // Chu kỳ 1 giây
            active_time = 100; // Bíp 100ms, tắt 900ms
            break;

        case BUZZER_FAST:
            period = 250;     // Chu kỳ 250ms
            active_time = 80;  // Bíp 80ms, tắt 170ms
            break;

        case BUZZER_EMERGENCY:
            period = 120;     // Chu kỳ 120ms
            active_time = 50;  // Bíp dồn dập: 50ms ON, 70ms OFF (Bíp bíp liên tục cực nhanh)
            break;
    }

    // Tính toán trạng thái pin dựa theo thời gian tương đối trong chu kỳ
    unsigned long relative_time = now % period;
    if (relative_time < active_time) {
        if (!physical_pin_state) {
            digitalWrite(MH_FMD_PIN, BUZZER_ON_LEVEL);
            physical_pin_state = true;
        }
    } else {
        if (physical_pin_state) {
            digitalWrite(MH_FMD_PIN, BUZZER_OFF_LEVEL);
            physical_pin_state = false;
        }
    }
}

// =============================================================================
// IMPLEMENTATION CÁC HÀM API GIAO TIẾP
// =============================================================================

void MH_FMD_Init() {
    // Giải phóng chân khỏi chức năng JTAG mặc định (GPIO41)
    gpio_reset_pin((gpio_num_t)MH_FMD_PIN);

    pinMode(MH_FMD_PIN, OUTPUT);
    digitalWrite(MH_FMD_PIN, BUZZER_OFF_LEVEL); // Đảm bảo còi tắt ngay lập tức
    
    is_enabled = true;
    warning_threshold = 50.0f; // Bắt đầu ở ngưỡng 50cm theo yêu cầu
    current_distance = 400.0f;
    active_mode = BUZZER_OFF;
    last_logged_mode = BUZZER_OFF;
    
    manual_force_on = false;
    is_beeping = false;
    beep_end_time = 0;
    is_testing = false;
    test_start_time = 0;
    physical_pin_state = false;
}

void MH_FMD_Update(float front, float rear) {
    unsigned long now = millis();

    // 1. Tìm khoảng cách nhỏ nhất hợp lệ giữa cảm biến trước và sau
    float min_dist = 400.0f;
    if (front > 0.0f && front < min_dist) min_dist = front;
    if (rear > 0.0f && rear < min_dist) min_dist = rear;
    current_distance = min_dist;

    // 2. Quyết định chế độ hoạt động của còi dựa trên thứ tự ưu tiên
    BuzzerMode target_mode = BUZZER_OFF;

    if (!is_enabled) {
        target_mode = BUZZER_OFF;
        is_testing = false;
        is_beeping = false;
        manual_force_on = false;
    } 
    else if (manual_force_on) {
        // Ưu tiên 1: Lệnh bật còi cưỡng bức bằng tay (ở chế độ này còi kêu liên tục)
        // Vẫn giữ để phục vụ debug thủ công qua MH_FMD_On
        // Có thể biến đổi nếu cần, nhưng còi cảnh báo tự động thì chỉ kêu bíp bíp
        #if MH_FMD_ACTIVE_LEVEL == 1
            digitalWrite(MH_FMD_PIN, HIGH);
        #else
            digitalWrite(MH_FMD_PIN, LOW);
        #endif
        physical_pin_state = true;
        active_mode = BUZZER_OFF; // Bypass executeBuzzerMode
        return;
    } 
    else if (is_beeping) {
        // Ưu tiên 2: Đang bíp hẹn giờ (Beep)
        if (now < beep_end_time) {
            #if MH_FMD_ACTIVE_LEVEL == 1
                digitalWrite(MH_FMD_PIN, HIGH);
            #else
                digitalWrite(MH_FMD_PIN, LOW);
            #endif
            physical_pin_state = true;
            active_mode = BUZZER_OFF; // Bypass executeBuzzerMode
            return;
        } else {
            is_beeping = false;
            digitalWrite(MH_FMD_PIN, BUZZER_OFF_LEVEL);
            physical_pin_state = false;
        }
    } 
    else if (is_testing) {
        // Ưu tiên 3: Chế độ chạy Test tuần tự
        unsigned long elapsed = now - test_start_time;
        if (elapsed < 2000) {
            target_mode = BUZZER_SLOW;
        } else if (elapsed < 4000) {
            target_mode = BUZZER_FAST;
        } else if (elapsed < 6000) {
            target_mode = BUZZER_EMERGENCY;
        } else {
            is_testing = false;
            target_mode = BUZZER_OFF;
        }
    } 
    else {
        // Chế độ tự động thông thường dựa trên khoảng cách đo được (Ngưỡng <= 50cm)
        if (current_distance > warning_threshold) {
            target_mode = BUZZER_OFF;
        } else if (current_distance > 30.0f) {
            target_mode = BUZZER_SLOW;       // 30cm -> 50cm: Kêu chậm
        } else if (current_distance > 20.0f) {
            target_mode = BUZZER_FAST;       // 20cm -> 30cm: Kêu nhanh
        } else {
            target_mode = BUZZER_EMERGENCY;  // <= 20cm: Cảnh báo dồn dập (chỉ bíp bíp, không rú)
        }
    }

    active_mode = target_mode;

    // 3. Thực thi trạng thái vật lý của chân còi
    executeBuzzerMode(now);

    // 4. Bắn log sự kiện chẩn đoán chênh lệch chế độ còi rõ ràng lý do
    if (active_mode != last_logged_mode) {
        Serial.print(F("📢 [MH-FMD Log] "));
        switch (active_mode) {
            case BUZZER_OFF:
                Serial.printf("COI TẮT (Khoảng cách an toàn: %.1f cm, Ngưỡng cảnh báo: %.1f cm)\n", current_distance, warning_threshold);
                break;
            case BUZZER_SLOW:
                Serial.printf("COI KÊU CHẬM [Bíp... Bíp...] (Khoảng cách vật cản gần nhất: %.1f cm <= Ngưỡng: %.1f cm)\n", current_distance, warning_threshold);
                break;
            case BUZZER_FAST:
                Serial.printf("COI KÊU NHANH [Bíp.Bíp.Bíp.] (Khoảng cách vật cản gần nhất: %.1f cm)\n", current_distance);
                break;
            case BUZZER_EMERGENCY:
                Serial.printf("COI CẢNH BÁO NGUY HIỂM KHẨN CẤP [BípBípBíp!] (Khoảng cách cực gần: %.1f cm!)\n", current_distance);
                break;
        }
        last_logged_mode = active_mode;
    }
}

void MH_FMD_On() {
    manual_force_on = true;
    is_beeping = false;
    is_testing = false;
    Serial.println(F("📢 [MH-FMD Log] Kích hoạt còi thủ công (FORCED ON)"));
}

void MH_FMD_Off() {
    manual_force_on = false;
    is_beeping = false;
    is_testing = false;
    active_mode = BUZZER_OFF;
    digitalWrite(MH_FMD_PIN, BUZZER_OFF_LEVEL);
    physical_pin_state = false;
    Serial.println(F("📢 [MH-FMD Log] Tắt còi thủ công (FORCED OFF)"));
}

void MH_FMD_Beep(uint16_t ms) {
    if (!is_enabled) return;
    is_beeping = true;
    beep_end_time = millis() + ms;
    manual_force_on = false;
    is_testing = false;
    Serial.printf("📢 [MH-FMD Log] Bíp còi thời gian ngắn: %d ms\n", ms);
}

void MH_FMD_SetEnable(bool enable) {
    is_enabled = enable;
    if (!is_enabled) {
        MH_FMD_Off();
    }
    Serial.printf("📢 [MH-FMD Log] Hệ thống còi cảnh báo: %s\n", is_enabled ? "BẬT (ENABLED)" : "TẮT (DISABLED)");
}

bool MH_FMD_IsEnable() {
    return is_enabled;
}

bool MH_FMD_IsWarning() {
    return (active_mode != BUZZER_OFF);
}

float MH_FMD_GetDistance() {
    return current_distance;
}

void MH_FMD_SetThreshold(float cm) {
    warning_threshold = cm;
    Serial.printf("📢 [MH-FMD Log] Thay đổi ngưỡng cảnh báo còi: %.1f cm\n", warning_threshold);
}

float MH_FMD_GetThreshold() {
    return warning_threshold;
}

void MH_FMD_Test() {
    if (!is_enabled) return;
    is_testing = true;
    test_start_time = millis();
    manual_force_on = false;
    is_beeping = false;
    Serial.println(F("📢 [MH-FMD Log] Bắt đầu tự động Test còi cảnh báo"));
}

void MH_FMD_PrintStatus() {
    Serial.print(F("[MH-FMD] System: "));
    Serial.print(is_enabled ? F("ENABLED") : F("DISABLED"));
    
    Serial.print(F(" | Active Level: "));
    Serial.print(MH_FMD_ACTIVE_LEVEL == 1 ? F("Active High") : F("Active Low"));

    Serial.print(F(" | Physical Pin Level: "));
    Serial.print(digitalRead(MH_FMD_PIN) == HIGH ? F("HIGH") : F("LOW"));

    Serial.print(F(" | Current Dist: "));
    Serial.print(current_distance, 1);
    Serial.print(F(" cm | Warn Limit: "));
    Serial.print(warning_threshold, 1);
    Serial.print(F(" cm"));

    Serial.print(F(" | Mode: "));
    switch (active_mode) {
        case BUZZER_OFF:         Serial.print(F("OFF")); break;
        case BUZZER_SLOW:        Serial.print(F("SLOW BEEP")); break;
        case BUZZER_FAST:        Serial.print(F("FAST BEEP")); break;
        case BUZZER_EMERGENCY:   Serial.print(F("EMERGENCY BEEP")); break;
    }

    if (manual_force_on) {
        Serial.print(F(" (FORCED ON)"));
    } else if (is_beeping) {
        Serial.print(F(" (BEEPING)"));
    } else if (is_testing) {
        Serial.print(F(" (TESTING)"));
    }
    Serial.println();
}
