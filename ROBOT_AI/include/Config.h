#ifndef CONFIG_H
#define CONFIG_H

#include <Arduino.h>

/**
 * ⚙️ HẰNG SỐ CẤU HÌNH HỆ THỐNG
 * Chứa các thông số vật lý của Robot và cấu hình truyền thông.
 */

// --- Cấu hình LEDC PWM (ESP32) ---
#define PWM_FREQ      15000 // Tần số PWM 15kHz (tránh tiếng rít cơ học)
#define PWM_RES       8     // Độ phân giải 8-bit (từ 0 - 255)

// Gán kênh LEDC cho 8 cổng điều khiển động cơ
#define CH_FL_R       0
#define CH_FL_L       1
#define CH_FR_R       2
#define CH_FR_L       3
#define CH_RL_R       4
#define CH_RL_L       5
#define CH_RR_R       6
#define CH_RR_L       7

// --- Thông số vật lý của Robot ---
const float WHEEL_DIAMETER = 0.08; // Đường kính bánh xe (0.08m = 80mm)
const float WHEEL_CIRCUMFERENCE = WHEEL_DIAMETER * PI; // Chu vi bánh xe
const float PPR = 11.0f;           // Pulse Per Revolution của động cơ
const float GEAR_RATIO = 30.0f;    // Tỷ số truyền động cơ

// Kích thước hình học của xe (Dùng cho động học Mecanum)
const float L_X = 0.15; // m (Khoảng cách từ tâm đến trục bánh xe theo chiều X)
const float L_Y = 0.15; // m (Khoảng cách từ tâm đến trục bánh xe theo chiều Y)

// --- Tốc độ Baud giao tiếp Serial ---
#define SERIAL_BAUD 115200

// =============================================================================
// 🤖 CẤU HÌNH CHẾ ĐỘ TỰ ĐỘNG (AUTO MODE CONFIGURATION)
// =============================================================================

// --- Cấu hình PID Giữ Hướng (Yaw Heading Hold) ---
#define AUTO_PID_ENABLED           true    // Bật/Tắt bộ điều khiển PID giữ hướng
const float AUTO_KP              = 1.2f;   // Hệ số Kp mượt (tránh rung lắc bánh)
const float AUTO_KI              = 0.02f;  // Hệ số Ki
const float AUTO_KD              = 0.4f;   // Hệ số Kd
const int   AUTO_MAX_CORRECTION  = 30;     // Tốc độ điều chỉnh PWM tối đa của PID
const int   AUTO_PID_OUTPUT_CLAMP = 25;    // Giới hạn Clamp đầu ra PID mượt (không gây khựng động cơ)

// --- Cấu hình Tốc độ PWM & Khoảng cách Cảm biến ---
const int   AUTO_MAX_SPEED       = 100;    // PWM tốc độ tối đa khi đường thoáng (Theo yêu cầu: 100)
const int   AUTO_MIN_SPEED       = 65;     // PWM tốc độ tối thiểu trước khi dừng hẳn (đủ lực quay bánh mượt)
const float AUTO_DIST_SLOW_CM    = 80.0f;  // Khoảng cách bắt đầu giảm tốc độ từ 100 xuống 65 (cm)
const float AUTO_DIST_STOP_CM    = 25.0f;  // Khoảng cách dừng hoàn toàn để chuyển tránh vật cản (cm)
const float AUTO_DIST_CLEAR_CM   = 85.0f;  // Khoảng cách an toàn để quay lại tiến thẳng (cm)

// --- Cấu hình Tăng/Giảm tốc mềm (PWM Acceleration/Deceleration Ramp) ---
const int   AUTO_RAMP_STEP       = 10;     // Số bước thay đổi PWM mỗi chu kỳ
const unsigned long AUTO_RAMP_INTERVAL_MS = 15; // Khoảng thời gian giữa các bước tăng PWM (ms)

// --- Bộ lọc xác nhận quyết định từ cảm biến (Debounce / Hysteresis) ---
const uint8_t AUTO_OBSTACLE_DEBOUNCE_COUNT = 3; // Số lần đọc liên tiếp nhỏ hơn ngưỡng mới xác nhận có vật cản

// --- Cấu hình Recovery & Quay Góc ---
const unsigned long AUTO_BACKWARD_TIME_MS  = 500;  // Thời gian lùi ngắn khi bắt đầu Recovery (ms)
const unsigned long AUTO_SCAN_TIME_MS      = 300;  // Thời gian dừng quét cảm biến giữa các bước (ms)
const int   AUTO_TURN_DEFAULT_SPEED        = 150;  // PWM mặc định khi quay góc
const float AUTO_TURN_TOLERANCE_DEG        = 4.0f; // Dung sai góc quay chấp nhận được (độ)
const unsigned long AUTO_TURN_TIMEOUT_MS   = 2500; // Timeout tối đa cho 1 lần quay (ms)
// --- Cấu hình Chống Mắc Kẹt (Stuck Prevention) ---
const unsigned long AUTO_STUCK_TIMEOUT_MS  = 10000;// Thời gian tối đa không tiến lên được sẽ coi là mắc kẹt (10s)
const uint8_t AUTO_RECOVERY_RETRY_LIMIT    = 3;    // Số lần thử Recovery tối đa trước khi báo lỗi về MANUAL

// --- Cấu hình Trạng thái Chờ AUTO_IDLE ---
const unsigned long AUTO_IDLE_TIMEOUT_MS   = 30000;// Thời gian chờ tự động trong IDLE (ms)

// --- Cấu hình Phản hồi Encoder & MPU6050 ---
#define ENCODER_ENABLED            false    // Bật/Tắt phản hồi cân bằng tốc độ bánh bằng Encoder

// --- Cấu hình Phát hiện Va Chạm (Collision Detection via MPU6050) ---
#define AUTO_COLLISION_ENABLED     true    // Bật/Tắt phát hiện va chạm bằng MPU6050
const float AUTO_COLLISION_THRESHOLD_MS2 = 25.0f; // Ngưỡng gia tốc tổng quát phát hiện va chạm mạnh (m/s^2, ~2.5g)
const uint8_t AUTO_COLLISION_DEBOUNCE_COUNT = 3; // Số mẫu gia tốc liên tiếp vượt ngưỡng mới báo va chạm

#endif // CONFIG_H
