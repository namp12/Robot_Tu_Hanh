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
const float PPR = 11.0;          // Số xung trên 1 vòng của Encoder (chưa qua giảm tốc)
const float GEAR_RATIO = 30.0;   // Tỉ số truyền hộp giảm tốc (ví dụ 1:30)
const float WHEEL_DIAMETER = 0.08; // Đường kính bánh xe (0.08m = 80mm)
const float WHEEL_CIRCUMFERENCE = WHEEL_DIAMETER * PI; // Chu vi bánh xe

// Kích thước hình học của xe (Dùng cho động học Mecanum)
const float L_X = 0.15; // m (Khoảng cách từ tâm đến trục bánh xe theo chiều X)
const float L_Y = 0.15; // m (Khoảng cách từ tâm đến trục bánh xe theo chiều Y)

// --- Tốc độ Baud giao tiếp Serial ---
#define SERIAL_BAUD 115200

#endif // CONFIG_H
