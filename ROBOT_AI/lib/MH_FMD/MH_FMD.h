/**
 * @file MH_FMD.h
 * @brief Module điều khiển còi báo động MH-FMD (Active Buzzer) không block CPU.
 * Sinh ra các âm báo có tần số và khoảng cách xung khác nhau dựa trên ngưỡng vật cản.
 * 
 * Dự án: Robot Tự Hành Mecanum
 * Chuẩn biên dịch: PlatformIO C++
 */

#ifndef MH_FMD_H
#define MH_FMD_H

#include <Arduino.h>

// =============================================================================
// CẤU HÌNH PHẦN CỨNG - PIN ASSIGNMENTS
// =============================================================================
#include "PinMap.h"
#ifndef MH_FMD_PIN
#define MH_FMD_PIN 41   ///< Chân kết nối còi Active Buzzer MH-FMD
#endif

// Cấu hình mức kích hoạt của còi (1: Active High, 0: Active Low)
// Còi Active Buzzer MH-FMD thường kích hoạt ở mức LOW (Active Low)
#ifndef MH_FMD_ACTIVE_LEVEL
#define MH_FMD_ACTIVE_LEVEL 0   
#endif

// =============================================================================
// ĐỊNH NGHĨA CÁC CHẾ ĐỘ CÒI BÁO (BUZZER MODES) - CHỈ KÊU BÍP BÍP PHÁT XUNG
// =============================================================================
enum BuzzerMode {
    BUZZER_OFF,          ///< Tắt còi hoàn toàn
    BUZZER_SLOW,         ///< Kêu chậm (Vật cản từ 30cm - 50cm)
    BUZZER_FAST,         ///< Kêu nhanh (Vật cản từ 20cm - 30cm)
    BUZZER_EMERGENCY     ///< Cảnh báo khẩn cấp (Vật cản <= 20cm)
};

// =============================================================================
// KHAI BÁO CÁC HÀM GIAO TIẾP (API)
// =============================================================================

/**
 * @brief Khởi tạo chân GPIO cho còi và tắt còi ban đầu.
 */
void MH_FMD_Init();

/**
 * @brief Cập nhật trạng thái còi dựa trên khoảng cách.
 * Tính toán chế độ còi, xử lý override từ Beep hoặc Test, và đổi trạng thái pin không block.
 * @param front Khoảng cách cảm biến trước (cm)
 */
void MH_FMD_Update(float front);

/**
 * @brief Bật còi thủ công (Kêu liên tục). Override khoảng cách tạm thời.
 */
void MH_FMD_On();

/**
 * @brief Tắt còi thủ công, đồng thời hủy bỏ các chế độ Override như Beep hoặc Test.
 */
void MH_FMD_Off();

/**
 * @brief Bíp còi trong một khoảng thời gian xác định (mili giây), không block CPU.
 * @param ms Thời gian bíp (mili giây)
 */
void MH_FMD_Beep(uint16_t ms);

/**
 * @brief Bật hoặc tắt hệ thống cảnh báo còi tự động.
 * @param enable true để cho phép cảnh báo tự động, false để tắt cảnh báo.
 */
void MH_FMD_SetEnable(bool enable);

/**
 * @brief Kiểm tra xem hệ thống còi cảnh báo có đang bật (enable) hay không.
 * @return true nếu đang bật, ngược lại là false.
 */
bool MH_FMD_IsEnable();

/**
 * @brief Kiểm tra xem còi có đang phát ra âm thanh cảnh báo/bíp hay không.
 * @return true nếu còi đang bật vật lý, ngược lại là false.
 */
bool MH_FMD_IsWarning();

/**
 * @brief Lấy khoảng cách nhỏ nhất hiện tại đang sử dụng để so ngưỡng.
 * @return Khoảng cách nhỏ nhất hợp lệ (cm).
 */
float MH_FMD_GetDistance();

/**
 * @brief Thay đổi ngưỡng khoảng cách bắt đầu cảnh báo còi.
 * Mặc định ban đầu là 70.0 cm.
 * @param cm Ngưỡng cảnh báo mới (cm).
 */
void MH_FMD_SetThreshold(float cm);

/**
 * @brief Lấy ngưỡng khoảng cách cảnh báo còi hiện tại.
 * @return Ngưỡng cảnh báo (cm).
 */
float MH_FMD_GetThreshold();

/**
 * @brief Kích hoạt chế độ kiểm tra còi (Test mode).
 * Tự động chạy qua các chế độ Slow -> Medium -> Fast -> Continuous -> Off mỗi 2 giây.
 */
void MH_FMD_Test();

/**
 * @brief In thông tin chẩn đoán chi tiết của còi ra Serial Monitor.
 */
void MH_FMD_PrintStatus();

#endif // MH_FMD_H
