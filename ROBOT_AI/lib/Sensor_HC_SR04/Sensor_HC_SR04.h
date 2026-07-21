/**
 * @file Sensor_HC_SR04.h
 * @brief Module đọc khoảng cách từ 2 cảm biến siêu âm HC-SR04 (trước và sau) không block CPU.
 * Sử dụng ngắt ngoài (External Interrupt) và Median Filter 5 mẫu để lọc nhiễu.
 * 
 * Dự án: Robot Tự Hành Mecanum
 * Chuẩn biên dịch: PlatformIO C++
 */

#ifndef SENSOR_HC_SR04_H
#define SENSOR_HC_SR04_H

#include <Arduino.h>

// =============================================================================
// CẤU HÌNH PHẦN CỨNG - PIN ASSIGNMENTS
// =============================================================================
#define HC_SR04_FRONT_TRIG 1    ///< Chân TRIG cảm biến trước
#define HC_SR04_FRONT_ECHO 2    ///< Chân ECHO cảm biến trước

// =============================================================================
// KHAI BÁO CÁC HÀM GIAO TIẾP (API)
// =============================================================================

/**
 * @brief Hàm đo khoảng cách chi tiết cho một cảm biến siêu âm.
 * @param name Tên cảm biến để hiển thị debug log (ví dụ: "FRONT", "REAR")
 * @param trigPin Chân GPIO TRIG
 * @param echoPin Chân GPIO ECHO
 * @return Khoảng cách đo được (cm), hoặc -1.0f nếu đo thất bại/timeout.
 */
float readSensor(const char* name, uint8_t trigPin, uint8_t echoPin);

/**
 * @brief Khởi tạo phần cứng cho cảm biến siêu âm.
 * Cấu hình chân GPIO, gắn ngắt ngoài cho chân ECHO và khởi tạo các biến.
 */
void HC_SR04_Init();

/**
 * @brief Cập nhật trạng thái đọc của cảm biến siêu âm phía trước.
 * Cần được gọi liên tục trong vòng lặp loop() chính của chương trình.
 */
void HC_SR04_Update();

void HC_SR04_TestGPIO();
void HC_SR04_TestTrigger();
void HC_SR04_CheckConflicts();

/**
 * @brief Lấy khoảng cách đã qua lọc Median của cảm biến trước.
 * @return Khoảng cách (cm), trả về khoảng cách mặc định cực đại hoặc âm nếu lỗi.
 */
float HC_SR04_GetFrontDistance();

/**
 * @brief Lấy khoảng cách nhỏ nhất (bằng khoảng cách cảm biến trước).
 * @return Khoảng cách nhỏ nhất (cm).
 */
float HC_SR04_GetMinDistance();

/**
 * @brief Lấy khoảng cách lớn nhất (bằng khoảng cách cảm biến trước).
 * @return Khoảng cách lớn nhất (cm).
 */
float HC_SR04_GetMaxDistance();

/**
 * @brief Kiểm tra xem phía trước có vật cản dưới ngưỡng xác định hay không.
 * @param cm Ngưỡng khoảng cách cần kiểm tra (cm)
 * @return true nếu có vật cản, ngược lại là false.
 */
bool HC_SR04_FrontObstacle(float cm);

/**
 * @brief Kiểm tra xem phía trước có vật cản dưới ngưỡng xác định hay không.
 * @param cm Ngưỡng khoảng cách cần kiểm tra (cm)
 * @return true nếu có vật cản, ngược lại là false.
 */
bool HC_SR04_HasObstacle(float cm);

/**
 * @brief Kiểm tra xem cảm biến trước có đang hoạt động trực tuyến (online) hay không.
 * @return true nếu hoạt động bình thường, false nếu mất kết nối (nhiều lần timeout liên tiếp).
 */
bool HC_SR04_FrontOnline();

/**
 * @brief Cấu hình khoảng cách cảnh báo mặc định.
 * @param cm Ngưỡng cảnh báo mới (cm).
 */
void HC_SR04_SetWarningDistance(float cm);

/**
 * @brief Lấy ngưỡng khoảng cách cảnh báo mặc định.
 * @return Ngưỡng cảnh báo hiện tại (cm).
 */
float HC_SR04_GetWarningDistance();

/**
 * @brief In thông tin chẩn đoán trạng thái cảm biến ra Serial Monitor.
 */
void HC_SR04_Print();

/**
 * @brief Bật/tắt chế độ in debug chi tiết cho hàm readSensor.
 * @param enable true để bật debug log, false để tắt.
 */
void HC_SR04_SetDebug(bool enable);
bool HC_SR04_GetDebug();

#endif // SENSOR_HC_SR04_H
