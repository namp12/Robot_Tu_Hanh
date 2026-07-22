/**
 * @file test_module.h
 * @brief Header file cho phân hệ chẩn đoán và kiểm tra các module phần cứng.
 */

#ifndef TEST_MODULE_H
#define TEST_MODULE_H

#include <Arduino.h>

/**
 * @brief Khởi tạo chế độ chẩn đoán, hỏi lựa chọn chế độ ở boot.
 */
void test_module_Init();

/**
 * @brief Cập nhật nhận diện phím Serial và thực thi test module.
 */
void test_module_Update();

/**
 * @brief Kiểm tra xem hệ thống có đang chạy ở chế độ chẩn đoán module hay không.
 * @return true nếu đang ở chế độ test, ngược lại là false.
 */
bool is_in_test_mode();

/**
 * @brief Kiểm tra xem cảm biến siêu âm có được phép cập nhật hay không.
 * @return true nếu được cập nhật, false nếu đang bị tạm dừng (pause).
 */
bool should_run_sensor_update();

/**
 * @brief Kiểm tra xem hệ thống có đang ở chế độ kiểm tra siêu âm cô lập hay không.
 */
bool is_sensor_isolated_mode();

#endif // TEST_MODULE_H
