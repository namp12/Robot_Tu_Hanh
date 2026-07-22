/**
 * @file ROS2Protocol.h
 * @brief Định nghĩa Cấu trúc Khung Gói tin Nhị phân (Binary Protocol Frame)
 *        dùng cho truyền thông 2 chiều tốc độ cao ESP32 <-> Raspberry Pi (ROS2).
 */

#ifndef ROS2_PROTOCOL_H
#define ROS2_PROTOCOL_H

#include <Arduino.h>

// =============================================================================
// CẰNG SỐ KHUNG GÓI TIN (FRAME CONSTANTS)
// =============================================================================
#define ROS2_HEADER1 0xFF
#define ROS2_HEADER2 0xFE
#define ROS2_TAIL    0xFD

// =============================================================================
// MÃ LOẠI THÔNG ĐIỆP (MESSAGE IDs)
// =============================================================================
#define MSG_ID_TELEMETRY   0x01  ///< ESP32 -> Pi: IMU, Siêu âm, Trạng thái, PWM 4 bánh (50Hz)
#define MSG_ID_CMD_VEL     0x02  ///< Pi -> ESP32: Lệnh vận tốc linear.x, linear.y, angular.z
#define MSG_ID_SET_MODE    0x03  ///< Pi -> ESP32: Lệnh đổi Mode (MANUAL, AUTO, ROS2, E-STOP)
#define MSG_ID_RESET_GOC   0x04  ///< Pi -> ESP32: Lệnh reset góc Yaw MPU6050 về 0
#define MSG_ID_ACK         0x05  ///< ESP32 -> Pi: Phản hồi xác nhận lệnh (Acknowledge)
#define MSG_ID_TRIGGER_BEEP 0x06 ///< Pi -> ESP32: Lệnh còi (Beep) kêu từ dashboard/RPi

// =============================================================================
// CẤU TRÚC PAYLOAD NHỊ PHÂN (PACKED STRUCTS)
// =============================================================================
#pragma pack(push, 1)

/**
 * @brief Telemetry Data Package (ESP32 -> Raspberry Pi)
 * Kích thước: 44 Bytes. Truyền định kỳ 20ms (50Hz).
 */
struct TelemetryPayload {
    uint32_t timestamp_ms;   ///< Thời gian millis() trên ESP32
    float accel_x;          ///< Gia tốc X (m/s^2)
    float accel_y;          ///< Gia tốc Y (m/s^2)
    float accel_z;          ///< Gia tốc Z (m/s^2)
    float gyro_x;           ///< Vận tốc góc X (rad/s)
    float gyro_y;           ///< Vận tốc góc Y (rad/s)
    float gyro_z;           ///< Vận tốc góc Z (rad/s)
    float roll;             ///< Góc Roll (độ)
    float pitch;            ///< Góc Pitch (độ)
    float yaw;              ///< Góc Yaw (độ)
    float front_distance;   ///< Khoảng cách cảm biến trước (cm)
    float rear_distance;    ///< Khoảng cách cảm biến sau (cm)
    uint8_t current_mode;   ///< 0: MANUAL, 1: AUTO, 2: ROS2
    uint8_t auto_state;     ///< Trạng thái AutoState hiện tại
    int16_t motor_fl_speed; ///< PWM bánh FL (-255 -> 255)
    int16_t motor_fr_speed; ///< PWM bánh FR (-255 -> 255)
    int16_t motor_rl_speed; ///< PWM bánh RL (-255 -> 255)
    int16_t motor_rr_speed; ///< PWM bánh RR (-255 -> 255)
    uint8_t flags;          ///< Bit 0: mpuOk, Bit 1: frontOnline, Bit 2: rearOnline, Bit 3: E-Stop
};

/**
 * @brief Lệnh Vận tốc từ ROS2 Node (Raspberry Pi -> ESP32)
 * Kích thước: 12 Bytes.
 */
struct CmdVelPayload {
    float linear_x;   ///< Vận tốc tiến/lùi (m/s)
    float linear_y;   ///< Vận tốc sang trái/phải (m/s)
    float angular_z;  ///< Vận tốc xoay tại chỗ (rad/s)
};

/**
 * @brief Lệnh Đổi Mode & Dừng Khẩn cấp (Raspberry Pi -> ESP32)
 * Kích thước: 2 Bytes.
 */
struct SetModePayload {
    uint8_t target_mode; ///< 0: MANUAL, 1: AUTO, 2: ROS2
    uint8_t e_stop;      ///< 1: Kích hoạt E-Stop, 0: Tắt E-Stop
};

#pragma pack(pop)

// =============================================================================
// HELPER: TÍNH CRC16 CHECKSUM (CRC16-MODBUS/CCITT)
// =============================================================================
inline uint16_t calculateCRC16(const uint8_t* data, size_t length) {
    uint16_t crc = 0xFFFF;
    for (size_t i = 0; i < length; i++) {
        crc ^= data[i];
        for (uint8_t j = 0; j < 8; j++) {
            if (crc & 0x0001) {
                crc = (crc >> 1) ^ 0xA001;
            } else {
                crc = crc >> 1;
            }
        }
    }
    return crc;
}

#endif // ROS2_PROTOCOL_H
