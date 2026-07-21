/**
 * @file PacketBuilder.h
 * @brief Đóng gói dữ liệu số và Struct thành mảng Byte nhị phân chuẩn bị gửi qua Serial.
 */

#ifndef PACKET_BUILDER_H
#define PACKET_BUILDER_H

#include "ROS2Protocol.h"

class PacketBuilder {
public:
    /**
     * @brief Đóng gói gói tin Telemetry (ESP32 -> Pi)
     * @param payload Struct dữ liệu Telemetry
     * @param buffer Mảng chứa kết quả đầu ra
     * @param maxLen Kích thước tối đa của buffer
     * @return Kích thước thực tế của gói tin đóng gói thành công (số bytes)
     */
    static size_t buildTelemetryPacket(const TelemetryPayload& payload, uint8_t* buffer, size_t maxLen);

    /**
     * @brief Đóng gói gói tin Phản hồi ACK (ESP32 -> Pi)
     * @param ackMsgId Msg ID của lệnh được ACK
     * @param status Trạng thái (0: OK, 1: Error)
     * @param buffer Mảng chứa kết quả đầu ra
     * @param maxLen Kích thước tối đa của buffer
     * @return Kích thước thực tế của gói tin
     */
    static size_t buildAckPacket(uint8_t ackMsgId, uint8_t status, uint8_t* buffer, size_t maxLen);
};

#endif // PACKET_BUILDER_H
