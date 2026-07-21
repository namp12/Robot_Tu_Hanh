#include "PacketBuilder.h"

size_t PacketBuilder::buildTelemetryPacket(const TelemetryPayload& payload, uint8_t* buffer, size_t maxLen) {
    size_t payloadLen = sizeof(TelemetryPayload);
    size_t totalPacketLen = 2 + 1 + 1 + payloadLen + 2 + 1; // Header(2) + MsgID(1) + Len(1) + Payload + CRC(2) + Tail(1)

    if (maxLen < totalPacketLen) {
        return 0; // Buffer không đủ chứa
    }

    size_t idx = 0;
    buffer[idx++] = ROS2_HEADER1;
    buffer[idx++] = ROS2_HEADER2;
    buffer[idx++] = MSG_ID_TELEMETRY;
    buffer[idx++] = (uint8_t)payloadLen;

    // Copy Payload byte nhị phân
    memcpy(&buffer[idx], &payload, payloadLen);
    idx += payloadLen;

    // Tính CRC16 trên các byte từ MsgID đến hết Payload
    uint16_t crc = calculateCRC16(&buffer[2], 1 + 1 + payloadLen);
    buffer[idx++] = (uint8_t)(crc & 0xFF);        // CRC Low Byte
    buffer[idx++] = (uint8_t)((crc >> 8) & 0xFF); // CRC High Byte

    buffer[idx++] = ROS2_TAIL;

    return idx;
}

size_t PacketBuilder::buildAckPacket(uint8_t ackMsgId, uint8_t status, uint8_t* buffer, size_t maxLen) {
    uint8_t payload[2] = { ackMsgId, status };
    size_t payloadLen = 2;
    size_t totalPacketLen = 2 + 1 + 1 + payloadLen + 2 + 1;

    if (maxLen < totalPacketLen) {
        return 0;
    }

    size_t idx = 0;
    buffer[idx++] = ROS2_HEADER1;
    buffer[idx++] = ROS2_HEADER2;
    buffer[idx++] = MSG_ID_ACK;
    buffer[idx++] = (uint8_t)payloadLen;

    memcpy(&buffer[idx], payload, payloadLen);
    idx += payloadLen;

    uint16_t crc = calculateCRC16(&buffer[2], 1 + 1 + payloadLen);
    buffer[idx++] = (uint8_t)(crc & 0xFF);
    buffer[idx++] = (uint8_t)((crc >> 8) & 0xFF);

    buffer[idx++] = ROS2_TAIL;

    return idx;
}
