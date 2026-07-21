/**
 * @file PacketParser.h
 * @brief Máy trạng thái (State Machine) đọc từng Byte Serial từ Raspberry Pi,
 *        giải mã gói tin nhị phân và kiểm tra CRC16 chống nhiễu.
 */

#ifndef PACKET_PARSER_H
#define PACKET_PARSER_H

#include "ROS2Protocol.h"

enum ParserState {
    STATE_WAIT_HEADER1,
    STATE_WAIT_HEADER2,
    STATE_READ_MSG_ID,
    STATE_READ_LENGTH,
    STATE_READ_PAYLOAD,
    STATE_READ_CRC_LOW,
    STATE_READ_CRC_HIGH,
    STATE_WAIT_TAIL
};

class PacketParser {
private:
    ParserState _state;
    uint8_t _msgId;
    uint8_t _payloadLen;
    uint8_t _payloadBuffer[64];
    uint8_t _payloadIndex;
    uint8_t _crcLow;
    uint8_t _crcHigh;

    // Buffer phụ phục vụ tính toán CRC
    uint8_t _crcCalcBuffer[70];

public:
    PacketParser();
    void reset();

    /**
     * @brief Nạp từng byte từ dòng Serial vào máy trạng thái
     * @param byteIn Byte vừa đọc từ Serial
     * @param outMsgId [Output] Trả về Msg ID của gói tin nếu giải mã thành công
     * @param outPayload [Output] Con trỏ tới bộ đệm chứa Payload nhị phân
     * @param outLen [Output] Độ dài Payload
     * @return true nếu đã nhận đủ 1 gói tin hoàn chỉnh và CRC16 đúng, ngược lại false
     */
    bool parseByte(uint8_t byteIn, uint8_t& outMsgId, uint8_t* outPayload, uint8_t& outLen);
};

#endif // PACKET_PARSER_H
