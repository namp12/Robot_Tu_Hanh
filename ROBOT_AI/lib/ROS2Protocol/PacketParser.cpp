#include "PacketParser.h"

PacketParser::PacketParser() {
    reset();
}

void PacketParser::reset() {
    _state = STATE_WAIT_HEADER1;
    _msgId = 0;
    _payloadLen = 0;
    _payloadIndex = 0;
    _crcLow = 0;
    _crcHigh = 0;
}

bool PacketParser::parseByte(uint8_t byteIn, uint8_t& outMsgId, uint8_t* outPayload, uint8_t& outLen) {
    switch (_state) {
        case STATE_WAIT_HEADER1:
            if (byteIn == ROS2_HEADER1) {
                _state = STATE_WAIT_HEADER2;
            }
            break;

        case STATE_WAIT_HEADER2:
            if (byteIn == ROS2_HEADER2) {
                _state = STATE_READ_MSG_ID;
            } else {
                _state = STATE_WAIT_HEADER1;
            }
            break;

        case STATE_READ_MSG_ID:
            _msgId = byteIn;
            _state = STATE_READ_LENGTH;
            break;

        case STATE_READ_LENGTH:
            _payloadLen = byteIn;
            if (_payloadLen > sizeof(_payloadBuffer)) {
                // Độ dài vượt ngưỡng cho phép -> Hủy gói
                reset();
            } else if (_payloadLen == 0) {
                _state = STATE_READ_CRC_LOW;
            } else {
                _payloadIndex = 0;
                _state = STATE_READ_PAYLOAD;
            }
            break;

        case STATE_READ_PAYLOAD:
            _payloadBuffer[_payloadIndex++] = byteIn;
            if (_payloadIndex >= _payloadLen) {
                _state = STATE_READ_CRC_LOW;
            }
            break;

        case STATE_READ_CRC_LOW:
            _crcLow = byteIn;
            _state = STATE_READ_CRC_HIGH;
            break;

        case STATE_READ_CRC_HIGH:
            _crcHigh = byteIn;
            _state = STATE_WAIT_TAIL;
            break;

        case STATE_WAIT_TAIL:
            if (byteIn == ROS2_TAIL) {
                // Đã nhận đủ gói -> Kiểm tra CRC16
                uint16_t receivedCRC = (uint16_t)_crcLow | ((uint16_t)_crcHigh << 8);

                // Gom MsgID + Len + Payload để tính CRC
                _crcCalcBuffer[0] = _msgId;
                _crcCalcBuffer[1] = _payloadLen;
                if (_payloadLen > 0) {
                    memcpy(&_crcCalcBuffer[2], _payloadBuffer, _payloadLen);
                }

                uint16_t calculatedCRC = calculateCRC16(_crcCalcBuffer, 2 + _payloadLen);

                if (calculatedCRC == receivedCRC) {
                    // CRC Khớp -> Xuất dữ liệu
                    outMsgId = _msgId;
                    outLen = _payloadLen;
                    if (_payloadLen > 0 && outPayload != nullptr) {
                        memcpy(outPayload, _payloadBuffer, _payloadLen);
                    }
                    reset();
                    return true;
                }
            }
            // Nếu Tail sai hoặc CRC không khớp -> Reset
            reset();
            break;
    }

    return false;
}
