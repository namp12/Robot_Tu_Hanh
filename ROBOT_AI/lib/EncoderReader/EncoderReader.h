#ifndef ENCODER_READER_H
#define ENCODER_READER_H

#include <Arduino.h>
#include "PinMap.h"
#include "Config.h"

/**
 * 📊 THƯ VIỆN ĐỌC XUNG ENCODER & TÍNH VẬN TỐC
 * Đọc xung ngắt tốc độ cao và quy đổi sang RPM, m/s độc lập.
 */
class EncoderReader {
public:
    EncoderReader();
    void begin();
    
    // Tính toán lại vận tốc (Chạy định kỳ trong loop)
    void update();
    
    // Lấy số xung tích lũy của 4 bánh
    void getTicks(long &fl, long &fr, long &rl, long &rr);
    
    // Lấy số vòng quay trên phút (RPM)
    void getRPM(float &fl, float &fr, float &rl, float &rr);
    
    // Lấy vận tốc dài (m/s)
    void getSpeeds(float &fl, float &fr, float &rl, float &rr);
    
    // Đặt lại (reset) số xung về 0
    void resetTicks();
};

#endif // ENCODER_READER_H
