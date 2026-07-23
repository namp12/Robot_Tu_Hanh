#ifndef BUZZER_MANAGER_H
#define BUZZER_MANAGER_H

#include <Arduino.h>
#include "MH_FMD.h"

class BuzzerManager {
private:
    BuzzerMode _currentMode;

    BuzzerManager();

public:
    static BuzzerManager& getInstance() {
        static BuzzerManager instance;
        return instance;
    }

    void begin();
    void setMode(BuzzerMode mode);
    BuzzerMode getMode() const { return _currentMode; }
    void beep(uint16_t duration_ms) { MH_FMD_Beep(duration_ms); }
    void update();
};

#endif // BUZZER_MANAGER_H
