#include "BuzzerManager.h"

BuzzerManager::BuzzerManager() 
    : _currentMode(BUZZER_OFF) {}

void BuzzerManager::begin() {
    MH_FMD_Init();
    _currentMode = BUZZER_OFF;
}

void BuzzerManager::setMode(BuzzerMode mode) {
    _currentMode = mode;
    switch (mode) {
        case BUZZER_OFF:
            MH_FMD_Off();
            break;
        case BUZZER_SLOW:
            MH_FMD_Beep(200);
            break;
        case BUZZER_FAST:
            MH_FMD_Beep(100);
            break;
        case BUZZER_EMERGENCY:
            MH_FMD_On();
            break;
    }
}

void BuzzerManager::update() {
    // Handled non-blocking internally
}
