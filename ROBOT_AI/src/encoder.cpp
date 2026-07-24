/**
 * @file encoder.cpp
 * @brief Implementations cho EncoderModule.
 */

#include "encoder.h"
#include "Config.h"

EncoderModule& EncoderModule::getInstance() {
    static EncoderModule instance;
    return instance;
}

EncoderModule::EncoderModule() {}

void EncoderModule::begin() {
#if ENCODER_ENABLED
    EncoderManager::getInstance().begin();
#endif
}

void EncoderModule::update() {
#if ENCODER_ENABLED
    EncoderManager::getInstance().update();
#endif
}

MecanumWheelSpeeds EncoderModule::getWheelSpeeds() const {
    MecanumWheelSpeeds ws;
#if ENCODER_ENABLED
    EncoderManager& em = EncoderManager::getInstance();
    ws.fl = em.getSpeed(0);
    ws.fr = em.getSpeed(1);
    ws.rl = em.getSpeed(2);
    ws.rr = em.getSpeed(3);
#else
    ws.fl = 0.0f;
    ws.fr = 0.0f;
    ws.rl = 0.0f;
    ws.rr = 0.0f;
#endif
    return ws;
}

float EncoderModule::getTotalDistance() const {
#if ENCODER_ENABLED
    return EncoderManager::getInstance().getWheelDistance();
#else
    return 0.0f;
#endif
}

void EncoderModule::reset() {
#if ENCODER_ENABLED
    EncoderManager::getInstance().resetAll();
#endif
}
