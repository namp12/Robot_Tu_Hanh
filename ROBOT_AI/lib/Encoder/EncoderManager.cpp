#include "EncoderManager.h"
#include "HallEncoderDriver.h"
#include "PinMap.h"
#include "Config.h"

EncoderManager::EncoderManager() {
    // Populate default config from Config.h constants
    _config.ppr = PPR;
    _config.gearRatio = GEAR_RATIO;
    _config.wheelDiameter = WHEEL_DIAMETER;
    _config.enableMinPulseFilter = true;
    _config.enableMedianFilter = true;
    _config.enableMovingAverage = true;
    _config.minPulseThreshold = 1;
    _config.timeoutMs = 2000;
    _config.maxSpeedThreshold = 5.0f; // Limit to 5 m/s

    // Instantiate drivers
    _drivers[0] = new HallEncoderDriver(ENC_FL_A, ENC_FL_B, 0);
    _drivers[1] = new HallEncoderDriver(ENC_FR_A, ENC_FR_B, 1);
    _drivers[2] = new HallEncoderDriver(ENC_RL_A, ENC_RL_B, 2);
    _drivers[3] = new HallEncoderDriver(ENC_RR_A, ENC_RR_B, 3);

    // Instantiate Encoders
    _encoders[0] = new Encoder(_drivers[0], 0, _config);
    _encoders[1] = new Encoder(_drivers[1], 1, _config);
    _encoders[2] = new Encoder(_drivers[2], 2, _config);
    _encoders[3] = new Encoder(_drivers[3], 3, _config);
}

EncoderManager::~EncoderManager() {
    for (int i = 0; i < 4; i++) {
        delete _encoders[i];
        delete _drivers[i];
    }
}

void EncoderManager::begin() {
    for (int i = 0; i < 4; i++) {
        if (_encoders[i] != nullptr) {
            _encoders[i]->begin();
        }
    }
}

void EncoderManager::update() {
    for (int i = 0; i < 4; i++) {
        if (_encoders[i] != nullptr) {
            _encoders[i]->update();
        }
    }
}

void EncoderManager::reset(int index) {
    if (index >= 0 && index < 4 && _encoders[index] != nullptr) {
        _encoders[index]->reset();
    }
}

void EncoderManager::resetAll() {
    for (int i = 0; i < 4; i++) {
        reset(i);
    }
}

long EncoderManager::getPulse(int index) const {
    if (index >= 0 && index < 4 && _encoders[index] != nullptr) {
        return _encoders[index]->getPulse();
    }
    return 0;
}

float EncoderManager::getRPM(int index) const {
    if (index >= 0 && index < 4 && _encoders[index] != nullptr) {
        return _encoders[index]->getRPM();
    }
    return 0.0f;
}

float EncoderManager::getSpeed(int index) const {
    if (index >= 0 && index < 4 && _encoders[index] != nullptr) {
        return _encoders[index]->getSpeed();
    }
    return 0.0f;
}

float EncoderManager::getDistance(int index) const {
    if (index >= 0 && index < 4 && _encoders[index] != nullptr) {
        return _encoders[index]->getDistance();
    }
    return 0.0f;
}

int EncoderManager::getDirection(int index) const {
    if (index >= 0 && index < 4 && _encoders[index] != nullptr) {
        return _encoders[index]->getDirection();
    }
    return 1;
}

bool EncoderManager::isHealthy(int index) const {
    if (index >= 0 && index < 4 && _encoders[index] != nullptr) {
        return _encoders[index]->isHealthy();
    }
    return false;
}

Encoder* EncoderManager::getEncoder(int index) const {
    if (index >= 0 && index < 4) {
        return _encoders[index];
    }
    return nullptr;
}

float EncoderManager::getLeftDistance() const {
    // Average of Front Left (0) and Rear Left (2)
    return (getDistance(0) + getDistance(2)) / 2.0f;
}

float EncoderManager::getRightDistance() const {
    // Average of Front Right (1) and Rear Right (3)
    return (getDistance(1) + getDistance(3)) / 2.0f;
}

float EncoderManager::getWheelDistance() const {
    return (getDistance(0) + getDistance(1) + getDistance(2) + getDistance(3)) / 4.0f;
}

float EncoderManager::getWheelVelocity(int index) const {
    return getSpeed(index);
}

float EncoderManager::getLinearVelocity() const {
    // Vx = (FL + FR + RL + RR) / 4.0
    return (getSpeed(0) + getSpeed(1) + getSpeed(2) + getSpeed(3)) / 4.0f;
}

float EncoderManager::getAngularVelocity() const {
    // Wz = (-FL + FR - RL + RR) / (4.0 * (L_X + L_Y))
    float denom = 4.0f * (L_X + L_Y);
    if (denom > 0.0f) {
        return (-getSpeed(0) + getSpeed(1) - getSpeed(2) + getSpeed(3)) / denom;
    }
    return 0.0f;
}

void EncoderManager::configureAll(const EncoderConfig& config) {
    _config = config;
    for (int i = 0; i < 4; i++) {
        if (_encoders[i] != nullptr) {
            _encoders[i]->configure(config);
        }
    }
}

EncoderTelemetry EncoderManager::getTelemetry(int index) const {
    if (index >= 0 && index < 4 && _encoders[index] != nullptr) {
        return _encoders[index]->getTelemetry();
    }
    EncoderTelemetry empty = {0};
    return empty;
}
