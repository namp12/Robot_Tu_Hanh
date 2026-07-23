#ifndef ULTRASONIC_MANAGER_H
#define ULTRASONIC_MANAGER_H

#include <Arduino.h>
#include "Sensor_HC_SR04.h"

class UltrasonicManager {
private:
    float _frontDist;
    float _rearDist;

    UltrasonicManager();

public:
    static UltrasonicManager& getInstance() {
        static UltrasonicManager instance;
        return instance;
    }

    void begin();
    void update(bool frontActive = true, bool rearActive = true);
    float getFrontDistance() const { return _frontDist; }
    float getRearDistance() const { return _rearDist; }
    bool isFrontOnline() const { return HC_SR04_FrontOnline(); }
    bool isRearOnline() const { return HC_SR04_RearOnline(); }
};

#endif // ULTRASONIC_MANAGER_H
