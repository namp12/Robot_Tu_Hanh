/**
 * @file distance.h
 * @brief Phân hệ đọc Cảm biến Khoảng cách Siêu âm HC-SR04 (Trước & Sau).
 */

#ifndef DISTANCE_MODULE_H
#define DISTANCE_MODULE_H

#include <Arduino.h>
#include "Sensor_HC_SR04.h"

class DistanceModule {
public:
    static DistanceModule& getInstance();

    void begin();
    void update(bool updateFront = true, bool updateRear = true);

    float getFrontDistance();
    float getRearDistance();

    bool isFrontOnline();
    bool isRearOnline();

private:
    DistanceModule();
};

#endif // DISTANCE_MODULE_H
