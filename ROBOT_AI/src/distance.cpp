/**
 * @file distance.cpp
 * @brief Implementations cho DistanceModule.
 */

#include "distance.h"

DistanceModule& DistanceModule::getInstance() {
    static DistanceModule instance;
    return instance;
}

DistanceModule::DistanceModule() {}

void DistanceModule::begin() {
    HC_SR04_Init();
}

void DistanceModule::update(bool updateFront, bool updateRear) {
    HC_SR04_Update(updateFront, updateRear);
}

float DistanceModule::getFrontDistance() {
    return HC_SR04_GetFrontDistance();
}

float DistanceModule::getRearDistance() {
    return HC_SR04_GetRearDistance();
}

bool DistanceModule::isFrontOnline() {
    return HC_SR04_FrontOnline();
}

bool DistanceModule::isRearOnline() {
    return HC_SR04_RearOnline();
}
