#include "UltrasonicManager.h"

UltrasonicManager::UltrasonicManager() 
    : _frontDist(400.0f), _rearDist(400.0f) {}

void UltrasonicManager::begin() {
    HC_SR04_Init();
}

void UltrasonicManager::update(bool frontActive, bool rearActive) {
    HC_SR04_Update(frontActive, rearActive);
    _frontDist = HC_SR04_GetFrontDistance();
    _rearDist = HC_SR04_GetRearDistance();
}
