/**
 * @file bluetooth.cpp
 * @brief Implementations cho BluetoothModule.
 */

#include "bluetooth.h"

BluetoothModule& BluetoothModule::getInstance() {
    static BluetoothModule instance;
    return instance;
}

BluetoothModule::BluetoothModule() {}

void BluetoothModule::begin() {
    BLEManager::getInstance().begin();
}

void BluetoothModule::update() {
    BLEManager::getInstance().update();
}
