/**
 * @file bluetooth.h
 * @brief Phân hệ giao tiếp Bluetooth BLE không dây.
 */

#ifndef BLUETOOTH_MODULE_H
#define BLUETOOTH_MODULE_H

#include <Arduino.h>
#include "BLEManager.h"

class BluetoothModule {
public:
    static BluetoothModule& getInstance();

    void begin();
    void update();

private:
    BluetoothModule();
};

#endif // BLUETOOTH_MODULE_H
