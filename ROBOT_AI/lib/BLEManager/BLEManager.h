#ifndef BLE_MANAGER_H
#define BLE_MANAGER_H

#include <Arduino.h>
#include <NimBLEDevice.h>
#include <vector>

// Nordic UART Service (NUS) UUIDs
#define NUS_SERVICE_UUID           "6E400001-B5A3-F393-E0A9-E50E24DCCA9E"
#define NUS_CHARACTERISTIC_RX_UUID "6E400002-B5A3-F393-E0A9-E50E24DCCA9E"
#define NUS_CHARACTERISTIC_TX_UUID "6E400003-B5A3-F393-E0A9-E50E24DCCA9E"

class BLEManager {
private:
    NimBLEServer* _pServer;
    NimBLECharacteristic* _pTxCharacteristic;
    NimBLECharacteristic* _pRxCharacteristic;
    bool _isClientConnected;
    std::vector<String> _rxQueue;

    BLEManager();

public:
    static BLEManager& getInstance() {
        static BLEManager instance;
        return instance;
    }

    void begin();
    void update();
    bool connected() const;
    void send(const String& message);
    bool available() const;
    String read();

    // Internal helpers for callbacks
    void pushRxQueue(const String& data);
    void setConnected(bool isConn);
};

#endif // BLE_MANAGER_H
