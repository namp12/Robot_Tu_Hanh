#include "BLEManager.h"
#include "Protocol/SerialProtocol.h"
#include "Managers/CommandManager.h"
#include "robot_global.h"

class ServerCallbacks : public NimBLEServerCallbacks {
    void onConnect(NimBLEServer* pServer) override {
        BLEManager::getInstance().setConnected(true);
        Serial.println(F("[BLE] Connected"));
    }

    void onDisconnect(NimBLEServer* pServer) override {
        BLEManager::getInstance().setConnected(false);
        Serial.println(F("[BLE] Disconnected"));
        Serial.println(F("[BLE] Restart Advertising"));
        NimBLEDevice::startAdvertising();
    }
};

class RXCallbacks : public NimBLECharacteristicCallbacks {
    void onWrite(NimBLECharacteristic* pCharacteristic) override {
        std::string rxValue = pCharacteristic->getValue();
        if (rxValue.length() > 0) {
            String incoming = String(rxValue.c_str());
            incoming.trim();
            if (incoming.length() > 0) {
                BLEManager::getInstance().pushRxQueue(incoming);
            }
        }
    }
};

BLEManager::BLEManager() 
    : _pServer(nullptr), 
      _pTxCharacteristic(nullptr), 
      _pRxCharacteristic(nullptr), 
      _isClientConnected(false) {}

void BLEManager::begin() {
    NimBLEDevice::init("ESP32_Robot");
    NimBLEDevice::setPower(ESP_PWR_LVL_P9);

    _pServer = NimBLEDevice::createServer();
    _pServer->setCallbacks(new ServerCallbacks());

    NimBLEService* pService = _pServer->createService(NUS_SERVICE_UUID);

    _pTxCharacteristic = pService->createCharacteristic(
        NUS_CHARACTERISTIC_TX_UUID,
        NIMBLE_PROPERTY::NOTIFY
    );

    _pRxCharacteristic = pService->createCharacteristic(
        NUS_CHARACTERISTIC_RX_UUID,
        NIMBLE_PROPERTY::WRITE | NIMBLE_PROPERTY::WRITE_NR
    );
    _pRxCharacteristic->setCallbacks(new RXCallbacks());

    pService->start();

    NimBLEAdvertising* pAdvertising = NimBLEDevice::getAdvertising();
    pAdvertising->addServiceUUID(NUS_SERVICE_UUID);
    pAdvertising->setScanResponse(true);
    pAdvertising->start();

    Serial.println(F("[BLE] Advertising started. Device Name: ESP32_Robot"));
}

bool BLEManager::connected() const {
    return _isClientConnected;
}

void BLEManager::setConnected(bool isConn) {
    _isClientConnected = isConn;
}

void BLEManager::pushRxQueue(const String& data) {
    _rxQueue.push_back(data);
}

bool BLEManager::available() const {
    return !_rxQueue.empty();
}

String BLEManager::read() {
    if (_rxQueue.empty()) return "";
    String val = _rxQueue.front();
    _rxQueue.erase(_rxQueue.begin());
    return val;
}

void BLEManager::send(const String& message) {
    if (_isClientConnected && _pTxCharacteristic != nullptr) {
        _pTxCharacteristic->setValue(message.c_str());
        _pTxCharacteristic->notify();
    }
}

void BLEManager::update() {
    while (available()) {
        String incoming = read();
        incoming.trim();
        if (incoming.length() == 0) continue;

        Serial.printf("[BLE RX] Command received: '%s'\n", incoming.c_str());

        RobotCommand cmd;
        if (SerialProtocol::getInstance().parseCommand(incoming, cmd)) {
            CommandManager::getInstance().executeCommand(cmd);
        } else {
            processCommand(incoming);
        }
    }
}
