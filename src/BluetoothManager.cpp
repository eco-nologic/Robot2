#include "../include/BluetoothManager.h"
#include "../include/CommandHandler.h"
#include "Config.h"
#include "../include/TelemetryPacket.h"
#include <BLEDevice.h>
#include <BLEServer.h>
#include <BLEUtils.h>
#include <BLE2902.h>
class MyServerCallbacks : public BLEServerCallbacks {
    BluetoothManager* _manager;
public:
    MyServerCallbacks(BluetoothManager* manager) : _manager(manager) {}
    void onConnect(BLEServer* pServer) {
        _manager->isConnected = true;
        Serial.println("[BLE] 📲 Client connecté");
    }
    void onDisconnect(BLEServer* pServer) {
        _manager->isConnected = false;
        Serial.println("[BLE] 📴 Client déconnecté");
        BLEDevice::startAdvertising();
    }
};

// Callbacks pour gérer la réception de commandes (WRITE)
class MyCharacteristicCallbacks : public BLECharacteristicCallbacks {
    BluetoothManager* _manager;
public:
    MyCharacteristicCallbacks(BluetoothManager* manager) : _manager(manager) {}
    void onWrite(BLECharacteristic* pCharacteristic) {
        std::string rxValue = pCharacteristic->getValue();
        if (_manager != nullptr) {
            _manager->processIncomingCommand((uint8_t*)rxValue.c_str(), rxValue.length());
        }
    }
};

BluetoothManager::BluetoothManager(CommandHandler* handler) 
    : isConnected(false), _isInitialized(false), _commandHandler(handler) {}

bool BluetoothManager::begin() {
    Serial.println("[BLE] Initialisation du service BLE...");
    
    try {
        BLEDevice::init(Config::BLE_DEVICE_NAME);
        pServer = BLEDevice::createServer();
        if (!pServer) return false;
        
        pServer->setCallbacks(new MyServerCallbacks(this));

        BLEService* pService = pServer->createService(SERVICE_UUID);
        if (!pService) return false;

        pCharacteristic = pService->createCharacteristic(
            CHARACTERISTIC_UUID,
            BLECharacteristic::PROPERTY_READ | BLECharacteristic::PROPERTY_NOTIFY | BLECharacteristic::PROPERTY_WRITE
        );
        
        if (!pCharacteristic) return false;
        pCharacteristic->setCallbacks(new MyCharacteristicCallbacks(this));
        pCharacteristic->addDescriptor(new BLE2902());
        pService->start();

        BLEAdvertising* pAdvertising = BLEDevice::getAdvertising();
        pAdvertising->addServiceUUID(SERVICE_UUID);
        pAdvertising->setScanResponse(true);
        // Ensure the advertisement contains the device name for discovery
        BLEAdvertisementData ad;
        ad.setName(Config::BLE_DEVICE_NAME);
        pAdvertising->setAdvertisementData(ad);
        BLEDevice::startAdvertising();

        Serial.println("[BLE] ✅ Serveur actif. Nom: 'GiRobot_BLE'");
        _isInitialized = true;
        return true;
    } catch (...) {
        _isInitialized = false;
        return false;
    }
}

void BluetoothManager::sendTelemetry(const TelemetryPacket& packet) {
    if (!isConnected || !_isInitialized) return;

    char buffer[256];
    snprintf(buffer, sizeof(buffer),
             "A:%.1f,%.1f,%.1f|G:%.1f,%.1f,%.1f|M:%.1f,%.1f,%.1f|H:%.1f|B:%.1f|T:%lu|X:%.1f|Y:%.1f|Lth:%.1f|Ath:%.1f|Rth:%.1f|TV:%.1f|AV:%.1f|TW:%.3f|AW:%.3f|EL:%ld|ER:%ld",
             packet.accelX, packet.accelY, packet.accelZ,
             packet.gyroX, packet.gyroY, packet.gyroZ,
             packet.magX, packet.magY, packet.magZ,
             packet.robotHeading,
             packet.batteryVoltage,
             millis(),
             packet.robotX, packet.robotY,
             packet.targetL, packet.targetTheta, packet.targetR,
             packet.targetV, packet.actualV, packet.targetW, packet.actualW,
             packet.leftWheelSteps, packet.rightWheelSteps);

    if (pCharacteristic != nullptr) {
        pCharacteristic->setValue(buffer);
        pCharacteristic->notify();
        lastPacketTime = millis();
    }
}

void BluetoothManager::processIncomingCommand(const uint8_t* data, size_t length) {
    if (_commandHandler != nullptr) {
        _commandHandler->processJSONCommand(const_cast<uint8_t*>(data), length);
    }
}

void BluetoothManager::stop() {
    if (_isInitialized) {
        BLEDevice::deinit(false);
        _isInitialized = false;
        isConnected = false;
    }
}

bool BluetoothManager::isActivated() const {
    return _isInitialized;
}
