#ifndef BLUETOOTH_MANAGER_H
#define BLUETOOTH_MANAGER_H

#include <BLEDevice.h>
#include <BLEServer.h>
#include <BLEUtils.h>
#include <BLE2902.h>
#include "Config.h"
#include "TelemetryPacket.h"

// Forward declaration
class CommandHandler;

/**
 * @class BluetoothManager
 * @brief Gère la télémétrie "Out-of-band" via Bluetooth pour robot_remote.py
 */
class BluetoothManager {
public:
    BluetoothManager(CommandHandler* handler = nullptr);

    // DEFENSE: "Pourquoi utiliser le Bluetooth en plus du WiFi ?"
    // ANSWER: Pour disposer d'un canal de données indépendant et pour permettre le contrôle
    // via robot_remote.py même quand le WiFi est saturé.
    bool begin();

    // DEFENSE: "Comment le Bluetooth aide-t-il à la validation ?"
    // ANSWER: Il permet de streamer les coordonnées (x,y,t) vers un script Python sur PC 
    // qui enregistre un fichier CSV pour l'analyse post-mortem.
    void sendTelemetry(const TelemetryPacket& packet);

    // Pour la gestion des commandes entrantes via BLE
    void processIncomingCommand(const uint8_t* data, size_t length);

    // Arrête le service BLE
    void stop();

    // Vérifie si le module BLE est initialisé et actif
    bool isActivated() const;

    // Indique si un client BLE est connecté
    bool isConnected;
    void setConnected(bool state) { isConnected = state; }

private:
    BLEServer* pServer = nullptr;
    BLECharacteristic* pCharacteristic = nullptr;
    CommandHandler* _commandHandler;
    bool _isInitialized = false;
    unsigned long lastPacketTime;

    // UUIDs pour le service et la caractéristique (doivent correspondre au robot_remote.py)
    const char* SERVICE_UUID = "4fafc201-1fb5-459e-8fcc-c5c9c331914b";
    const char* CHARACTERISTIC_UUID = "beb5483e-36e1-4688-b7f5-ea07361b26a8";
};

#endif
