#ifndef COMMS_MANAGER_H
#define COMMS_MANAGER_H

#include <WiFi.h>
#include <ESPAsyncWebServer.h>
#include <ArduinoJson.h>
#include "TelemetryPacket.h"

// Forward declarations
class BatteryMonitor;
class IMotor;
class CommandHandler;

/**
 * DEFENSE: "Pourquoi utiliser des WebSockets et du JSON ?"
 * ANSWER: Les WebSockets permettent une communication bi-directionnelle en temps réel 
 * sans le surpoids du HTTP. Le JSON est standard, léger et facile à traiter en JavaScript.
 */
class CommsManager {
public:
    CommsManager(BatteryMonitor* bm = nullptr, IMotor* left = nullptr, IMotor* right = nullptr, CommandHandler* handler = nullptr);

    void begin();
    
    // Method called from main loop to process and send telemetry
    void sendTelemetry();

    // Envoie l'état du robot (X, Y, Cap, Batterie) au tableau de bord
    void broadcastTelemetry(const TelemetryPacket& packet);

private:
    AsyncWebServer _server;
    AsyncWebSocket _ws;
    
    // Pointers to subsystems for data retrieval
    BatteryMonitor* _batteryMonitor;
    IMotor* _leftMotor;
    IMotor* _rightMotor;
    CommandHandler* _commandHandler;

    void onEvent(AsyncWebSocket* s, AsyncWebSocketClient* c, AwsEventType t, void* arg, uint8_t* data, size_t len);
};
#endif
