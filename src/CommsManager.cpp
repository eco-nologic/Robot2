#include "../include/CommsManager.h"
#include "Config.h"
#include <LittleFS.h>
#include <ArduinoJson.h>
#include "../include/TelemetryPacket.h"
#include "../include/CommandHandler.h"
#include "../include/BatteryMonitor.h"
#include "../include/IMotor.h"

CommsManager::CommsManager(BatteryMonitor* bm, IMotor* left, IMotor* right, CommandHandler* handler) 
    : _server(80), _ws("/ws"), 
      _batteryMonitor(bm), 
      _leftMotor(left), 
      _rightMotor(right),
      _commandHandler(handler) {}

void CommsManager::begin() {
    _ws.onEvent([this](AsyncWebSocket* s, AsyncWebSocketClient* c, AwsEventType t, void* arg, uint8_t* data, size_t len) {
        if (t == WS_EVT_CONNECT) Serial.printf("[WS] Client %u connected. Total: %u\n", c->id(), s->count());
        this->onEvent(s, c, t, arg, data, len);
    });

    _server.addHandler(&_ws);

    _server.on("/favicon.ico", HTTP_GET, [](AsyncWebServerRequest *request) {
        request->send(204);
    });

    auto handleIndex = [](AsyncWebServerRequest *request) {
        request->send(LittleFS, "/index.html", "text/html");
    };
    _server.on("/", HTTP_GET, handleIndex);
    _server.on("/index.html", HTTP_GET, handleIndex);

    _server.serveStatic("/", LittleFS, "/");

    _server.begin();
    Serial.println("[CommsManager] WebSocket server started on port 80");
}

void CommsManager::onEvent(AsyncWebSocket* s, AsyncWebSocketClient* c, AwsEventType t, void* arg, uint8_t* data, size_t len) {
    if (t == WS_EVT_DATA) {
        if (_commandHandler) {
            _commandHandler->processJSONCommand(data, len);
        }
    }
}

void CommsManager::sendTelemetry() {
    if (!_batteryMonitor || !_leftMotor || !_rightMotor) return;

    TelemetryPacket packet;
    memset(&packet, 0, sizeof(TelemetryPacket));
    
    packet.robotX = 0;
    packet.robotY = 0;
    packet.robotHeading = 0;
    packet.batteryVoltage = _batteryMonitor->getVoltage();
    packet.leftWheelSteps = _leftMotor->getEncoderCount();
    packet.rightWheelSteps = _rightMotor->getEncoderCount();
    packet.isCalibrated = false;

    broadcastTelemetry(packet);
}

void CommsManager::broadcastTelemetry(const TelemetryPacket& packet) {
    if (_ws.count() > 0) {
        _ws.cleanupClients();

        JsonDocument doc;
        doc["x"] = packet.robotX;
        doc["y"] = packet.robotY;
        doc["h"] = packet.robotHeading;
        doc["b"] = packet.batteryVoltage;
        doc["t"] = millis();

        doc["ax"] = packet.accelX;
        doc["ay"] = packet.accelY;
        doc["az"] = packet.accelZ;
        doc["gx"] = packet.gyroX;
        doc["gy"] = packet.gyroY;
        doc["gz"] = packet.gyroZ;
        doc["mx"] = packet.magX;
        doc["my"] = packet.magY;
        doc["mz"] = packet.magZ;

        doc["gx_o"] = packet.ghostX;
        doc["gy_o"] = packet.ghostY;
        doc["gh_o"] = packet.ghostHeading;

        doc["ls"] = packet.leftWheelSpeed;
        doc["rs"] = packet.rightWheelSpeed;
        doc["lc"] = packet.leftWheelSteps;
        doc["rc"] = packet.rightWheelSteps;
        doc["EL"] = packet.leftWheelSteps;
        doc["ER"] = packet.rightWheelSteps;
        doc["lth"] = packet.targetL;
        doc["ath"] = packet.targetTheta;
        doc["rth"] = packet.targetR;
        doc["tv"] = packet.targetV;
        doc["av"] = packet.actualV;
        doc["tw"] = packet.targetW;
        doc["aw"] = packet.actualW;
        
        doc["calib"] = packet.isCalibrated; 
        String output;
        serializeJson(doc, output);
        _ws.textAll(output);
    }
}
