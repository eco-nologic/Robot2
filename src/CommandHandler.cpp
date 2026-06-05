#include "CommandHandler.h"
#include "Navigation.h"
#include "HeadingPIDController.h"
#include "ConfigManager.h"
#include "LIS3MDLManager.h"

extern HeadingPIDController headingPID;
extern bool headingHoldEnabled;
extern float targetHeading;
extern ConfigManager configManager;

CommandHandler::CommandHandler(DriveTrain* drive, Navigation* nav) 
    : _drive(drive), _navigation(nav) {}

void CommandHandler::processJSONCommand(uint8_t* data, size_t len) {
    JsonDocument doc;
    DeserializationError error = deserializeJson(doc, data, len);

    if (error) {
        Serial.println("❌ JSON Parse Error");
        return;
    }

    const char* cmd = doc["cmd"];
    if (!cmd) return;

    if (strcmp(cmd, "CONFIG") == 0) {
        float akp = doc["akp"] | 2.0f;
        float aki = doc["aki"] | 0.05f;
        float akd = doc["akd"] | 0.1f;
        
        headingPID.setGains(akp, aki, akd);
        // Persist values
        configManager.savePidGains(akp, aki, akd);
    } 
    else if (strcmp(cmd, "NORTH") == 0) {
        targetHeading = 0.0f; // North
        headingHoldEnabled = true;
        Serial.println("🧭 Heading Hold: NORTH");
    }
    else if (strcmp(cmd, "STOP") == 0) {
        headingHoldEnabled = false;
        if (_drive) _drive->stop();
    }
    else if (strcmp(cmd, "CALIBRATEMAG") == 0) {
        Serial.println("🧲 Starting Magnetometer Calibration...");
        // This is non-blocking if magManager is on another task
        // Otherwise, trigger the sequence
        magManager.calibrate(30000, 100); 
    }
    // Add other manual controls (FORWARD, etc) here
}