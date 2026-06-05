#include "CommandHandler.h"
#include "Navigation.h"
#include "HeadingPIDController.h"
#include "ConfigManager.h"
#include "LIS3MDLManager.h"

extern HeadingPIDController headingPID;
extern bool headingHoldEnabled;
extern float targetHeading;
extern float manualLinearVelocity;
extern float manualAngularVelocity;
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

    Serial.printf("[CommandHandler] Received command: %s\n", cmd);

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
        manualLinearVelocity = 0.0f;
        manualAngularVelocity = 0.0f;
        Serial.println("🧭 Heading Hold: NORTH");
    }
    else if (strcmp(cmd, "STOP") == 0) {
        headingHoldEnabled = false;
        manualLinearVelocity = 0.0f;
        manualAngularVelocity = 0.0f;
        if (_drive) _drive->stop();
        if (_navigation && _navigation->isCalibrating()) {
            _navigation->stopMagnetometerCalibration();
        }
    }
    else if (strcmp(cmd, "FORWARD") == 0) {
        headingHoldEnabled = false;
        manualLinearVelocity = 80.0f;
        manualAngularVelocity = 0.0f;
        Serial.println("➡️ Manual control: FORWARD");
    }
    else if (strcmp(cmd, "BACKWARD") == 0) {
        headingHoldEnabled = false;
        manualLinearVelocity = -80.0f;
        manualAngularVelocity = 0.0f;
        Serial.println("⬅️ Manual control: BACKWARD");
    }
    else if (strcmp(cmd, "TURN_LEFT") == 0) {
        headingHoldEnabled = false;
        manualLinearVelocity = 0.0f;
        manualAngularVelocity = 0.60f;
        Serial.println("↰ Manual control: TURN_LEFT");
    }
    else if (strcmp(cmd, "TURN_RIGHT") == 0) {
        headingHoldEnabled = false;
        manualLinearVelocity = 0.0f;
        manualAngularVelocity = -0.60f;
        Serial.println("↱ Manual control: TURN_RIGHT");
    }
    else if (strcmp(cmd, "CALIBRATEMAG") == 0) {
        Serial.println("🧲 Starting Magnetometer Calibration...");
        headingHoldEnabled = false;
        manualLinearVelocity = 0.0f;
        manualAngularVelocity = 0.0f;
        if (_navigation) {
            _navigation->startMagnetometerCalibration();
        }
    }
    // Add other manual controls here
}