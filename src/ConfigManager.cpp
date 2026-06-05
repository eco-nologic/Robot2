#include "ConfigManager.h"
#include "HeadingPIDController.h"

extern HeadingPIDController headingPID;

void ConfigManager::begin() {
    _prefs.begin("robot_cfg", false);
}

void ConfigManager::loadConfiguration() {
    // Load Heading PID (Angular)
    float kp = _prefs.getFloat("akp", 2.0f);
    float ki = _prefs.getFloat("aki", 0.05f);
    float kd = _prefs.getFloat("akd", 0.1f);
    
    headingPID.setGains(kp, ki, kd);
    Serial.printf("[Config] Loaded Heading PID: P:%.2f I:%.2f D:%.2f\n", kp, ki, kd);
}

void ConfigManager::saveFloat(const char* key, float value) {
    _prefs.putFloat(key, value);
}

void ConfigManager::savePidGains(float kp, float ki, float kd) {
    _prefs.putFloat("akp", kp);
    _prefs.putFloat("aki", ki);
    _prefs.putFloat("akd", kd);
    Serial.println("[Config] PID gains saved to NVS");
}