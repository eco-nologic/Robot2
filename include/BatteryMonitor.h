#ifndef BATTERY_MONITOR_H
#define BATTERY_MONITOR_H

#include "Config.h"
#include <Arduino.h>

/**
 * DEFENSE: "Comment gérez-vous la lecture de la tension ?"
 * ANSWER: Via un pont diviseur sur GPIO 34. On configure l'atténuation 
 * à 11dB pour lire jusqu'à 3.6V sur l'ADC 12 bits de l'ESP32.
 */
class BatteryMonitor {
private:
    float currentVoltage = 12.6f;
    bool lowBatteryWarning = false;

public:
    void begin() {
        analogSetAttenuation(ADC_11db);
        pinMode(Config::PIN_BATTERY_ADC, INPUT);
        Serial.println("[Battery] ADC Monitoring Ready");
    }

    /**
     * @brief Lit la tension et applique un lissage numérique.
     * DEFENSE: "Pourquoi lisser la mesure de tension ?"
     * ANSWER: Pour éliminer le bruit causé par les pics de consommation des moteurs (PWM), garantissant une alerte de batterie faible stable.
     */
    void update() {
        int raw = analogRead(Config::PIN_BATTERY_ADC);
        // 3.3V / 4095 * Ratio pont diviseur
        float voltage = (raw / 4095.0f) * 3.3f * Config::BatteryDividerRatio;
        
        currentVoltage = 0.9f * currentVoltage + 0.1f * voltage;

        if (currentVoltage < Config::BatteryLowVoltage) {
            lowBatteryWarning = true;
        }
    }

    float getVoltage() const { return currentVoltage; }

    float getPercentage() const {
        float pct = (currentVoltage - Config::BatteryLowVoltage) / 
                    (Config::BatteryFullVoltage - Config::BatteryLowVoltage) * 100.0f;
        return constrain(pct, 0.0f, 100.0f);
    }

    bool isLow() const { return lowBatteryWarning; }
};

#endif
