#ifndef CONFIG_MANAGER_H
#define CONFIG_MANAGER_H

#include <Preferences.h>
#include "Config.h"

/**
 * @class ConfigManager
 * @brief Gère le stockage permanent des paramètres (PID, Calibration) en NVS.
 */
class ConfigManager {
public:
    ConfigManager() = default;

    // Initialise l'accès à la mémoire Flash
    void begin();

    // Charge les valeurs sauvegardées ou utilise les défauts de Config.h
    void loadConfiguration();

    // Sauvegarde une valeur flottante spécifique (ex: Kp)
    void saveFloat(const char* key, float value);

    // Sauvegarde les gains PID
    void savePidGains(float kp, float ki, float kd);

private:
    Preferences _prefs;
};

#endif
