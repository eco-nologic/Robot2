#ifndef COMMAND_HANDLER_H
#define COMMAND_HANDLER_H

#include <ArduinoJson.h>
#include "DriveTrain.h"

// Forward declaration
class Navigation;

/**
 * @class CommandHandler
 * @brief Centralizes command processing logic for all transports (WiFi, BLE).
 */
class CommandHandler {
public:
    CommandHandler(DriveTrain* drive = nullptr, Navigation* nav = nullptr);
    void processJSONCommand(uint8_t* data, size_t len);

private:
    DriveTrain* _drive;
    Navigation* _navigation;
};

#endif
