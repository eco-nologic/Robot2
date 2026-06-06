#ifndef CONFIG_H
#define CONFIG_H

#include <Arduino.h>
#include <cmath>
#include "ConfigHardware.h"

/**
 * @file Config.h
 * @brief Central configuration for DrawRobot firmware, communication, and control.
 *
 * Hardware-specific constants are stored in ConfigHardware.h.
 */

namespace Config {

// ============================================================================
// FIRMWARE VERSION & BUILD INFO
// ============================================================================
constexpr char FirmwareVersion[] = "2.0.0-Redesign";
constexpr char RobotName[] = "GiRobot";

// Runtime-configurable compass offset (degrees). Stored in NVS and applied at startup.
extern float CompassOffsetDeg;

// ============================================================================
// WIFI & BLE CONFIGURATION
// ============================================================================
constexpr char WIFI_SSID[] = "GiRobot_AP";
constexpr char WIFI_PASSWORD[] = "12345678";
constexpr bool WIFI_AP_MODE = true;           // Create WiFi access point
constexpr int WIFI_CHANNEL = 1;
constexpr int WIFI_MAX_CONNECTIONS = 4;

constexpr char BLE_DEVICE_NAME[] = "GiRobot_BLE";
constexpr char BLE_SERVICE_UUID[] = "4fafc201-1fb5-459e-8fcc-c5c9c331914b";
constexpr char BLE_CHAR_UUID[] = "beb5483e-36e1-4688-b7f5-ea07361b26a8";

// ============================================================================
// TELEMETRY & COMMUNICATION
// ============================================================================
constexpr int TELEMETRY_INTERVAL_MS = 100;    // Send telemetry every 100ms
constexpr int SERIAL_BAUD = 115200;           // Serial monitor speed
constexpr int WEBSOCKET_PORT = 80;            // HTTP/WebSocket port

// ============================================================================
// CONTROL LOOP TIMING
// ============================================================================
constexpr int MAIN_LOOP_FREQUENCY_HZ = 50;    // 50 Hz = 20ms per cycle
constexpr int ENCODER_READ_FREQUENCY_HZ = 100; // 100 Hz encoder sampling
constexpr int IMU_FREQUENCY_HZ = 100;         // 100 Hz IMU sampling

// ============================================================================
// PID CONTROL PARAMETERS (placeholder, to be tuned)
// ============================================================================
constexpr float PID_KP_LINEAR = 0.5f;         // Proportional gain for linear motion
constexpr float PID_KI_LINEAR = 0.1f;         // Integral gain
constexpr float PID_KD_LINEAR = 0.2f;         // Derivative gain

constexpr float PID_KP_ANGULAR = 1.0f;        // Proportional gain for angular motion
constexpr float PID_KI_ANGULAR = 0.1f;
constexpr float PID_KD_ANGULAR = 0.3f;

} // namespace Config

#endif // CONFIG_H
