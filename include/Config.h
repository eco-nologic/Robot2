#ifndef CONFIG_H
#define CONFIG_H

#include <Arduino.h>
#include <cmath>

/**
 * @file Config.h
 * @brief Central configuration for DrawRobot hardware, physics, and communication parameters
 * 
 * All pins and constants are defined here. Modify this file to adapt to different
 * hardware variants (different encoder counts, motor power, etc.)
 */

namespace Config {

// ============================================================================
// FIRMWARE VERSION & BUILD INFO
// ============================================================================
constexpr char FirmwareVersion[] = "2.0.0-Redesign";
constexpr char RobotName[] = "GiRobot";

// ============================================================================
// PHYSICAL ROBOT PARAMETERS (from travail.pdf / pin.pdf)
// ============================================================================
constexpr float WheelDiameterMm = 90.0f;      // Drive wheel diameter
constexpr float WheelBaseMm = 83.0f;          // Distance between wheel centers (track width)
constexpr float PenOffsetMm = 130.0f;         // Distance from axle to pen tip (forward)
constexpr int EncoderStepsPerRev = 1070;      // Encoder ticks per wheel revolution
constexpr float MaxLinearSpeedMmS = 120.0f;   // Maximum forward speed
constexpr float MaxAngularSpeedRadS = 2.9f;   // Maximum rotation speed (rad/s)
constexpr int MotorPwmDeadband = 151;         // Minimum PWM to overcome static friction
constexpr int MotorPwmMax = 255;              // Maximum PWM value

// Derived constants (computed at compile time)
constexpr float WheelCircumferenceMm = WheelDiameterMm * M_PI;
constexpr float MmPerEncoderStep = WheelCircumferenceMm / EncoderStepsPerRev;
constexpr float RadiusFromWheelMm = WheelBaseMm / 2.0f;

// ============================================================================
// MOTOR CONTROL - LEFT MOTOR (from pin.pdf)
// ============================================================================
constexpr int PIN_MOTOR_LEFT_EN = 4;          // PWM enable (0-255)
constexpr int PIN_MOTOR_LEFT_IN1 = 17;        // Direction control
constexpr int PIN_MOTOR_LEFT_IN2 = 16;        // Direction control
constexpr int PWM_CHANNEL_LEFT = 0;           // ESP32 PWM channel
constexpr int PWM_FREQUENCY_LEFT = 5000;      // 5 kHz

// ============================================================================
// MOTOR CONTROL - RIGHT MOTOR (from pin.pdf)
// ============================================================================
constexpr int PIN_MOTOR_RIGHT_EN = 23;        // PWM enable (0-255)
constexpr int PIN_MOTOR_RIGHT_IN1 = 19;       // Direction control
constexpr int PIN_MOTOR_RIGHT_IN2 = 18;       // Direction control
constexpr int PWM_CHANNEL_RIGHT = 1;          // ESP32 PWM channel
constexpr int PWM_FREQUENCY_RIGHT = 5000;     // 5 kHz

// Motor inversion flags (set true if a motor is wired/reversed)
constexpr bool MOTOR_LEFT_INVERTED = false;   // Set to true to invert left motor direction
constexpr bool MOTOR_RIGHT_INVERTED = true;   // Set to true to invert right motor direction

// ============================================================================
// ENCODER FEEDBACK - QUADRATURE A/B CHANNELS
// ============================================================================
constexpr int PIN_ENCODER_LEFT_A = 33;        // Left encoder phase A
constexpr int PIN_ENCODER_LEFT_B = 32;        // Left encoder phase B
constexpr int PIN_ENCODER_RIGHT_A = 27;       // Right encoder phase A
constexpr int PIN_ENCODER_RIGHT_B = 14;       // Right encoder phase B

// ============================================================================
// I2C BUS (IMU & Magnetometer Communication)
// ============================================================================
constexpr int PIN_I2C_SDA = 21;               // Serial data line
constexpr int PIN_I2C_SCL = 22;               // Serial clock line
constexpr int I2C_FREQUENCY = 100000;         // 100 kHz for improved stability
constexpr uint8_t ADDR_MPU9250 = 0x68;        // MPU9250 IMU (accel + gyro + mag)
constexpr uint8_t ADDR_BNO055 = 0x28;         // BNO055 IMU (alternative)
constexpr float CompassOffsetDeg = 348.0f;    // Calibration: facing North

// ============================================================================
// USER INTERFACE LEDs
// ============================================================================
constexpr int PIN_LED_USER1 = 25;             // Status LED 1 (WiFi status)
constexpr int PIN_LED_USER2 = 26;             // Status LED 2 (BLE status)

// ============================================================================
// BATTERY MONITORING (ADC)
// ============================================================================
constexpr int PIN_BATTERY_ADC = 0;            // Battery voltage ADC input
constexpr float BatteryDividerRatio = 2.0f;   // Voltage divider: battery / ADC
constexpr float BatteryLowVoltage = 6.6f;     // Low battery threshold (4S LiPo min)
constexpr float BatteryFullVoltage = 16.8f;   // Full charge (4S LiPo max)
constexpr float BatteryAdcRefVoltage = 3.3f;  // ADC reference voltage

// ============================================================================
// PEN CONTROL (Third Wheel - Passive Contact)
// ============================================================================
constexpr int PIN_PEN_SERVO = 12;             // Servo PWM for pen lift (if used)
constexpr int PIN_PEN_DOWN = 13;              // Digital output for pen down
constexpr int PIN_PEN_UP = 15;                // Digital output for pen up

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
