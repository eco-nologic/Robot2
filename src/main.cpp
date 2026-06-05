#include <Arduino.h>
#include <Wire.h>

// Core components
#include "Config.h"
#include "IMotor.h"
#include "DCMotor.h"
#include "DriveTrain.h"
#include "Navigation.h"
#include "BatteryMonitor.h"
#include "ConfigManager.h"
#include "TelemetryPacket.h"
#include "CommandHandler.h"
#include "BluetoothManager.h"
#include "LIS3MDLManager.h"
#include "HeadingPIDController.h"

// Global I2C Mutex - Used to prevent collisions between IMU and Magnetometer tasks
SemaphoreHandle_t xI2CSemaphore = NULL;

// Global instances
DCMotor motorLeft(0);
DCMotor motorRight(1);
DriveTrain driveTrain(&motorLeft, &motorRight);
Navigation navigation;
BatteryMonitor battery;
ConfigManager configManager;
CommandHandler commandHandler(&driveTrain, &navigation);
BluetoothManager bluetoothManager(&commandHandler);

// Heading Control State
HeadingPIDController headingPID(2.0f, 0.05f, 0.1f); // Adjust Kp, Ki, Kd as needed
float targetHeading = 0.0f;
bool headingHoldEnabled = false;
float baseForwardSpeed = 60.0f;

// Timing
unsigned long lastTelemetryTime = 0;
const unsigned long TELEMETRY_INTERVAL = 100;  // Send telemetry every 100ms



/**
 * @brief Initialize all hardware and communication systems
 */
void setup() {
    // 1. Serial console (debugging)
    Serial.begin(Config::SERIAL_BAUD);
    delay(500);
    Serial.println("\n\n================================");
    Serial.println("🤖 DrawRobot v2.0 - Startup");
    Serial.println("================================\n");
    
    // 0. Initialize Synchronization Primitives
    xI2CSemaphore = xSemaphoreCreateMutex();
    if (xI2CSemaphore == NULL) {
        Serial.println("❌ CRITICAL: Failed to create I2C Semaphore!");
        while (true) delay(1000);
    }

    // 1. I2C bus recovery before initialization
    Serial.println("[Setup] Performing I2C bus recovery...");
    pinMode(Config::PIN_I2C_SCL, OUTPUT);
    pinMode(Config::PIN_I2C_SDA, OUTPUT);
    digitalWrite(Config::PIN_I2C_SDA, HIGH);
    for (int i = 0; i < 10; i++) {
        digitalWrite(Config::PIN_I2C_SCL, LOW);
        delayMicroseconds(10);
        digitalWrite(Config::PIN_I2C_SCL, HIGH);
        delayMicroseconds(10);
    }
    pinMode(Config::PIN_I2C_SDA, INPUT_PULLUP);
    pinMode(Config::PIN_I2C_SCL, INPUT_PULLUP);
    delay(50);

    // 2. I2C bus for IMU sensors
    Wire.begin(Config::PIN_I2C_SDA, Config::PIN_I2C_SCL, Config::I2C_FREQUENCY);
    Wire.setTimeOut(100); // Set 100ms timeout for I2C recovery
    Serial.printf("[Setup] I2C initialized (SDA:%d, SCL:%d, %dkHz)\n", 
                  Config::PIN_I2C_SDA, Config::PIN_I2C_SCL, Config::I2C_FREQUENCY / 1000);

    // 2b. Initialize Magnetometer Manager (Task starts here)
    magManager.begin();
    Serial.println("✅ Magnetometer Manager started");

    // 3. Initialize motors
    Serial.println("\n[Setup] Initializing motors...");
    if (!driveTrain.begin()) {
        Serial.println("❌ CRITICAL: DriveTrain initialization failed!");
        while (true) delay(1000);  // Halt
    }
    Serial.println("✅ Motors ready");

    // 4. Initialize IMU + magnetometer
    Serial.println("\n[Setup] Initializing IMU sensors...");
    if (!navigation.begin()) {
        Serial.println("❌ CRITICAL: Navigation (IMU) initialization failed!");
        while (true) delay(1000);  // Halt
    }
    Serial.println("✅ IMU ready");

    // 5. Initialize battery monitor
    Serial.println("\n[Setup] Initializing battery monitor...");
    battery.begin();
    battery.update();
    Serial.printf("🔋 Battery: %.2fV (%d%%)\n", battery.getVoltage(), (int)battery.getPercentage());

    // 6. Initialize configuration manager (NVS)
    Serial.println("\n[Setup] Initializing config manager...");
    configManager.begin();
    configManager.loadConfiguration();

    // 7. Initialize Bluetooth (BLE)
    Serial.println("\n[Setup] Initializing Bluetooth...");
    if (!bluetoothManager.begin()) {
        Serial.println("⚠️  WARNING: BLE initialization failed (continuing anyway)");
    } else {
        Serial.println("✅ Bluetooth ready - Connect with robot_remote.py");
    }

    // WiFi disabled to avoid ADC2 conflicts; BLE-only telemetry
    Serial.println("\n[Setup] WiFi disabled (ADC2 conflict avoidance)");

    // 9. Set initial position
    Serial.println("\n[Setup] Calibrating position...");
    driveTrain.resetPosition();
    Serial.println("✅ Position reset to (0, 0, 0°)");

    Serial.println("\n✅ STARTUP COMPLETE - Robot ready!");
    Serial.println("\n📡 Telemetry streaming to robot_remote.py (100ms interval)");
    Serial.println("🧭 Next: Rotate robot 360° slowly for magnetometer calibration\n");

    lastTelemetryTime = millis();
}

/**
 * @brief Main control loop (~50 Hz)
 */
void loop() {
    unsigned long loopStart = millis();

    // ======== UPDATE SENSORS ========
    
    // 1. Read motors + update odometry
    driveTrain.updateOdometry(20);  // Assume ~20ms loop time
    
    // 2. Update IMU + heading
    navigation.update();

    // 2b. Apply Heading PID Correction
    if (headingHoldEnabled && !navigation.isCalibrating()) {
        float currentH = (float)magManager.getHeading();
        float correction = headingPID.compute(targetHeading, currentH, 0.020f); // 20ms = 0.02s

        // Apply correction differentially
        // If correction is positive, turn right (left motor faster, right motor slower)
        motorLeft.setPwm(baseForwardSpeed + correction);
        motorRight.setPwm(baseForwardSpeed - correction);
    }

    // 3. Update battery monitor
    battery.update();

    // ======== TELEMETRY (100ms interval) ========
    
    unsigned long now = millis();
    if (now - lastTelemetryTime >= TELEMETRY_INTERVAL) {
        // Build telemetry packet
        TelemetryPacket packet;
        memset(&packet, 0, sizeof(TelemetryPacket));

        // Position (odometry)
        float posX, posY, heading;
        driveTrain.getPosition(&posX, &posY, &heading);
        packet.robotX = posX;
        packet.robotY = posY;
        // Use fused, tilt-compensated heading for telemetry
        packet.robotHeading = navigation.getCorrectedHeadingDeg();
        // Provide odometry-only heading as ghostHeading for diagnostics
        packet.ghostHeading = heading * 180.0f / M_PI;  // Convert to degrees

        // Motors
        packet.leftWheelSpeed = motorLeft.getSpeed();
        packet.rightWheelSpeed = motorRight.getSpeed();
        packet.leftWheelSteps = motorLeft.getEncoderCount();
        packet.rightWheelSteps = motorRight.getEncoderCount();

        // IMU raw data
        ImuData imu = navigation.getRawData();
        packet.accelX = imu.accelX;
        packet.accelY = imu.accelY;
        packet.accelZ = imu.accelZ;
        packet.gyroX = imu.gyroX;
        packet.gyroY = imu.gyroY;
        packet.gyroZ = imu.gyroZ;
        magManager.getCalibratedXYZ(packet.magX, packet.magY, packet.magZ);

        // Battery
        packet.batteryVoltage = battery.getVoltage();
        packet.isCalibrated = navigation.isMagnetometerCalibrated();

        // Calibration progress
        packet.targetTheta = navigation.getCalibrationProgress(); 

        // Send to BLE
        if (bluetoothManager.isActivated() && bluetoothManager.isConnected) {
            bluetoothManager.sendTelemetry(packet);
        }

        // WiFi telemetry disabled to avoid ADC2 conflicts

        // Serial debug (every 1 second)
        static unsigned long lastDebugTime = 0;
        if (now - lastDebugTime >= 1000) {
            Serial.printf("[Loop] X:%.1f Y:%.1f H:%.1f° | Bat:%.2fV | Calib:%d%% | BLE:%s\n",
                          packet.robotX, packet.robotY, packet.robotHeading,
                          packet.batteryVoltage,
                          (int)packet.targetTheta,
                          bluetoothManager.isConnected ? "✅" : "❌");
            lastDebugTime = now;
        }

        lastTelemetryTime = now;
    }

    // ======== HANDLE CALIBRATION COMMAND ========
    
    // Use navigation.getCalibrationProgress() to detect active calibration
    int calibProgress = navigation.getCalibrationProgress();
    if (calibProgress > 0) {
        if (calibProgress >= 100) {
            Serial.println("\n✅ Magnetometer calibration complete!");
            navigation.stopMagnetometerCalibration();
            // Stop motion now that calibration finished
            driveTrain.stop();
            Serial.println("[Main] Robot stopped after calibration");
        }
    }

    // ======== TIMING ========
    
    // Maintain ~50Hz loop frequency (20ms per cycle)
    unsigned long loopDuration = millis() - loopStart;
    if (loopDuration < 20) {
        delayMicroseconds((20 - loopDuration) * 1000);
    } else if (loopDuration > 25) {
        Serial.printf("⚠️  Loop overrun: %lu ms\n", loopDuration);
    }
}
