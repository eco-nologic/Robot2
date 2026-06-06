#include "Navigation.h"
#include "LIS3MDLManager.h"
#include <math.h>

extern SemaphoreHandle_t xI2CSemaphore;
extern LIS3MDLManager magManager;

Navigation::Navigation() {
    memset(&_rawData, 0, sizeof(ImuData));
    memset(&_orientation, 0, sizeof(OrientationData));
}

bool Navigation::begin() {
    Serial.println("\n[Nav] 🚀 Initialisation Navigation...");
    delay(200);
    
    // 1. Detect LSM6DS3 (Accel + Gyro)
    bool lsmFound = false;
    uint8_t lsmAddrs[] = {0x6A, 0x6B};
    for (uint8_t addr : lsmAddrs) {
        if (xSemaphoreTake(xI2CSemaphore, pdMS_TO_TICKS(100)) == pdTRUE) {
            if (_lsm.begin_I2C(addr, &Wire)) {
                Serial.printf("[Nav] ✅ LSM6DS3 found at 0x%02X\n", addr);
                lsmFound = true;
            }
            xSemaphoreGive(xI2CSemaphore);
        }
        if (lsmFound) break;
        delay(10);
    }

    if (!lsmFound) {
        Serial.println("[Nav] ❌ ERROR: LSM6DS3 not found on I2C bus");
        return false;
    }
    
    delay(100);

    // 2. Magnetometer is managed by the shared LIS3MDL manager task
    Serial.println("[Nav] ✅ LSM6DS3 initialized; LIS3MDL handled by magManager");
    
    // Calibrate gyro at startup
    calibrateGyro();

    _data.heading = 0;
    _data.rawMagHeading = 0;
    _data.gyroZ = 0;
    _data.isCalibrated = false;
    _lastUpdate = millis();
    _headingIntegral = 0;
    return true;
}

void Navigation::calibrateGyro() {
    bool isStable = false;
    while (!isStable) {
        Serial.println("[Nav] ⚖️ Gyro Calibration... DO NOT MOVE");
        float sumZ = 0, sumSqZ = 0;
        const int samples = 200;

        for (int i = 0; i < samples; i++) {
            sensors_event_t accel, gyro, temp;
            if (xSemaphoreTake(xI2CSemaphore, pdMS_TO_TICKS(20)) == pdTRUE) {
                _lsm.getEvent(&accel, &gyro, &temp);
                xSemaphoreGive(xI2CSemaphore);
            } else {
                delay(5);
                continue;
            }
            float val = gyro.gyro.z;
            sumZ += val;
            sumSqZ += val * val; 
            if (i % 20 == 0) Serial.print(".");
            delay(5);
        }

        float mean = sumZ / samples;
        float variance = (sumSqZ / samples) - (mean * mean);
        float stdDev = sqrt(fmax(0.0f, variance));

        if (stdDev < 0.015f) {
            _gyroBiasZ = mean;
            Serial.printf("\n[Nav] ✅ Stable. Bias: %.4f rad/s (StdDev: %.4f)\n", _gyroBiasZ, stdDev);
            isStable = true;
        } else {
            Serial.printf("\n[Nav] ⚠️ Unstable (StdDev: %.4f). Retry in 1s...\n", stdDev);
            delay(1000);
        }
    }

    _headingIntegral = 0;
    _lastUpdate = 0;
    _magFilter.reset();
}

void Navigation::update() {
    unsigned long now = millis();
    bool firstUpdate = (_lastUpdate == 0);
    float dt = firstUpdate ? 0.0f : (now - _lastUpdate) / 1000.0f;
    if (!firstUpdate && dt < 0.001f) return;

    // Read raw sensor data
    sensors_event_t accelEvent, gyroEvent, tempEvent;
    if (xSemaphoreTake(xI2CSemaphore, pdMS_TO_TICKS(20)) == pdTRUE) {
        _lsm.getEvent(&accelEvent, &gyroEvent, &tempEvent);
        xSemaphoreGive(xI2CSemaphore);
    } else {
        return;
    }

    float magX, magY, magZ;
    magManager.getCalibratedXYZ(magX, magY, magZ);

    // Map sensor axes to robot frame (X=Forward, Y=Left, Z=Up)
    _rawData.accelX = accelEvent.acceleration.y;
    _rawData.accelY = -accelEvent.acceleration.x;
    _rawData.accelZ = accelEvent.acceleration.z;

    _rawData.gyroX = gyroEvent.gyro.y;
    _rawData.gyroY = -gyroEvent.gyro.x;
    _rawData.gyroZ = -gyroEvent.gyro.z;

    // Magnetometer (negated for correct North alignment, then filtered)
    MagData rawMag(-magY, magX, magZ);
    MagData filteredMag = _magFilter.update(rawMag);

    _rawData.magX = filteredMag.x;
    _rawData.magY = filteredMag.y;
    _rawData.magZ = filteredMag.z;

    // Data from magManager is already calibrated and now filtered. Use for tilt compensation.
    float correctedMagX = _rawData.magX;
    float correctedMagY = _rawData.magY;
    float correctedMagZ = _rawData.magZ;

    // Calculate roll and pitch for tilt compensation
    _orientation.roll = atan2(_rawData.accelY, _rawData.accelZ);
    _orientation.pitch = atan2(-_rawData.accelX, sqrt(_rawData.accelY * _rawData.accelY + _rawData.accelZ * _rawData.accelZ));

    // Tilt compensation: project mag vector to horizontal plane
    float Xh = correctedMagX * cos(_orientation.pitch) + correctedMagZ * sin(_orientation.pitch);
    float Yh = correctedMagX * sin(_orientation.roll) * sin(_orientation.pitch) + correctedMagY * cos(_orientation.roll) - correctedMagZ * sin(_orientation.roll) * cos(_orientation.pitch);

    // Calculate absolute magnetic heading
    float magHeading = atan2(-Yh, Xh);

    // Convert raw mag heading to degrees (0-360) for diagnostics
    float rawMagDeg = magHeading * (180.0f / M_PI);
    if (rawMagDeg < 0) rawMagDeg += 360.0f;
    _data.rawMagHeading = rawMagDeg;

    // Complementary filter: Gyro (fast) + Magnetometer (stable)
    float gyroZRate = _rawData.gyroZ - _gyroBiasZ;

    // Suppress tiny gyro noise (residual bias below calibration threshold)
    const float GYRO_DEADBAND = 0.005f; // rad/s (~0.3 °/s)
    if (fabsf(gyroZRate) < GYRO_DEADBAND) gyroZRate = 0.0f;

    float headingError = magHeading - _headingIntegral;

    // Normalize error to [-PI, PI]
    while (headingError > M_PI) headingError -= 2.0f * M_PI;
    while (headingError < -M_PI) headingError += 2.0f * M_PI;

    // Snap to mag on first update (avoid slow convergence)
    if (firstUpdate) {
        _headingIntegral = magHeading;
    } else {
        _headingIntegral += (gyroZRate + 0.3f * headingError) * dt;
    }
    
    _lastUpdate = now;

    // Wrap heading to [-PI, PI]
    while (_headingIntegral > M_PI) _headingIntegral -= 2.0f * M_PI;
    while (_headingIntegral < -M_PI) _headingIntegral += 2.0f * M_PI;

    // Update nav data
    _data.gyroZ = gyroZRate;
    _data.heading = _headingIntegral * (180.0f / M_PI);
    if (_data.heading < 0) _data.heading += 360.0f;
}

void Navigation::startMagnetometerCalibration() {
    // DEFENSE: "Pourquoi déléguer la calibration au magManager ?"
    // ANSWER: Pour respecter le principe de responsabilité unique. Navigation gère la fusion 
    // de cap, mais c'est le magManager qui possède le matériel et calcule les offsets.
    magManager.startCalibration(); 
    Serial.println("[Nav] Magnetometer calibration initiated via magManager");
}

void Navigation::stopMagnetometerCalibration() {
    magManager.stopCalibration();

    // Reset both heading and signal processing filters to snap to new calibration
    _lastUpdate = 0;
    _headingIntegral = 0;
    _magFilter.reset();
    Serial.println("[Nav] Calibration stopped. Heading and mag filters reset to new baseline.");
}

bool Navigation::isMagnetometerCalibrated() const {
    return magManager.isCalibrated();
}

bool Navigation::isCalibrating() const {
    return magManager.isCalibrating();
}

int Navigation::getCalibrationProgress() const {
    return magManager.getCalibrationProgress();
}

float Navigation::getCorrectedHeadingDeg() const {
    float h = _data.heading - Config::CompassOffsetDeg;
    while (h < 0) h += 360.0f;
    while (h >= 360.0f) h -= 360.0f;
    return h;
}
