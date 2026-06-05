#include "Navigation.h"
#include <math.h>

Navigation::Navigation() {
    memset(&_rawData, 0, sizeof(ImuData));
    memset(&_orientation, 0, sizeof(OrientationData));
    _magMinX = 1000.0f;
    _magMaxX = -1000.0f;
    _magMinY = 1000.0f;
    _magMaxY = -1000.0f;
}

bool Navigation::begin() {
    Serial.println("\n[Nav] 🚀 Initialisation Navigation...");
    delay(200);
    
    // 1. Detect LSM6DS3 (Accel + Gyro)
    bool lsmFound = false;
    uint8_t lsmAddrs[] = {0x6A, 0x6B};
    for (uint8_t addr : lsmAddrs) {
        if (_lsm.begin_I2C(addr, &Wire)) {
            Serial.printf("[Nav] ✅ LSM6DS3 found at 0x%02X\n", addr);
            lsmFound = true;
            break;
        }
        delay(10);
    }

    if (!lsmFound) {
        Serial.println("[Nav] ❌ ERROR: LSM6DS3 not found on I2C bus");
        return false;
    }
    
    delay(100);

    // 2. Detect LIS3MDL (Magnetometer)
    bool lisFound = false;
    uint8_t lisAddrs[] = {0x1C, 0x1E};
    for (uint8_t addr : lisAddrs) {
        if (_lis.begin_I2C(addr, &Wire)) {
            Serial.printf("[Nav] ✅ LIS3MDL found at 0x%02X\n", addr);
            lisFound = true;
            break;
        }
        delay(10);
    }

    if (!lisFound) {
        Serial.println("[Nav] ❌ ERROR: LIS3MDL not found on I2C bus");
        return false;
    }

    Serial.println("[Nav] ✅ Both IMU sensors initialized");
    
    // Calibrate gyro at startup
    calibrateGyro();

    _data.heading = 0;
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
            _lsm.getEvent(&accel, &gyro, &temp);
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
}

void Navigation::update() {
    unsigned long now = millis();
    float dt = (now - _lastUpdate) / 1000.0f;
    if (dt < 0.001f) return; 
    _lastUpdate = now;

    // Read raw sensor data
    sensors_event_t accelEvent, gyroEvent, tempEvent, magEvent;
    _lsm.getEvent(&accelEvent, &gyroEvent, &tempEvent);
    _lis.getEvent(&magEvent);

    // Map sensor axes to robot frame (X=Forward, Y=Left, Z=Up)
    _rawData.accelX = accelEvent.acceleration.y;
    _rawData.accelY = -accelEvent.acceleration.x;
    _rawData.accelZ = accelEvent.acceleration.z;
    
    _rawData.gyroX = gyroEvent.gyro.y;
    _rawData.gyroY = -gyroEvent.gyro.x;
    _rawData.gyroZ = -gyroEvent.gyro.z;

    // Magnetometer (negated for correct North alignment)
    _rawData.magX = -magEvent.magnetic.y;
    _rawData.magY = magEvent.magnetic.x;
    _rawData.magZ = magEvent.magnetic.z;

    // Apply hard-iron offsets and soft-iron scale
    float correctedMagX = (_rawData.magX - _magOffsetX) * _magScaleX;
    float correctedMagY = (_rawData.magY - _magOffsetY) * _magScaleY;
    float correctedMagZ = (_rawData.magZ - _magOffsetZ) * _magScaleZ;

    // Collect samples during calibration using the calibrator
    if (_isCalibrating) {
        _calibrator.addSample(_rawData.magX, _rawData.magY, _rawData.magZ);
        _calibrationSamples = (int)_calibrator.sampleCount();
    }

    // Calculate roll and pitch for tilt compensation
    _orientation.roll = atan2(_rawData.accelY, _rawData.accelZ);
    _orientation.pitch = atan2(-_rawData.accelX, sqrt(_rawData.accelY * _rawData.accelY + _rawData.accelZ * _rawData.accelZ));

    // Tilt compensation: project mag vector to horizontal plane
    float Xh = correctedMagX * cos(_orientation.pitch) + correctedMagZ * sin(_orientation.pitch);
    float Yh = correctedMagX * sin(_orientation.roll) * sin(_orientation.pitch) + correctedMagY * cos(_orientation.roll) - correctedMagZ * sin(_orientation.roll) * cos(_orientation.pitch);

    // Calculate absolute magnetic heading
    float magHeading = atan2(-Yh, Xh);

    // Complementary filter: Gyro (fast) + Magnetometer (stable)
    float gyroZRate = _rawData.gyroZ - _gyroBiasZ;
    float headingError = magHeading - _headingIntegral;

    // Normalize error to [-PI, PI]
    while (headingError > M_PI) headingError -= 2.0f * M_PI;
    while (headingError < -M_PI) headingError += 2.0f * M_PI;

    // Snap to mag on first update (avoid slow convergence)
    if (_lastUpdate == 0) {
        _headingIntegral = magHeading;
    } else {
        _headingIntegral += (gyroZRate + 0.05f * headingError) * dt;
    }
    
    // Wrap heading to [-PI, PI]
    while (_headingIntegral > M_PI) _headingIntegral -= 2.0f * M_PI;
    while (_headingIntegral < -M_PI) _headingIntegral += 2.0f * M_PI;

    // Update nav data
    _data.gyroZ = gyroZRate;
    _data.heading = _headingIntegral * (180.0f / M_PI);
    if (_data.heading < 0) _data.heading += 360.0f;
}

void Navigation::startMagnetometerCalibration() {
    _calibrator.clear();
    _isCalibrating = true;
    _calibrationSamples = 0;
    _data.isCalibrated = false;
    Serial.println("[Nav] Starting magnetometer calibration (rotate 360° slowly)");
}

void Navigation::stopMagnetometerCalibration() {
    _isCalibrating = false;
    // Compute calibrator results
    _calibrator.compute();
    float ox, oy, oz, sx, sy, sz;
    _calibrator.getOffsets(ox, oy, oz);
    _calibrator.getScales(sx, sy, sz);
    _magOffsetX = ox;
    _magOffsetY = oy;
    _magOffsetZ = oz;
    _magScaleX = sx;
    _magScaleY = sy;
    _magScaleZ = sz;
    _data.isCalibrated = true;

    // Reset heading filter to snap to new offsets
    _lastUpdate = 0;
    _headingIntegral = 0;
    Serial.printf("[Nav] Calibration complete. Offsets X:%.2f, Y:%.2f, Z:%.2f (%d samples)\n", 
                  _magOffsetX, _magOffsetY, _magOffsetZ, _calibrationSamples);
    Serial.printf("[Nav] Scale X:%.3f, Y:%.3f, Z:%.3f\n", _magScaleX, _magScaleY, _magScaleZ);
}

bool Navigation::isMagnetometerCalibrated() const {
    return _data.isCalibrated;
}

int Navigation::getCalibrationProgress() const {
    if (!_isCalibrating) return 0;
    return constrain((int)(_calibrationSamples * 100 / CALIBRATION_SAMPLE_LIMIT), 0, 100);
}

float Navigation::getCorrectedHeadingDeg() const {
    float h = _data.heading - Config::CompassOffsetDeg;
    while (h < 0) h += 360.0f;
    while (h >= 360.0f) h -= 360.0f;
    return h;
}
