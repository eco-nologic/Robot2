#ifndef NAVIGATION_H
#define NAVIGATION_H

#include <Arduino.h>
#include <Wire.h>
#include <Adafruit_LSM6DS3.h>
#include "Config.h"
#include "MagnetometerCalibrator.h"
#include "MagnetometerFilter.h"

// Raw IMU data structure
struct ImuData {
    float accelX, accelY, accelZ;
    float gyroX, gyroY, gyroZ;
    float magX, magY, magZ;
};

// Fused orientation data
struct OrientationData {
    float roll, pitch, heading;
};

/**
 * @class Navigation
 * @brief IMU sensor fusion with magnetometer calibration
 * 
 * Uses LSM6DS3 (accel/gyro) plus the shared LIS3MDL manager instance
 * for magnetometer data, with hard-iron calibration and complementary filtering
 */
class Navigation {
public:
    struct NavData {
        float heading;       // Heading in degrees (0-360)
        float rawMagHeading; // Raw magnetometer heading (tilt-compensated) in degrees (0-360)
        float gyroZ;         // Angular velocity (deg/s)
        bool isCalibrated;   // Magnetometer calibration state
    };

    Navigation();

    // Initialize I2C bus and sensors
    bool begin();

    // Update sensor fusion (call from main loop ~100Hz)
    void update();

    // Calibrate gyro bias (robot must be still)
    void calibrateGyro();

    // Start magnetometer calibration (requires 360° rotation)
    void startMagnetometerCalibration();
    void stopMagnetometerCalibration();
    bool isMagnetometerCalibrated() const;

    NavData getNavData() const { return _data; }
    ImuData getRawData() const { return _rawData; }
    float getHeading() const { return _data.heading; }
    float getRawMagHeading() const { return _data.rawMagHeading; }
    float getCorrectedHeadingDeg() const;

    // Calibration progress (0-100%)
    int getCalibrationProgress() const;
    bool isCalibrating() const;

private:
    NavData _data;
    Adafruit_LSM6DS3 _lsm;
    ImuData _rawData;
    OrientationData _orientation;

    // Hard-iron calibration (min/max collection)
    float _magMinX, _magMaxX, _magMinY, _magMaxY;
    float _magOffsetX, _magOffsetY, _magOffsetZ;
    // Soft-iron scale factors (computed by calibrator)
    float _magScaleX = 1.0f, _magScaleY = 1.0f, _magScaleZ = 1.0f;
    MagnetometerCalibrator _calibrator;

    // Magnetometer signal processing filter (RCMA: low latency, superior noise rejection)
    MagnetometerFilter3D _magFilter{FilterType::RCMA, 15};

    // Gyro bias from startup calibration
    float _gyroBiasZ = 0;

    // Heading integration for complementary filter
    float _headingIntegral = 0;
    unsigned long _lastUpdate = 0;

    // Calibration state
    bool _isCalibrating = false;
    int _calibrationSamples = 0;
    const int CALIBRATION_SAMPLE_LIMIT = 10000;  // Collect up to 10k samples during 360° rotation

    void initializeMagnetometerCalibration();
    void updateMagnetometerCalibration();
};

#endif // NAVIGATION_H
