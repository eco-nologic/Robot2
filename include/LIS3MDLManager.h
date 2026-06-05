#ifndef LIS3MDL_MANAGER_H
#define LIS3MDL_MANAGER_H

#include <Arduino.h>
#include <Preferences.h>
#include <vector>

class Adafruit_LIS3MDL; // Forward declaration for LIS3MDL

/**
 * @brief Manager class for the LIS3MDL Magnetometer.
 * Handles I2C concurrency, calibration, and persistent storage of offsets/scales.
 */
class LIS3MDLManager {
public:
    LIS3MDLManager();
    
    // Initialization and Task Management
    void begin();
    static void taskEntry(void* pvParameters);

    // Calibration Logic
    void calibrate(int test_duration_ms = 30000, int sample_delay = 100);
    void loadCalibration();
    void saveCalibration(float ox, float oy, float oz, float sx, float sy, float sz);

    /**
     * @brief Sets the smoothing factor (0.01 to 1.0). Lower is smoother, higher is more responsive.
     */
    void setSmoothingAlpha(float alpha) { _alpha = constrain(alpha, 0.01f, 1.0f); }

    /**
     * @brief Check if the device has been calibrated (non-default scales).
     */
    bool isCalibrated() const { return _calibrated; }

    // Data Retrieval
    /**
     * @brief Returns the latest calibrated and smoothed heading in degrees (0-359.9).
     */
    float getHeading() const { return _heading; }

    /**
     * @brief Returns the latest calibrated X, Y, Z raw values.
     */
    void getCalibratedXYZ(float &x, float &y, float &z) const;

private:
    Adafruit_LIS3MDL* _mag; // Pointer to the Adafruit LIS3MDL instance
    Preferences _prefs;
    
    void runLoop();
    void applyOffsetCorrection(std::vector<float>& x, std::vector<float>& y, std::vector<float>& z, 
                               float& ox, float& oy, float& oz);
    void applyScaleCorrection(std::vector<float>& x, std::vector<float>& y, std::vector<float>& z, 
                              float& sx, float& sy, float& sz);
    void resetCalibration();

    // Internal storage for latest samples
    float _x = 0;
    float _y = 0;
    float _z = 0;
    float _heading = 0.0f;
    bool _calibrated = false;

    // Calibration parameters
    float _ox = 0, _oy = 0, _oz = 0;
    float _sx = 1.0, _sy = 1.0, _sz = 1.0;

    // Smoothing Filter variables
    float _alpha = 0.2f;    // EMA factor (0.2 is a good balance between lag and stability)
    float _filtSin = 0.0f;
    float _filtCos = 0.0f;
};

extern LIS3MDLManager magManager;

#endif // LIS3MDL_MANAGER_H