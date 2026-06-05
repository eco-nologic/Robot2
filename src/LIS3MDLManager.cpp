#include "LIS3MDLManager.h"
#include <Adafruit_LIS3MDL.h>
#include <Adafruit_Sensor.h>
#include "Config.h"
#include "soc/rtc_wdt.h"
#include <algorithm>
#include <cmath>

// External references from main/config
extern SemaphoreHandle_t xI2CSemaphore;
static constexpr uint32_t I2C_xBlockTime = 10; 
static constexpr uint32_t I2C_RETRY_DELAY = 5;
static constexpr uint32_t DEFAULT_STACK_SIZE = 4096;

// Global instance
LIS3MDLManager magManager;
static const char* TAG = "LIS3MDL";

LIS3MDLManager::LIS3MDLManager() {
    _mag = new Adafruit_LIS3MDL();
}

void LIS3MDLManager::begin() {
    bool success = false;
    // Initializing I2C and loading previous calibration
    if (xSemaphoreTake(xI2CSemaphore, pdMS_TO_TICKS(I2C_xBlockTime)) == pdTRUE) {
        uint8_t lisAddrs[] = {0x1E, 0x1C};
        for (uint8_t addr : lisAddrs) {
            if (_mag->begin_I2C(addr, &Wire)) {
                Serial.printf("[LIS3MDL] ✅ Found LIS3MDL at 0x%02X\n", addr);
                success = true;
                break;
            }
            delay(10);
        }

        if (success) {
            _mag->setPerformanceMode(LIS3MDL_ULTRAHIGHMODE);
            _mag->setOperationMode(LIS3MDL_CONTINUOUSMODE);
            _mag->setDataRate(LIS3MDL_DATARATE_155_HZ);
            _mag->setRange(LIS3MDL_RANGE_4_GAUSS);
        } else {
            ESP_LOGE(TAG, "Failed to find LIS3MDL chip");
        }
        xSemaphoreGive(xI2CSemaphore);
    }

    if (!success) return; // Do not start the task if hardware init failed

    loadCalibration();

    xTaskCreatePinnedToCore(
        this->taskEntry,
        "taskMag",
        DEFAULT_STACK_SIZE,
        this,
        1,
        NULL,
        1
    );
}

void LIS3MDLManager::taskEntry(void* pvParameters) {
    static_cast<LIS3MDLManager*>(pvParameters)->runLoop();
}

void LIS3MDLManager::runLoop() {
    bool initialized = false;
    sensors_event_t event;
    for (;;) {
        if (xSemaphoreTake(xI2CSemaphore, pdMS_TO_TICKS(20)) == pdTRUE) {
            _mag->getEvent(&event);
            
            // Apply calibration manually
            _x = (event.magnetic.x - _ox) * _sx;
            _y = (event.magnetic.y - _oy) * _sy;
            _z = (event.magnetic.z - _oz) * _sz;
            
            // Calculate raw heading in radians
            float rad = atan2(_y, _x);

            // Initialize or apply Exponential Moving Average (EMA) filter
            // We smooth the Sine and Cosine components to safely handle the 0/360 wrap-around.
            if (!initialized) {
                _filtSin = sin(rad);
                _filtCos = cos(rad);
                initialized = true;
            } else {
                _filtSin = (_alpha * sin(rad)) + ((1.0f - _alpha) * _filtSin);
                _filtCos = (_alpha * cos(rad)) + ((1.0f - _alpha) * _filtCos);
            }

            float smoothedAz = atan2(_filtSin, _filtCos) * 180.0f / PI;
            _heading = (smoothedAz < 0) ? (smoothedAz + 360.0f) : smoothedAz;

            xSemaphoreGive(xI2CSemaphore);
            vTaskDelay(pdMS_TO_TICKS(100)); // Sample rate for navigation
        } else {
            vTaskDelay(pdMS_TO_TICKS(I2C_RETRY_DELAY));
        }
    }
}

void LIS3MDLManager::getCalibratedXYZ(float &x, float &y, float &z) const {
    if (xSemaphoreTake(xI2CSemaphore, pdMS_TO_TICKS(I2C_xBlockTime)) == pdTRUE) {
        x = _x;
        y = _y;
        z = _z;
        xSemaphoreGive(xI2CSemaphore);
    } else {
        x = 0.0f;
        y = 0.0f;
        z = 0.0f;
    }
}

void LIS3MDLManager::calibrate(int test_duration_ms, int sample_delay) {
    ESP_LOGI(TAG, "Calibration Started: Rotate robot on all axes...");
    _calibrated = false; 
    
    size_t expected_samples = test_duration_ms / sample_delay;
    std::vector<float> x_vals, y_vals, z_vals;
    
    // Memory optimization: prevent heap fragmentation
    x_vals.reserve(expected_samples + 1);
    y_vals.reserve(expected_samples + 1);
    z_vals.reserve(expected_samples + 1);

    sensors_event_t event;
    unsigned long start = millis();
    while ((millis() - start) < (unsigned long)test_duration_ms) {
        if (xSemaphoreTake(xI2CSemaphore, pdMS_TO_TICKS(I2C_xBlockTime)) == pdTRUE) {
            _mag->getEvent(&event);
            x_vals.push_back(event.magnetic.x);
            y_vals.push_back(event.magnetic.y);
            z_vals.push_back(event.magnetic.z);
            xSemaphoreGive(xI2CSemaphore);
        }
        rtc_wdt_feed();
        delay(sample_delay);
    }

    float ox, oy, oz, sx, sy, sz;
    
    // Offset calculation (Hard-iron)
    ox = (*std::max_element(x_vals.begin(), x_vals.end()) + *std::min_element(x_vals.begin(), x_vals.end())) / 2.0f;
    oy = (*std::max_element(y_vals.begin(), y_vals.end()) + *std::min_element(y_vals.begin(), y_vals.end())) / 2.0f;
    oz = (*std::max_element(z_vals.begin(), z_vals.end()) + *std::min_element(z_vals.begin(), z_vals.end())) / 2.0f;

    // Scale calculation (Soft-iron)
    float dx = (*std::max_element(x_vals.begin(), x_vals.end()) - *std::min_element(x_vals.begin(), x_vals.end())) / 2.0f;
    float dy = (*std::max_element(y_vals.begin(), y_vals.end()) - *std::min_element(y_vals.begin(), y_vals.end())) / 2.0f;
    float dz = (*std::max_element(z_vals.begin(), z_vals.end()) - *std::min_element(z_vals.begin(), z_vals.end())) / 2.0f;
    float avg = (dx + dy + dz) / 3.0f;
    sx = avg / dx; sy = avg / dy; sz = avg / dz;

    saveCalibration(ox, oy, oz, sx, sy, sz);
    ESP_LOGI(TAG, "Calibration Saved to NVS.");
}

void LIS3MDLManager::loadCalibration() {
    _prefs.begin("mag_cal", true);
    _ox = _prefs.getFloat("ox", 0.0f);
    _oy = _prefs.getFloat("oy", 0.0f);
    _oz = _prefs.getFloat("oz", 0.0f);
    _sx = _prefs.getFloat("sx", 1.0f);
    _sy = _prefs.getFloat("sy", 1.0f);
    _sz = _prefs.getFloat("sz", 1.0f);
    _calibrated = _prefs.getBool("is_cal", false);
    _prefs.end();
    ESP_LOGI(TAG, "Loaded Calibration: Offsets(%.1f,%.1f,%.1f)", _ox, _oy, _oz);
}

void LIS3MDLManager::startCalibration() {
    _isCalibrating = true;
    _calStartTime = millis();
    Serial.println("[LIS3MDL] Live calibration started...");
}

void LIS3MDLManager::stopCalibration() {
    _isCalibrating = false;
    Serial.println("[LIS3MDL] Live calibration stopped.");
}

int LIS3MDLManager::getCalibrationProgress() const {
    if (!_isCalibrating) return 0;
    return constrain((int)((millis() - _calStartTime) * 100 / _calDuration), 0, 100);
}

void LIS3MDLManager::saveCalibration(float ox, float oy, float oz, float sx, float sy, float sz) {
    _prefs.begin("mag_cal", false);
    _prefs.putFloat("ox", ox);
    _prefs.putFloat("oy", oy);
    _prefs.putFloat("oz", oz);
    _prefs.putFloat("sx", sx);
    _prefs.putFloat("sy", sy);
    _prefs.putFloat("sz", sz);
    _prefs.putBool("is_cal", true);
    _prefs.end();
    _ox = ox; _oy = oy; _oz = oz;
    _sx = sx; _sy = sy; _sz = sz;
    _calibrated = true;
}

void LIS3MDLManager::applyOffsetCorrection(std::vector<float>& x, std::vector<float>& y, std::vector<float>& z, 
                                           float& ox, float& oy, float& oz) {
    if (x.empty()) return;
    ox = (*std::max_element(x.begin(), x.end()) + *std::min_element(x.begin(), x.end())) / 2.0f;
    oy = (*std::max_element(y.begin(), y.end()) + *std::min_element(y.begin(), y.end())) / 2.0f;
    oz = (*std::max_element(z.begin(), z.end()) + *std::min_element(z.begin(), z.end())) / 2.0f;
}

void LIS3MDLManager::applyScaleCorrection(std::vector<float>& x, std::vector<float>& y, std::vector<float>& z, 
                                          float& sx, float& sy, float& sz) {
    if (x.empty()) return;
    float dx = (*std::max_element(x.begin(), x.end()) - *std::min_element(x.begin(), x.end())) / 2.0f;
    float dy = (*std::max_element(y.begin(), y.end()) - *std::min_element(y.begin(), y.end())) / 2.0f;
    float dz = (*std::max_element(z.begin(), z.end()) - *std::min_element(z.begin(), z.end())) / 2.0f;
    if (dx == 0) dx = 1; if (dy == 0) dy = 1; if (dz == 0) dz = 1;
    float avg = (dx + dy + dz) / 3.0f;
    sx = avg / dx;
    sy = avg / dy;
    sz = avg / dz;
}