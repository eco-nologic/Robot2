#ifndef MAGNETOMETER_FILTER_H
#define MAGNETOMETER_FILTER_H

#include <Arduino.h>
#include <cmath>
#include <cstring>

#ifndef M_PI
#define M_PI 3.14159265358979323846f
#endif

/**
 * @struct MagData
 * @brief 3-axis magnetometer data container
 */
struct MagData {
    float x;
    float y;
    float z;

    MagData() : x(0.0f), y(0.0f), z(0.0f) {}
    MagData(float x_, float y_, float z_) : x(x_), y(y_), z(z_) {}
};

/**
 * @enum FilterType
 * @brief Available magnetometer filter algorithms
 */
enum class FilterType {
    SMA,   // Simple Moving Average
    LDMA,  // Linear Decay Moving Average (Bartlett)
    RCMA,  // Raised Cosine Moving Average (Hanning)
    EMA    // Exponential Moving Average (IIR)
};

/**
 * @class MagnetometerFilter
 * @brief Single-channel streaming signal processor for magnetometer denoising
 *
 * Implements FIR and IIR filter variants with minimal latency overhead,
 * designed for embedded real-time navigation systems.
 *
 * Reference: MDPI Electronics 2024, 13(11), 2006
 */
class MagnetometerFilter {
private:
    static constexpr int MAX_WINDOW = 31;  // Maximum window size for fixed allocation

    FilterType _filterType;
    int _windowSize;
    float _beta;

    float _buffer[MAX_WINDOW];     // Circular ring buffer
    float _weights[MAX_WINDOW];    // Precomputed filter coefficients
    int _bufferIndex;
    bool _isInitialized;
    float _prevEmaOutput;

    /**
     * @brief Compute normalized weight distribution based on filter type
     */
    void computeWeights();

public:
    /**
     * @brief Constructor for FIR filters (SMA, LDMA, RCMA)
     * @param filterType Filter algorithm to use
     * @param windowSize Number of samples in the window (max 31)
     */
    MagnetometerFilter(FilterType filterType, int windowSize);

    /**
     * @brief Constructor for IIR filter (EMA)
     * @param filterType Must be FilterType::EMA
     * @param smoothingCoeff Exponential smoothing coefficient (0.0 < beta < 1.0)
     */
    MagnetometerFilter(FilterType filterType, float smoothingCoeff);

    /**
     * @brief Process a single sample through the filter
     * @param input Raw sensor reading
     * @return Filtered output value
     */
    float update(float input);

    /**
     * @brief Reset filter state (useful after startup or recalibration)
     */
    void reset();

    /**
     * @brief Factory method: create filter with target group delay constraint
     * @param filterType Desired algorithm
     * @param targetGroupDelay Maximum acceptable latency in samples
     * @return Configured MagnetometerFilter instance
     */
    static MagnetometerFilter createWithGroupDelay(FilterType filterType, int targetGroupDelay);

    /**
     * @brief Get current window size (for FIR filters)
     */
    int getWindowSize() const { return _windowSize; }

    /**
     * @brief Get smoothing coefficient (for EMA only)
     */
    float getSmoothingCoeff() const { return _beta; }

    /**
     * @brief Get filter type
     */
    FilterType getFilterType() const { return _filterType; }
};

/**
 * @class MagnetometerFilter3D
 * @brief 3-axis vector filter wrapping three independent single-channel filters
 *
 * Applies identical filtering to X, Y, Z components in parallel,
 * maintaining symmetry across all orthogonal axes.
 */
class MagnetometerFilter3D {
private:
    MagnetometerFilter _filterX;
    MagnetometerFilter _filterY;
    MagnetometerFilter _filterZ;

public:
    /**
     * @brief Constructor for FIR filter types
     * @param filterType Filter algorithm
     * @param windowSize Window length
     */
    MagnetometerFilter3D(FilterType filterType, int windowSize);

    /**
     * @brief Constructor for IIR filter type
     * @param filterType FilterType::EMA
     * @param smoothingCoeff Exponential decay coefficient
     */
    MagnetometerFilter3D(FilterType filterType, float smoothingCoeff);

    /**
     * @brief Process 3D vector through the filter pipeline
     * @param raw Unfiltered magnetometer reading
     * @return Filtered 3-axis data
     */
    MagData update(const MagData& raw);

    /**
     * @brief Reset all three channel filters
     */
    void reset();

    /**
     * @brief Get configuration details
     */
    FilterType getFilterType() const { return _filterX.getFilterType(); }
    int getWindowSize() const { return _filterX.getWindowSize(); }
};

#endif // MAGNETOMETER_FILTER_H
