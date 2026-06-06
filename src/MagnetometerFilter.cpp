#include "MagnetometerFilter.h"

// ============================================================================
// MagnetometerFilter Implementation
// ============================================================================

MagnetometerFilter::MagnetometerFilter(FilterType filterType, int windowSize)
    : _filterType(filterType), _windowSize(windowSize), _beta(0.0f),
      _bufferIndex(0), _isInitialized(false), _prevEmaOutput(0.0f) {
    if (_windowSize < 1) _windowSize = 1;
    if (_windowSize > MAX_WINDOW) _windowSize = MAX_WINDOW;

    memset(_buffer, 0, sizeof(_buffer));
    memset(_weights, 0, sizeof(_weights));
    computeWeights();
}

MagnetometerFilter::MagnetometerFilter(FilterType filterType, float smoothingCoeff)
    : _filterType(filterType), _windowSize(0), _beta(smoothingCoeff),
      _bufferIndex(0), _isInitialized(false), _prevEmaOutput(0.0f) {
    if (_beta < 0.0f) _beta = 0.0f;
    if (_beta > 1.0f) _beta = 1.0f;

    memset(_buffer, 0, sizeof(_buffer));
    memset(_weights, 0, sizeof(_weights));
    computeWeights();
}

void MagnetometerFilter::computeWeights() {
    if (_filterType == FilterType::EMA) {
        return; // EMA doesn't use weight array
    }

    int N = _windowSize;

    if (_filterType == FilterType::SMA) {
        // Simple Moving Average: h(n) = 1 / N
        float invN = 1.0f / N;
        for (int n = 0; n < N; ++n) {
            _weights[n] = invN;
        }
    }
    else if (_filterType == FilterType::LDMA) {
        // Linear Decay Moving Average: h(n) = [2 / (N * (N + 1))] * (N - n)
        float denom = static_cast<float>(N * (N + 1));
        for (int n = 0; n < N; ++n) {
            _weights[n] = (2.0f * (N - n)) / denom;
        }
    }
    else if (_filterType == FilterType::RCMA) {
        // Raised Cosine Moving Average: h(n) = [1 / (N + 1)] * (1 + cos(n * pi / N))
        float denom = static_cast<float>(N + 1);
        for (int n = 0; n < N; ++n) {
            float angle = (n * M_PI) / N;
            _weights[n] = (1.0f + cosf(angle)) / denom;
        }
    }
}

float MagnetometerFilter::update(float input) {
    // Cold-start transient mitigation: fill buffer with first value
    if (!_isInitialized) {
        if (_filterType == FilterType::EMA) {
            _prevEmaOutput = input;
        } else {
            for (int i = 0; i < _windowSize; ++i) {
                _buffer[i] = input;
            }
        }
        _isInitialized = true;
    }

    if (_filterType == FilterType::EMA) {
        // IIR EMA: y[t] = (1 - beta) * x[t] + beta * y[t-1]
        float output = (1.0f - _beta) * input + _beta * _prevEmaOutput;
        _prevEmaOutput = output;
        return output;
    } else {
        // FIR filters: multiply-accumulate over circular buffer
        _buffer[_bufferIndex] = input;
        float output = 0.0f;

        for (int n = 0; n < _windowSize; ++n) {
            int idx = _bufferIndex - n;
            if (idx < 0) {
                idx += _windowSize;
            }
            output += _weights[n] * _buffer[idx];
        }

        _bufferIndex = (_bufferIndex + 1) % _windowSize;
        return output;
    }
}

void MagnetometerFilter::reset() {
    memset(_buffer, 0, sizeof(_buffer));
    _bufferIndex = 0;
    _prevEmaOutput = 0.0f;
    _isInitialized = false;
}

MagnetometerFilter MagnetometerFilter::createWithGroupDelay(FilterType filterType, int targetGroupDelay) {
    if (filterType == FilterType::EMA) {
        // EMA group delay: GD = beta / (1 - beta)
        // Inverse: beta = GD / (1 + GD)
        float beta = static_cast<float>(targetGroupDelay) / (1.0f + targetGroupDelay);
        return MagnetometerFilter(FilterType::EMA, beta);
    }

    int N;
    if (filterType == FilterType::SMA) {
        // SMA group delay: GD = (N - 1) / 2 => N = (2 * GD) + 1
        N = (2 * targetGroupDelay) + 1;
    }
    else if (filterType == FilterType::LDMA) {
        // LDMA group delay: GD = (N - 1) / 3 => N = (3 * GD) + 1
        N = (3 * targetGroupDelay) + 1;
    }
    else { // RCMA
        // RCMA group delay: GD = (8/9) * ((N - 1) / 3) => N ≈ ((27 * GD) / 8) + 1
        N = ((27 * targetGroupDelay) / 8) + 1;
    }

    if (N < 1) N = 1;
    if (N > MAX_WINDOW) N = MAX_WINDOW;

    return MagnetometerFilter(filterType, N);
}

// ============================================================================
// MagnetometerFilter3D Implementation
// ============================================================================

MagnetometerFilter3D::MagnetometerFilter3D(FilterType filterType, int windowSize)
    : _filterX(filterType, windowSize),
      _filterY(filterType, windowSize),
      _filterZ(filterType, windowSize) {}

MagnetometerFilter3D::MagnetometerFilter3D(FilterType filterType, float smoothingCoeff)
    : _filterX(filterType, smoothingCoeff),
      _filterY(filterType, smoothingCoeff),
      _filterZ(filterType, smoothingCoeff) {}

MagData MagnetometerFilter3D::update(const MagData& raw) {
    return MagData(
        _filterX.update(raw.x),
        _filterY.update(raw.y),
        _filterZ.update(raw.z)
    );
}

void MagnetometerFilter3D::reset() {
    _filterX.reset();
    _filterY.reset();
    _filterZ.reset();
}
