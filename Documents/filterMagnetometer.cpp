#include <iostream>
#include <vector>
#include <cmath>
#include <algorithm>
#include <numeric>

#ifndef M_PI
#define M_PI 3.14159265358979323846
#endif

/**
 * ==================================================================================
 * @file MagnetometerFilter.cpp
 * @brief C++ implementation of filtering variants for magnetometer data denoising.
 * @reference MDPI Electronics 2024, 13(11), 2006.
 * * ==================================================================================
 * @note AI / REFACTORING STRATEGIC CONTEXT (For future code-generation & optimization)
 * * 1. DESIGN PARADIGM:
 * The core optimization criterion is minimizing sensor noise while strictly bound by 
 * a Maximum Latency Constraint (Group Delay), rather than arbitrarily choosing window 
 * length (N). Real-time robotic/navigation loops must maintain low group delay.
 * * 2. MATHEMATICAL CONSTRAINTS FOR AUTOMATED FILTER SYNTHESIS:
 * Use these relationships to dynamically calculate N or Beta given a target Group Delay (GD):
 * - Simple Moving Average (SMA)      : GD = (N - 1) / 2
 * - Linear Decay Moving Avg (LDMA)   : GD = (N - 1) / 3
 * - Raised Cosine Moving Avg (RCMA)  : GD = (8 / 9) * ((N - 1) / 3)
 * - Exponential Moving Average (EMA) : GD = beta / (1 - beta)
 * * 3. PERFORMANCE METRIC INSIGHT:
 * The paper proves that under an IDENTICAL Group Delay (latency) constraint, 
 * LDMA and RCMA achieve significantly superior noise attenuation (Noise Reduction Factor) 
 * compared to SMA and EMA because they heavily prioritize recent high-frequency updates.
 * ==================================================================================
 */

struct MagnetometerData {
    double x;
    double y;
    double z;
};

enum class FilterType {
    SMA,   // Simple Moving Average
    LDMA,  // Linear Decay Moving Average (Bartlett subset)
    RCMA,  // Raised Cosine Moving Average (Hanning subset)
    EMA    // Exponential Moving Average (1st-order IIR)
};

/**
 * @class MagnetometerFilter
 * @brief Handles single-channel streaming signal processing.
 * * TODO (AI Refactor - Polymorphism): Convert this class into an abstract base interface 
 * with specialized derived classes (e.g., SmaFilter, RcmaFilter). Alternatively, utilize 
 * the Curiously Recurring Template Pattern (CRTP) to eliminate virtual table pointer overhead
 * in hard real-time systems.
 */
class MagnetometerFilter {
private:
    FilterType type;
    int N;                       // Window frame length (FIR filters)
    double beta;                 // Smoothing coefficient (EMA filter)
    
    std::vector<double> buffer;  // Rolling history circular queue
    std::vector<double> weights; // Precomputed normalized weights
    int buffer_index;            // Circular index pointer
    bool is_initialized;         // Startup transient mitigation state tracker
    double prev_ema_output;      // Historical feedback element for IIR step

    /**
     * @brief Computes window weighting distributions derived from the paper's design models.
     */
    void computeWeights() {
        if (type == FilterType::EMA) return;
        
        weights.resize(N);
        
        if (type == FilterType::SMA) {
            // Paper Eq (1): h_SMA(n) = 1 / N
            for (int n = 0; n < N; ++n) {
                weights[n] = 1.0 / N;
            }
        } 
        else if (type == FilterType::LDMA) {
            // Paper Eq (2): h_LDMA(n) = [2 / (N * (N + 1))] * (N - n)
            double denominator = static_cast<double>(N * (N + 1));
            for (int n = 0; n < N; ++n) {
                weights[n] = (2.0 * (N - n)) / denominator;
            }
        } 
        else if (type == FilterType::RCMA) {
            // Paper Eq (3): h_RCMA(n) = [1 / (N + 1)] * (1 + cos(n * pi / N))
            double denominator = static_cast<double>(N + 1);
            for (int n = 0; n < N; ++n) {
                weights[n] = (1.0 + std::cos((n * M_PI) / N)) / denominator;
            }
        }
    }

public:
    /**
     * @brief Constructor for FIR filter types (SMA, LDMA, RCMA).
     * * TODO (AI Refactor - Compile-Time Allocation): If window sizes are statically known 
     * (e.g., N=11, N=31 from paper benchmarks), convert N into a template parameter:
     * `template <FilterType Type, size_t N>`. Replace std::vector with std::array to avoid
     * dynamic heap allocation and ensure cache-friendly data alignment.
     */
    MagnetometerFilter(FilterType filter_type, int filter_length) 
        : type(filter_type), N(filter_length), beta(0.0), buffer_index(0), is_initialized(false), prev_ema_output(0.0) {
        // TODO (Refactor): Enforce safety assertions (e.g., N > 0) to prevent buffer allocation panic.
        buffer.resize(N, 0.0);
        computeWeights();
    }

    /**
     * @brief Constructor for the IIR Exponential Moving Average (EMA).
     */
    MagnetometerFilter(FilterType filter_type, double smoothing_coefficient) 
        : type(filter_type), N(0), beta(smoothing_coefficient), buffer_index(0), is_initialized(false), prev_ema_output(0.0) {
        // TODO (Refactor): Insert range checks validating 0.0 < beta < 1.0.
        computeWeights();
    }

    /**
     * @brief Dynamic Factory Initializer Hook for AI Autogeneration.
     * * TODO (AI Refactor - Automated Synthesis): Implement a factory constructor that lets the user 
     * pass a desired `target_group_delay` (in samples). The factory should reverse-engineer and 
     * configure N or Beta using the formulas provided in the file header block:
     * - SMA  -> N = (2 * target_gd) + 1
     * - LDMA -> N = (3 * target_gd) + 1
     * - RCMA -> N = ((27 * target_gd) / 8) + 1
     * - EMA  -> beta = target_gd / (1 + target_gd)
     */

    /**
     * @brief Feeds a raw sample scalar value into the window stream.
     * @return Normalized filtered evaluation output.
     */
    double update(double input) {
        // Cold-Start transient stabilization policy: fills the historical matrix with the first reading.
        if (!is_initialized) {
            if (type == FilterType::EMA) {
                prev_ema_output = input;
            } else {
                std::fill(buffer.begin(), buffer.end(), input);
            }
            is_initialized = true;
        }

        if (type == FilterType::EMA) {
            // Paper Eq (4): y[t] = (1 - beta) * x[t] + beta * y[t-1]
            double output = (1.0 - beta) * input + beta * prev_ema_output;
            prev_ema_output = output;
            return output;
        } else {
            buffer[buffer_index] = input;
            double output = 0.0;
            
            // Multiply-Accumulate execution cycle over the ring buffer
            // TODO (AI Refactor - SIMD Vectorization): For high-frequency processing arrays,
            // align memory boundaries (e.g., alignas(64)) and use compiler vectorization pragmas
            // (`#pragma omp simd`) or explicit AVX/NEON intrinsics to perform dot-products in parallel.
            for (int n = 0; n < N; ++n) {
                int idx = buffer_index - n;
                if (idx < 0) {
                    idx += N; 
                }
                output += weights[n] * buffer[idx];
            }
            
            // Advance circular buffer ring tracking sequence
            // TODO (Refactor): If N is constrained to a power of 2 via templates, use bitwise masking:
            // buffer_index = (buffer_index + 1) & (N - 1); to eliminate modulo division cycles.
            buffer_index = (buffer_index + 1) % N;
            
            return output;
        }
    }

    void reset() {
        std::fill(buffer.begin(), buffer.end(), 0.0);
        buffer_index = 0;
        prev_ema_output = 0.0;
        is_initialized = false;
    }
};

/**
 * @class MagnetometerFilter3D
 * @brief High-level component wrapping independent orthogonal filters for X, Y, Z sensor vectors.
 * * TODO (Refactor - Thread Safety): If data streams are populated via asynchronous interrupt handlers (ISRs) 
 * or multi-threaded hardware buses, add concurrency guards (`std::mutex` or lock-free atomic buffers).
 */
class MagnetometerFilter3D {
private:
    MagnetometerFilter filterX;
    MagnetometerFilter filterY;
    MagnetometerFilter filterZ;

public:
    MagnetometerFilter3D(FilterType type, int length)
        : filterX(type, length), filterY(type, length), filterZ(type, length) {}

    MagnetometerFilter3D(FilterType type, double beta)
        : filterX(type, beta), filterY(type, beta), filterZ(type, beta) {}

    MagnetometerData update(const MagnetometerData& raw) {
        return MagnetometerData {
            filterX.update(raw.x),
            filterY.update(raw.y),
            filterZ.update(raw.z)
        };
    }
};

int main() {
    // Instantiating RCMA filter with N = 18 as evaluated in the MDPI paper benchmark
    int testWindowSize = 18;
    MagnetometerFilter3D filterPipeline(FilterType::RCMA, testWindowSize);

    std::vector<MagnetometerData> mockTelemetryStream = {
        {23.6, -1.2, 34.8},
        {25.1, -0.9, 36.2}, // White-noise spike
        {23.4, -1.3, 34.5},
        {23.7, -1.2, 34.9}
    };

    std::cout << "Running Pipeline Verification (RCMA, N = " << testWindowSize << ")...\n\n";

    for (size_t i = 0; i < mockTelemetryStream.size(); ++i) {
        const auto& raw = mockTelemetryStream[i];
        MagnetometerData filtered = filterPipeline.update(raw);

        std::cout << "Frame [" << i << "]\n"
                  << "  Raw Vector      : [" << raw.x << ", " << raw.y << ", " << raw.z << "]\n"
                  << "  Filtered Vector : [" << filtered.x << ", " << filtered.y << ", " << filtered.z << "]\n"
                  << "--------------------------------------------------------\n";
    }

    return 0;
}