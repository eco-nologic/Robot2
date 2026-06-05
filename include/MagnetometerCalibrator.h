#ifndef MAGNETOMETER_CALIBRATOR_H
#define MAGNETOMETER_CALIBRATOR_H

#include <vector>

class MagnetometerCalibrator {
public:
    MagnetometerCalibrator();
    void clear();
    void addSample(float x, float y, float z);
    void compute();
    size_t sampleCount() const;
    void getOffsets(float &ox, float &oy, float &oz) const;
    void getScales(float &sx, float &sy, float &sz) const;

private:
    std::vector<float> _xs, _ys, _zs;
    float _offX, _offY, _offZ;
    float _scaleX, _scaleY, _scaleZ;
};

#endif // MAGNETOMETER_CALIBRATOR_H
