#include "../include/MagnetometerCalibrator.h"
#include <algorithm>
#include <limits>

MagnetometerCalibrator::MagnetometerCalibrator()
{
    clear();
}

void MagnetometerCalibrator::clear()
{
    _xs.clear();
    _ys.clear();
    _zs.clear();
    _offX = _offY = _offZ = 0.0f;
    _scaleX = _scaleY = _scaleZ = 1.0f;
}

void MagnetometerCalibrator::addSample(float x, float y, float z)
{
    _xs.push_back(x);
    _ys.push_back(y);
    _zs.push_back(z);
}

size_t MagnetometerCalibrator::sampleCount() const { return _xs.size(); }

void MagnetometerCalibrator::compute()
{
    if (_xs.empty()) return;

    auto minmax_x = std::minmax_element(_xs.begin(), _xs.end());
    auto minmax_y = std::minmax_element(_ys.begin(), _ys.end());
    auto minmax_z = std::minmax_element(_zs.begin(), _zs.end());

    float minx = *minmax_x.first;
    float maxx = *minmax_x.second;
    float miny = *minmax_y.first;
    float maxy = *minmax_y.second;
    float minz = *minmax_z.first;
    float maxz = *minmax_z.second;

    _offX = (maxx + minx) / 2.0f;
    _offY = (maxy + miny) / 2.0f;
    _offZ = (maxz + minz) / 2.0f;

    float half_dx = (maxx - minx) / 2.0f;
    float half_dy = (maxy - miny) / 2.0f;
    float half_dz = (maxz - minz) / 2.0f;

    float avg = (half_dx + half_dy + half_dz) / 3.0f;
    _scaleX = (half_dx > 0.0f) ? (avg / half_dx) : 1.0f;
    _scaleY = (half_dy > 0.0f) ? (avg / half_dy) : 1.0f;
    _scaleZ = (half_dz > 0.0f) ? (avg / half_dz) : 1.0f;
}

void MagnetometerCalibrator::getOffsets(float &ox, float &oy, float &oz) const
{
    ox = _offX; oy = _offY; oz = _offZ;
}

void MagnetometerCalibrator::getScales(float &sx, float &sy, float &sz) const
{
    sx = _scaleX; sy = _scaleY; sz = _scaleZ;
}
