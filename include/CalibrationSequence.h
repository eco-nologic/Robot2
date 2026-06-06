#pragma once

#include <Arduino.h>
#include <stdint.h>

// Minimal odometry-only calibration sequence.
class CalibrationSequence {
public:
    CalibrationSequence();
    void begin();
    void stop();
    void update(unsigned long now_ms);

private:
    enum State { Idle, Move, Completed, Aborted } state_;
    int current_move_;
    unsigned long move_start_ms_;
    float V_; // base speed (m/s) or normalized unit depending on motor API
    void startMove(int idx, unsigned long now_ms);
    void endMove();
    void logSample(unsigned long now_ms);
};
