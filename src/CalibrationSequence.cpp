#include "CalibrationSequence.h"
#include "DriveTrain.h"
#include "DCMotor.h"
#include "Config.h"
#include "LIS3MDLManager.h"
#include "Navigation.h"
#include <Arduino.h>
#include <stdio.h>

// Extern globals defined in main.cpp
extern DriveTrain driveTrain;
extern DCMotor motorLeft;
extern DCMotor motorRight;
extern LIS3MDLManager magManager;
extern Navigation navigation;

// Durations are in milliseconds
static const unsigned long MOVE_DURATIONS_MS[] = {3000,3000,3000,3000,3000,3000,2000,600};
static const int MOVE_COUNT = 8;

CalibrationSequence::CalibrationSequence()
    : state_(Idle), current_move_(0), move_start_ms_(0), V_(0.2f) {}

void CalibrationSequence::begin() {
    state_ = Move;
    current_move_ = 0;
    move_start_ms_ = millis();
    // start magnetometer calibration via Navigation/magManager
    navigation.startMagnetometerCalibration();
    startMove(current_move_, move_start_ms_);
}

void CalibrationSequence::stop() {
    state_ = Aborted;
    driveTrain.stop();
    navigation.stopMagnetometerCalibration();
}

void CalibrationSequence::update(unsigned long now_ms) {
    if (state_ != Move) return;
    // log sample
    logSample(now_ms);
    unsigned long elapsed = now_ms - move_start_ms_;
    if (elapsed >= MOVE_DURATIONS_MS[current_move_]) {
        // end current move
        endMove();
        current_move_++;
        if (current_move_ >= MOVE_COUNT) {
            // finished
            driveTrain.stop();
            navigation.stopMagnetometerCalibration();
            state_ = Completed;
            return;
        }
        startMove(current_move_, now_ms);
    }
}

void CalibrationSequence::startMove(int idx, unsigned long now_ms) {
    move_start_ms_ = now_ms;
    // compute left/right speeds (m/s) according to move index, then convert to mm/s
    float left_m=0, right_m=0;
    switch(idx) {
        case 0: case 2: case 4:
            left_m = V_;
            right_m = 0.4f*V_;
            break;
        case 1: case 3: case 5:
            left_m = V_;
            right_m = 1.6f*V_;
            break;
        case 6:
            left_m = 0.9f*V_;
            right_m = 0.9f*V_;
            break;
        case 7:
            left_m = V_;
            right_m = -V_;
            break;
    }
    // convert to mm/s
    float left_mm = left_m * 1000.0f;
    float right_mm = right_m * 1000.0f;
    float linear_mm = (left_mm + right_mm) / 2.0f;
    float angular_rad = 0.0f;
    if (fabs(right_mm - left_mm) > 0.0f) {
        angular_rad = (right_mm - left_mm) / Config::WheelBaseMm; // rad/s
    }
    driveTrain.setVelocity(linear_mm, angular_rad);
}

void CalibrationSequence::endMove() {
    // small no-op; motors continue until next startMove changes speeds
}

void CalibrationSequence::logSample(unsigned long now_ms) {
    float mx=0, my=0, mz=0;
    magManager.getCalibratedXYZ(mx, my, mz);
    long left_enc = motorLeft.getEncoderCount();
    long right_enc = motorRight.getEncoderCount();
    Serial.printf("%lu,%.3f,%.3f,%.3f,%ld,%ld\n", now_ms, mx, my, mz, left_enc, right_enc);
}
