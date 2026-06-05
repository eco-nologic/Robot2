#ifndef HEADING_PID_CONTROLLER_H
#define HEADING_PID_CONTROLLER_H

#include <Arduino.h>

class HeadingPIDController {
public:
    HeadingPIDController(float kp, float ki, float kd) 
        : _kp(kp), _ki(ki), _kd(kd), _integral(0), _lastError(0) {}

    /**
     * @brief Computes steering correction
     * @param target Target heading in degrees (0-359)
     * @param current Current heading (0-359)
     * @param dt Time delta in seconds
     * @return Correction value to add/subtract from motor speeds
     */
    float compute(float target, float current, float dt) {
        // Calculate shortest-path error (-180 to 180)
        float error = target - current;
        if (error > 180.0f) error -= 360.0f;
        else if (error < -180.0f) error += 360.0f;

        // Proportional
        float pTerm = _kp * error;

        // Integral with anti-windup
        _integral += error * dt;
        _integral = constrain(_integral, -50.0f, 50.0f); 
        float iTerm = _ki * _integral;

        // Derivative
        float dTerm = _kd * ((error - _lastError) / dt);
        _lastError = error;

        return pTerm + iTerm + dTerm;
    }

    void setGains(float kp, float ki, float kd) {
        _kp = kp;
        _ki = ki;
        _kd = kd;
    }

    // Getters for persistence
    float getKp() const { return _kp; }
    float getKi() const { return _ki; }
    float getKd() const { return _kd; }

    void reset() {
        _integral = 0;
        _lastError = 0;
    }

private:
    float _kp, _ki, _kd;
    float _integral;
    float _lastError;
};

#endif // HEADING_PID_CONTROLLER_H