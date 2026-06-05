#ifndef DCMOTOR_H
#define DCMOTOR_H

#include "IMotor.h"
#include "Config.h"
#include <driver/pcnt.h>

/**
 * @class DCMotor
 * @brief Real DC motor driver with encoder feedback
 * 
 * Implements IMotor interface for actual ESP32 hardware motor control.
 * Uses:
 * - GPIO PWM for speed control
 * - GPIO direction pins (IN1/IN2) for direction
 * - PCNT (Pulse Counter) hardware for high-speed encoder reading
 */
class DCMotor : public IMotor {
public:
    /**
     * Constructor
     * @param motorId 0 = left motor, 1 = right motor
     */
    explicit DCMotor(int motorId);

    bool begin() override;
    void setPwm(int pwm) override;
    void setDirection(bool forward) override;
    long getEncoderCount() const override;
    void resetEncoder() override;
    float getSpeed() const override;
    void setSpeedMmPerSec(float speedMmPerSec) override;
    void stop() override;
    bool isReady() const override { return _isInitialized; }

    /**
     * Update speed estimation (call from main loop periodically)
     * @param deltaTimeMs Time since last call (milliseconds)
     */
    void updateSpeed(unsigned long deltaTimeMs);

private:
    int _motorId;                    // 0=left, 1=right
    int _pinEn;                      // PWM enable pin
    int _pinIn1, _pinIn2;            // Direction control pins
    int _pwmChannel;                 // ESP32 PWM channel
    int _pinEncoderA, _pinEncoderB;  // Encoder phase pins
    pcnt_unit_t _pcntUnit;           // PCNT hardware unit
    
    bool _isInitialized = false;
    bool _isForward = true;
    bool _inverted = false;            // true if wiring inverts forward/backward
    int _currentPwm = 0;

    // Speed estimation
    long _lastEncoderCount = 0;
    unsigned long _lastSpeedUpdateMs = 0;
    float _estimatedSpeed = 0.0f;    // ticks/s

    /**
     * Initialize PCNT (Pulse Counter) hardware for encoder reading
     */
    bool initializePcnt();

    /**
     * Select motor parameters based on motorId
     */
    void selectMotorPins();

    /**
     * Convert speed (mm/s) to PWM (0-255)
     */
    int speedTopwm(float speedMmPerSec);
};

#endif // DCMOTOR_H
