#ifndef IMOTOR_H
#define IMOTOR_H

#include <Arduino.h>

/**
 * @class IMotor
 * @brief Abstract interface for motor control - allows swapping real vs. simulated motors
 * 
 * This interface defines the contract for any motor implementation (real hardware or simulation).
 * Both DCMotor and VirtualStepper implement this interface.
 */
class IMotor {
public:
    virtual ~IMotor() = default;

    /**
     * Initialize motor pins and PWM configuration
     * @return true if successful, false if setup failed
     */
    virtual bool begin() = 0;

    /**
     * Set motor PWM duty cycle
     * @param pwm Value from 0 (off) to 255 (full speed)
     */
    virtual void setPwm(int pwm) = 0;

    /**
     * Set motor rotation direction
     * @param forward true = forward direction, false = backward
     */
    virtual void setDirection(bool forward) = 0;

    /**
     * Get current encoder count (ticks accumulated since startup)
     * @return Encoder tick count (long integer)
     */
    virtual long getEncoderCount() const = 0;

    /**
     * Reset encoder counter to zero
     */
    virtual void resetEncoder() = 0;

    /**
     * Get motor speed in encoder ticks per second
     * @return Current speed estimate (ticks/s)
     */
    virtual float getSpeed() const = 0;

    /**
     * Set motor speed directly (alternative to setPwm + setDirection)
     * @param speedMmPerSec Speed in mm/s (positive = forward, negative = backward)
     */
    virtual void setSpeedMmPerSec(float speedMmPerSec) = 0;

    /**
     * Stop motor immediately (set PWM to 0)
     */
    virtual void stop() = 0;

    /**
     * Check if motor is operational
     * @return true if motor is ready to use
     */
    virtual bool isReady() const = 0;
};

#endif // IMOTOR_H
