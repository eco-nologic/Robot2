#ifndef DRIVETRAIN_H
#define DRIVETRAIN_H

#include "IMotor.h"
#include "Config.h"
#include <cmath>

/**
 * @class DriveTrain
 * @brief Differential drive kinematics controller
 * 
 * Manages two motors for a differential-drive robot.
 * Converts desired velocities (linear v, angular ω) to individual motor speeds.
 * 
 * Kinematics equations:
 *   v_left  = v - ω * L/2    where L = wheelbase
 *   v_right = v + ω * L/2
 */
class DriveTrain {
public:
    /**
     * Constructor - takes ownership of two motor pointers
     * @param leftMotor Left wheel motor (must not be nullptr)
     * @param rightMotor Right wheel motor (must not be nullptr)
     */
    DriveTrain(IMotor* leftMotor, IMotor* rightMotor);

    /**
     * Initialize both motors
     * @return true if both motors initialized successfully
     */
    bool begin();

    /**
     * Set linear velocity (mm/s) and angular velocity (rad/s)
     * @param linearVelocityMmPerSec Forward speed (positive = forward, negative = backward)
     * @param angularVelocityRadPerSec Rotation speed (positive = counter-clockwise)
     */
    void setVelocity(float linearVelocityMmPerSec, float angularVelocityRadPerSec);

    /**
     * Set linear velocity only (stops rotation)
     * @param mmPerSec Speed in mm/s
     */
    void setLinearVelocity(float mmPerSec);

    /**
     * Set angular velocity only (rotation in place)
     * @param radPerSec Angular speed in rad/s
     */
    void setAngularVelocity(float radPerSec);

    /**
     * Stop all motors immediately
     */
    void stop();

    /**
     * Get position estimate (from odometry)
     * @param outX Pointer to store X position (mm)
     * @param outY Pointer to store Y position (mm)
     * @param outHeadingRad Pointer to store heading (radians)
     */
    void getPosition(float* outX, float* outY, float* outHeadingRad) const;

    /**
     * Reset position to (0, 0, 0)
     */
    void resetPosition();

    /**
     * Update odometry (call from main loop)
     * @param deltaTimeMs Time since last update (milliseconds)
     */
    void updateOdometry(unsigned long deltaTimeMs);

    /**
     * Get current left/right motor speeds (for debugging)
     */
    float getLeftMotorSpeed() const;
    float getRightMotorSpeed() const;

    /**
     * Check if both motors are operational
     */
    bool isReady() const;

private:
    IMotor* _leftMotor;
    IMotor* _rightMotor;

    // Odometry state
    float _posX = 0.0f;           // X position (mm)
    float _posY = 0.0f;           // Y position (mm)
    float _heading = 0.0f;        // Heading (radians, 0 = +X direction)

    // Encoder state tracking
    long _lastLeftEncoderCount = 0;
    long _lastRightEncoderCount = 0;

    /**
     * Compute motor speeds from desired linear and angular velocities
     * @param linVel Linear velocity (mm/s)
     * @param angVel Angular velocity (rad/s)
     * @param outLeftSpeed Output left motor speed (mm/s)
     * @param outRightSpeed Output right motor speed (mm/s)
     */
    void computeMotorSpeeds(float linVel, float angVel, 
                            float& outLeftSpeed, float& outRightSpeed);

    /**
     * Update robot position from encoder deltas
     * @param leftDeltaMm Left wheel displacement (mm)
     * @param rightDeltaMm Right wheel displacement (mm)
     */
    void updatePositionFromEncoders(float leftDeltaMm, float rightDeltaMm);
};

#endif // DRIVETRAIN_H
