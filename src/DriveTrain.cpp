#include "DriveTrain.h"
#include <Arduino.h>

DriveTrain::DriveTrain(IMotor* leftMotor, IMotor* rightMotor)
    : _leftMotor(leftMotor), _rightMotor(rightMotor) {
    if (!_leftMotor) Serial.println("[DriveTrain] WARNING: leftMotor is nullptr!");
    if (!_rightMotor) Serial.println("[DriveTrain] WARNING: rightMotor is nullptr!");
}

bool DriveTrain::begin() {
    Serial.println("[DriveTrain] Initializing...");
    
    if (!_leftMotor || !_rightMotor) {
        Serial.println("[DriveTrain] ERROR: Motor pointers are null!");
        return false;
    }

    if (!_leftMotor->begin()) {
        Serial.println("[DriveTrain] ERROR: Left motor init failed!");
        return false;
    }

    if (!_rightMotor->begin()) {
        Serial.println("[DriveTrain] ERROR: Right motor init failed!");
        return false;
    }

    Serial.println("[DriveTrain] Both motors initialized successfully");
    return true;
}

bool DriveTrain::isReady() const {
    return _leftMotor && _rightMotor && _leftMotor->isReady() && _rightMotor->isReady();
}

void DriveTrain::setVelocity(float linearVelocityMmPerSec, float angularVelocityRadPerSec) {
    float leftSpeed, rightSpeed;
    computeMotorSpeeds(linearVelocityMmPerSec, angularVelocityRadPerSec, leftSpeed, rightSpeed);
    
    _leftMotor->setSpeedMmPerSec(leftSpeed);
    _rightMotor->setSpeedMmPerSec(rightSpeed);
}

void DriveTrain::setLinearVelocity(float mmPerSec) {
    setVelocity(mmPerSec, 0.0f);
}

void DriveTrain::setAngularVelocity(float radPerSec) {
    setVelocity(0.0f, radPerSec);
}

void DriveTrain::stop() {
    _leftMotor->stop();
    _rightMotor->stop();
}

void DriveTrain::resetPosition() {
    _posX = 0.0f;
    _posY = 0.0f;
    _heading = 0.0f;
    _leftMotor->resetEncoder();
    _rightMotor->resetEncoder();
    _lastLeftEncoderCount = 0;
    _lastRightEncoderCount = 0;
}

void DriveTrain::getPosition(float* outX, float* outY, float* outHeadingRad) const {
    if (outX) *outX = _posX;
    if (outY) *outY = _posY;
    if (outHeadingRad) *outHeadingRad = _heading;
}

void DriveTrain::updateOdometry(unsigned long deltaTimeMs) {
    if (!isReady()) return;

    // Read current encoder values
    long leftCount = _leftMotor->getEncoderCount();
    long rightCount = _rightMotor->getEncoderCount();

    // Calculate displacement since last update
    long leftDelta = leftCount - _lastLeftEncoderCount;
    long rightDelta = rightCount - _lastRightEncoderCount;

    float leftDeltaMm = leftDelta * Config::MmPerEncoderStep;
    float rightDeltaMm = rightDelta * Config::MmPerEncoderStep;

    // Update position
    updatePositionFromEncoders(leftDeltaMm, rightDeltaMm);

    // Remember current encoder values for next update
    _lastLeftEncoderCount = leftCount;
    _lastRightEncoderCount = rightCount;
}

float DriveTrain::getLeftMotorSpeed() const {
    if (!_leftMotor) return 0.0f;
    return _leftMotor->getSpeed();  // ticks/s
}

float DriveTrain::getRightMotorSpeed() const {
    if (!_rightMotor) return 0.0f;
    return _rightMotor->getSpeed();  // ticks/s
}

void DriveTrain::computeMotorSpeeds(float linVel, float angVel,
                                    float& outLeftSpeed, float& outRightSpeed) {
    /**
     * Differential drive kinematics:
     * 
     * For a robot with two wheels separated by distance L (wheelbase):
     *   v_left  = v_linear - (L/2) * v_angular
     *   v_right = v_linear + (L/2) * v_angular
     * 
     * Where:
     *   v = linear velocity (mm/s)
     *   ω = angular velocity (rad/s)
     *   L = wheelbase (mm)
     */
    
    float halfWheelbase = Config::WheelBaseMm / 2.0f;
    
    outLeftSpeed = linVel - (halfWheelbase * angVel);
    outRightSpeed = linVel + (halfWheelbase * angVel);

    // Clamp to max speed
    float maxSpeed = Config::MaxLinearSpeedMmS;
    
    if (abs(outLeftSpeed) > maxSpeed || abs(outRightSpeed) > maxSpeed) {
        float speedRatio = maxSpeed / max(abs(outLeftSpeed), abs(outRightSpeed));
        outLeftSpeed *= speedRatio;
        outRightSpeed *= speedRatio;
    }
}

void DriveTrain::updatePositionFromEncoders(float leftDeltaMm, float rightDeltaMm) {
    /**
     * Odometry integration:
     * 
     * For small time steps, assume constant velocity over the interval:
     *   Δs = (Δs_left + Δs_right) / 2       (average displacement)
     *   Δθ = (Δs_right - Δs_left) / L      (rotation)
     * 
     * Then update pose:
     *   x' = x + Δs * cos(θ + Δθ/2)
     *   y' = y + Δs * sin(θ + Δθ/2)
     *   θ' = θ + Δθ
     */
    
    if (leftDeltaMm == 0.0f && rightDeltaMm == 0.0f) {
        return;  // No movement
    }

    float wheelbase = Config::WheelBaseMm;
    
    // Average displacement
    float deltaS = (leftDeltaMm + rightDeltaMm) / 2.0f;
    
    // Rotation
    float deltaTheta = (rightDeltaMm - leftDeltaMm) / wheelbase;
    
    // Update heading
    _heading += deltaTheta;
    
    // Normalize heading to [-π, π]
    while (_heading > M_PI) _heading -= 2 * M_PI;
    while (_heading < -M_PI) _heading += 2 * M_PI;
    
    // Update position (use heading midpoint for better accuracy)
    float headingMidpoint = _heading - deltaTheta / 2.0f;
    _posX += deltaS * cos(headingMidpoint);
    _posY += deltaS * sin(headingMidpoint);
}
