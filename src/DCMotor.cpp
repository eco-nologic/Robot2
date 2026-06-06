#include "DCMotor.h"
#include <Arduino.h>

/**
 * ESP32 PCNT (Pulse Counter) Configuration Notes:
 * - PCNT_UNIT_0 through PCNT_UNIT_7 available (8 units total)
 * - We use PCNT_UNIT_0 for left motor, PCNT_UNIT_1 for right motor
 * - PCNT can count up to 65536 ticks before overflow (16-bit counter)
 * - Perfect for quadrature encoding with glitch filter
 */

DCMotor::DCMotor(int motorId) 
    : _motorId(motorId),
      _pcntUnit(motorId == 0 ? PCNT_UNIT_0 : PCNT_UNIT_1) {
    selectMotorPins();
}

void DCMotor::selectMotorPins() {
    if (_motorId == 0) {
        // LEFT MOTOR
        _pinEn = Config::PIN_MOTOR_LEFT_EN;
        _pinIn1 = Config::PIN_MOTOR_LEFT_IN1;
        _pinIn2 = Config::PIN_MOTOR_LEFT_IN2;
        _pwmChannel = Config::PWM_CHANNEL_LEFT;
        _pinEncoderA = Config::PIN_ENCODER_LEFT_A;
        _pinEncoderB = Config::PIN_ENCODER_LEFT_B;
        _inverted = Config::MOTOR_LEFT_INVERTED;
    } else {
        // RIGHT MOTOR
        _pinEn = Config::PIN_MOTOR_RIGHT_EN;
        _pinIn1 = Config::PIN_MOTOR_RIGHT_IN1;
        _pinIn2 = Config::PIN_MOTOR_RIGHT_IN2;
        _pwmChannel = Config::PWM_CHANNEL_RIGHT;
        _pinEncoderA = Config::PIN_ENCODER_RIGHT_A;
        _pinEncoderB = Config::PIN_ENCODER_RIGHT_B;
        _inverted = Config::MOTOR_RIGHT_INVERTED;
    }
}

bool DCMotor::begin() {
    Serial.printf("[DCMotor] Initializing motor %d...\n", _motorId);

    // Configure direction pins as outputs
    pinMode(_pinIn1, OUTPUT);
    pinMode(_pinIn2, OUTPUT);
    digitalWrite(_pinIn1, LOW);
    digitalWrite(_pinIn2, LOW);

    // Configure PWM on enable pin
    ledcSetup(_pwmChannel, 
              (_motorId == 0 ? Config::PWM_FREQUENCY_LEFT : Config::PWM_FREQUENCY_RIGHT),
              8);  // 8-bit resolution (0-255)
    ledcAttachPin(_pinEn, _pwmChannel);
    ledcWrite(_pwmChannel, 0);  // Start at 0 speed

    // Configure encoder pins as inputs
    pinMode(_pinEncoderA, INPUT);
    pinMode(_pinEncoderB, INPUT);

    // Initialize PCNT for encoder reading
    if (!initializePcnt()) {
        Serial.printf("[DCMotor] ERROR: Failed to initialize PCNT for motor %d\n", _motorId);
        return false;
    }

    _isInitialized = true;
    Serial.printf("[DCMotor] Motor %d initialized successfully\n", _motorId);
    return true;
}

bool DCMotor::initializePcnt() {
    // PCNT configuration structure
    pcnt_config_t pcnt_config = {
        .pulse_gpio_num = _pinEncoderA,      // Phase A (increment count)
        .ctrl_gpio_num = _pinEncoderB,       // Phase B (direction)
        .lctrl_mode = PCNT_MODE_REVERSE,     // When B is low, count up
        .hctrl_mode = PCNT_MODE_KEEP,        // When B is high, count up
        .pos_mode = PCNT_COUNT_INC,          // Increment on rising edge
        .neg_mode = PCNT_COUNT_DEC,          // Decrement on falling edge
        .counter_h_lim = 32767,              // High limit (don't overflow)
        .counter_l_lim = -32768,             // Low limit
        .unit = _pcntUnit,
        .channel = PCNT_CHANNEL_0,
    };

    // Apply configuration
    esp_err_t err = pcnt_unit_config(&pcnt_config);
    if (err != ESP_OK) {
        Serial.printf("[DCMotor] PCNT config failed: %s\n", esp_err_to_name(err));
        return false;
    }

    // Enable glitch filter (ignore noise)
    pcnt_set_filter_value(_pcntUnit, 10);
    pcnt_filter_enable(_pcntUnit);

    // Start counting
    pcnt_counter_pause(_pcntUnit);
    pcnt_counter_clear(_pcntUnit);
    pcnt_counter_resume(_pcntUnit);

    Serial.printf("[DCMotor] PCNT unit %d configured for motor %d\n", _pcntUnit, _motorId);
    return true;
}

void DCMotor::setPwm(int pwm) {
    // Clamp PWM value
    pwm = constrain(pwm, 0, Config::MotorPwmMax);

    // Apply deadband (minimum PWM to overcome static friction)
    if (pwm > 0 && pwm < Config::MotorPwmDeadband) {
        pwm = Config::MotorPwmDeadband;
    }

    _currentPwm = pwm;
    ledcWrite(_pwmChannel, pwm);
}

void DCMotor::setDirection(bool forward) {
    // Apply wiring inversion if necessary
    bool actualForward = forward ^ _inverted;
    _isForward = actualForward;
    if (actualForward) {
        digitalWrite(_pinIn1, HIGH);
        digitalWrite(_pinIn2, LOW);
    } else {
        digitalWrite(_pinIn1, LOW);
        digitalWrite(_pinIn2, HIGH);
    }
}

void DCMotor::stop() {
    setPwm(0);
}

long DCMotor::getEncoderCount() const {
    int16_t count = 0;
    pcnt_get_counter_value(_pcntUnit, &count);
    return (long)count;
}

void DCMotor::resetEncoder() {
    pcnt_counter_pause(_pcntUnit);
    pcnt_counter_clear(_pcntUnit);
    pcnt_counter_resume(_pcntUnit);
    _lastEncoderCount = 0;
}

float DCMotor::getSpeed() const {
    return _estimatedSpeed;  // ticks/s
}

void DCMotor::updateSpeed(unsigned long deltaTimeMs) {
    if (deltaTimeMs < 10) return;  // Don't update too frequently

    long currentCount = getEncoderCount();
    long deltaTicks = currentCount - _lastEncoderCount;
    float deltaSeconds = deltaTimeMs / 1000.0f;
    
    _estimatedSpeed = (float)deltaTicks / deltaSeconds;  // ticks/s
    
    _lastEncoderCount = currentCount;
    _lastSpeedUpdateMs = millis();
}

void DCMotor::setSpeedMmPerSec(float speedMmPerSec) {
    // Determine direction
    if (speedMmPerSec < 0) {
        setDirection(false);
        speedMmPerSec = -speedMmPerSec;
    } else {
        setDirection(true);
    }

    // Convert speed to PWM
    int pwm = speedTopwm(speedMmPerSec);
    setPwm(pwm);
}

int DCMotor::speedTopwm(float speedMmPerSec) {
    // Convert linear speed in mm/s to encoder ticks per second
    float ticksPerSec = speedMmPerSec / Config::MmPerEncoderStep;
    float maxTicksPerSec = Config::MaxLinearSpeedMmS / Config::MmPerEncoderStep;

    // Map the encoder-rate ratio to PWM, using the hardware speed calibration.
    float speedRatio = constrain(ticksPerSec / maxTicksPerSec, 0.0f, 1.0f);
    int pwm = (int)(speedRatio * Config::MotorPwmMax);

    return constrain(pwm, 0, Config::MotorPwmMax);
}
