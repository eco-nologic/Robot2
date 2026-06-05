
Eric Copet
20:12 (il y a 12 minutes)
À Nicolas

// ================================================
// Drawing Robot - ESP32 Differential Drive with Pen Offset + PID Wheel Velocity Control
// G-code support + PID
// ================================================

#include <Arduino.h>
#include <ESP32Encoder.h>
#include <math.h>

// ====================== CONFIGURATION ======================
const float WHEEL_BASE = 0.20f; // meters
const float WHEEL_RADIUS = 0.032f; // meters
const float PEN_OFFSET_D = 0.10f; // meters
const float MAX_SPEED = 0.3f; // m/s

// PID Tunings - START HERE and tune!
const float PID_KP = 8.0f; // Proportional
const float PID_KI = 2.0f; // Integral
const float PID_KD = 0.1f; // Derivative

// Pins
#define LEFT_PWM_PIN 25
#define LEFT_DIR_PIN 26
#define RIGHT_PWM_PIN 27
#define RIGHT_DIR_PIN 14

// Encoder pins (change to yours)
#define ENC_LEFT_A 16
#define ENC_LEFT_B 17
#define ENC_RIGHT_A 18
#define ENC_RIGHT_B 19

ESP32Encoder leftEncoder;
ESP32Encoder rightEncoder;

// ====================== PID CONTROLLER ======================
struct PID {
    float kp, ki, kd;
    float setpoint = 0.0f;
    float integral = 0.0f;
    float prevError = 0.0f;
    unsigned long lastTime = 0;
    
    PID(float p, float i, float d) : kp(p), ki(i), kd(d) {}
    
    float compute(float measured, unsigned long now) {
        float dt = (now - lastTime) / 1000.0f; // seconds
        if (dt <= 0.001f) dt = 0.01f;
        
        float error = setpoint - measured;
        integral += error * dt;
        float derivative = (error - prevError) / dt;
        
        // Anti-windup (simple clamp)
        if (integral > 100.0f) integral = 100.0f;
        if (integral < -100.0f) integral = -100.0f;
        
        float output = kp * error + ki * integral + kd * derivative;
        
        prevError = error;
        lastTime = now;
        return constrain(output, -255.0f, 255.0f); // PWM range
    }
};

PID pidLeft(PID_KP, PID_KI, PID_KD);
PID pidRight(PID_KP, PID_KI, PID_KD);

// ====================== STATE ======================
struct Pose {
    float x = 0.0f, y = 0.0f, theta = 0.0f;
};

Pose currentPose;

volatile long leftTicks = 0;
volatile long rightTicks = 0;
long prevLeftTicks = 0;
long prevRightTicks = 0;

unsigned long lastOdometryTime = 0;
unsigned long lastSpeedTime = 0;

float currentSpeedLeft = 0.0f; // m/s
float currentSpeedRight = 0.0f;

float targetX_pen = 0.0f;
float targetY_pen = 0.0f;
bool isMoving = false;

// Ticks per revolution - MEASURE / CALCULATE YOURS
const float TICKS_PER_REV = 360.0f; // Example - change to your encoder spec
const float DIST_PER_TICK = (2.0f * M_PI * WHEEL_RADIUS) / TICKS_PER_REV;

// ====================== HELPER FUNCTIONS ======================

void IRAM_ATTR leftEncoderISR() {
    leftEncoder.handle();
}

void IRAM_ATTR rightEncoderISR() {
    rightEncoder.handle();
}

void updateOdometry() {
    unsigned long now = millis();
    if (now - lastOdometryTime < 10) return; // \~100Hz
    
    long leftDelta = leftEncoder.getCount() - prevLeftTicks;
    long rightDelta = rightEncoder.getCount() - prevRightTicks;
    
    prevLeftTicks = leftEncoder.getCount();
    prevRightTicks = rightEncoder.getCount();
    
    float dl = leftDelta * DIST_PER_TICK;
    float dr = rightDelta * DIST_PER_TICK;
    
    float d_center = (dl + dr) / 2.0f;
    float d_theta = (dr - dl) / WHEEL_BASE;
    
    currentPose.x += d_center * cos(currentPose.theta + d_theta/2.0f);
    currentPose.y += d_center * sin(currentPose.theta + d_theta/2.0f);
    currentPose.theta += d_theta;
    
    while (currentPose.theta > M_PI) currentPose.theta -= 2*M_PI;
    while (currentPose.theta < -M_PI) currentPose.theta += 2*M_PI;
    
    lastOdometryTime = now;
}

void updateWheelSpeeds() {
    unsigned long now = millis();
    if (now - lastSpeedTime < 50) return; // 20Hz speed update - tune
    
    long leftDelta = leftEncoder.getCount() - prevLeftTicks; // reuse from odometry if synced
    long rightDelta = rightEncoder.getCount() - prevRightTicks;
    
    float dt = (now - lastSpeedTime) / 1000.0f;
    currentSpeedLeft = (leftDelta * DIST_PER_TICK) / dt;
    currentSpeedRight = (rightDelta * DIST_PER_TICK) / dt;
    
    lastSpeedTime = now;
}

void getPenPosition(float& px, float& py) {
    px = currentPose.x + PEN_OFFSET_D * cos(currentPose.theta);
    py = currentPose.y + PEN_OFFSET_D * sin(currentPose.theta);
}

void penToRobot(float px, float py, float desiredTheta, float& rx, float& ry, float& rtheta) {
    rx = px - PEN_OFFSET_D * cos(desiredTheta);
    ry = py - PEN_OFFSET_D * sin(desiredTheta);
    rtheta = desiredTheta;
}

void diffIK(float v, float omega, float& vL, float& vR) {
    vR = v + omega * (WHEEL_BASE / 2.0f);
    vL = v - omega * (WHEEL_BASE / 2.0f);
}

// ====================== MOTOR CONTROL WITH PID ======================
void setMotorSpeeds(float desiredVL, float desiredVR) { // m/s
    unsigned long now = millis();
    
    pidLeft.setpoint = desiredVL;
    pidRight.setpoint = desiredVR;
    
    float pwmLeft = pidLeft.compute(currentSpeedLeft, now);
    float pwmRight = pidRight.compute(currentSpeedRight, now);
    
    // Left motor
    digitalWrite(LEFT_DIR_PIN, pwmLeft >= 0 ? HIGH : LOW);
    analogWrite(LEFT_PWM_PIN, abs(pwmLeft));
    
    // Right motor
    digitalWrite(RIGHT_DIR_PIN, pwmRight >= 0 ? HIGH : LOW);
    analogWrite(RIGHT_PWM_PIN, abs(pwmRight));
}

// ====================== G-CODE PARSER (unchanged) ======================
String gcodeBuffer = "";

void processGCode(String line) {
    line.trim();
    if (line.length() == 0 || line.startsWith(";") || line.startsWith("(")) return;
    
    float x = NAN, y = NAN;
    // ... (same simple parser as previous version)
    
    if (line.startsWith("G00") || line.startsWith("G01")) {
        if (!isnan(x) || !isnan(y)) {
            targetX_pen = isnan(x) ? targetX_pen : x / 1000.0f;
            targetY_pen = isnan(y) ? targetY_pen : y / 1000.0f;
            isMoving = true;
        }
    }
}

// ====================== PATH FOLLOWER ======================
void followTarget() {
    if (!isMoving) {
        setMotorSpeeds(0, 0);
        return;
    }
    
    float px, py;
    getPenPosition(px, py);
    
    float dx = targetX_pen - px;
    float dy = targetY_pen - py;
    float dist = sqrt(dx*dx + dy*dy);
    
    if (dist < 0.008f) { // \~8mm tolerance
        isMoving = false;
        setMotorSpeeds(0, 0);
        return;
    }
    
    float desiredTheta = atan2(dy, dx);
    float angleError = desiredTheta - currentPose.theta;
    while (angleError > M_PI) angleError -= 2*M_PI;
    while (angleError < -M_PI) angleError += 2*M_PI;
    
    float omega = 2.5f * angleError; // yaw gain
    float v = constrain(dist * 1.8f, 0.08f, MAX_SPEED);
    
    float vL, vR;
    diffIK(v, omega, vL, vR);
    setMotorSpeeds(vL, vR);
}

// ====================== SETUP & LOOP ======================
void setup() {
    Serial.begin(115200);
    Serial.println("Drawing Robot with PID Ready");
    
    // Encoders
    ESP32Encoder::useInternalWeakPullResistors = UP;
    leftEncoder.attach(ENC_LEFT_A, ENC_LEFT_B);
    rightEncoder.attach(ENC_RIGHT_A, ENC_RIGHT_B);
    
    // Optional ISRs for high speed
    // attachInterrupt(digitalPinToInterrupt(ENC_LEFT_A), leftEncoderISR, CHANGE); etc.
    
    // Motor pins
    pinMode(LEFT_PWM_PIN, OUTPUT);
    pinMode(LEFT_DIR_PIN, OUTPUT);
    pinMode(RIGHT_PWM_PIN, OUTPUT);
    pinMode(RIGHT_DIR_PIN, OUTPUT);
    
    analogWriteFrequency(30000); // ESP32 PWM freq
}

void loop() {
    // G-code input
    while (Serial.available()) {
        char c = Serial.read();
        if (c == '\n') {
            processGCode(gcodeBuffer);
            gcodeBuffer = "";
        } else gcodeBuffer += c;
    }
    
    updateOdometry();
    updateWheelSpeeds();
    followTarget();
    
    delay(5); // Main loop \~200Hz, adjust as needed
}