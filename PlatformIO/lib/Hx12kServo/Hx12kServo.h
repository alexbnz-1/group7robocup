#pragma once

#include <Arduino.h>
#include <Servo.h>

class Hx12kServo {
public:
    static constexpr uint16_t MIN_PULSE_US = 1000;
    static constexpr uint16_t CENTRE_PULSE_US = 1500;
    static constexpr uint16_t MAX_PULSE_US = 2000;
    static constexpr float DEFAULT_TRAVEL_DEG = 135.0f;

    explicit Hx12kServo(uint8_t signalPin, float travelDeg = DEFAULT_TRAVEL_DEG);

    // begin() deliberately leaves the output disabled so startup cannot move
    // the mechanism. The first position command attaches the pulse output.
    void begin();
    void setAngle(float angleDeg);
    void setPulseMicroseconds(uint16_t pulseUs);
    void centre();
    void disable();

    bool enabled() const;
    float commandedAngle() const;
    uint16_t commandedPulseMicroseconds() const;
    float travelDegrees() const;

private:
    void ensureAttached();

    uint8_t signalPin_;
    float travelDeg_;
    Servo output_;
    bool enabled_ = false;
    float commandedAngleDeg_;
    uint16_t commandedPulseUs_ = CENTRE_PULSE_US;
};
