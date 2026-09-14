#include "Hx12kServo.h"

#include <math.h>

Hx12kServo::Hx12kServo(uint8_t signalPin, float travelDeg)
    : signalPin_(signalPin),
      travelDeg_(travelDeg > 0.0f ? travelDeg : DEFAULT_TRAVEL_DEG),
      commandedAngleDeg_(travelDeg_ * 0.5f)
{
}

void Hx12kServo::begin()
{
    disable();
}

void Hx12kServo::setAngle(float angleDeg)
{
    angleDeg = constrain(angleDeg, 0.0f, travelDeg_);
    const float fraction = angleDeg / travelDeg_;
    const uint16_t pulseUs = static_cast<uint16_t>(lroundf(
        MIN_PULSE_US + fraction * (MAX_PULSE_US - MIN_PULSE_US)
    ));
    commandedAngleDeg_ = angleDeg;
    setPulseMicroseconds(pulseUs);
}

void Hx12kServo::setPulseMicroseconds(uint16_t pulseUs)
{
    pulseUs = constrain(pulseUs, MIN_PULSE_US, MAX_PULSE_US);
    ensureAttached();
    commandedPulseUs_ = pulseUs;
    commandedAngleDeg_ =
        (pulseUs - MIN_PULSE_US) * travelDeg_ /
        static_cast<float>(MAX_PULSE_US - MIN_PULSE_US);
    output_.writeMicroseconds(commandedPulseUs_);
}

void Hx12kServo::centre()
{
    setPulseMicroseconds(CENTRE_PULSE_US);
}

void Hx12kServo::disable()
{
    if (output_.attached())
        output_.detach();
    pinMode(signalPin_, OUTPUT);
    digitalWrite(signalPin_, LOW);
    enabled_ = false;
}

bool Hx12kServo::enabled() const
{
    return enabled_;
}

float Hx12kServo::commandedAngle() const
{
    return commandedAngleDeg_;
}

uint16_t Hx12kServo::commandedPulseMicroseconds() const
{
    return commandedPulseUs_;
}

float Hx12kServo::travelDegrees() const
{
    return travelDeg_;
}

void Hx12kServo::ensureAttached()
{
    if (!output_.attached())
        output_.attach(signalPin_);
    enabled_ = true;
}
