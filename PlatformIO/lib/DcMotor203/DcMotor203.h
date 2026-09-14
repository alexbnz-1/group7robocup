#pragma once

#include <Arduino.h>
#include <Servo.h>

class DcMotor203 {
public:
    static constexpr uint16_t FULL_REVERSE_US = 1050;
    static constexpr uint16_t STOP_US = 1500;
    static constexpr uint16_t FULL_FORWARD_US = 1950;

    DcMotor203(uint8_t channelAPin, uint8_t channelBPin);

    void begin();
    void setPercent(int16_t channelAPercent, int16_t channelBPercent);
    void stop();

    int16_t channelAPercent() const;
    int16_t channelBPercent() const;
    uint16_t channelAPulseUs() const;
    uint16_t channelBPulseUs() const;

private:
    uint8_t channelAPin_;
    uint8_t channelBPin_;
    Servo channelA_;
    Servo channelB_;
    int16_t channelAPercent_ = 0;
    int16_t channelBPercent_ = 0;
    uint16_t channelAPulseUs_ = STOP_US;
    uint16_t channelBPulseUs_ = STOP_US;
};
