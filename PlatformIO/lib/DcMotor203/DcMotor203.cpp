#include "DcMotor203.h"

DcMotor203::DcMotor203(uint8_t channelAPin, uint8_t channelBPin)
    : channelAPin_(channelAPin),
      channelBPin_(channelBPin)
{
}

void DcMotor203::begin()
{
    // Match the supplied 203_DCMotor example: each controller channel is a
    // normal Servo output. Serial3 is only the physical connector name here.
    channelA_.attach(channelAPin_);
    channelB_.attach(channelBPin_);
    stop();
}

void DcMotor203::setPercent(int16_t channelAPercent, int16_t channelBPercent)
{
    channelAPercent_ = constrain(channelAPercent, -100, 100);
    channelBPercent_ = constrain(channelBPercent, -100, 100);
    channelAPulseUs_ = static_cast<uint16_t>(map(
        channelAPercent_,
        -100,
        100,
        FULL_REVERSE_US,
        FULL_FORWARD_US
    ));
    channelBPulseUs_ = static_cast<uint16_t>(map(
        channelBPercent_,
        -100,
        100,
        FULL_REVERSE_US,
        FULL_FORWARD_US
    ));
    channelA_.writeMicroseconds(channelAPulseUs_);
    channelB_.writeMicroseconds(channelBPulseUs_);
}

void DcMotor203::stop()
{
    channelAPercent_ = 0;
    channelBPercent_ = 0;
    channelAPulseUs_ = STOP_US;
    channelBPulseUs_ = STOP_US;
    channelA_.writeMicroseconds(channelAPulseUs_);
    channelB_.writeMicroseconds(channelBPulseUs_);
}

int16_t DcMotor203::channelAPercent() const
{
    return channelAPercent_;
}

int16_t DcMotor203::channelBPercent() const
{
    return channelBPercent_;
}

uint16_t DcMotor203::channelAPulseUs() const
{
    return channelAPulseUs_;
}

uint16_t DcMotor203::channelBPulseUs() const
{
    return channelBPulseUs_;
}
