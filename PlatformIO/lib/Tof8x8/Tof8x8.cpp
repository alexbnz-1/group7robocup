#include "Tof8x8.h"

#include <new>

Tof8x8::Tof8x8(uint8_t address, TwoWire& wire)
    : wire_(wire), preferredAddress_(address)
{
}

bool Tof8x8::begin()
{
    wire_.begin();
    addressAckMask_ = 0;
    address52Acknowledged_ = false;
    memset(i2cAckBits_, 0, sizeof(i2cAckBits_));
    detectedAddress_ = 0;

    // Scan the complete usable 7-bit range. Besides locating the SEN0628, this
    // proves whether the selected CPU socket is electrically on this Wire bus.
    for (uint8_t address = 0x08; address <= 0x77; ++address)
    {
        wire_.beginTransmission(address);
        if (wire_.endTransmission() == 0)
            i2cAckBits_[address >> 3] |= static_cast<uint8_t>(1U << (address & 7));
    }

    for (uint8_t address = 0x30; address <= 0x33; ++address)
        if (addressAcknowledged(address))
            addressAckMask_ |= static_cast<uint8_t>(1U << (address - 0x30));

    address52Acknowledged_ = addressAcknowledged(0x52);

    uint8_t address = 0;
    if (preferredAddress_ == 0x52 && address52Acknowledged_)
        address = 0x52;
    else if (preferredAddress_ >= 0x30 && preferredAddress_ <= 0x33 &&
        (addressAckMask_ & (1U << (preferredAddress_ - 0x30))))
        address = preferredAddress_;
    else
        for (uint8_t index = 0; index < 4 && address == 0; ++index)
            if (addressAckMask_ & (1U << index))
                address = 0x30 + index;

    if (address != 0)
    {
        sensor_ = new (std::nothrow) DFRobot_MatrixLidar_I2C(address, &wire_);
        if (sensor_ != nullptr && sensor_->begin() == 0 &&
            sensor_->setRangingMode(eMatrix_8X8) == 0)
        {
            available_ = true;
            detectedAddress_ = address;
            return true;
        }
    }

    available_ = false;
    return false;
}

bool Tof8x8::read()
{
    if (!available_)
        return false;

    lastReadSucceeded_ = sensor_ != nullptr && sensor_->getAllData(frame_) == 0;
    if (lastReadSucceeded_)
        ++frameNumber_;
    return lastReadSucceeded_;
}

bool Tof8x8::available() const { return available_; }
bool Tof8x8::lastReadSucceeded() const { return lastReadSucceeded_; }
uint32_t Tof8x8::frameNumber() const { return frameNumber_; }
uint8_t Tof8x8::detectedAddress() const { return detectedAddress_; }
uint8_t Tof8x8::addressAckMask() const { return addressAckMask_; }
bool Tof8x8::address52Acknowledged() const { return address52Acknowledged_; }

bool Tof8x8::addressAcknowledged(uint8_t address) const
{
    return address < 128 && (i2cAckBits_[address >> 3] & (1U << (address & 7))) != 0;
}

uint16_t Tof8x8::distanceMm(uint8_t row, uint8_t column) const
{
    if (row >= HEIGHT || column >= WIDTH)
        return 0;
    return frame_[row * WIDTH + column];
}

const uint16_t* Tof8x8::data() const { return frame_; }
