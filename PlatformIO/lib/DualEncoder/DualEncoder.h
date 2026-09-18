#pragma once

#include <Arduino.h>
#include <Encoder.h>

// Two quadrature encoders on the four signal pins of Digital Raw 2.
// Encoder.h handles both A and B transitions with interrupts (x4 decoding).
class DualEncoder {
public:
    DualEncoder(uint8_t firstA, uint8_t firstB, uint8_t secondA, uint8_t secondB);

    void begin();
    void sample(uint32_t nowMs);
    void zero();

    int32_t firstCount() const { return firstCount_; }
    int32_t secondCount() const { return secondCount_; }
    int32_t firstDelta() const { return firstDelta_; }
    int32_t secondDelta() const { return secondDelta_; }
    float firstCountsPerSecond() const { return firstCountsPerSecond_; }
    float secondCountsPerSecond() const { return secondCountsPerSecond_; }

private:
    uint8_t firstA_;
    uint8_t firstB_;
    uint8_t secondA_;
    uint8_t secondB_;
    Encoder first_;
    Encoder second_;
    int32_t firstCount_ = 0;
    int32_t secondCount_ = 0;
    int32_t firstDelta_ = 0;
    int32_t secondDelta_ = 0;
    float firstCountsPerSecond_ = 0.0f;
    float secondCountsPerSecond_ = 0.0f;
    uint32_t lastSampleMs_ = 0;
};
