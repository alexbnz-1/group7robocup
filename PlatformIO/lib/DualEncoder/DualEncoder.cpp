#include "DualEncoder.h"

DualEncoder::DualEncoder(uint8_t firstA, uint8_t firstB, uint8_t secondA, uint8_t secondB)
    : firstA_(firstA), firstB_(firstB), secondA_(secondA), secondB_(secondB)
{
}

void DualEncoder::begin()
{
    // Encoder.begin configures both pins as INPUT_PULLUP and attaches CHANGE
    // interrupts. This permits open-collector encoder outputs as well as
    // push-pull 3.3 V signals.
    first_.begin(firstA_, firstB_);
    second_.begin(secondA_, secondB_);
    zero();
}

void DualEncoder::sample(uint32_t nowMs)
{
    const int32_t firstNow = first_.read();
    const int32_t secondNow = second_.read();
    const uint32_t elapsedMs = nowMs - lastSampleMs_;
    firstDelta_ = firstNow - firstCount_;
    secondDelta_ = secondNow - secondCount_;
    firstCount_ = firstNow;
    secondCount_ = secondNow;
    if (elapsedMs != 0)
    {
        firstCountsPerSecond_ = firstDelta_ * 1000.0f / elapsedMs;
        secondCountsPerSecond_ = secondDelta_ * 1000.0f / elapsedMs;
    }
    lastSampleMs_ = nowMs;
}

void DualEncoder::zero()
{
    first_.write(0);
    second_.write(0);
    firstCount_ = 0;
    secondCount_ = 0;
    firstDelta_ = 0;
    secondDelta_ = 0;
    firstCountsPerSecond_ = 0.0f;
    secondCountsPerSecond_ = 0.0f;
    lastSampleMs_ = millis();
}
