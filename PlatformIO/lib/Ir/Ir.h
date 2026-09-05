#pragma once

#include <stdint.h>

namespace Ir {

// Raw ADC counts from sensor `index`.
uint16_t readRaw(uint8_t index);

// Distance estimate in mm using the team's fitted short-range calibration
// curve. Noisy off-angle per bench testing — prefer detected() where possible.
float readDistanceMm(uint8_t index);

// True if something is closer than the configured presence threshold.
bool detected(uint8_t index);

} // namespace Ir
