#pragma once

#include <stdint.h>

namespace Tof {

// Brings up the SX1509 expander and all configured short/long TOF sensors.
void begin();

// Latest range reading in mm for short/long sensor `index`.
int16_t readShort(uint8_t index);
int16_t readLong(uint8_t index);

// True if the last read on that sensor timed out (treat the reading as stale).
bool shortTimedOut(uint8_t index);
bool longTimedOut(uint8_t index);

} // namespace Tof
