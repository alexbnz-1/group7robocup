#include "Tof.h"
#include "config.h"

#include <Wire.h>
#include <VL53L0X.h>
#include <VL53L1X.h>
#include <SparkFunSX1509.h>

namespace {

SX1509 expander;
VL53L0X shortSensors[TofConfig::SHORT_COUNT];
VL53L1X longSensors[TofConfig::LONG_COUNT];

// Holds every sensor's XSHUT low so they all boot at the same default I2C
// address (0x29) without colliding, ready to be brought up one at a time.
void resetAllSensors() {
  for (uint8_t i = 0; i < TofConfig::SHORT_COUNT; i++) {
    expander.pinMode(TofConfig::SHORT_XSHUT_PINS[i], OUTPUT);
    expander.digitalWrite(TofConfig::SHORT_XSHUT_PINS[i], LOW);
  }
  for (uint8_t i = 0; i < TofConfig::LONG_COUNT; i++) {
    expander.pinMode(TofConfig::LONG_XSHUT_PINS[i], OUTPUT);
    expander.digitalWrite(TofConfig::LONG_XSHUT_PINS[i], LOW);
  }
}

} // namespace

void Tof::begin() {
  Wire.begin();
  Wire.setClock(400000);
  expander.begin(TofConfig::EXPANDER_I2C_ADDRESS);

  resetAllSensors();

  // Bring each short sensor up one at a time, giving it a unique address
  // before the next one is powered on.
  for (uint8_t i = 0; i < TofConfig::SHORT_COUNT; i++) {
    expander.digitalWrite(TofConfig::SHORT_XSHUT_PINS[i], HIGH);
    delay(10);

    shortSensors[i].setTimeout(500);
    if (!shortSensors[i].init()) {
      Serial.print("Tof: failed to init short sensor ");
      Serial.println(i);
      while (true) {} // halt — a missing/misaddressed sensor can't be recovered here
    }
    shortSensors[i].setAddress(TofConfig::VL53L0X_ADDRESS_START + i);
    shortSensors[i].startContinuous(50);
  }

  // Same bring-up sequence for the long-range sensors.
  for (uint8_t i = 0; i < TofConfig::LONG_COUNT; i++) {
    expander.digitalWrite(TofConfig::LONG_XSHUT_PINS[i], HIGH);
    delay(10);

    longSensors[i].setTimeout(500);
    if (!longSensors[i].init()) {
      Serial.print("Tof: failed to init long sensor ");
      Serial.println(i);
      while (true) {}
    }
    longSensors[i].setAddress(TofConfig::VL53L1X_ADDRESS_START + i);
    longSensors[i].startContinuous(50);
  }
}

// Latest continuous-mode reading, mm.
int16_t Tof::readShort(uint8_t index) {
  return shortSensors[index].readRangeContinuousMillimeters();
}

int16_t Tof::readLong(uint8_t index) {
  return longSensors[index].read();
}

bool Tof::shortTimedOut(uint8_t index) {
  return shortSensors[index].timeoutOccurred();
}

bool Tof::longTimedOut(uint8_t index) {
  return longSensors[index].timeoutOccurred();
}
