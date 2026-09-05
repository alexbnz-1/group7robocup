#include "Ir.h"
#include "config.h"

#include <Arduino.h>
#include <math.h>

uint16_t Ir::readRaw(uint8_t index) {
  return analogRead(IrConfig::PINS[index]);
}

// Inverse of the team's fitted curve ADC = 3790.7110 * distance^-0.3313 - 500.
float Ir::readDistanceMm(uint8_t index) {
  float adc = readRaw(index);
  return powf((adc + 500.0f) / 3790.7110f, -3.0181f);
}

bool Ir::detected(uint8_t index) {
  return readRaw(index) > IrConfig::PRESENCE_THRESHOLD;
}
