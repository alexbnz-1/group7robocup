#pragma once

#include <Arduino.h>

namespace Pins {

// Drivetrain — tracked chassis, PPM-controlled DC motor driver (DFR0513)
constexpr uint8_t DRIVE_LEFT = 2;
constexpr uint8_t DRIVE_RIGHT = 3;

// Reel intake — 2x SKU365231 motor, PPM-controlled DC motor driver (DFR0513)
constexpr uint8_t REEL_LEFT = 4;
constexpr uint8_t REEL_RIGHT = 5;

// Sorting gate + kicker cam — DRS-0101 smart servos, chained on one HerkuleX
// serial bus and addressed by ID rather than a dedicated pin each.
constexpr uint8_t SMART_SERVO_SERIAL = 1; // Teensy Serial1
namespace HerkuleXId {
constexpr uint8_t SORTING_GATE = 1;
constexpr uint8_t KICKER_CAM = 2;
} // namespace HerkuleXId

// --- Sensors: placeholder layout, pending final sensor selection ---

// TOF and colour sensors share I2C address 0x29 by default; split across
// Teensy's two I2C buses until a mux/address-translator choice is made.
constexpr uint8_t TOF_I2C_BUS = 0; // Wire
constexpr uint8_t COLOUR_I2C_BUS = 1; // Wire1

constexpr uint8_t IR_LEFT = A0;
constexpr uint8_t IR_RIGHT = A1;

constexpr uint8_t ULTRASOUND_FRONT_TRIG = 9;
constexpr uint8_t ULTRASOUND_FRONT_ECHO = 10;

constexpr uint8_t INDUCTIVE_PROXIMITY = 11;

} // namespace Pins

// TOF sensors reset (XSHUT) over the onboard SX1509 IO expander, not raw
// Teensy pins — placeholder counts/pins, confirm against final sensor layout.
namespace TofConfig {
constexpr uint8_t EXPANDER_I2C_ADDRESS = 0x3F;
constexpr uint8_t VL53L0X_ADDRESS_START = 0x30; // short-range
constexpr uint8_t VL53L1X_ADDRESS_START = 0x35; // long-range

constexpr uint8_t SHORT_COUNT = 2;
constexpr uint8_t SHORT_XSHUT_PINS[SHORT_COUNT] = {0, 1};

constexpr uint8_t LONG_COUNT = 2;
constexpr uint8_t LONG_XSHUT_PINS[LONG_COUNT] = {2, 3};
} // namespace TofConfig

// IR sensors — placeholder count/pins, confirm against final sensor layout.
// The team's own bench testing (see CDR) found analogue IR too noisy to
// trust for ranging, especially off-angle — prefer the digital presence
// check over the distance conversion where possible.
namespace IrConfig {
constexpr uint8_t COUNT = 2;
constexpr uint8_t PINS[COUNT] = {Pins::IR_LEFT, Pins::IR_RIGHT};

// Raw ADC threshold above which an object is considered "present".
// Tune on the bench against the actual mounted sensors.
constexpr uint16_t PRESENCE_THRESHOLD = 400;
} // namespace IrConfig
