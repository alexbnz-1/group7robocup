#pragma once

#include <Arduino.h>

// ============================================================
// PIN / HARDWARE ASSIGNMENTS
// ============================================================

namespace Pins {

    // --------------------------------------------------------
    // Drivetrain
    // Tracked chassis, PPM-controlled DC motor driver (DFR0513)
    // --------------------------------------------------------

    constexpr uint8_t DRIVE_LEFT  = 2;
    constexpr uint8_t DRIVE_RIGHT = 3;


    // --------------------------------------------------------
    // Reel intake
    // 2x SKU365231 motors, PPM-controlled DC motor driver (DFR0513)
    // --------------------------------------------------------

    constexpr uint8_t REEL_LEFT  = 4;
    constexpr uint8_t REEL_RIGHT = 5;


    // --------------------------------------------------------
    // Herkulex smart servo bus
    //
    // Teensy 4.0 Serial1:
    // RX1 = pin 0
    // TX1 = pin 1
    //
    // Sorting gate + kicker cam share this serial bus and are
    // addressed by servo ID rather than individual signal pins.
    // --------------------------------------------------------

    constexpr uint8_t SMART_SERVO_RX = 0;
    constexpr uint8_t SMART_SERVO_TX = 1;


    // --------------------------------------------------------
    // Sensors
    // --------------------------------------------------------

    // TOF and colour sensors share I2C address 0x29 by default;
    // split across Teensy's two I2C buses until a mux/address
    // translator choice is finalised.

    constexpr uint8_t TOF_I2C_BUS    = 0; // Wire
    constexpr uint8_t COLOUR_I2C_BUS = 1; // Wire1

    constexpr uint8_t IR_LEFT  = A0;
    constexpr uint8_t IR_RIGHT = A1;

    constexpr uint8_t ULTRASOUND_FRONT_TRIG = 9;
    constexpr uint8_t ULTRASOUND_FRONT_ECHO = 10;

    constexpr uint8_t INDUCTIVE_PROXIMITY = 11;

} // namespace Pins


// ============================================================
// HERKULEX CONFIG
// ============================================================

namespace HerkulexConfig {

    // Teensy hardware UART used for all Herkulex servos.
    inline HardwareSerial& SERIAL = Serial1;

    constexpr uint32_t BAUD = 115200;


    // --------------------------------------------------------
    // Servo IDs
    // --------------------------------------------------------

    namespace Id {

        constexpr uint8_t SORTING_GATE = 1;
        constexpr uint8_t KICKER_CAM   = 2;

    } // namespace Id


    // --------------------------------------------------------
    // Mechanical / positional limits
    //
    // Current Herkulex library uses positional mode and clamps
    // commands to approximately +/-160 degrees.
    // --------------------------------------------------------

    constexpr float MIN_ANGLE_DEG = -160.0f;
    constexpr float MAX_ANGLE_DEG =  160.0f;


    // --------------------------------------------------------
    // Default movement times
    // --------------------------------------------------------

    constexpr uint16_t DEFAULT_MOVE_TIME_MS = 500;

    constexpr uint16_t SORTING_GATE_MOVE_TIME_MS = 300;
    constexpr uint16_t KICKER_CAM_MOVE_TIME_MS   = 250;


    // --------------------------------------------------------
    // Library / servo startup timing
    // --------------------------------------------------------

    constexpr uint16_t REBOOT_DELAY_MS      = 700;
    constexpr uint16_t CLEAR_ERROR_DELAY_MS = 100;
    constexpr uint16_t TORQUE_ON_DELAY_MS   = 100;


    // --------------------------------------------------------
    // Sorting gate positions
    //
    // Placeholder values.
    // Tune these once the actual mechanism is assembled.
    // --------------------------------------------------------

    namespace SortingGate {

        constexpr float LEFT_DEG   = -45.0f;
        constexpr float CENTRE_DEG =   0.0f;
        constexpr float RIGHT_DEG  =  45.0f;

    } // namespace SortingGate


    // --------------------------------------------------------
    // Kicker cam positions
    //
    // Placeholder values.
    // Tune these once the actual mechanism is assembled.
    // --------------------------------------------------------

    namespace KickerCam {

        constexpr float HOME_DEG = 0.0f;
        constexpr float KICK_DEG = 90.0f;

    } // namespace KickerCam

} // namespace HerkulexConfig


// ============================================================
// TOF SENSOR CONFIG
// ============================================================
//
// TOF sensor XSHUT pins are controlled using the onboard
// SX1509 IO expander rather than raw Teensy GPIO.
//
// Placeholder counts / expander pins.
// Confirm against the final sensor layout.
// ============================================================

namespace TofConfig {

    constexpr uint8_t EXPANDER_I2C_ADDRESS = 0x3F;

    // New addresses assigned to sensors after startup.
    constexpr uint8_t VL53L0X_ADDRESS_START = 0x30; // short-range
    constexpr uint8_t VL53L1X_ADDRESS_START = 0x35; // long-range


    // --------------------------------------------------------
    // Short-range TOF sensors
    // --------------------------------------------------------

    constexpr uint8_t SHORT_COUNT = 2;

    constexpr uint8_t SHORT_XSHUT_PINS[SHORT_COUNT] = {
        0,
        1
    };


    // --------------------------------------------------------
    // Long-range TOF sensors
    // --------------------------------------------------------

    constexpr uint8_t LONG_COUNT = 2;

    constexpr uint8_t LONG_XSHUT_PINS[LONG_COUNT] = {
        2,
        3
    };

} // namespace TofConfig


// ============================================================
// IR SENSOR CONFIG
// ============================================================
//
// The team's bench testing found analogue IR distance readings
// too noisy to trust for accurate ranging, particularly when
// objects are viewed off-angle.
//
// Prefer using these as object-presence sensors where possible.
// ============================================================

namespace IrConfig {

    constexpr uint8_t COUNT = 2;

    constexpr uint8_t PINS[COUNT] = {
        Pins::IR_LEFT,
        Pins::IR_RIGHT
    };

    // Raw ADC threshold above which an object is considered
    // present.
    //
    // Tune this using the actual mounted sensors.
    constexpr uint16_t PRESENCE_THRESHOLD = 400;

} // namespace IrConfig