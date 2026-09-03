#include <Arduino.h>
#include <HerkulexTeensy.h>

static const uint8_t SERVO_ID = 0x04;

// Teensy 4.0 Serial4:
// RX = pin 16
// TX = pin 17
//
// This lets you keep the same physical pins
// you were using on the ESP32.
HerkulexTeensy herkulex(Serial4, 115200);

void setup()
{
    Serial.begin(115200);
    delay(1500);

    herkulex.begin();
    herkulex.initializeServo(SERVO_ID);
}

void loop()
{
    herkulex.moveAngle(
        SERVO_ID,
        -100.0f,
        250,
        HerkulexTeensy::LED_BLUE
    );

    delay(1000);

    herkulex.moveAngle(
        SERVO_ID,
        -150.0f,
        500,
        HerkulexTeensy::LED_GREEN
    );

    delay(1000);

    herkulex.moveAngle(
        SERVO_ID,
        0.0f,
        1000,
        HerkulexTeensy::LED_RED
    );

    delay(1000);

    herkulex.moveAngle(
        SERVO_ID,
        100.0f,
        250,
        HerkulexTeensy::LED_BLUE
    );

    delay(1000);

    herkulex.moveAngle(
        SERVO_ID,
        150.0f,
        500,
        HerkulexTeensy::LED_GREEN
    );

    delay(1000);

    herkulex.moveAngle(
        SERVO_ID,
        0.0f,
        1000,
        HerkulexTeensy::LED_RED
    );

    delay(1000);
}
