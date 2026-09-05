#include <Arduino.h>
#include <HerkulexTeensy.h>

static const uint8_t SERVO_ID = 0x01;

// Teensy 4.0 Serial1:
// RX = pin 0
// TX = pin 1
HerkulexTeensy herkulex(Serial1, 115200);

void setup()
{
    Serial.begin(115200);
    delay(1500);

    Serial.println("Starting Herkulex library test...");

    herkulex.begin();

    Serial.println("Initialising servo...");
    herkulex.initializeServo(SERVO_ID);

    Serial.println("Ready.");
}

void loop()
{
    // -100 degrees in 250 ms
    Serial.println("-100newcode");
    herkulex.moveAngle(
        SERVO_ID,
        -100.0f,
        250,
        HerkulexTeensy::LED_BLUE
    );

    delay(1000);


    // -150 degrees in 500 ms
    Serial.println("-150");
    herkulex.moveAngle(
        SERVO_ID,
        -150.0f,
        500,
        HerkulexTeensy::LED_GREEN
    );

    delay(1000);


    // Centre in 1000 ms
    Serial.println("0");
    herkulex.moveAngle(
        SERVO_ID,
        0.0f,
        1000,
        HerkulexTeensy::LED_RED
    );

    delay(1000);


    // +100 degrees in 250 ms
    Serial.println("+100");
    herkulex.moveAngle(
        SERVO_ID,
        100.0f,
        250,
        HerkulexTeensy::LED_BLUE
    );

    delay(1000);


    // +150 degrees in 500 ms
    Serial.println("+150");
    herkulex.moveAngle(
        SERVO_ID,
        150.0f,
        500,
        HerkulexTeensy::LED_GREEN
    );

    delay(1000);


    // Centre in 1000 ms
    Serial.println("0");
    herkulex.moveAngle(
        SERVO_ID,
        0.0f,
        1000,
        HerkulexTeensy::LED_RED
    );

    delay(1000);
}