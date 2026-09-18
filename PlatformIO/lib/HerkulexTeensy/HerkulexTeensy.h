#pragma once

#include <Arduino.h>

class HerkulexTeensy {
public:
    enum Led : uint8_t {
        LED_OFF   = 0x00,
        LED_GREEN = 0x04,
        LED_BLUE  = 0x08,
        LED_RED   = 0x10
    };

    // Pass Serial1 / Serial2 / Serial3 / Serial4 etc.
    HerkulexTeensy(HardwareSerial& serial, uint32_t baud = 115200);

    void begin();
    void end();

    void reboot(uint8_t id);
    void clearError(uint8_t id);
    void torqueOn(uint8_t id);
    void torqueOff(uint8_t id);

    void movePosition(
        uint8_t id,
        uint16_t position,
        uint16_t playTimeMs = 1000,
        Led led = LED_OFF
    );

    void moveAngle(
        uint8_t id,
        float angleDeg,
        uint16_t playTimeMs = 1000,
        Led led = LED_OFF
    );

    // DRS-series continuous-turn control. Speed is signed -1023..1023;
    // negative and positive values select opposite directions.
    void moveVelocity(
        uint8_t id,
        int16_t speed,
        uint16_t playTimeMs = 0,
        Led led = LED_OFF
    );

    void initializeServo(uint8_t id, bool rebootFirst = true);
    void initializeAll();

    int readPosition(uint8_t id, uint16_t timeoutMs = 10);
    float readAngle(uint8_t id, uint16_t timeoutMs = 10);
    int readStatus(uint8_t id, uint16_t timeoutMs = 10);

    void flushRx();

private:
    HardwareSerial& _serial;
    uint32_t _baud;

    static constexpr uint8_t CMD_RAM_WRITE = 0x03;
    static constexpr uint8_t CMD_RAM_READ  = 0x04;
    static constexpr uint8_t CMD_S_JOG     = 0x06;
    static constexpr uint8_t CMD_STAT      = 0x07;
    static constexpr uint8_t CMD_REBOOT    = 0x09;

    uint8_t checksum1(const uint8_t* packet, size_t packetSize) const;
    void sendPacket(uint8_t* packet, size_t size);

    void writeRam(uint8_t id, uint8_t address, uint8_t value);
    void writeRam(uint8_t id, uint8_t address, const uint8_t* data, uint8_t length);

    bool readPacket(uint8_t* buffer, size_t expectedSize, uint16_t timeoutMs);
    bool validatePacket(const uint8_t* packet, size_t size) const;
};
