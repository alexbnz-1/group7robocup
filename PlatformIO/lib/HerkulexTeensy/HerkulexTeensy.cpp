#include "HerkulexTeensy.h"

HerkulexTeensy::HerkulexTeensy(HardwareSerial& serial, uint32_t baud)
    : _serial(serial), _baud(baud)
{
}

void HerkulexTeensy::begin()
{
    _serial.begin(_baud);
    flushRx();
}

void HerkulexTeensy::end()
{
    _serial.end();
}

uint8_t HerkulexTeensy::checksum1(const uint8_t* packet, size_t packetSize) const
{
    uint8_t checksum = 0;

    checksum ^= packet[2];
    checksum ^= packet[3];
    checksum ^= packet[4];

    for (size_t i = 7; i < packetSize; i++)
    {
        checksum ^= packet[i];
    }

    return checksum & 0xFE;
}

void HerkulexTeensy::sendPacket(uint8_t* packet, size_t size)
{
    packet[5] = checksum1(packet, size);
    packet[6] = (~packet[5]) & 0xFE;

    for (size_t i = 0; i < size; i++)
    {
        _serial.write(packet[i]);
    }

    _serial.flush();
}

void HerkulexTeensy::flushRx()
{
    while (_serial.available())
    {
        _serial.read();
    }
}

void HerkulexTeensy::writeRam(
    uint8_t id,
    uint8_t address,
    uint8_t value
)
{
    writeRam(id, address, &value, 1);
}

void HerkulexTeensy::writeRam(
    uint8_t id,
    uint8_t address,
    const uint8_t* data,
    uint8_t length
)
{
    const size_t size = 9 + length;

    uint8_t packet[32];

    if (size > sizeof(packet))
        return;

    packet[0] = 0xFF;
    packet[1] = 0xFF;
    packet[2] = (uint8_t)size;
    packet[3] = id;
    packet[4] = CMD_RAM_WRITE;
    packet[5] = 0x00;
    packet[6] = 0x00;
    packet[7] = address;
    packet[8] = length;

    for (uint8_t i = 0; i < length; i++)
    {
        packet[9 + i] = data[i];
    }

    sendPacket(packet, size);
}

void HerkulexTeensy::reboot(uint8_t id)
{
    uint8_t packet[7];

    packet[0] = 0xFF;
    packet[1] = 0xFF;
    packet[2] = 0x07;
    packet[3] = id;
    packet[4] = CMD_REBOOT;
    packet[5] = 0x00;
    packet[6] = 0x00;

    sendPacket(packet, sizeof(packet));
}

void HerkulexTeensy::clearError(uint8_t id)
{
    uint8_t data[2] = {
        0x00,
        0x00
    };

    writeRam(
        id,
        0x30,
        data,
        2
    );
}

void HerkulexTeensy::torqueOn(uint8_t id)
{
    writeRam(
        id,
        0x34,
        0x60
    );
}

void HerkulexTeensy::torqueOff(uint8_t id)
{
    writeRam(
        id,
        0x34,
        0x00
    );
}

void HerkulexTeensy::movePosition(
    uint8_t id,
    uint16_t position,
    uint16_t playTimeMs,
    Led led
)
{
    if (position > 1023)
        position = 1023;

    if (playTimeMs > 2856)
        playTimeMs = 2856;

    uint8_t playTime =
        (uint8_t)(playTimeMs / 11.2f);

    uint8_t posLSB =
        position & 0xFF;

    uint8_t posMSB =
        (position >> 8) & 0xFF;

    uint8_t setValue = 0x00;

    if (led & LED_GREEN)
        setValue |= 0x04;

    if (led & LED_BLUE)
        setValue |= 0x08;

    if (led & LED_RED)
        setValue |= 0x10;

    uint8_t packet[12];

    packet[0] = 0xFF;
    packet[1] = 0xFF;
    packet[2] = 0x0C;
    packet[3] = id;
    packet[4] = CMD_S_JOG;
    packet[5] = 0x00;
    packet[6] = 0x00;
    packet[7] = playTime;
    packet[8] = posLSB;
    packet[9] = posMSB;
    packet[10] = setValue;
    packet[11] = id;

    sendPacket(packet, sizeof(packet));
}

void HerkulexTeensy::moveAngle(
    uint8_t id,
    float angleDeg,
    uint16_t playTimeMs,
    Led led
)
{
    if (angleDeg > 160.0f)
        angleDeg = 160.0f;

    if (angleDeg < -160.0f)
        angleDeg = -160.0f;

    int position =
        (int)(angleDeg / 0.325f) + 512;

    if (position < 0)
        position = 0;

    if (position > 1023)
        position = 1023;

    movePosition(
        id,
        (uint16_t)position,
        playTimeMs,
        led
    );
}

void HerkulexTeensy::initializeServo(
    uint8_t id,
    bool rebootFirst
)
{
    if (rebootFirst)
    {
        reboot(id);
        delay(700);
    }

    clearError(id);
    delay(100);

    torqueOn(id);
    delay(100);
}

void HerkulexTeensy::initializeAll()
{
    clearError(0xFE);
    delay(100);

    torqueOn(0xFE);
    delay(100);
}

bool HerkulexTeensy::validatePacket(
    const uint8_t* packet,
    size_t size
) const
{
    if (size < 7)
        return false;

    if (packet[0] != 0xFF ||
        packet[1] != 0xFF)
        return false;

    if (packet[2] != size)
        return false;

    uint8_t c1 =
        checksum1(packet, size);

    uint8_t c2 =
        (~c1) & 0xFE;

    return
        packet[5] == c1 &&
        packet[6] == c2;
}

bool HerkulexTeensy::readPacket(
    uint8_t* buffer,
    size_t expectedSize,
    uint16_t timeoutMs
)
{
    uint32_t start =
        millis();

    size_t index = 0;

    while ((millis() - start) < timeoutMs)
    {
        while (_serial.available())
        {
            uint8_t b =
                (uint8_t)_serial.read();

            if (index == 0)
            {
                if (b == 0xFF)
                {
                    buffer[index++] = b;
                }

                continue;
            }

            if (index == 1)
            {
                if (b == 0xFF)
                {
                    buffer[index++] = b;
                }
                else
                {
                    index = 0;
                }

                continue;
            }

            buffer[index++] = b;

            if (index >= expectedSize)
            {
                return validatePacket(
                    buffer,
                    expectedSize
                );
            }
        }
    }

    return false;
}

int HerkulexTeensy::readPosition(
    uint8_t id,
    uint16_t timeoutMs
)
{
    flushRx();

    uint8_t packet[9];

    packet[0] = 0xFF;
    packet[1] = 0xFF;
    packet[2] = 0x09;
    packet[3] = id;
    packet[4] = CMD_RAM_READ;
    packet[5] = 0x00;
    packet[6] = 0x00;
    packet[7] = 0x3A;
    packet[8] = 0x02;

    sendPacket(
        packet,
        sizeof(packet)
    );

    uint8_t response[13];

    if (!readPacket(
        response,
        sizeof(response),
        timeoutMs
    ))
    {
        return -1;
    }

    return
        ((response[10] & 0x03) << 8) |
        response[9];
}

float HerkulexTeensy::readAngle(
    uint8_t id,
    uint16_t timeoutMs
)
{
    int position =
        readPosition(
            id,
            timeoutMs
        );

    if (position < 0)
        return NAN;

    return
        (position - 512) *
        0.325f;
}

int HerkulexTeensy::readStatus(
    uint8_t id,
    uint16_t timeoutMs
)
{
    flushRx();

    uint8_t packet[7];

    packet[0] = 0xFF;
    packet[1] = 0xFF;
    packet[2] = 0x07;
    packet[3] = id;
    packet[4] = CMD_STAT;
    packet[5] = 0x00;
    packet[6] = 0x00;

    sendPacket(
        packet,
        sizeof(packet)
    );

    uint8_t response[9];

    if (!readPacket(
        response,
        sizeof(response),
        timeoutMs
    ))
    {
        return -1;
    }

    return response[7];
}
