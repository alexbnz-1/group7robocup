#include "RobotDebug.h"

RobotDebug::RobotDebug(Stream& stream, MessageHandler handler, void* context)
    : stream_(stream), handler_(handler), context_(context)
{
}

void RobotDebug::update()
{
    while (stream_.available() > 0)
    {
        const char c = static_cast<char>(stream_.read());
        if (c == '\n')
        {
            finishLine();
            continue;
        }
        if (c == '\r' || droppingLine_)
            continue;
        if (length_ >= sizeof(buffer_) - 1)
        {
            length_ = 0;
            droppingLine_ = true;
            continue;
        }
        buffer_[length_++] = c;
    }
}

void RobotDebug::finishLine()
{
    if (droppingLine_)
    {
        droppingLine_ = false;
        error("Incoming message exceeded 767 bytes");
        return;
    }
    if (length_ == 0)
        return;

    buffer_[length_] = '\0';
    JsonDocument message;
    const DeserializationError result = deserializeJson(message, buffer_);
    length_ = 0;
    if (result)
    {
        error("Malformed JSON message");
        return;
    }
    if (!message.is<JsonObject>())
    {
        error("Message must be a JSON object");
        return;
    }
    if (handler_ != nullptr)
        handler_(message, context_);
}

void RobotDebug::send(JsonDocument& message)
{
    const size_t required = measureJson(message);
    if (required >= sizeof(txBuffer_))
        return;

    const size_t length = serializeJson(message, txBuffer_, sizeof(txBuffer_));
    size_t offset = 0;
    while (offset < length)
    {
        const size_t remaining = length - offset;
        const size_t chunkLength = remaining < TX_CHUNK_SIZE
            ? remaining
            : TX_CHUNK_SIZE;
        stream_.write(
            reinterpret_cast<const uint8_t*>(txBuffer_ + offset),
            chunkLength
        );
        offset += chunkLength;
        if (offset < length)
            delay(3);
    }
    stream_.write('\n');
    stream_.flush();
}

void RobotDebug::log(const char* level, const char* text)
{
    JsonDocument message;
    message["type"] = "log";
    message["level"] = level;
    message["message"] = text;
    send(message);
}

void RobotDebug::error(const char* text)
{
    JsonDocument message;
    message["type"] = "error";
    message["message"] = text;
    send(message);
}

void RobotDebug::state(bool debugMode, bool stopped, bool fault)
{
    JsonDocument message;
    message["type"] = "state";
    message["debug_mode"] = debugMode;
    message["stopped"] = stopped;
    message["fault"] = fault;
    message["uptime_ms"] = millis();
    send(message);
}

void RobotDebug::parameterValue(const char* name, float value)
{
    JsonDocument message;
    message["type"] = "parameter_value";
    message["name"] = name;
    message["value"] = value;
    send(message);
}

void RobotDebug::telemetry(const char* name, float value)
{
    JsonDocument message;
    message["type"] = "telemetry";
    message["time"] = millis();
    message["name"] = name;
    message["value"] = value;
    send(message);
}
