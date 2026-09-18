#pragma once

#include <Arduino.h>
#include <ArduinoJson.h>

class RobotDebug {
public:
    using MessageHandler = void (*)(JsonDocument& message, void* context);

    RobotDebug(Stream& stream, MessageHandler handler, void* context = nullptr);
    void update();
    void log(const char* level, const char* message);
    void error(const char* message);
    void state(bool debugMode, bool stopped, bool fault = false);
    void parameterValue(const char* name, float value);
    void telemetry(const char* name, float value);
    void send(JsonDocument& message);

private:
    static constexpr size_t TX_BUFFER_SIZE = 2048;
    static constexpr size_t TX_CHUNK_SIZE = 20;
    Stream& stream_;
    MessageHandler handler_;
    void* context_;
    char buffer_[768] = {};
    char txBuffer_[TX_BUFFER_SIZE] = {};
    size_t length_ = 0;
    bool droppingLine_ = false;

    void finishLine();
};
