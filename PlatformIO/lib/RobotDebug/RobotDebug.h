#pragma once

#include <Arduino.h>
#include <ArduinoJson.h>

class RobotDebug {
public:
    using MessageHandler = void (*)(JsonDocument& message);

    RobotDebug(Stream& stream, MessageHandler handler);
    void update();
    void log(const char* level, const char* message);
    void error(const char* message);
    void state(bool debugMode, bool stopped, bool fault = false);
    void parameterValue(const char* name, float value);
    void send(JsonDocument& message);

private:
    Stream& stream_;
    MessageHandler handler_;
    char buffer_[768] = {};
    size_t length_ = 0;
    bool droppingLine_ = false;

    void finishLine();
};
