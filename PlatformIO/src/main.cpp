#include <Arduino.h>
#include <ArduinoJson.h>
#include <HerkulexTeensy.h>
#include <RobotDebug.h>
#include <config.h>

namespace {

bool debugMode = false;
bool stopped = true;
HerkulexTeensy servos(HerkulexConfig::PORT, HerkulexConfig::BAUD);
uint8_t lastServoId = HerkulexConfig::Id::SORTING_GATE;
float lastServoAngleDeg = 0.0f;
uint32_t telemetryIntervalMs = BluetoothConfig::TELEMETRY_INTERVAL_MS;
uint32_t lastTelemetryMs = 0;
uint32_t lastDefinitionsMs = 0;
uint32_t receivedMessages = 0;

void handleBluetoothMessage(JsonDocument& message);
RobotDebug debugLink(BluetoothConfig::PORT, handleBluetoothMessage);

void sendState()
{
    debugLink.state(debugMode, stopped);
}

void sendDefinitions()
{
    JsonDocument parameter;
    parameter["type"] = "parameter_definition";
    parameter["name"] = "debug.telemetry_interval_ms";
    parameter["label"] = "Telemetry interval";
    parameter["description"] = "Delay between telemetry packets";
    parameter["datatype"] = "int";
    parameter["unit"] = "ms";
    parameter["value"] = telemetryIntervalMs;
    parameter["min"] = 50;
    parameter["max"] = 2000;
    parameter["step"] = 50;
    debugLink.send(parameter);
    delay(40);

    JsonDocument servoCommand;
    servoCommand["type"] = "command_definition";
    servoCommand["name"] = "set_herkulex_angle";
    servoCommand["label"] = "Set Herkulex angle";
    JsonArray servoArgs = servoCommand["args"].to<JsonArray>();
    JsonObject servoId = servoArgs.add<JsonObject>();
    servoId["name"] = "id";
    servoId["type"] = "int";
    servoId["min"] = 1;
    servoId["max"] = 253;
    servoId["default"] = HerkulexConfig::Id::SORTING_GATE;
    JsonObject angle = servoArgs.add<JsonObject>();
    angle["name"] = "angle_deg";
    angle["type"] = "float";
    angle["min"] = HerkulexConfig::MIN_ANGLE_DEG;
    angle["max"] = HerkulexConfig::MAX_ANGLE_DEG;
    angle["step"] = 1;
    angle["default"] = 0;
    JsonObject moveTime = servoArgs.add<JsonObject>();
    moveTime["name"] = "move_time_ms";
    moveTime["type"] = "int";
    moveTime["min"] = 50;
    moveTime["max"] = 2850;
    moveTime["step"] = 50;
    moveTime["default"] = HerkulexConfig::DEFAULT_MOVE_TIME_MS;
    debugLink.send(servoCommand);
    delay(40);

    const char* commands[] = {"ping", "stop"};
    for (const char* name : commands)
    {
        JsonDocument definition;
        definition["type"] = "command_definition";
        definition["name"] = name;
        definition["label"] = strcmp(name, "ping") == 0 ? "Ping Teensy" : "Stop Robot";
        definition["args"].to<JsonArray>();
        debugLink.send(definition);
        delay(40);
    }
    lastDefinitionsMs = millis();
}

void handleCommand(JsonDocument& message)
{
    const char* command = message["command"] | "";

    if (strcmp(command, "set_debug_mode") == 0)
    {
        debugMode = message["enabled"] | false;
        if (!debugMode)
        {
            stopped = true;
            servos.torqueOff(0xFE);
        }
        sendState();
        debugLink.log("INFO", debugMode ? "Debug mode enabled" : "Debug mode disabled");
    }
    else if (strcmp(command, "ping") == 0)
    {
        debugLink.log("INFO", "Pong from Teensy");
        sendState();
    }
    else if (strcmp(command, "set_herkulex_angle") == 0)
    {
        if (!debugMode)
        {
            debugLink.error("Set Herkulex angle requires Debug Mode");
            return;
        }

        if (!message["id"].is<int>() ||
            (!message["angle_deg"].is<float>() && !message["angle_deg"].is<int>()) ||
            !message["move_time_ms"].is<int>())
        {
            debugLink.error("Servo command requires id, angle_deg, and move_time_ms");
            return;
        }

        const int requestedId = message["id"].as<int>();
        const float requestedAngle = message["angle_deg"].as<float>();
        const int requestedTime = message["move_time_ms"].as<int>();
        if (requestedId < 1 || requestedId > 253 ||
            requestedAngle < HerkulexConfig::MIN_ANGLE_DEG ||
            requestedAngle > HerkulexConfig::MAX_ANGLE_DEG ||
            requestedTime < 50 || requestedTime > 2850)
        {
            debugLink.error("Servo command is outside its safe range");
            return;
        }

        lastServoId = static_cast<uint8_t>(requestedId);
        lastServoAngleDeg = requestedAngle;
        servos.clearError(lastServoId);
        servos.torqueOn(lastServoId);
        servos.moveAngle(
            lastServoId,
            lastServoAngleDeg,
            static_cast<uint16_t>(requestedTime),
            HerkulexTeensy::LED_BLUE
        );
        stopped = false;
        sendState();
        debugLink.log("INFO", "Herkulex angle command sent");
    }
    else if (strcmp(command, "stop") == 0)
    {
        servos.torqueOff(0xFE);
        stopped = true;
        sendState();
        debugLink.log("WARNING", "STOP received");
    }
    else
    {
        debugLink.error("Unknown command");
    }
}

void handleBluetoothMessage(JsonDocument& message)
{
    ++receivedMessages;
    const char* type = message["type"] | "";

    if (strcmp(type, "hello") == 0)
    {
        debugLink.log("INFO", "Robot Debug GUI connected");
        sendState();
        // Advertise immediately as well as handling request_definitions. This
        // keeps startup reliable on transparent radio links that may lose one
        // of two back-to-back packets while the connection settles.
        sendDefinitions();
    }
    else if (strcmp(type, "request_definitions") == 0)
    {
        if (millis() - lastDefinitionsMs >= 500)
            sendDefinitions();
    }
    else if (strcmp(type, "command") == 0)
    {
        handleCommand(message);
    }
    else if (strcmp(type, "parameter") == 0)
    {
        const char* name = message["name"] | "";
        if (strcmp(name, "debug.telemetry_interval_ms") != 0 || !message["value"].is<int>())
        {
            debugLink.error("Unknown parameter or invalid value");
            return;
        }
        telemetryIntervalMs = static_cast<uint32_t>(constrain(message["value"].as<int>(), 50, 2000));
        debugLink.parameterValue(name, telemetryIntervalMs);
    }
    else if (strcmp(type, "parameter_request") == 0)
    {
        const char* name = message["name"] | "";
        if (strcmp(name, "debug.telemetry_interval_ms") == 0)
            debugLink.parameterValue(name, telemetryIntervalMs);
        else
            debugLink.error("Unknown parameter");
    }
    else
    {
        debugLink.error("Unknown message type");
    }
}

void sendTelemetry()
{
    const uint32_t now = millis();
    if (now - lastTelemetryMs < telemetryIntervalMs)
        return;
    lastTelemetryMs = now;

    JsonDocument message;
    message["type"] = "telemetry";
    message["time"] = now;
    JsonObject data = message["data"].to<JsonObject>();
    data["system.uptime_s"] = now / 1000.0f;
    data["system.debug_mode"] = debugMode;
    data["system.stopped"] = stopped;
    data["bluetooth.messages_received"] = receivedMessages;
    data["servo.last_id"] = lastServoId;
    data["servo.commanded_angle_deg"] = lastServoAngleDeg;
    debugLink.send(message);
}

} // namespace

void setup()
{
    BluetoothConfig::PORT.begin(BluetoothConfig::BAUD);
    servos.begin();
    servos.torqueOff(0xFE);
    delay(100);
    debugLink.log("INFO", "Teensy Bluetooth link ready on Serial1");
    sendState();
}

void loop()
{
    debugLink.update();
    sendTelemetry();
}
