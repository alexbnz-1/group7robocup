#include "BluetoothDebugWorkflow.h"

#include <config.h>
#include <debug_config.generated.h>

BluetoothDebugWorkflow::BluetoothDebugWorkflow(
    HardwareSerialIMXRT& bluetoothPort,
    HardwareSerial& herkulexPort
)
    : bluetoothPort_(bluetoothPort),
      servos_(herkulexPort, HerkulexConfig::BAUD),
      dcMotor203_(Pins::DC_MOTOR_203_CHANNEL_A, Pins::DC_MOTOR_203_CHANNEL_B),
      dcMotor203Second_(Pins::DC_MOTOR_203_SECOND_CHANNEL_A, Pins::DC_MOTOR_203_SECOND_CHANNEL_B),
      hx12kA_(Pins::HX12K_OUTPUT_A),
      hx12kB_(Pins::HX12K_OUTPUT_B),
      hx12kC_(Pins::HX12K_OUTPUT_C),
      hx12kD_(Pins::HX12K_OUTPUT_D),
      link_(bluetoothPort, dispatch, this),
      tof8x8_(Tof8x8Config::I2C_ADDRESS, Wire1)
{
}

void BluetoothDebugWorkflow::begin()
{
    // Teensy's default hardware-serial RX storage is too small for the longer
    // JSON motion commands when telemetry transmission temporarily delays the
    // parser. Extra storage prevents complete newline-terminated commands from
    // being truncated before link_.update() can consume them.
    bluetoothPort_.addMemoryForRead(bluetoothRxBuffer_, sizeof(bluetoothRxBuffer_));
    bluetoothPort_.begin(BluetoothConfig::BAUD);
    servos_.begin();
    servos_.torqueOff(0xFE);
    dcMotor203_.begin();
    dcMotor203Second_.begin();
    hx12kA_.begin();
    hx12kB_.begin();
    hx12kC_.begin();
    hx12kD_.begin();
    const DeserializationError error = deserializeJson(config_, EmbeddedDebugConfig::JSON);
    if (error)
    {
        link_.error("Embedded debug_config.json is invalid");
        return;
    }

    initialiseTofSensors();
    const bool tof8x8Ready = tof8x8_.begin();

    JsonObject interval = findParameter("debug.telemetry_interval_ms");
    if (!interval.isNull())
        telemetryIntervalMs_ = constrain(interval["value"] | 200UL, 50UL, 2000UL);

    delay(100);
    link_.log("INFO", "Teensy Bluetooth workflow ready");
    link_.log(tof8x8Ready ? "INFO" : "WARNING",
              tof8x8Ready ? "SEN0628 8x8 TOF ready" : "SEN0628 8x8 TOF not detected");
    sendState();
}

void BluetoothDebugWorkflow::update()
{
    link_.update();
    updateAutomaticServoRead();
    updateTof8x8();
    sendTelemetry();
}

void BluetoothDebugWorkflow::dispatch(JsonDocument& message, void* context)
{
    static_cast<BluetoothDebugWorkflow*>(context)->handleMessage(message);
}

void BluetoothDebugWorkflow::handleMessage(JsonDocument& message)
{
    ++receivedMessages_;
    const char* type = message["type"] | "";

    if (strcmp(type, "hello") == 0)
    {
        link_.log("INFO", "Robot Debug GUI connected");
        sendState();
        sendDefinitions();
    }
    else if (strcmp(type, "request_definitions") == 0)
    {
        if (millis() - lastDefinitionsMs_ >= 500)
            sendDefinitions();
    }
    else if (strcmp(type, "command") == 0)
    {
        handleCommand(message);
    }
    else if (strcmp(type, "parameter") == 0)
    {
        handleParameter(message);
    }
    else if (strcmp(type, "parameter_request") == 0)
    {
        const char* name = message["name"] | "";
        JsonObject parameter = findParameter(name);
        if (parameter.isNull())
            link_.error("Unknown parameter");
        else
            link_.parameterValue(name, parameter["value"].as<float>());
    }
    else
    {
        link_.error("Unknown message type");
    }
}

void BluetoothDebugWorkflow::handleCommand(JsonDocument& message)
{
    const char* commandName = message["command"] | "";

    // This command is built into the desktop GUI, so it does not need to be
    // repeated in debug_config.json.
    if (strcmp(commandName, "set_debug_mode") == 0)
    {
        debugMode_ = message["enabled"] | false;
        if (!debugMode_)
        {
            stopped_ = true;
            servos_.torqueOff(0xFE);
            dcMotor203_.stop();
            dcMotor203Active_ = false;
            dcMotor203Second_.stop();
            dcMotor203SecondActive_ = false;
            hx12kA_.disable();
            hx12kB_.disable();
            hx12kC_.disable();
            hx12kD_.disable();
        }
        sendState();
        link_.log("INFO", debugMode_ ? "Debug mode enabled" : "Debug mode disabled; torque off");
        return;
    }

    JsonObject definition = findCommand(commandName);
    if (definition.isNull())
    {
        link_.error("Unknown command");
        return;
    }

    if ((definition["debug_mode_required"] | false) && !debugMode_)
    {
        link_.error("Command requires Debug Mode");
        return;
    }

    const char* action = definition["action"] | "";
    if (strcmp(action, "ping") == 0)
    {
        link_.log("INFO", "Pong from Teensy");
        sendState();
    }
    else if (strcmp(action, "stop") == 0)
    {
        servos_.torqueOff(0xFE);
        continuousVelocityActive_ = false;
        commandedVelocity_ = 0;
        dcMotor203_.stop();
        dcMotor203Active_ = false;
        dcMotor203Second_.stop();
        dcMotor203SecondActive_ = false;
        hx12kA_.disable();
        hx12kB_.disable();
        hx12kC_.disable();
        hx12kD_.disable();
        stopped_ = true;
        sendState();
        link_.log("WARNING", "STOP received; servo torque disabled");
    }
    else if (strcmp(action, "run") == 0)
    {
        stopped_ = false;
        trackingFault_ = false;
        consecutiveTrackingErrors_ = 0;
        sendState();
        link_.log("INFO", "Run mode enabled; servo remains still until commanded");
    }
    else if (strcmp(action, "herkulex_angle") == 0)
    {
        if (!message["id"].is<int>() ||
            (!message["angle_deg"].is<float>() && !message["angle_deg"].is<int>()) ||
            !message["move_time_ms"].is<int>())
        {
            link_.error("Servo command requires id, angle_deg, and move_time_ms");
            return;
        }

        const int id = message["id"].as<int>();
        const float angle = message["angle_deg"].as<float>();
        const int moveTime = message["move_time_ms"].as<int>();
        if (id < 1 || id > 253 ||
            angle < HerkulexConfig::MIN_ANGLE_DEG || angle > HerkulexConfig::MAX_ANGLE_DEG ||
            moveTime < 50 || moveTime > 2850)
        {
            link_.error("Servo command is outside its safe range");
            return;
        }

        if (!servoZeroed_[id])
        {
            link_.error("Servo must be zeroed before it can move");
            return;
        }

        if (stopped_)
        {
            link_.error("Robot is stopped; press Run Robot before moving");
            return;
        }

        const float absoluteAngle = servoZeroOffsetsDeg_[id] + angle;
        if (absoluteAngle < HerkulexConfig::MIN_ANGLE_DEG ||
            absoluteAngle > HerkulexConfig::MAX_ANGLE_DEG)
        {
            link_.error("Relative target exceeds the servo's absolute range");
            return;
        }

        lastServoId_ = static_cast<uint8_t>(id);
        lastServoAngleDeg_ = angle;
        lastMoveStartMs_ = millis();
        lastMoveDurationMs_ = static_cast<uint16_t>(moveTime);
        trackingFault_ = false;
        consecutiveTrackingErrors_ = 0;
        // The DRS-0101 must be torque-free while changing from continuous-turn
        // mode back to position mode. The first JOG selects the mode; the
        // second executes after torque is restored.
        servos_.torqueOff(lastServoId_);
        delay(5);
        servos_.clearError(lastServoId_);
        servos_.moveAngle(lastServoId_, absoluteAngle, static_cast<uint16_t>(moveTime), HerkulexTeensy::LED_BLUE);
        delay(5);
        servos_.torqueOn(lastServoId_);
        delay(5);
        servos_.moveAngle(lastServoId_, absoluteAngle, static_cast<uint16_t>(moveTime), HerkulexTeensy::LED_BLUE);
        continuousVelocityActive_ = false;
        commandedVelocity_ = 0;
        stopped_ = false;
        sendState();
        link_.log("INFO", "Herkulex angle command sent");
    }
    else if (strcmp(action, "herkulex_read_angle") == 0)
    {
        const int id = message["id"] | -1;
        if (id < 1 || id > 253)
        {
            link_.error("Read angle requires a servo id from 1 to 253");
            return;
        }

        const float angle = servos_.readAngle(static_cast<uint8_t>(id), 30);
        if (isnan(angle))
        {
            link_.error("No valid angle response from Herkulex servo");
            return;
        }

        lastServoId_ = static_cast<uint8_t>(id);
        measuredServoAngleDeg_ = servoZeroed_[id]
            ? angle - servoZeroOffsetsDeg_[id]
            : angle;
        autoReadServoId_ = static_cast<uint8_t>(id);
        autoReadEnabled_ = true;
        servoResponding_ = true;
        consecutiveReadFailures_ = 0;
        link_.telemetry("servo.measured_angle_deg", measuredServoAngleDeg_);
        link_.log("INFO", "Herkulex angle read successfully");
    }
    else if (strcmp(action, "herkulex_zero_here") == 0)
    {
        const int id = message["id"] | -1;
        if (id < 1 || id > 253)
        {
            link_.error("Zero command requires a servo id from 1 to 253");
            return;
        }
        if (continuousVelocityActive_ && id == lastServoId_)
        {
            link_.error("Stop continuous rotation before setting zero");
            return;
        }

        // Reading does not enable torque or move the mechanism.
        const float currentAngle = servos_.readAngle(static_cast<uint8_t>(id), 30);
        if (isnan(currentAngle))
        {
            link_.error("Cannot zero: no valid response from Herkulex servo");
            return;
        }

        servoZeroOffsetsDeg_[id] = currentAngle;
        servoZeroed_[id] = true;
        lastServoId_ = static_cast<uint8_t>(id);
        lastServoAngleDeg_ = 0.0f;
        measuredServoAngleDeg_ = 0.0f;
        link_.telemetry("servo.measured_angle_deg", 0.0f);
        link_.telemetry("servo.zero_offset_deg", currentAngle);
        link_.log("INFO", "Current Herkulex position stored as software zero");
    }
    else if (strcmp(action, "herkulex_velocity") == 0)
    {
        if (stopped_)
        {
            link_.error("Robot is stopped; press Run Robot before turning");
            return;
        }

        const int id = message["id"] | -1;
        const int speed = message["speed"] | 0;
        if (id < 1 || id > 253 || speed < -1023 || speed > 1023)
        {
            link_.error("Velocity command requires id 1..253 and speed -1023..1023");
            return;
        }

        lastServoId_ = static_cast<uint8_t>(id);
        commandedVelocity_ = static_cast<int16_t>(speed);
        servos_.torqueOff(lastServoId_);
        delay(5);
        servos_.clearError(lastServoId_);
        servos_.moveVelocity(lastServoId_, 0, 0, HerkulexTeensy::LED_GREEN);
        delay(5);
        servos_.torqueOn(lastServoId_);
        delay(5);
        servos_.moveVelocity(lastServoId_, commandedVelocity_, 0, HerkulexTeensy::LED_GREEN);
        continuousVelocityActive_ = commandedVelocity_ != 0;
        trackingFault_ = false;
        consecutiveTrackingErrors_ = 0;
        sendState();
        link_.log("WARNING", continuousVelocityActive_
            ? "Herkulex continuous rotation started"
            : "Herkulex continuous velocity set to zero");
    }
    else if (strcmp(action, "herkulex_stop_velocity") == 0)
    {
        const int id = message["id"] | -1;
        if (id < 1 || id > 253)
        {
            link_.error("Stop turn requires a servo id from 1 to 253");
            return;
        }

        servos_.moveVelocity(static_cast<uint8_t>(id), 0);
        delay(5);
        servos_.torqueOff(static_cast<uint8_t>(id));
        if (id == lastServoId_)
        {
            continuousVelocityActive_ = false;
            commandedVelocity_ = 0;
        }
        link_.log("INFO", "Herkulex continuous rotation stopped; torque off");
    }
    else if (strcmp(action, "dc_motor_203_speed") == 0)
    {
        if (stopped_)
        {
            link_.error("Robot is stopped; press Run Robot before driving motor");
            return;
        }

        const int channelA = message["channel_a_percent"] | 0;
        const int channelB = message["channel_b_percent"] | 0;
        if (channelA < -100 || channelA > 100 || channelB < -100 || channelB > 100)
        {
            link_.error("Both 203 DC motor speeds must be from -100 to 100 percent");
            return;
        }

        dcMotor203_.setPercent(static_cast<int16_t>(channelA), static_cast<int16_t>(channelB));
        dcMotor203Active_ = channelA != 0 || channelB != 0;
        link_.log(dcMotor203Active_ ? "WARNING" : "INFO",
                  dcMotor203Active_ ? "203 DC motor command applied" : "203 DC motor stopped");
    }
    else if (strcmp(action, "dc_motor_203_stop") == 0)
    {
        dcMotor203_.stop();
        dcMotor203Active_ = false;
        link_.log("INFO", "203 DC motor stopped at neutral pulse");
    }
    else if (strcmp(action, "dc_motor_203_second_speed") == 0)
    {
        if (stopped_)
        {
            link_.error("Robot is stopped; press Run Robot before driving motor");
            return;
        }

        const int channelA = message["channel_a_percent"] | 0;
        const int channelB = message["channel_b_percent"] | 0;
        if (channelA < -100 || channelA > 100 || channelB < -100 || channelB > 100)
        {
            link_.error("Both second 203 DC motor speeds must be from -100 to 100 percent");
            return;
        }

        dcMotor203Second_.setPercent(static_cast<int16_t>(channelA), static_cast<int16_t>(channelB));
        dcMotor203SecondActive_ = channelA != 0 || channelB != 0;
        link_.log(dcMotor203SecondActive_ ? "WARNING" : "INFO",
                  dcMotor203SecondActive_ ? "Second 203 DC motor command applied" : "Second 203 DC motor stopped");
    }
    else if (strcmp(action, "dc_motor_203_second_stop") == 0)
    {
        dcMotor203Second_.stop();
        dcMotor203SecondActive_ = false;
        link_.log("INFO", "Second 203 DC motor stopped at neutral pulse");
    }
    else if (strcmp(action, "hx12k_angles") == 0)
    {
        if (stopped_)
        {
            link_.error("Robot is stopped; press Run Robot before moving HX12K servos");
            return;
        }

        const bool selectA = message["select_a"] | false;
        const bool selectB = message["select_b"] | false;
        const bool selectC = message["select_c"] | false;
        const bool selectD = message["select_d"] | false;
        if (!selectA && !selectB && !selectC && !selectD)
        {
            link_.error("Select at least one HX12K servo");
            return;
        }

        const float angleA = message["angle_a_deg"] | 67.5f;
        const float angleB = message["angle_b_deg"] | 67.5f;
        const float angleC = message["angle_c_deg"] | 67.5f;
        const float angleD = message["angle_d_deg"] | 67.5f;
        if (angleA < 0.0f || angleA > 135.0f ||
            angleB < 0.0f || angleB > 135.0f ||
            angleC < 0.0f || angleC > 135.0f ||
            angleD < 0.0f || angleD > 135.0f)
        {
            link_.error("Every HX12K angle must be from 0 to 135 degrees");
            return;
        }

        if (selectA) hx12kA_.setAngle(angleA);
        if (selectB) hx12kB_.setAngle(angleB);
        if (selectC) hx12kC_.setAngle(angleC);
        if (selectD) hx12kD_.setAngle(angleD);
        link_.log("INFO", "Selected HX12K angles applied");
        sendState();
    }
    else if (strcmp(action, "hx12k_disable") == 0)
    {
        hx12kA_.disable();
        hx12kB_.disable();
        hx12kC_.disable();
        hx12kD_.disable();
        link_.log("INFO", "All HX12K pulse outputs disabled");
    }
    else if (strcmp(action, "tof_read_all") == 0)
    {
        lastTelemetryMs_ = millis() - telemetryIntervalMs_;
        sendTelemetry();
        link_.log("INFO", "Configured TOF sensors read");
    }
    else
    {
        link_.error("Command action has no firmware handler");
    }
}

void BluetoothDebugWorkflow::handleParameter(JsonDocument& message)
{
    const char* name = message["name"] | "";
    JsonObject parameter = findParameter(name);
    if (parameter.isNull() || message["value"].isNull())
    {
        link_.error("Unknown parameter or missing value");
        return;
    }

    const char* datatype = parameter["datatype"] | "float";
    if (strcmp(datatype, "int") == 0)
    {
        if (!message["value"].is<int>())
        {
            link_.error("Parameter requires an integer");
            return;
        }
        int value = message["value"].as<int>();
        value = constrain(value, parameter["min"] | value, parameter["max"] | value);
        parameter["value"] = value;
    }
    else
    {
        if (!message["value"].is<float>() && !message["value"].is<int>())
        {
            link_.error("Parameter requires a number");
            return;
        }
        float value = message["value"].as<float>();
        value = constrain(value, parameter["min"] | value, parameter["max"] | value);
        parameter["value"] = value;
    }

    if (strcmp(name, "debug.telemetry_interval_ms") == 0)
        telemetryIntervalMs_ = parameter["value"].as<uint32_t>();

    link_.parameterValue(name, parameter["value"].as<float>());
}

void BluetoothDebugWorkflow::sendDefinitions()
{
    for (JsonObject parameter : config_["parameters"].as<JsonArray>())
    {
        JsonDocument outgoing;
        outgoing.set(parameter);
        outgoing["type"] = "parameter_definition";
        link_.send(outgoing);
    }

    for (JsonObject command : config_["commands"].as<JsonArray>())
    {
        JsonDocument outgoing;
        outgoing.set(command);
        outgoing.remove("action");
        outgoing.remove("debug_mode_required");
        outgoing["type"] = "command_definition";
        link_.send(outgoing);
    }
    lastDefinitionsMs_ = millis();
}

void BluetoothDebugWorkflow::sendState()
{
    link_.state(debugMode_, stopped_, trackingFault_);
}

void BluetoothDebugWorkflow::updateAutomaticServoRead()
{
    constexpr uint32_t READ_INTERVAL_MS = 200;
    const uint32_t now = millis();
    if (!autoReadEnabled_ || now - lastServoReadMs_ < READ_INTERVAL_MS)
        return;
    lastServoReadMs_ = now;

    const float absoluteAngle = servos_.readAngle(autoReadServoId_, 30);
    if (isnan(absoluteAngle))
    {
        if (consecutiveReadFailures_ < 255)
            ++consecutiveReadFailures_;
        if (consecutiveReadFailures_ >= 3)
        {
            const bool wasResponding = servoResponding_;
            servoResponding_ = false;
            if (wasResponding)
            {
                trackingFault_ = true;
                sendState();
                link_.error("Herkulex position monitoring lost servo response");
            }
        }
        return;
    }

    servoResponding_ = true;
    consecutiveReadFailures_ = 0;
    measuredServoAngleDeg_ = servoZeroed_[autoReadServoId_]
        ? absoluteAngle - servoZeroOffsetsDeg_[autoReadServoId_]
        : absoluteAngle;
    updateTrackingState(absoluteAngle);
}

void BluetoothDebugWorkflow::updateTrackingState(float absoluteAngle)
{
    if (continuousVelocityActive_)
        return;
    if (autoReadServoId_ != lastServoId_ || !servoZeroed_[autoReadServoId_])
        return;

    const float measuredRelative = absoluteAngle - servoZeroOffsetsDeg_[autoReadServoId_];
    servoPositionErrorDeg_ = lastServoAngleDeg_ - measuredRelative;

    // Do not judge tracking while the requested movement should still be in progress.
    if (millis() - lastMoveStartMs_ <= static_cast<uint32_t>(lastMoveDurationMs_) + 300U)
        return;

    constexpr float MAX_TRACKING_ERROR_DEG = 8.0f;
    if (!stopped_ && fabsf(servoPositionErrorDeg_) > MAX_TRACKING_ERROR_DEG)
    {
        if (consecutiveTrackingErrors_ < 255)
            ++consecutiveTrackingErrors_;
    }
    else
    {
        consecutiveTrackingErrors_ = 0;
    }

    const bool newFault = consecutiveTrackingErrors_ >= 3;
    if (newFault != trackingFault_)
    {
        trackingFault_ = newFault;
        sendState();
        link_.log(newFault ? "ERROR" : "INFO",
                  newFault ? "Herkulex tracking error exceeds 8 degrees" : "Herkulex tracking recovered");
    }
}

void BluetoothDebugWorkflow::sendTelemetry()
{
    const uint32_t now = millis();
    if (now - lastTelemetryMs_ < telemetryIntervalMs_)
        return;
    lastTelemetryMs_ = now;

    readTofSensors();

    JsonDocument message;
    message["type"] = "telemetry";
    message["time"] = now;
    JsonObject data = message["data"].to<JsonObject>();
    data["system.uptime_s"] = now / 1000.0f;
    data["system.debug_mode"] = debugMode_;
    data["system.stopped"] = stopped_;
    data["bluetooth.messages_received"] = receivedMessages_;
    data["servo.last_id"] = lastServoId_;
    data["servo.commanded_angle_deg"] = lastServoAngleDeg_;
    data["servo.zeroed"] = servoZeroed_[lastServoId_];
    data["servo.auto_read_enabled"] = autoReadEnabled_;
    data["servo.responding"] = servoResponding_;
    data["servo.tracking_fault"] = trackingFault_;
    data["servo.continuous_velocity_active"] = continuousVelocityActive_;
    data["servo.commanded_velocity"] = commandedVelocity_;
    data["dc_motor_203.active"] = dcMotor203Active_;
    data["dc_motor_203.channel_a_percent"] = dcMotor203_.channelAPercent();
    data["dc_motor_203.channel_b_percent"] = dcMotor203_.channelBPercent();
    data["dc_motor_203.channel_a_pulse_us"] = dcMotor203_.channelAPulseUs();
    data["dc_motor_203.channel_b_pulse_us"] = dcMotor203_.channelBPulseUs();
    data["dc_motor_203_second.active"] = dcMotor203SecondActive_;
    data["dc_motor_203_second.channel_a_percent"] = dcMotor203Second_.channelAPercent();
    data["dc_motor_203_second.channel_b_percent"] = dcMotor203Second_.channelBPercent();
    data["dc_motor_203_second.channel_a_pulse_us"] = dcMotor203Second_.channelAPulseUs();
    data["dc_motor_203_second.channel_b_pulse_us"] = dcMotor203Second_.channelBPulseUs();
    data["hx12k.a.enabled"] = hx12kA_.enabled();
    data["hx12k.a.angle_deg"] = hx12kA_.commandedAngle();
    data["hx12k.b.enabled"] = hx12kB_.enabled();
    data["hx12k.b.angle_deg"] = hx12kB_.commandedAngle();
    data["hx12k.c.enabled"] = hx12kC_.enabled();
    data["hx12k.c.angle_deg"] = hx12kC_.commandedAngle();
    data["hx12k.d.enabled"] = hx12kD_.enabled();
    data["hx12k.d.angle_deg"] = hx12kD_.commandedAngle();
    for (uint8_t i = 0; i < tofSensorCount_; ++i)
    {
        char key[56];
        snprintf(key, sizeof(key), "tof.%s.available", tofSensorNames_[i]);
        data[key] = tofAvailable_[i];
        snprintf(key, sizeof(key), "tof.%s.timed_out", tofSensorNames_[i]);
        data[key] = tofTimedOut_[i];
        if (tofAvailable_[i] && !tofTimedOut_[i] && tofDistanceMm_[i] >= 0)
        {
            snprintf(key, sizeof(key), "tof.%s.distance_mm", tofSensorNames_[i]);
            data[key] = tofDistanceMm_[i];
        }
    }
    if (servoZeroed_[lastServoId_])
        data["servo.zero_offset_deg"] = servoZeroOffsetsDeg_[lastServoId_];
    if (!isnan(measuredServoAngleDeg_))
        data["servo.measured_angle_deg"] = measuredServoAngleDeg_;
    if (!isnan(servoPositionErrorDeg_))
        data["servo.position_error_deg"] = servoPositionErrorDeg_;
    link_.send(message);
}

void BluetoothDebugWorkflow::initialiseTofSensors()
{
    Tof::SensorDefinition definitions[MAX_TOF_SENSORS];
    JsonArray configured = config_["tof_sensors"].as<JsonArray>();

    for (JsonObject sensor : configured)
    {
        if (tofSensorCount_ >= MAX_TOF_SENSORS)
        {
            link_.error("Maximum of eight TOF sensors exceeded");
            break;
        }

        const char* name = sensor["name"] | "";
        const char* type = sensor["type"] | "";
        int port = -1;
        if (sensor["port"].is<int>())
            port = sensor["port"].as<int>();
        else
        {
            const char* portText = sensor["port"] | "";
            if ((strncmp(portText, "XSHUT", 5) == 0 || strncmp(portText, "xshut", 5) == 0) &&
                portText[5] >= '0' && portText[5] <= '7' && portText[6] == '\0')
                port = portText[5] - '0';
        }

        const bool validType = strcmp(type, "long") == 0 || strcmp(type, "short") == 0;
        bool duplicate = false;
        for (uint8_t i = 0; i < tofSensorCount_; ++i)
            duplicate = duplicate || strcmp(name, tofSensorNames_[i]) == 0 ||
                        definitions[i].xshutPin == port;

        if (name[0] == '\0' || strlen(name) >= sizeof(tofSensorNames_[0]) ||
            !validType || port < 0 || port > 7 || duplicate)
        {
            link_.error("Invalid or duplicate TOF JSON entry");
            continue;
        }

        strncpy(tofSensorNames_[tofSensorCount_], name, sizeof(tofSensorNames_[0]) - 1);
        definitions[tofSensorCount_].type = strcmp(type, "long") == 0
            ? Tof::SensorType::Long : Tof::SensorType::Short;
        definitions[tofSensorCount_].xshutPin = static_cast<uint8_t>(port);
        definitions[tofSensorCount_].address = TofConfig::FIRST_ASSIGNED_ADDRESS + port;
        ++tofSensorCount_;
    }

    Tof::begin(definitions, tofSensorCount_);
    for (uint8_t i = 0; i < tofSensorCount_; ++i)
    {
        tofAvailable_[i] = Tof::available(i);
        tofTimedOut_[i] = !tofAvailable_[i];
        tofDistanceMm_[i] = -1;
    }

    link_.log("INFO", "JSON-configured TOF sensor setup complete");
}

void BluetoothDebugWorkflow::readTofSensors()
{
    for (uint8_t i = 0; i < tofSensorCount_; ++i)
    {
        tofAvailable_[i] = Tof::available(i);
        if (!tofAvailable_[i])
        {
            tofTimedOut_[i] = true;
            tofDistanceMm_[i] = -1;
            continue;
        }

        tofDistanceMm_[i] = Tof::read(i);
        tofTimedOut_[i] = Tof::timedOut(i);
    }
}

void BluetoothDebugWorkflow::updateTof8x8()
{
    const uint32_t now = millis();
    if (now - lastTof8x8FrameMs_ < Tof8x8Config::FRAME_INTERVAL_MS)
        return;
    lastTof8x8FrameMs_ = now;

    if (tof8x8_.available())
        tof8x8_.read();
    sendTof8x8Frame();
}

void BluetoothDebugWorkflow::sendTof8x8Frame()
{
    JsonDocument message;
    message["type"] = "tof_8x8";
    message["time"] = millis();
    message["name"] = "matrix";
    message["available"] = tof8x8_.available();
    message["valid"] = tof8x8_.lastReadSucceeded();
    message["frame"] = tof8x8_.frameNumber();
    message["address"] = tof8x8_.detectedAddress();
    message["address_ack_mask"] = tof8x8_.addressAckMask();
    message["address_52_ack"] = tof8x8_.address52Acknowledged();
    message["bus"] = "I2C1";
    JsonArray i2cAddresses = message["i2c_addresses"].to<JsonArray>();
    for (uint8_t address = 0x08; address <= 0x77; ++address)
        if (tof8x8_.addressAcknowledged(address))
            i2cAddresses.add(address);

    JsonArray zones = message["data"].to<JsonArray>();
    if (tof8x8_.lastReadSucceeded())
        for (uint8_t i = 0; i < Tof8x8::ZONE_COUNT; ++i)
            zones.add(tof8x8_.data()[i]);

    link_.send(message);
}

JsonObject BluetoothDebugWorkflow::findParameter(const char* name)
{
    for (JsonObject parameter : config_["parameters"].as<JsonArray>())
        if (strcmp(parameter["name"] | "", name) == 0)
            return parameter;
    return JsonObject();
}

JsonObject BluetoothDebugWorkflow::findCommand(const char* name)
{
    for (JsonObject command : config_["commands"].as<JsonArray>())
        if (strcmp(command["name"] | "", name) == 0)
            return command;
    return JsonObject();
}
