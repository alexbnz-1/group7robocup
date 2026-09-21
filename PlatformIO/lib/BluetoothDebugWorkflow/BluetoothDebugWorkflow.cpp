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
      encoders_(Pins::ENCODER_1_A, Pins::ENCODER_1_B, Pins::ENCODER_2_A, Pins::ENCODER_2_B),
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
    encoders_.begin();
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

    initialiseDigitalInputs();
    initialiseUltrasoundSensors();
    initialiseTofSensors();
    const bool tof8x8Ready = tof8x8_.begin();
    const bool imuReady = imu_.begin();

    JsonObject interval = findParameter("debug.telemetry_interval_ms");
    if (!interval.isNull())
        telemetryIntervalMs_ = constrain(interval["value"] | 200UL, 50UL, 2000UL);

    delay(100);
    link_.log("INFO", "Teensy Bluetooth workflow ready");
    link_.log(tof8x8Ready ? "INFO" : "WARNING",
              tof8x8Ready ? "SEN0628 8x8 TOF ready" : "SEN0628 8x8 TOF not detected");
    link_.log(imuReady ? "INFO" : "WARNING",
              imuReady ? "BNO055 IMU ready" : "BNO055 IMU not detected on I2C0/I2C1");
    sendState();
}

void BluetoothDebugWorkflow::update()
{
    link_.update();
    const uint32_t now = millis();
    for (uint8_t i = 0; i < digitalInputCount_; ++i)
        digitalInputs_[i].update(now);
    if (dcMotor203SecondDeadman_ && dcMotor203SecondActive_ &&
        millis() - lastDcMotor203SecondCommandMs_ > KEYBOARD_DRIVE_TIMEOUT_MS)
    {
        dcMotor203Second_.stop();
        dcMotor203SecondActive_ = false;
        dcMotor203SecondDeadman_ = false;
        link_.log("WARNING", "Keyboard drive heartbeat lost; motor bank 2 stopped");
    }
    updateAutomaticServoRead();
    updateTof8x8();
    updateMotionAndRangeSensors();
    // Make the safety decision before potentially lengthy JSON transmission.
    updateNavigation();
    sendTelemetry();
    // Start/sample echo after the potentially slower I2C/telemetry work so a
    // fresh trigger cannot be hidden inside those operations.
    updateUltrasoundSensors();
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
    else if (strcmp(type, "drive") == 0)
    {
        // Compact, heartbeat-driven packet used by desktop arrow-key drive.
        // Keeping this packet short makes it much less vulnerable to losses
        // on the half-duplex CH9143 Bluetooth bridge than a full command JSON.
        if (!debugMode_ || stopped_)
        {
            link_.error("Keyboard drive requires Debug Mode and Run state");
            return;
        }
        if (!message["a"].is<int>() || !message["b"].is<int>())
        {
            link_.error("Keyboard drive requires integer a and b fields");
            return;
        }
        const int channelA = message["a"].as<int>();
        const int channelB = message["b"].as<int>();
        if (channelA < -100 || channelA > 100 ||
            channelB < -100 || channelB > 100)
        {
            link_.error("Keyboard drive values must be from -100 to 100");
            return;
        }
        const bool changed = channelA != dcMotor203Second_.channelAPercent() ||
                             channelB != dcMotor203Second_.channelBPercent();
        dcMotor203Second_.setPercent(channelA, channelB);
        dcMotor203SecondActive_ = channelA != 0 || channelB != 0;
        dcMotor203SecondDeadman_ = true;
        lastDcMotor203SecondCommandMs_ = millis();
        if (changed)
            link_.log(dcMotor203SecondActive_ ? "WARNING" : "INFO",
                      dcMotor203SecondActive_ ? "Keyboard drive applied" :
                                                "Keyboard drive stopped");
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
            stopNavigation();
            stopped_ = true;
            servos_.torqueOff(0xFE);
            dcMotor203_.stop();
            dcMotor203Active_ = false;
            dcMotor203Second_.stop();
            dcMotor203SecondActive_ = false;
            dcMotor203SecondDeadman_ = false;
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
        stopNavigation();
        servos_.torqueOff(0xFE);
        continuousVelocityActive_ = false;
        commandedVelocity_ = 0;
        dcMotor203_.stop();
        dcMotor203Active_ = false;
        dcMotor203Second_.stop();
        dcMotor203SecondActive_ = false;
        dcMotor203SecondDeadman_ = false;
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

        if (navigationActive_)
            stopNavigation("Navigation stopped by manual motor command");

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

        if (navigationActive_)
            stopNavigation("Navigation stopped by manual motor command");

        const int channelA = message["channel_a_percent"] | 0;
        const int channelB = message["channel_b_percent"] | 0;
        if (channelA < -100 || channelA > 100 || channelB < -100 || channelB > 100)
        {
            link_.error("Both second 203 DC motor speeds must be from -100 to 100 percent");
            return;
        }

        const bool changed = channelA != dcMotor203Second_.channelAPercent() ||
                             channelB != dcMotor203Second_.channelBPercent();
        dcMotor203Second_.setPercent(static_cast<int16_t>(channelA), static_cast<int16_t>(channelB));
        dcMotor203SecondActive_ = channelA != 0 || channelB != 0;
        dcMotor203SecondDeadman_ = message["deadman"] | false;
        lastDcMotor203SecondCommandMs_ = millis();
        if (changed)
            link_.log(dcMotor203SecondActive_ ? "WARNING" : "INFO",
                      dcMotor203SecondActive_ ? "Second 203 DC motor command applied" : "Second 203 DC motor stopped");
    }
    else if (strcmp(action, "dc_motor_203_second_stop") == 0)
    {
        stopNavigation();
        dcMotor203Second_.stop();
        dcMotor203SecondActive_ = false;
        dcMotor203SecondDeadman_ = false;
        link_.log("INFO", "Second 203 DC motor stopped at neutral pulse");
    }
    else if (strcmp(action, "navigation_toggle") == 0)
    {
        if (!message["enabled"].is<bool>())
        {
            link_.error("Navigation command requires enabled=true or false");
            return;
        }
        if (!message["enabled"].as<bool>())
        {
            stopNavigation("Autonomous navigation stopped");
            return;
        }
        if (!debugMode_)
        {
            link_.error("Autonomous navigation requires Debug Mode");
            return;
        }
        navigationActive_ = true;
        navigationState_ = NAV_SEEK_WALL;
        navigationMotionStartedMs_ = millis();
        navigationHeadingReferenceDeg_ = imu_.headingDeg();
        navigationTargetHeadingDeg_ = navigationHeadingReferenceDeg_;
        navigationLaneIndex_ = 0;
        navigationSweepTurnRight_ = true;
        navigationSweepLeftReferenceValid_ = false;
        navigationSweepRightReferenceValid_ = false;
        navigationSweepLateralErrorMm_ = 0;
        navigationMotionConsistent_ = true;
        stopped_ = false;
        dcMotor203SecondDeadman_ = false;
        sendState();
        link_.log("WARNING", "Autonomous navigation started");
    }
    else if (strcmp(action, "encoder_zero") == 0)
    {
        encoders_.zero();
        link_.log("INFO", "Both encoder counts zeroed");
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
    else if (strcmp(action, "hx12k_bumpers") == 0)
    {
        if (stopped_)
        {
            link_.error("Robot is stopped; press Run Robot before moving bumper servos");
            return;
        }
        if (navigationActive_)
            stopNavigation("Navigation stopped by manual drive");
        if (!message["enabled"].is<bool>())
        {
            link_.error("Bumper servo command requires enabled=true or false");
            return;
        }

        const bool enabled = message["enabled"].as<bool>();
        hx12kC_.setAngle(enabled ? 0.0f : 130.0f);
        hx12kD_.setAngle(enabled ? 130.0f : 0.0f);
        link_.log("INFO", enabled ? "Bumper servos ON: C=0, D=130" :
                                  "Bumper servos OFF: C=130, D=0");
        sendState();
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
    data["navigation.active"] = navigationActive_;
    data["navigation.state"] = navigationState_;
    data["navigation.front_mm"] = navigationFrontMm_;
    data["navigation.left_mm"] = navigationLeftMm_;
    data["navigation.right_mm"] = navigationRightMm_;
    const char* navigationPhase = "idle";
    switch (navigationState_)
    {
        case NAV_SEEK_WALL: navigationPhase = "seek_wall"; break;
        case NAV_INITIAL_TURN: navigationPhase = "initial_turn"; break;
        case NAV_FOLLOW_WALL: navigationPhase = "follow_wall"; break;
        case NAV_CORNER_TURN: navigationPhase = "corner_turn"; break;
        case NAV_SWEEP: navigationPhase = "sweep"; break;
        case NAV_LANE_TURN_OUT: navigationPhase = "lane_turn_out"; break;
        case NAV_LANE_SHIFT: navigationPhase = "lane_shift"; break;
        case NAV_LANE_TURN_IN: navigationPhase = "lane_turn_in"; break;
        case NAV_COMPLETE: navigationPhase = "complete"; break;
        default: break;
    }
    data["navigation.phase"] = navigationPhase;
    data["navigation.lane"] = navigationLaneIndex_;
    data["navigation.target_heading_deg"] = navigationTargetHeadingDeg_;
    data["navigation.sweep_left_reference_mm"] =
        navigationSweepLeftReferenceValid_ ? navigationSweepLeftReferenceMm_ : 0;
    data["navigation.sweep_right_reference_mm"] =
        navigationSweepRightReferenceValid_ ? navigationSweepRightReferenceMm_ : 0;
    data["navigation.sweep_lateral_error_mm"] = navigationSweepLateralErrorMm_;
    data["navigation.motion_consistent"] = navigationMotionConsistent_;
    data["encoder.1.count"] = encoders_.firstCount();
    data["encoder.1.delta"] = encoders_.firstDelta();
    data["encoder.1.counts_per_s"] = encoders_.firstCountsPerSecond();
    data["encoder.2.count"] = encoders_.secondCount();
    data["encoder.2.delta"] = encoders_.secondDelta();
    data["encoder.2.counts_per_s"] = encoders_.secondCountsPerSecond();
    for (uint8_t i = 0; i < digitalInputCount_; ++i)
    {
        char key[64];
        snprintf(key, sizeof(key), "digital.%s.detected", digitalInputNames_[i]);
        data[key] = digitalInputs_[i].detected();
        snprintf(key, sizeof(key), "digital.%s.raw_high", digitalInputNames_[i]);
        data[key] = digitalInputs_[i].rawHigh();
        snprintf(key, sizeof(key), "digital.%s.transitions", digitalInputNames_[i]);
        data[key] = digitalInputs_[i].transitionCount();
    }
    for (uint8_t i = 0; i < ultrasoundSensorCount_; ++i)
    {
        char key[64];
        snprintf(key, sizeof(key), "ultrasound.%s.valid", ultrasoundSensorNames_[i]);
        data[key] = ultrasoundSensors_[i].valid();
        snprintf(key, sizeof(key), "ultrasound.%s.timed_out", ultrasoundSensorNames_[i]);
        data[key] = ultrasoundSensors_[i].timedOut();
        snprintf(key, sizeof(key), "ultrasound.%s.echo_us", ultrasoundSensorNames_[i]);
        data[key] = ultrasoundSensors_[i].echoUs();
        if (ultrasoundSensors_[i].valid())
        {
            snprintf(key, sizeof(key), "ultrasound.%s.distance_mm", ultrasoundSensorNames_[i]);
            data[key] = ultrasoundSensors_[i].distanceMm();
        }
    }
    data["imu.available"] = imu_.available();
    data["imu.valid"] = lastImuSampleValid_;
    if (imu_.available()) {
        data["imu.bus"] = imu_.bus();
        data["imu.address"] = imu_.address();
        data["imu.calibration.system"] = imu_.systemCalibration();
        data["imu.calibration.gyro"] = imu_.gyroCalibration();
        data["imu.calibration.accel"] = imu_.accelCalibration();
        data["imu.calibration.mag"] = imu_.magCalibration();
        data["imu.system_status"] = imu_.systemStatus();
        data["imu.system_error"] = imu_.systemError();
        data["imu.system_error_active"] = imu_.systemErrorActive();
        data["imu.operation_mode"] = imu_.operationMode();
        data["imu.fusion_running"] = imu_.fusionRunning();
        data["imu.self_test_result"] = imu_.selfTestResult();
        data["imu.self_test_passed"] = imu_.selfTestPassed();
        if (lastImuSampleValid_) {
            data["imu.heading_deg"] = imu_.headingDeg();
            data["imu.roll_deg"] = imu_.rollDeg();
            data["imu.pitch_deg"] = imu_.pitchDeg();
            data["imu.quaternion.w"] = imu_.quaternionW();
            data["imu.quaternion.x"] = imu_.quaternionX();
            data["imu.quaternion.y"] = imu_.quaternionY();
            data["imu.quaternion.z"] = imu_.quaternionZ();
            data["imu.linear_accel.x_mps2"] = imu_.linearAccelX();
            data["imu.linear_accel.y_mps2"] = imu_.linearAccelY();
            data["imu.linear_accel.z_mps2"] = imu_.linearAccelZ();
            data["imu.gravity.x_mps2"] = imu_.gravityX();
            data["imu.gravity.y_mps2"] = imu_.gravityY();
            data["imu.gravity.z_mps2"] = imu_.gravityZ();
            data["imu.temperature_c"] = imu_.temperatureC();
        }
    }
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

    // Keep electrical diagnostics in a second, slower packet. These raw pin
    // values change far less often than motion state and sending them at the
    // full telemetry rate needlessly saturates the 115200-baud radio link.
    if (now - lastElectricalDiagnosticsMs_ < 1000U)
        return;
    lastElectricalDiagnosticsMs_ = now;
    JsonDocument diagnosticMessage;
    diagnosticMessage["type"] = "telemetry";
    diagnosticMessage["time"] = now;
    JsonObject diagnostics = diagnosticMessage["data"].to<JsonObject>();
    for (uint8_t i = 0; i < ultrasoundSensorCount_; ++i)
    {
        char key[64];
        snprintf(key, sizeof(key), "ultrasound.%s.echo_high", ultrasoundSensorNames_[i]);
        diagnostics[key] = ultrasoundSensors_[i].echoHigh();
        snprintf(key, sizeof(key), "ultrasound.%s.trigger_count", ultrasoundSensorNames_[i]);
        diagnostics[key] = ultrasoundSensors_[i].triggerCount();
        snprintf(key, sizeof(key), "ultrasound.%s.rise_count", ultrasoundSensorNames_[i]);
        diagnostics[key] = ultrasoundSensors_[i].riseCount();
        snprintf(key, sizeof(key), "ultrasound.%s.fall_count", ultrasoundSensorNames_[i]);
        diagnostics[key] = ultrasoundSensors_[i].fallCount();
        snprintf(key, sizeof(key), "ultrasound.%s.echo_adc", ultrasoundSensorNames_[i]);
        diagnostics[key] = ultrasoundSensors_[i].currentEchoAdc();
        snprintf(key, sizeof(key), "ultrasound.%s.echo_voltage_v", ultrasoundSensorNames_[i]);
        diagnostics[key] = ultrasoundSensors_[i].currentEchoAdc() * (3.3f / 4095.0f);
        snprintf(key, sizeof(key), "ultrasound.%s.ping_min_adc", ultrasoundSensorNames_[i]);
        diagnostics[key] = ultrasoundSensors_[i].lastMinEchoAdc();
        snprintf(key, sizeof(key), "ultrasound.%s.ping_max_adc", ultrasoundSensorNames_[i]);
        diagnostics[key] = ultrasoundSensors_[i].lastMaxEchoAdc();
        snprintf(key, sizeof(key), "ultrasound.%s.ping_min_voltage_v", ultrasoundSensorNames_[i]);
        diagnostics[key] = ultrasoundSensors_[i].lastMinEchoAdc() * (3.3f / 4095.0f);
        snprintf(key, sizeof(key), "ultrasound.%s.ping_max_voltage_v", ultrasoundSensorNames_[i]);
        diagnostics[key] = ultrasoundSensors_[i].lastMaxEchoAdc() * (3.3f / 4095.0f);
        snprintf(key, sizeof(key), "ultrasound.%s.trigger_low_adc", ultrasoundSensorNames_[i]);
        diagnostics[key] = ultrasoundSensors_[i].triggerLowAdc();
        snprintf(key, sizeof(key), "ultrasound.%s.trigger_high_adc", ultrasoundSensorNames_[i]);
        diagnostics[key] = ultrasoundSensors_[i].triggerHighAdc();
        snprintf(key, sizeof(key), "ultrasound.%s.trigger_high_voltage_v", ultrasoundSensorNames_[i]);
        diagnostics[key] = ultrasoundSensors_[i].triggerHighAdc() * (3.3f / 4095.0f);
    }
    link_.send(diagnosticMessage);
}

void BluetoothDebugWorkflow::initialiseDigitalInputs()
{
    for (JsonObject input : config_["digital_inputs"].as<JsonArray>())
    {
        if (digitalInputCount_ >= MAX_DIGITAL_INPUTS)
        {
            link_.error("Maximum of eight digital inputs exceeded");
            break;
        }

        const char* name = input["name"] | "";
        const int pin = input["pin"] | -1;
        bool duplicate = false;
        for (uint8_t i = 0; i < digitalInputCount_; ++i)
            duplicate = duplicate || strcmp(name, digitalInputNames_[i]) == 0 ||
                        pin == digitalInputs_[i].pin();

        if (name[0] == '\0' || strlen(name) >= sizeof(digitalInputNames_[0]) ||
            pin < 0 || pin > 41 || duplicate)
        {
            link_.error("Invalid or duplicate digital input JSON entry");
            continue;
        }

        strncpy(digitalInputNames_[digitalInputCount_], name,
                sizeof(digitalInputNames_[0]) - 1);
        const bool activeLow = input["active_low"] | true;
        const bool pullup = input["pullup"] | true;
        const uint16_t debounceMs = constrain(input["debounce_ms"] | 20, 0, 1000);
        digitalInputs_[digitalInputCount_].begin(
            static_cast<uint8_t>(pin), activeLow, pullup, debounceMs);
        ++digitalInputCount_;
    }
}

void BluetoothDebugWorkflow::initialiseUltrasoundSensors()
{
    for (JsonObject sensor : config_["ultrasound_sensors"].as<JsonArray>())
    {
        if (ultrasoundSensorCount_ >= MAX_ULTRASOUND_SENSORS)
        {
            link_.error("Maximum of two ultrasound sensors exceeded");
            break;
        }

        const char* name = sensor["name"] | "";
        const int triggerPin = sensor["trigger_pin"] | -1;
        const int echoPin = sensor["echo_pin"] | -1;
        bool duplicate = triggerPin == echoPin;
        for (uint8_t i = 0; i < ultrasoundSensorCount_; ++i)
            duplicate = duplicate || strcmp(name, ultrasoundSensorNames_[i]) == 0 ||
                        triggerPin == ultrasoundSensors_[i].triggerPin() ||
                        triggerPin == ultrasoundSensors_[i].echoPin() ||
                        echoPin == ultrasoundSensors_[i].triggerPin() ||
                        echoPin == ultrasoundSensors_[i].echoPin();

        if (name[0] == '\0' || strlen(name) >= sizeof(ultrasoundSensorNames_[0]) ||
            triggerPin < 0 || triggerPin > 41 || echoPin < 0 || echoPin > 41 || duplicate)
        {
            link_.error("Invalid or duplicate ultrasound JSON entry");
            continue;
        }

        strncpy(ultrasoundSensorNames_[ultrasoundSensorCount_], name,
                sizeof(ultrasoundSensorNames_[0]) - 1);
        const uint32_t intervalMs = constrain(sensor["interval_ms"] | 100UL, 50UL, 2000UL);
        const uint32_t timeoutUs = constrain(sensor["timeout_us"] | 30000UL, 1000UL, 50000UL);
        ultrasoundSensors_[ultrasoundSensorCount_].begin(
            static_cast<uint8_t>(triggerPin), static_cast<uint8_t>(echoPin),
            intervalMs, timeoutUs);
        ++ultrasoundSensorCount_;
    }
}

void BluetoothDebugWorkflow::updateUltrasoundSensors()
{
    if (ultrasoundSensorCount_ == 0)
        return;

    const uint32_t nowUs = micros();
    for (uint8_t i = 0; i < ultrasoundSensorCount_; ++i)
        ultrasoundSensors_[i].sampleEchoLevel();
    if (activeUltrasoundIndex_ >= 0)
    {
        UltrasoundSensor& active = ultrasoundSensors_[activeUltrasoundIndex_];
        active.update(nowUs);
        if (active.busy())
            return;
        nextUltrasoundIndex_ = (static_cast<uint8_t>(activeUltrasoundIndex_) + 1) %
                               ultrasoundSensorCount_;
        activeUltrasoundIndex_ = -1;
    }

    // Only one transducer may transmit/listen at a time. This avoids channel A
    // receiving channel B's ping on the two-socket interface board.
    for (uint8_t offset = 0; offset < ultrasoundSensorCount_; ++offset)
    {
        const uint8_t index = (nextUltrasoundIndex_ + offset) % ultrasoundSensorCount_;
        if (!ultrasoundSensors_[index].readyToTrigger(nowUs))
            continue;
        ultrasoundSensors_[index].trigger(nowUs);
        activeUltrasoundIndex_ = index;
        return;
    }
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

void BluetoothDebugWorkflow::updateMotionAndRangeSensors()
{
    const uint32_t now = millis();
    if (now - lastMotionSampleMs_ >= 50U)
    {
        lastMotionSampleMs_ = now;
        encoders_.sample(now);
        lastImuSampleValid_ = imu_.update();
    }
    if (now - lastRangePollMs_ >= 100U)
    {
        lastRangePollMs_ = now;
        readTofSensors();
    }
}

void BluetoothDebugWorkflow::stopNavigation(const char* reason)
{
    navigationActive_ = false;
    navigationState_ = NAV_IDLE;
    dcMotor203Second_.stop();
    dcMotor203SecondActive_ = false;
    dcMotor203SecondDeadman_ = false;
    if (reason != nullptr)
        link_.log("INFO", reason);
}

void BluetoothDebugWorkflow::updateNavigation()
{
    if (!navigationActive_)
        return;
    const uint32_t now = millis();
    if (!debugMode_ || stopped_)
    {
        stopNavigation();
        return;
    }

    constexpr uint16_t FRONT_AVOID_MM = 300;
    constexpr uint16_t WALL_FOLLOW_TARGET_MM = 200;
    constexpr uint16_t WALL_FOLLOW_DEADBAND_MM = 10;
    constexpr int16_t SWEEP_LATERAL_DEADBAND_MM = 15;
    constexpr float LANE_SPACING_MM = 250.0f;
    constexpr uint8_t SWEEP_LANE_COUNT = 10;
    constexpr float ENCODER_1_MM_PER_COUNT = 0.09094f;
    constexpr float ENCODER_2_MM_PER_COUNT = 0.09592f;

    uint16_t front = 0xFFFF, left = 0xFFFF, right = 0xFFFF;
    auto includeMinimum = [](uint16_t& target, int value) {
        if (value >= 30 && value <= 3500)
            target = min(target, static_cast<uint16_t>(value));
    };

    // Navigation deliberately ignores every bottom point TOF. Those sensors
    // can see one another/chassis hardware and are reserved for weight sensing
    // and mapping. Only the two top point TOFs feed forward navigation.
    uint16_t frontCandidates[MAX_TOF_SENSORS + 1] = {};
    uint8_t frontCandidateCount = 0;
    for (uint8_t i = 0; i < tofSensorCount_; ++i)
    {
        if (!tofAvailable_[i] || tofTimedOut_[i])
            continue;
        if (strncmp(tofSensorNames_[i], "top_", 4) != 0)
            continue;
        const int value = tofDistanceMm_[i];
        if (value >= 30 && value <= 3500)
            frontCandidates[frontCandidateCount++] = static_cast<uint16_t>(value);
    }
    for (uint8_t i = 0; i < ultrasoundSensorCount_; ++i)
    {
        if (!ultrasoundSensors_[i].valid() || ultrasoundSensors_[i].timedOut())
            continue;
        // Ultrasound A faces robot-left and B faces robot-right.
        if (i == 0) includeMinimum(left, ultrasoundSensors_[i].distanceMm());
        else includeMinimum(right, ultrasoundSensors_[i].distanceMm());
    }
    if (tof8x8_.available() && tof8x8_.lastReadSucceeded())
    {
        uint16_t centreValues[32];
        uint8_t centreCount = 0;
        for (uint8_t row = 0; row < 8; ++row)
            for (uint8_t col = 0; col < 8; ++col)
            {
                const uint16_t value = tof8x8_.distanceMm(row, col);
                // Preserve the established 8x8 rejection of chassis/noise
                // returns below 200 mm. The side-facing ultrasound channels
                // provide the separate 200 mm side clearance.
                if (value < 200 || value > 3500)
                    continue;
                if (col >= 2 && col <= 5) centreValues[centreCount++] = value;
            }
        auto lowerQuartile = [](uint16_t* values, uint8_t count) -> uint16_t {
            if (count == 0) return 0xFFFF;
            for (uint8_t i = 1; i < count; ++i)
            {
                const uint16_t value = values[i];
                uint8_t j = i;
                while (j > 0 && values[j - 1] > value)
                {
                    values[j] = values[j - 1];
                    --j;
                }
                values[j] = value;
            }
            return values[(count - 1) / 4];
        };
        const uint16_t matrixForward = lowerQuartile(centreValues, centreCount);
        if (matrixForward != 0xFFFF)
            frontCandidates[frontCandidateCount++] = matrixForward;
    }

    // A median across the top-left, top-right and matrix forward estimates
    // rejects one isolated short return without hiding a broad wall.
    for (uint8_t i = 1; i < frontCandidateCount; ++i)
    {
        const uint16_t value = frontCandidates[i];
        uint8_t j = i;
        while (j > 0 && frontCandidates[j - 1] > value)
        {
            frontCandidates[j] = frontCandidates[j - 1];
            --j;
        }
        frontCandidates[j] = value;
    }
    if (frontCandidateCount > 0)
        front = frontCandidates[frontCandidateCount / 2];

    navigationFrontMm_ = front == 0xFFFF ? 0 : front;
    navigationLeftMm_ = left == 0xFFFF ? 0 : left;
    navigationRightMm_ = right == 0xFFFF ? 0 : right;
    const bool frontBlocked = front != 0xFFFF && front < FRONT_AVOID_MM;

    auto normaliseHeading = [](float heading) {
        while (heading >= 360.0f) heading -= 360.0f;
        while (heading < 0.0f) heading += 360.0f;
        return heading;
    };
    auto headingDelta = [](float from, float to) {
        float delta = to - from;
        while (delta > 180.0f) delta -= 360.0f;
        while (delta < -180.0f) delta += 360.0f;
        return delta;
    };

    auto setDrive = [&](int16_t channelA, int16_t channelB) {
        dcMotor203Second_.setPercent(channelA, channelB);
        dcMotor203SecondActive_ = channelA != 0 || channelB != 0;
        dcMotor203SecondDeadman_ = false;
    };
    auto beginRightAngleTurn = [&](bool rightTurn, NavigationState state) {
        // Always turn from the exact heading of the current leg, not from the
        // slightly imperfect measured heading at the transition. Otherwise a
        // few degrees of turn tolerance accumulate on every sweep lane.
        navigationTargetHeadingDeg_ = normaliseHeading(
            navigationHeadingReferenceDeg_ + (rightTurn ? 90.0f : -90.0f));
        navigationState_ = state;
        navigationMotionStartedMs_ = now;
        navigationMotionConsistent_ = true;
    };
    auto runHeadingTurn = [&]() {
        const float error = headingDelta(imu_.headingDeg(), navigationTargetHeadingDeg_);
        if (lastImuSampleValid_ && fabsf(error) <= 7.0f &&
            now - navigationMotionStartedMs_ >= 350U)
        {
            setDrive(0, 0);
            return true;
        }
        const bool turnRight = error >= 0.0f;
        setDrive(turnRight ? -80 : 80, turnRight ? 80 : -80);
        return false;
    };
    auto driveOnHeading = [&](float targetHeading) {
        const float error = headingDelta(imu_.headingDeg(), targetHeading);
        if (error > 4.0f)
            setDrive(-80, -75);  // correct right while retaining forward motion
        else if (error < -4.0f)
            setDrive(-75, -80);  // correct left while retaining forward motion
        else
            setDrive(-80, -80);
    };
    auto beginForwardLeg = [&](float exactHeading) {
        navigationHeadingReferenceDeg_ = normaliseHeading(exactHeading);
        navigationMotionStartedMs_ = now;
        navigationMotionConsistent_ = true;
    };
    auto captureSweepSideReferences = [&]() {
        navigationSweepLeftReferenceValid_ = left != 0xFFFF;
        navigationSweepRightReferenceValid_ = right != 0xFFFF;
        if (navigationSweepLeftReferenceValid_)
            navigationSweepLeftReferenceMm_ = left;
        if (navigationSweepRightReferenceValid_)
            navigationSweepRightReferenceMm_ = right;
        navigationSweepLateralErrorMm_ = 0;
    };

    // Encoder/IMU agreement is recorded for diagnosis only. It never cancels
    // the coverage program.
    if (now - navigationMotionStartedMs_ >= 500U)
    {
        const float e1 = encoders_.firstCountsPerSecond();
        const float e2 = encoders_.secondCountsPerSecond();
        const bool forwardState = navigationState_ == NAV_SEEK_WALL ||
            navigationState_ == NAV_FOLLOW_WALL ||
            navigationState_ == NAV_SWEEP ||
            navigationState_ == NAV_LANE_SHIFT;
        if (forwardState)
            navigationMotionConsistent_ = lastImuSampleValid_ &&
                e1 < -10.0f && e2 > 10.0f;
        else if (navigationState_ != NAV_IDLE && navigationState_ != NAV_COMPLETE)
            navigationMotionConsistent_ = lastImuSampleValid_ &&
                (fabsf(e1) > 10.0f || fabsf(e2) > 10.0f);
    }

    switch (navigationState_)
    {
        case NAV_SEEK_WALL:
            if (frontBlocked)
            {
                setDrive(0, 0);
                beginRightAngleTurn(true, NAV_INITIAL_TURN);
                link_.log("INFO", "Navigation reached first wall; turning right");
            }
            else if (front == 0xFFFF)
                setDrive(-75, 75);  // rotate until forward ranging is recovered
            else
                driveOnHeading(navigationHeadingReferenceDeg_);
            break;

        case NAV_INITIAL_TURN:
            if (runHeadingTurn())
            {
                navigationState_ = NAV_FOLLOW_WALL;
                beginForwardLeg(navigationTargetHeadingDeg_);
                link_.log("INFO", "Navigation following first wall to corner");
            }
            break;

        case NAV_FOLLOW_WALL:
            if (frontBlocked)
            {
                setDrive(0, 0);
                beginRightAngleTurn(true, NAV_CORNER_TURN);
                link_.log("INFO", "Navigation reached corner; entering sweep");
            }
            else if (left != 0xFFFF &&
                     left + WALL_FOLLOW_DEADBAND_MM < WALL_FOLLOW_TARGET_MM)
            {
                // Too close: speed up the left wheel relative to the right so
                // the robot arcs away from the wall without leaving forward drive.
                const int16_t correction = constrain(
                    static_cast<int16_t>((WALL_FOLLOW_TARGET_MM - left) / 8),
                    static_cast<int16_t>(5), static_cast<int16_t>(10));
                setDrive(-85, static_cast<int16_t>(-85 + correction));
            }
            else if (left != 0xFFFF &&
                     left > WALL_FOLLOW_TARGET_MM + WALL_FOLLOW_DEADBAND_MM)
            {
                // Too far: the correction increases with distance, actively
                // bringing the robot back toward the wall instead of allowing
                // the separation to grow on every pass.
                const int16_t correction = constrain(
                    static_cast<int16_t>((left - WALL_FOLLOW_TARGET_MM) / 8),
                    static_cast<int16_t>(5), static_cast<int16_t>(10));
                setDrive(static_cast<int16_t>(-85 + correction), -85);
            }
            else
                driveOnHeading(navigationHeadingReferenceDeg_);
            break;

        case NAV_CORNER_TURN:
            if (runHeadingTurn())
            {
                navigationState_ = NAV_SWEEP;
                navigationLaneIndex_ = 0;
                navigationSweepTurnRight_ = true;
                beginForwardLeg(navigationTargetHeadingDeg_);
                captureSweepSideReferences();
                link_.log("INFO", "Arena sweep lane 1 started");
            }
            break;

        case NAV_SWEEP:
            // If a side echo was unavailable at the exact lane transition,
            // adopt it as soon as it becomes valid rather than running the
            // complete lane without lateral feedback.
            if (!navigationSweepLeftReferenceValid_ && left != 0xFFFF)
            {
                navigationSweepLeftReferenceMm_ = left;
                navigationSweepLeftReferenceValid_ = true;
            }
            if (!navigationSweepRightReferenceValid_ && right != 0xFFFF)
            {
                navigationSweepRightReferenceMm_ = right;
                navigationSweepRightReferenceValid_ = true;
            }
            if (frontBlocked)
            {
                if (navigationLaneIndex_ + 1 >= SWEEP_LANE_COUNT)
                {
                    navigationState_ = NAV_COMPLETE;
                    setDrive(0, 0);
                    link_.log("INFO", "Arena sweep complete");
                }
                else
                {
                    setDrive(0, 0);
                    beginRightAngleTurn(navigationSweepTurnRight_, NAV_LANE_TURN_OUT);
                }
            }
            else
            {
                int32_t lateralErrorSum = 0;
                uint8_t lateralSamples = 0;
                if (navigationSweepLeftReferenceValid_ && left != 0xFFFF)
                {
                    // Positive when the robot has moved away from its left
                    // reference wall (drifted right).
                    lateralErrorSum += static_cast<int32_t>(left) -
                                       navigationSweepLeftReferenceMm_;
                    ++lateralSamples;
                }
                if (navigationSweepRightReferenceValid_ && right != 0xFFFF)
                {
                    // Also positive for rightward drift because the right
                    // distance becomes smaller than its lane-start value.
                    lateralErrorSum += static_cast<int32_t>(
                        navigationSweepRightReferenceMm_) - right;
                    ++lateralSamples;
                }
                navigationSweepLateralErrorMm_ = lateralSamples == 0 ? 0 :
                    static_cast<int16_t>(lateralErrorSum / lateralSamples);

                if (navigationSweepLateralErrorMm_ > SWEEP_LATERAL_DEADBAND_MM)
                {
                    const int16_t correction = constrain(
                        static_cast<int16_t>(navigationSweepLateralErrorMm_ / 8),
                        static_cast<int16_t>(5), static_cast<int16_t>(10));
                    setDrive(static_cast<int16_t>(-85 + correction), -85);
                }
                else if (navigationSweepLateralErrorMm_ < -SWEEP_LATERAL_DEADBAND_MM)
                {
                    const int16_t correction = constrain(
                        static_cast<int16_t>(-navigationSweepLateralErrorMm_ / 8),
                        static_cast<int16_t>(5), static_cast<int16_t>(10));
                    setDrive(-85, static_cast<int16_t>(-85 + correction));
                }
                else
                    driveOnHeading(navigationHeadingReferenceDeg_);
            }
            break;

        case NAV_LANE_TURN_OUT:
            if (runHeadingTurn())
            {
                navigationState_ = NAV_LANE_SHIFT;
                navigationShiftStartEncoder1_ = encoders_.firstCount();
                navigationShiftStartEncoder2_ = encoders_.secondCount();
                beginForwardLeg(navigationTargetHeadingDeg_);
            }
            break;

        case NAV_LANE_SHIFT:
        {
            const float firstDistance = fabsf(
                (encoders_.firstCount() - navigationShiftStartEncoder1_) *
                ENCODER_1_MM_PER_COUNT);
            const float secondDistance = fabsf(
                (encoders_.secondCount() - navigationShiftStartEncoder2_) *
                ENCODER_2_MM_PER_COUNT);
            const float shiftedMm = (firstDistance + secondDistance) * 0.5f;
            if (shiftedMm >= LANE_SPACING_MM || frontBlocked)
            {
                setDrive(0, 0);
                beginRightAngleTurn(navigationSweepTurnRight_, NAV_LANE_TURN_IN);
            }
            else
                driveOnHeading(navigationHeadingReferenceDeg_);
            break;
        }

        case NAV_LANE_TURN_IN:
            if (runHeadingTurn())
            {
                ++navigationLaneIndex_;
                navigationSweepTurnRight_ = !navigationSweepTurnRight_;
                navigationState_ = NAV_SWEEP;
                beginForwardLeg(navigationTargetHeadingDeg_);
                captureSweepSideReferences();
                link_.log("INFO", "Next arena sweep lane started");
            }
            break;

        case NAV_COMPLETE:
            setDrive(0, 0);
            break;

        default:
            navigationState_ = NAV_SEEK_WALL;
            beginForwardLeg(imu_.headingDeg());
            break;
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
