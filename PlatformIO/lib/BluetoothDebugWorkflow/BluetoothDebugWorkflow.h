#pragma once

#include <Arduino.h>
#include <ArduinoJson.h>
#include <DcMotor203.h>
#include <HerkulexTeensy.h>
#include <Hx12kServo.h>
#include <RobotDebug.h>
#include <Tof.h>
#include <Tof8x8.h>

class BluetoothDebugWorkflow {
public:
    BluetoothDebugWorkflow(HardwareSerialIMXRT& bluetoothPort, HardwareSerial& herkulexPort);

    void begin();
    void update();

private:
    static constexpr uint8_t MAX_TOF_SENSORS = 8;
    HardwareSerialIMXRT& bluetoothPort_;
    uint8_t bluetoothRxBuffer_[2048] = {};
    HerkulexTeensy servos_;
    DcMotor203 dcMotor203_;
    DcMotor203 dcMotor203Second_;
    Hx12kServo hx12kA_;
    Hx12kServo hx12kB_;
    Hx12kServo hx12kC_;
    Hx12kServo hx12kD_;
    RobotDebug link_;
    Tof8x8 tof8x8_;
    JsonDocument config_;

    bool debugMode_ = false;
    bool stopped_ = true;
    uint8_t lastServoId_ = 1;
    float lastServoAngleDeg_ = 0.0f;
    float measuredServoAngleDeg_ = NAN;
    float servoZeroOffsetsDeg_[254] = {};
    bool servoZeroed_[254] = {};
    bool autoReadEnabled_ = false;
    uint8_t autoReadServoId_ = 1;
    bool servoResponding_ = false;
    bool trackingFault_ = false;
    uint8_t consecutiveReadFailures_ = 0;
    uint8_t consecutiveTrackingErrors_ = 0;
    float servoPositionErrorDeg_ = NAN;
    uint32_t lastServoReadMs_ = 0;
    uint32_t lastMoveStartMs_ = 0;
    uint16_t lastMoveDurationMs_ = 0;
    bool continuousVelocityActive_ = false;
    int16_t commandedVelocity_ = 0;
    bool dcMotor203Active_ = false;
    bool dcMotor203SecondActive_ = false;
    uint32_t telemetryIntervalMs_ = 200;
    uint32_t lastTelemetryMs_ = 0;
    uint32_t lastDefinitionsMs_ = 0;
    uint32_t receivedMessages_ = 0;
    uint8_t tofSensorCount_ = 0;
    char tofSensorNames_[MAX_TOF_SENSORS][25] = {};
    bool tofAvailable_[MAX_TOF_SENSORS] = {};
    bool tofTimedOut_[MAX_TOF_SENSORS] = {};
    int16_t tofDistanceMm_[MAX_TOF_SENSORS] = {};
    uint32_t lastTof8x8FrameMs_ = 0;

    static void dispatch(JsonDocument& message, void* context);
    void handleMessage(JsonDocument& message);
    void handleCommand(JsonDocument& message);
    void handleParameter(JsonDocument& message);
    void sendDefinitions();
    void sendState();
    void sendTelemetry();
    void initialiseTofSensors();
    void readTofSensors();
    void updateTof8x8();
    void sendTof8x8Frame();
    void updateAutomaticServoRead();
    void updateTrackingState(float absoluteAngle);
    JsonObject findParameter(const char* name);
    JsonObject findCommand(const char* name);
};
