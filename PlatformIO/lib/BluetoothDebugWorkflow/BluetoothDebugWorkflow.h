#pragma once

#include <Arduino.h>
#include <ArduinoJson.h>
#include <Bno055Imu.h>
#include <DcMotor203.h>
#include <DigitalInputSensor.h>
#include <DualEncoder.h>
#include <HerkulexTeensy.h>
#include <Hx12kServo.h>
#include <RobotDebug.h>
#include <Tof.h>
#include <Tof8x8.h>
#include <UltrasoundSensor.h>

class BluetoothDebugWorkflow {
public:
    BluetoothDebugWorkflow(HardwareSerialIMXRT& bluetoothPort, HardwareSerial& herkulexPort);

    void begin();
    void update();

private:
    static constexpr uint8_t MAX_TOF_SENSORS = 8;
    static constexpr uint8_t MAX_DIGITAL_INPUTS = 8;
    static constexpr uint8_t MAX_ULTRASOUND_SENSORS = 2;
    HardwareSerialIMXRT& bluetoothPort_;
    uint8_t bluetoothRxBuffer_[2048] = {};
    HerkulexTeensy servos_;
    DcMotor203 dcMotor203_;
    DcMotor203 dcMotor203Second_;
    DualEncoder encoders_;
    Bno055Imu imu_;
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
    bool dcMotor203SecondDeadman_ = false;
    uint32_t lastDcMotor203SecondCommandMs_ = 0;
    static constexpr uint32_t KEYBOARD_DRIVE_TIMEOUT_MS = 500;
    uint32_t telemetryIntervalMs_ = 200;
    uint32_t lastTelemetryMs_ = 0;
    uint32_t lastElectricalDiagnosticsMs_ = 0;
    uint32_t lastDefinitionsMs_ = 0;
    uint32_t receivedMessages_ = 0;
    uint8_t tofSensorCount_ = 0;
    char tofSensorNames_[MAX_TOF_SENSORS][25] = {};
    bool tofAvailable_[MAX_TOF_SENSORS] = {};
    bool tofTimedOut_[MAX_TOF_SENSORS] = {};
    int16_t tofDistanceMm_[MAX_TOF_SENSORS] = {};
    uint32_t lastTof8x8FrameMs_ = 0;
    uint8_t digitalInputCount_ = 0;
    char digitalInputNames_[MAX_DIGITAL_INPUTS][25] = {};
    DigitalInputSensor digitalInputs_[MAX_DIGITAL_INPUTS];
    uint8_t ultrasoundSensorCount_ = 0;
    char ultrasoundSensorNames_[MAX_ULTRASOUND_SENSORS][25] = {};
    UltrasoundSensor ultrasoundSensors_[MAX_ULTRASOUND_SENSORS];
    int8_t activeUltrasoundIndex_ = -1;
    uint8_t nextUltrasoundIndex_ = 0;
    enum NavigationState : uint8_t {
        NAV_IDLE = 0,
        NAV_SEEK_WALL = 1,
        NAV_INITIAL_TURN = 2,
        NAV_FOLLOW_WALL = 3,
        NAV_CORNER_TURN = 4,
        NAV_SWEEP = 5,
        NAV_LANE_TURN_OUT = 6,
        NAV_LANE_SHIFT = 7,
        NAV_LANE_TURN_IN = 8,
        NAV_COMPLETE = 9
    };
    bool navigationActive_ = false;
    uint8_t navigationState_ = 0;
    uint32_t lastRangePollMs_ = 0;
    uint32_t lastMotionSampleMs_ = 0;
    bool lastImuSampleValid_ = false;
    uint16_t navigationFrontMm_ = 0;
    uint16_t navigationLeftMm_ = 0;
    uint16_t navigationRightMm_ = 0;
    uint32_t navigationMotionStartedMs_ = 0;
    float navigationHeadingReferenceDeg_ = 0.0f;
    float navigationTargetHeadingDeg_ = 0.0f;
    int32_t navigationShiftStartEncoder1_ = 0;
    int32_t navigationShiftStartEncoder2_ = 0;
    uint8_t navigationLaneIndex_ = 0;
    bool navigationSweepTurnRight_ = true;
    uint16_t navigationSweepLeftReferenceMm_ = 0;
    uint16_t navigationSweepRightReferenceMm_ = 0;
    bool navigationSweepLeftReferenceValid_ = false;
    bool navigationSweepRightReferenceValid_ = false;
    int16_t navigationSweepLateralErrorMm_ = 0;
    bool navigationMotionConsistent_ = true;

    static void dispatch(JsonDocument& message, void* context);
    void handleMessage(JsonDocument& message);
    void handleCommand(JsonDocument& message);
    void handleParameter(JsonDocument& message);
    void sendDefinitions();
    void sendState();
    void sendTelemetry();
    void initialiseTofSensors();
    void initialiseDigitalInputs();
    void initialiseUltrasoundSensors();
    void updateUltrasoundSensors();
    void updateMotionAndRangeSensors();
    void updateNavigation();
    void stopNavigation(const char* reason = nullptr);
    void readTofSensors();
    void updateTof8x8();
    void sendTof8x8Frame();
    void updateAutomaticServoRead();
    void updateTrackingState(float absoluteAngle);
    JsonObject findParameter(const char* name);
    JsonObject findCommand(const char* name);
};
