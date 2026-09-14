# Debug GUI configuration

Edit `debug_config.json` to add or remove controls in the desktop GUI. Building
the PlatformIO project validates the JSON and embeds it in the Teensy firmware.
Do not edit `include/debug_config.generated.h`; it is generated automatically.

The desktop GUI also reads this same file directly at startup. This guarantees
that controls appear even if the wireless serial link drops a definition frame;
definitions received from the Teensy still update the existing controls.

## Parameters

Add an object to the `parameters` array:

```json
{
  "name": "drive.pid.kp",
  "label": "Drive Kp",
  "description": "Proportional speed-controller gain",
  "datatype": "float",
  "value": 1.2,
  "min": 0.0,
  "max": 10.0,
  "step": 0.01
}
```

The library automatically advertises the parameter, accepts updates, clamps
numeric values to `min`/`max`, stores the current value in RAM, and confirms the
accepted value to the GUI. Removing the object removes its GUI control.

The special parameter `debug.telemetry_interval_ms` also controls the actual
telemetry interval. Other parameters are available through the workflow's JSON
registry; connecting them to new robot algorithms requires a matching C++ use.

## TOF sensors

Add each VL53 TOF sensor to the top-level `tof_sensors` array. No change to
`main.cpp` or the desktop GUI is required:

```json
"tof_sensors": [
  {"name": "front", "label": "Front TOF", "type": "long", "port": "XSHUT1"},
  {"name": "left", "label": "Left TOF", "type": "short", "port": "XSHUT2"}
]
```

- `name` is a unique short identifier used in telemetry names.
- `label` documents the physical sensor location.
- `type` is `long` for VL53L1X or `short` for VL53L0X.
- `port` is `XSHUT0` through `XSHUT7` (the integers 0-7 are also accepted).

Names and ports must be unique, and at most eight sensors can be configured.
The firmware holds all XSHUT lines low, starts listed sensors one at a time,
and assigns address `0x30 + XSHUT number`. Each entry appears separately in
Live Telemetry as `tof.<name>.available`, `tof.<name>.timed_out`, and
`tof.<name>.distance_mm`. Unlisted sensors remain shut down.

## Commands

Add an object to the `commands` array. The `action` selects firmware behaviour.
The optional `category` is displayed as a badge inside the command card, for
example `System`, `Herkulex`, `HX12K`, or `203 DC Motor`:

- `ping`: responds with a log and state.
- `stop`: disables all Herkulex torque, sends the 203 DC motor neutral, and
  reports stopped state.
- `run`: leaves stopped mode without commanding motion or enabling torque.
- `herkulex_angle`: validates and sends a Herkulex angle command.
- `herkulex_read_angle`: requests position feedback and publishes the measured
  angle as `servo.measured_angle_deg` telemetry. After the first successful
  manual read, the workflow continues reading that servo every 200 ms.
- `herkulex_zero_here`: reads the current position without moving, stores it as
  a temporary software zero, and makes later `herkulex_angle` targets relative
  to that point. The zero is cleared by a Teensy reset or power cycle.
- `herkulex_velocity`: switches a DRS-0101 into continuous-turn mode and applies
  signed speed from -1023 to +1023. It requires Debug Mode and Run state.
- `herkulex_stop_velocity`: commands zero velocity and disables that servo's
  torque. Use it before setting zero or returning to position control.
- `dc_motor_203_speed`: applies independent signed -100% to +100% commands to
  the two 203 DC motor channels on Serial7 TX7/pin 29 and RX7/pin 28. It
  requires Debug Mode and Run.
- `dc_motor_203_stop`: immediately writes the 1500 us neutral pulse to both
  channels.
- `hx12k_angles`: accepts independent selection flags and 0-135 degree targets
  for level-shifter outputs A-D. Only selected outputs are changed. It requires
  Debug Mode and Run.
- `hx12k_disable`: detaches the PWM output. Global STOP and leaving Debug Mode
  also detach all four outputs.
- `tof_read_all`: immediately reads every configured TOF sensor. All sensors
  are also read automatically with normal telemetry packets.

Example:

```json
{
  "name": "set_herkulex_angle",
  "label": "Set Herkulex angle",
  "action": "herkulex_angle",
  "debug_mode_required": true,
  "args": [
    {"name": "id", "type": "int", "min": 1, "max": 253, "default": 1},
    {"name": "angle_deg", "type": "float", "min": -160, "max": 160, "step": 1, "default": 0},
    {"name": "move_time_ms", "type": "int", "min": 50, "max": 2850, "step": 50, "default": 500}
  ]
}
```

Removing a command object removes its GUI button. A new `action` name also
needs a corresponding implementation in `BluetoothDebugWorkflow.cpp`; JSON can
describe controls and select existing actions, but cannot create new physical
robot behaviour by itself.

## Main program

`src/main.cpp` intentionally contains only construction plus `begin()` and
`update()`. Normal robot code can run beside `debugGui.update()` later without
moving protocol or GUI-definition code back into `main.cpp`.
