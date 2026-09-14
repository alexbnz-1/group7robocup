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

## Commands

Add an object to the `commands` array. The `action` selects firmware behaviour:

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
