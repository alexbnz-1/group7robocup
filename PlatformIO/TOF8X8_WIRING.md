# SEN0628 8x8 TOF wiring and test guide

## Hardware configuration

The supplied 106_TOF_X8 is the DFRobot SEN0628. Configure its switches for I2C
address **0x33** and completely remove and restore sensor power after changing
them. The current hardware-isolation test connects it directly to raw I2C1 and
the firmware therefore uses Teensy `Wire1`.

## Wiring to the CPU board

For the current isolation test, connect the sensor directly to **raw I2C1 /
Wire1**. Match signals by the PCB labels rather than relying only on colours:

| CPU raw I2C1 label | SEN0628 label | Purpose |
| --- | --- | --- |
| `3V` | `VCC` / `+` | 3.3 V sensor power |
| `G` | `GND` / `-` | Ground |
| `SC` | `SCL` / `C` | I2C clock |
| `SD` | `SDA` / `D` | I2C data |

Do not connect it to a Serial socket and do not connect the sensor's USB-C port
to the Teensy. USB-C is an alternative computer connection, not the robot I2C
connection.

The SEN0628 is isolated on I2C1 for this test. Its expected address is 0x33.

## Software behaviour

`lib/Tof8x8` wraps the official `DFRobot_MatrixLidar` driver. It detects the
sensor, selects 8x8 mode, retrieves all 64 row-major distances in millimetres,
and reports availability/read status without trapping the rest of the robot in
an infinite retry loop.

The Bluetooth workflow sends a `tof_8x8` frame every 250 ms. The desktop app's
**8x8 TOF** tab displays Y0-Y7 from top to bottom and X0-X7 from left to right,
matching the manufacturer's data ordering. Each square shows millimetres and is
coloured red for near objects through to blue for far objects.

## First test

1. Confirm the sensor is configured for I2C address 0x33.
2. Connect the four signals above with robot power off.
3. Power-cycle the sensor and Teensy.
4. Start the debug GUI and connect to COM10.
5. Open **8x8 TOF**.
6. Move a flat target across the field of view and confirm the coloured region
   moves across the grid.

If the tab says unavailable, first check the mode/address selectors, power-cycle
the sensor, and verify SDA/SCL have not been swapped.

Manufacturer documentation:

- https://wiki.dfrobot.com/sen0628/docs/21562
- https://github.com/DFRobot/DFRobot_MatrixLidar
