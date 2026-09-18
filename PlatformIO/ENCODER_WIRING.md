# Dual quadrature encoder board

The CPU board's **Digital Raw 2** connector exposes `3V, D5, D4, D3, D2, G, 3V`
across the signal header (check the printed PCB labels before plugging in).
The supplied `112_Encoder.ino` pairs the four signal pins as follows:

| Encoder | A signal | B signal |
| --- | --- | --- |
| 1 | D2 | D3 |
| 2 | D4 | D5 |

Connect the encoder board's ground to `G` and its supply to the appropriate
`3V` pin only if its own documentation confirms 3.3 V operation. Teensy 4.0
digital inputs are **not 5 V tolerant**; use a suitable level shifter if the
encoder board produces 5 V outputs. Do not use the old example's USB serial
printout as the robot interface; values are sent over the existing Bluetooth
debug telemetry.

`DualEncoder` uses interrupts on both A and B of each encoder (x4 quadrature
decoding). The GUI receives `encoder.1.count`, `encoder.2.count`, per-sample
`delta`, and `counts_per_s` at the normal telemetry rate. Negative values mean
the measured A/B phase order is opposite; swap A and B for that channel if the
direction convention should be reversed. **Zero both encoder counts** resets
both totals and speed estimates without moving a motor.

Counts/second is not RPM. To calculate physical RPM, first measure or obtain
the exact x4 counts per revolution of the installed encoder *at the shaft being
measured*. Motor-shaft and gearbox-output counts per revolution differ.

The older D2-D5 drivetrain/reel constants in `include/config.h` are inactive
placeholders and conflict with Digital Raw 2. Do not enable outputs on those
pins while this encoder board is connected.
