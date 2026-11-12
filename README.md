# PIDpatrolbot

Two-classic-Nano self-balancing patrol robot.

## Ready firmware

- **Balance Nano firmware:** BalanceNano/BalanceNano.ino — BMI160 pitch estimation, self-balancing PID, TB6612FNG motor control, yaw/movement commands, sensor safety, patrol mode, and EEPROM obstacle-distance setting.
- **Sensor Nano firmware:** SensorNano/SensorNano.ino — two VL53L0X floor sensors, RCWL-9610 front distance sensor, HC-05 Bluetooth bridge, and inter-Nano communication.
- **Link headers:** BalanceNano/RobotLink.h and SensorNano/RobotLink.h — matching packet protocol. Keep both copies identical.
- **Wiring:** TWO_NANO_SETUP.md and Two_Nano_Wiring.svg.
- **Upload and calibration:** CALIBRATION_GUIDE.md.
- **Testing and commands:** TESTING_GUIDE.md.

BalanceTest/ and SensorTest/ are optional diagnostic sketches. They are not required for normal operation. Use the two production sketches above for normal robot operation.

## Current calibration starting point

| Setting | Value |
| --- | ---: |
| Pitch Kp | 35.0 |
| Pitch Ki | 0.8 |
| Pitch Kd | 7.2 |
| Upright clearance | 2.0° |
| Motor minimum PWM | 22 |
| Maximum PWM | 210 |
| Motor slew step | 8.5 |

These are starting values for physical calibration, not a claim that the robot is already perfectly tuned. Keep the gains in BalanceNano/BalanceNano.ino as the single tuning location.

## Communication

- Inter-Nano hardware UART: **38400 baud**, crossed D1/D0, common GND.
- HC-05 Bluetooth serial: **9600 baud**, newline-terminated commands.
- Binary inter-Nano packets: A5 5A, type, sequence, payload length, payload (maximum 20 bytes), CRC-8 (polynomial 0x07).
- The parser discards malformed or incomplete packets and resets after a 20 ms inter-byte gap.
- Sensor frames are sent every 50 ms. Balance diagnostics are deliberately paced to avoid long bursts over the Sensor Nano SoftwareSerial Bluetooth connection.
- Upload both production sketches from the same branch/revision. The two RobotLink.h files must match.

## Build

Install the Arduino AVR Nano core and these libraries:

- DFRobot_BMI160
- Pololu VL53L0X (1.3.1)

Compile from the repository root:

~~~sh
arduino-cli compile --fqbn arduino:avr:nano BalanceNano
arduino-cli compile --fqbn arduino:avr:nano SensorNano
~~~

Some classic Nano clones require **ATmega328P (Old Bootloader)** in the IDE. Disconnect both D0/D1 inter-Nano wires from the Nano being uploaded, then reconnect with power off.

## Safety

Forward travel requires fresh front-floor data and a valid ultrasonic reading beyond the obstacle limit. Missing, invalid, or stale distance data blocks forward travel. Reverse requires valid rear-floor data; fixed-point pivots require both floor sensors. Loss of sensor data cancels travel commands while the balance loop continues. A confirmed loss of floor under both ends disables the motor driver; automatic recovery is permitted only after both floor readings are valid and safe for the recovery interval.

This is experimental hardware, not a certified safety system. Keep a physical motor-power disconnect within reach. Complete the restrained checks in the guides before a low-speed floor trial.
