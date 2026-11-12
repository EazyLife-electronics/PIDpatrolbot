# PID Patrol Bot: Two Nano Build

This build uses two classic Arduino Nano / ATmega328P boards. The **Balance Nano** owns the BMI160, TB6612FNG and time-critical balance loop. The **Sensor Nano** owns the two downward VL53L0X sensors, front-facing RCWL-9610 and HC-05 Bluetooth bridge.

## Wiring

### Balance Nano

| Signal | Nano pin |
| --- | --- |
| TB6612 PWMA | D5 (PWM) |
| TB6612 AIN1 | D3 |
| TB6612 AIN2 | D2 |
| TB6612 PWMB | D6 (PWM) |
| TB6612 BIN1 | A2 |
| TB6612 BIN2 | A1 |
| TB6612 STBY | D8 |
| Buzzer | D12 |
| Onboard status LED | D13 |
| BMI160 SDA / SCL | A4 / A5 |
| GY-BMI160 SAO | GND for I2C address 0x68 |
| Inter-Nano UART RX / TX | D0 / D1 |

### Sensor Nano

| Signal | Nano pin |
| --- | --- |
| Inter-Nano UART RX / TX | D0 / D1 |
| HC-05 TX to Nano RX | D2 |
| Nano TX to HC-05 RX | D3 through a 5 V-to-3.3 V divider unless the breakout explicitly level-shifts |
| Front VL53L0X XSHUT | A2 |
| Rear VL53L0X XSHUT | A1 |
| Onboard status LED | D13 |
| Shared sensor I2C SDA / SCL | A4 / A5 |
| Front VL53L0X address | 0x30 |
| Rear VL53L0X address | 0x31 |
| RCWL-9610 address | 0x57 |

Cross the inter-Nano UART at **38400 baud**:

- Balance D1 (TX) → Sensor D0 (RX)
- Sensor D1 (TX) → Balance D0 (RX)
- Balance GND ↔ Sensor GND

Never connect TX to TX. Disconnect both D0/D1 link wires from the Nano being programmed; these are also its USB bootloader serial pins. Reconnect only with power off.

Motor power must be a separate supply suitable for the motors. Join logic and motor grounds. Do not power TT motors from a Nano 5 V pin. Keep a physical motor-power disconnect within reach.

## Inter-Nano packet protocol

Both production folders contain their own RobotLink.h; Arduino builds each sketch folder separately. **The two files must be identical and both Nanos must run firmware from the same revision.**

Frame layout:

| Field | Size |
| --- | ---: |
| Sync bytes A5 5A | 2 bytes |
| Packet type | 1 byte |
| Sequence | 1 byte |
| Payload length | 1 byte |
| Payload | 0–20 bytes |
| CRC-8 | 1 byte |

CRC-8 uses polynomial 0x07, initial value 0x00, over packet type, sequence, payload length and payload. The parser validates packet type and payload length, rejects bad CRC, and resets an incomplete frame after a 20 ms inter-byte gap. Sensor frames are sent every 50 ms.

The Balance Nano sends one compact diagnostic frame every 250 ms, rotating through pitch/gyro, controller/motor output, sensor safety state, and fault/IMU age. This pacing is intentional: the Sensor Nano uses SoftwareSerial for the HC-05, which cannot receive Bluetooth bytes while it is transmitting. Do not restore a burst of multiple diagnostic lines every 250 ms.

## I2C and module notes

- The BMI160 bus on the Balance Nano is separate from the Sensor Nano sensor bus. Do not join the two A4/A5 buses.
- Both VL53L0X modules share the Sensor Nano I2C bus but use separate XSHUT pins. Startup holds both in shutdown, initializes the front unit at 0x30, then the rear at 0x31.
- The RCWL-9610 shares that Sensor Nano bus at 0x57. Configure I2C mode with M1 open and M2 closed.
- Verify the exact breakout supply, pull-ups and logic-level requirements. XSHUT may connect directly to the sensor IC and may not tolerate a 5 V Nano output; use suitable level shifting if required.
- For the specified purple GY-BMI160 3.5–5 V breakout revision, VCC may use Nano 5 V and SAO to GND selects 0x68; verify your exact board revision before applying 5 V.
- The HC-05 RX input must not receive 5 V logic directly unless the breakout explicitly includes level shifting.

## Build and upload

Install the Arduino AVR Nano core, DFRobot_BMI160 and Pololu VL53L0X 1.3.1.

From the repository root:

~~~sh
arduino-cli compile --fqbn arduino:avr:nano BalanceNano
arduino-cli compile --fqbn arduino:avr:nano SensorNano
~~~

For some Nano clones, select **ATmega328P (Old Bootloader)**. Upload Sensor Nano and Balance Nano separately with the inter-Nano UART disconnected from the board being programmed. Reconnect with power off.

## Bluetooth commands

Connect to the HC-05 using a Bluetooth Classic serial-terminal app at **9600 baud**, with newline or CRLF line endings. Send one command per line.

| Command | Behavior |
| --- | --- |
| MANUAL or M | Select manual mode; balance remains active |
| PATROL, ROAM or A | Start autonomous patrol |
| FWD or F | Forward jog for 650 ms |
| REV or B | Reverse jog for 650 ms |
| LEFT or L / RIGHT or R | Short forward arc |
| PIVOT_L or PL / PIVOT_R or PR | Fixed-point turn request |
| UTURN_L or UL / UTURN_R or UR | Forward-arc U-turn request |
| STOP or S | Cancel travel and return to manual upright balancing |
| DIST=45 | Save a forward obstacle limit in cm (15–200) |
| STATUS or ? | Report current mode and obstacle limit |

Manual movement commands and STOP switch out of patrol mode. A jog expires automatically; send a new command to continue. The default obstacle limit is 35 cm and is stored in EEPROM when changed. RCWL readings are accepted for navigation only from 1 to 90 cm; invalid, out-of-range and stale readings block forward travel.

## Behavior and limits

The balance loop does not wait for sensor ranging or Bluetooth input. If the sensor frame is stale or a required sensor is invalid, travel commands are suppressed while the balance loop continues. Forward requires fresh front-floor data and a valid ultrasonic reading beyond the configured limit. Reverse requires valid rear-floor data. A pivot requires both floor sensors.

If both floor sensors report no floor for the confirmation interval, the motor driver is disabled. With the current non-latching setting, it can recover only after both floor sensors report safe floor continuously for the recovery interval; otherwise the controller remains stopped. IMU startup calibration must be completed while the chassis is still and held at its true upright reference.

There are no wheel encoders, no measured stopping distance, and no certified safety controller. Yaw turns are approximate and gyro-integrated yaw drifts. The forward ultrasonic sensor does not protect the rear during reverse. Perform all commissioning in a clear padded area with a physical motor-power disconnect.
