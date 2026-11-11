# PIDpatrolbot

Two-Arduino-Nano self-balancing patrol robot.

## Firmware architecture
- **BalanceNano/** — BMI160, TB6612FNG motor control, balance/PID loop, movement safety checks, patrol behavior, and EEPROM obstacle-distance setting.
- **SensorNano/** — front/rear VL53L0X floor sensing, RCWL-9610 front distance sensing, HC-05 Bluetooth bridge, and inter-Nano sensor packets.
- **BalanceTest/** and **SensorTest/** — bench diagnostics, not production balance firmware.
- **RobotLink.h** — packet framing shared by the two nodes. Keep production copies synchronized.

## Authoritative wiring
Pin constants in the production sketches are authoritative. These references match those constants:
- [Two-Nano setup and pin table](TWO_NANO_SETUP.md)
- [Wiring diagram (SVG)](Two_Nano_Wiring.svg)

### Balance Nano
| Function | Pin |
| --- | --- |
| TB6612 PWMA / AIN1 / AIN2 | D5 / D3 / D2 |
| TB6612 PWMB / BIN1 / BIN2 | D6 / A2 / A1 |
| TB6612 STBY | D8 |
| Buzzer / status LED | D12 / D13 |
| BMI160 SDA / SCL | A4 / A5 |
| Inter-Nano UART RX / TX | D0 / D1 |

### Sensor Nano
| Function | Pin |
| --- | --- |
| HC-05 SoftwareSerial RX / TX | D2 / D3 |
| Front / rear VL53L0X XSHUT | A2 / A1 |
| Status LED | D13 |
| Shared sensor I2C SDA / SCL | A4 / A5 |
| Inter-Nano UART RX / TX | D0 / D1 |

## Testing
Follow the [step-by-step testing guide](TESTING_GUIDE.md) to test SensorTest and BalanceTest in order, verify sensor readings and the inter-Nano link, check safety responses, and then restore the production firmware.

## Build
Install the Arduino AVR Nano core, DFRobot_BMI160, and Pololu VL53L0X. Compile each sketch for the classic Nano / ATmega328P:

```sh
arduino-cli compile --fqbn arduino:avr:nano BalanceNano
arduino-cli compile --fqbn arduino:avr:nano SensorNano
arduino-cli compile --fqbn arduino:avr:nano BalanceTest
arduino-cli compile --fqbn arduino:avr:nano SensorTest
```

The two production sketches each contain a copy of `RobotLink.h`; update both copies together if the protocol changes.

## Safety
Forward movement requires fresh floor data and a valid ultrasonic reading beyond the obstacle limit. Missing or expired ultrasonic data blocks forward travel. Reverse movement requires a valid rear floor reading; pivoting requires both floor sensors to confirm floor. The sensor node retries failed ToF initialization and expires stale ultrasonic measurements. The balance node refuses startup if IMU calibration is incomplete or unstable.

This is an experimental robot, not a certified safety system. Test restrained first, keep motor power disconnectable, and only progress to low-speed trials in a clear, padded area away from people, stairs, pets, and fragile objects.
