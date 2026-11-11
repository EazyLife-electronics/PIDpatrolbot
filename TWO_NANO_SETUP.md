# PID Patrol Bot: Two Nano Build

This version uses two classic Nano/ATmega328P boards. The balance Nano owns the BMI160, TB6612FNG, and time-critical balance loop. The sensor Nano owns both downward VL53L0X units, the front-facing RCWL-9610 ultrasonic module, and the HC-05 Bluetooth link.

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
| Onboard status LED | D13 (held HIGH) |
| BMI160 SDA / SCL | A4 / A5 |
| Purple GY-BMI160 SAO | GND (selects address `0x68`) |
| UART to sensor Nano RX / TX | D0 / D1 |

### Sensor / Bluetooth Nano

| Signal | Nano pin |
| --- | --- |
| UART to balance Nano RX / TX | D0 / D1 |
| HC-05 TX to Nano RX | D3 |
| Nano TX to HC-05 RX | D2 through a 5 V-to-3.3 V divider unless the breakout explicitly includes level shifting |
| Front floor sensor XSHUT | A2 |
| Rear floor sensor XSHUT | A1 |
| Onboard status LED | D13 (held HIGH) |
| Both ToFs, RCWL-9610 SDA / SCL | A4 / A5 |
| Sensor Nano I2C addresses | Front ToF `0x30`, rear ToF `0x31`, RCWL-9610 `0x57` |

Cross the two UART pairs at 38400 baud: balance D1 (TX) to sensor D0 (RX), and sensor D1 (TX) to balance D0 (RX). Connect grounds. Disconnect these D0/D1 link wires while uploading firmware to either Nano; those pins are also the bootloader/USB serial pins. Motor power must use a separate supply appropriate for the motors, with a common ground. Keep a physical motor-power disconnect within reach.

## Pin Rerouting And Timer Notes

Most direction, standby, XSHUT, and buzzer signals are ordinary digital GPIO and can move to another unused digital pin if the matching `const uint8_t` in that sketch is changed too. Keep each XSHUT on its own pin. Avoid moving signals onto a pin already used by another part on the same Nano.

These pins have fixed roles in the current design:

| Pins | Why they are reserved |
| --- | --- |
| A4 / A5 on both Nanos | Hardware I2C (`Wire`) for the BMI160 on the balance Nano and ToF/RCWL sensors on the sensor Nano |
| D0 / D1 on both Nanos | Hardware UART for the inter-Nano link and also the Nano bootloader/USB serial connection |
| D13 on both Nanos | Reserved as the onboard status LED output, held HIGH from startup |
| Sensor Nano D3 | HC-05 receive input for `SoftwareSerial`; if moved, choose another unused pin supported by AVR pin-change interrupts and update `BT_RX_PIN` |
| Sensor Nano D2 | HC-05 transmit output; if moved, update `BT_TX_PIN` and preserve the voltage divider/level shifting on the Nano-to-HC-05 RX line |

PWM can only move to hardware PWM pins: D3, D5, D6, D9, D10, or D11. The current balance sketch uses D5/D6, which are Timer0 PWM outputs. Timer0 also provides Arduino `millis()`/`micros()`; ordinary `analogWrite()` on D5/D6 is fine, but do not reconfigure Timer0 if the control timing is needed. D9/D10 use Timer1 and are available in this firmware, but adding a library such as `Servo` may claim Timer1 and change that. The balance loop itself is scheduled with `micros()`; it does not reserve a hardware timer output pin.

The TMB12A05 is self-oscillating, so the balance sketch drives D12 HIGH/LOW for alert pulses and does not use `tone()`. This does not claim Timer2, so D3/D11 are no longer excluded by the buzzer; D3 is still assigned to TB6612 AIN2. If the buzzer module draws more than a Nano GPIO can safely supply (about 20 mA recommended), drive it through a transistor instead of directly from D12.

For a reroute, preserve the role: motor PWM stays on an available PWM pin; direction, standby, and XSHUT only need GPIO; I2C stays on A4/A5; and the inter-Nano hardware UART stays on D0/D1 unless the serial architecture is deliberately changed. Recompile both sketches after any pin-map change.

For the purple GY-BMI160 3.5-5V breakout revision, VCC can use Nano 5V and SDA/SCL connect directly to the balance Nano's A4/A5; tie SAO to GND to select the `0x68` address used by the firmware. Confirm your exact board revision has its regulator and I2C pull-ups before applying 5V. The balance Nano's BMI160 I2C bus is separate from the sensor Nano's A4/A5 bus; do not connect the two buses together. Many VL53L0X breakouts likewise include regulation and bidirectional I2C level shifting and can use direct 5V-side I2C, but check the exact module. XSHUT may bypass its level shifter and connect to the 2.8V sensor, so verify its allowed input voltage before connecting a Nano output; add a suitable transistor or level shifter if needed. Do not put ordinary fixed resistor dividers on SDA/SCL. The RCWL-9610 shares the sensor Nano I2C bus; verify that module's I2C voltage/pull-ups too. Configure it for I2C with M1 open and M2 closed. Its fixed address `0x57` is distinct from the reassigned VL53L0X addresses.

The sensor sketch starts both ToFs held in shutdown, enables the front unit and changes its address, then enables and addresses the rear unit. The RCWL-9610 is triggered and read asynchronously from the sensor Nano, so its conversion delay cannot pause balance control.

## Build And Upload

Install `DFRobot_BMI160`, `VL53L0X` by Pololu (1.3.1), and `AlashUltrasonic` (1.0.9). The current sensor sketch uses the RCWL-9610's documented I2C transaction directly so it can schedule the conversion without blocking; the library is useful for stand-alone checks.

From the `PIDpatrolbot` folder, compile each sketch separately:

```powershell
arduino-cli compile --fqbn arduino:avr:nano --output-dir build/balance_nano BalanceNano
arduino-cli compile --fqbn arduino:avr:nano --output-dir build/sensor_nano SensorNano
```

Select **Arduino Nano / ATmega328P** in the IDE for each corresponding folder. Disconnect the inter-Nano UART while uploading, reconnect it afterward, then power the sensor/control logic with motor power disconnected. Calibrate the balance loop restrained with the wheels raised before allowing the robot to drive.

The UART packet definition is copied into each sketch folder because Arduino builds sketches in isolation. Keep the two `RobotLink.h` copies synchronized when changing the link protocol.

## Bluetooth Commands

Use a Bluetooth Classic serial-terminal app at 9600 baud, with newline or CRLF line endings. Pair with the HC-05, connect, then send one command per line:

| Command | Behavior |
| --- | --- |
| `MANUAL` or `M` | Manual mode; balance remains active with no travel command |
| `PATROL`, `ROAM`, or `A` | Autonomous patrol |
| `FWD` or `F` | Forward jog, 650 ms |
| `REV` or `B` | Reverse jog, 650 ms |
| `LEFT` or `L` / `RIGHT` or `R` | Forward arc turn |
| `PIVOT_L` or `PL` / `PIVOT_R` or `PR` | Fixed-point turn request |
| `UTURN_L` or `UL` / `UTURN_R` or `UR` | Forward-arc U-turn request |
| `STOP` or `S` | Cancel travel and continue balancing upright |
| `DIST=45` | Set and save the forward obstacle limit in cm (15-200) |
| `STATUS` or `?` | Report mode and obstacle limit |

The default forward obstacle limit is 35 cm and is stored in EEPROM after changing it. Movement requests expire; send a new jog command to continue. Turn angles are approximate because the robot has no wheel encoders and gyro-integrated yaw drifts. Tune the floor-distance limit in `BalanceNano.ino` to your sensor mounting height and actual floor readings.

An HC-05 in slave mode waits for the phone to connect; it cannot force a phone to reconnect by itself. Configure the phone app to reconnect to its paired device if that option exists. Most HC-05 breakouts stop accepting another connection while one is active. AT-mode commands vary among clones, so verify the exact module firmware before changing its role or binding settings.

## Safety Behavior And Limits

The balance loop runs on the balance Nano and never waits for a sensor read or Bluetooth command. If the sensor packet is stale or a required sensor is invalid, commanded travel is suppressed while the controller continues balancing. Forward travel requires fresh front-floor data and a valid ultrasonic reading above the configured limit; missing, invalid, or expired ultrasonic data is treated as blocked, not as a clear path. Reverse travel requires valid rear-floor data. A fixed-point pivot requires both floor sensors to confirm floor. The sensor Nano expires ultrasonic readings after 750 ms without a successful measurement and retries failed ToF initialization/timeouts. The balance Nano rejects IMU calibration if too few samples arrive or gyro data indicates excessive movement/vibration; keep the robot still during startup. If both floor sensors report no floor for the confirmation interval, motor standby is disabled and the fault latches until power is cycled.

The forward ultrasonic module does not protect the rear from collision while reversing. There are no wheel encoders, no measured stopping distance, and no certified safety controller. The floor threshold, PID gains, motor polarity, recovery behavior, and Bluetooth connection loss behavior require restrained tests in a clear, padded area. Do not test around people, stairs, pets, traffic, or fragile objects.

