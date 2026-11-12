# PID Patrol Bot: Production Bring-Up and Test Guide

This guide is for the **production** firmware in SensorNano/ and BalanceNano/. The diagnostic sketches in SensorTest/ and BalanceTest/ are optional bench tools; they are not required for normal operation.

## 1. Before powering the robot

1. Put the robot in a clear, padded test area.
2. Keep the motor supply physically disconnected.
3. Check all wiring against TWO_NANO_SETUP.md.
4. Confirm both Nanos share GND, but their I2C buses remain separate.
5. Check the HC-05 RX voltage divider/level shifting and both VL53L0X XSHUT voltage limits.
6. Install the Arduino AVR Nano core, DFRobot_BMI160 and Pololu VL53L0X 1.3.1.
7. Keep a physical motor-power disconnect within reach for every powered motor test.

## 2. Upload the production firmware

Upload the two production sketches from the **same repository branch/revision**:

1. Switch all power off.
2. Disconnect the inter-Nano D0/D1 wires.
3. Upload SensorNano/SensorNano.ino to the Sensor Nano.
4. Upload BalanceNano/BalanceNano.ino to the Balance Nano.
5. Reconnect the UART crossed: Balance D1 TX → Sensor D0 RX; Sensor D1 TX → Balance D0 RX.
6. Connect common GND and power the logic boards with motor power still disconnected.

Do not leave one Nano running the old XOR-checksum protocol while the other runs the CRC-8 protocol. The two RobotLink.h copies must match.

## 3. Startup and sensor checks

1. Place the robot on a level surface and hold the chassis at its true upright reference angle.
2. Keep it completely still while the Balance Nano performs its startup IMU calibration.
3. The Balance Nano should send READY MANUAL after a successful BMI160 initialization and calibration.
4. Pair the phone with the HC-05 and connect using a Bluetooth Classic serial terminal at **9600 baud**. Set the line ending to Newline or CRLF.
5. Send STATUS; expect a response containing the current mode and obstacle distance.
6. Check the rotating diagnostic messages:
   - P:...,G:... — pitch and pitch rate, scaled by 10.
   - A:...,L:...,R:... — controller output and left/right motor commands.
   - ST:...,S:...,F:...,R:... — driver standby, sensor freshness, front and rear floor safety.
   - IMU:...,SL:...,CF:... — IMU data age, surface-loss state and controller fault.
7. Check that front and rear floor readings are valid and independently respond to the floor beneath each sensor. The current floor acceptance limit is 220 mm.
8. Check that the RCWL-9610 returns a valid distance. The default navigation limit is 35 cm; valid navigation readings must be 1–90 cm.

If the phone shows no controller messages, power off and check crossed TX/RX, common GND, HC-05 pairing, and the 38400 baud inter-Nano link. Do not connect the two TX pins together.

## 4. First balance test

The balance controller runs as soon as startup calibration succeeds; it is not armed by a Bluetooth command. Keep the motor supply disconnected until the previous checks pass.

1. Raise the drive wheels and restrain the chassis securely.
2. Connect motor power only when ready, with a hand on the physical disconnect.
3. Verify that both wheels respond and that the controller attempts to correct a tilt rather than drive farther into it.
4. If motor direction is wrong, disconnect motor power and correct the motor wiring or direction mapping deliberately.
5. Do not proceed to a floor test if the chassis accelerates away, oscillates violently, or a motor fails to respond.
6. For the first floor trial, use a clear padded area, keep a hand on the disconnect, and support the robot until it can be released safely.

## 5. Bluetooth movement commands

The balance loop remains active in manual mode. Commands request short travel actions; they do not replace balance control.

| Command | Expected behavior |
| --- | --- |
| MANUAL | Cancel patrol and remain in upright balancing mode |
| FWD | Forward jog for 650 ms; requires safe front floor and valid clear ultrasonic data |
| REV | Reverse jog for 650 ms; requires safe rear floor |
| LEFT / RIGHT | Short forward arc; requires forward safety |
| PIVOT_L / PIVOT_R | Turn request; requires both floor sensors |
| UTURN_L / UTURN_R | Longer forward-arc turn request |
| STOP | Cancel travel, cancel patrol, and return to manual upright balancing |
| PATROL | Start autonomous patrol when valid sensor data is available |
| DIST=45 | Save 45 cm as the forward obstacle limit |
| STATUS | Report mode and saved obstacle limit |

Every travel request is time-limited. Send a new command to request another jog. If a command is blocked, restore valid sensor readings rather than bypassing the safety check.

## 6. Calibration and PID tuning

Follow CALIBRATION_GUIDE.md. Change one tuning variable at a time and record the result. Do not change the communication protocol or safety thresholds while tuning the balance response.

## Troubleshooting

| Symptom | First checks |
| --- | --- |
| No READY MANUAL | BMI160 supply, GND, A4/A5, SAO address 0x68, stillness during startup |
| ERR BMI160 | BMI160 wiring, supply and I2C address |
| ERR IMU CAL | Chassis moved/vibrated during calibration, sensor mounting or wiring |
| No Bluetooth response | HC-05 pairing, 9600 baud, D2/D3, RX voltage divider, newline ending |
| No inter-Nano data | Both boards powered, crossed D0/D1, common GND, both sketches from same revision |
| Floor data invalid | ToF power, SDA/SCL, individual XSHUT wiring, breakout voltage |
| Ultrasonic invalid | RCWL-9610 I2C mode, address 0x57, SDA/SCL, target range |
| Forward command blocked | Front floor must be valid and ultrasonic distance must be valid and beyond the obstacle limit |
| Robot drives the wrong way | Disconnect motor supply; check motor polarity and mapping |
| Robot oscillates or vibrates | Stop the trial; record pitch, pitch rate and motor output before changing one PID value |

## Final safety checks

- [ ] Both Nanos use production firmware from the same branch/revision.
- [ ] Both RobotLink.h copies match.
- [ ] BMI160 starts and calibrates while the chassis is still and upright.
- [ ] Front and rear floor sensors respond independently.
- [ ] Ultrasonic data is valid and changes with a target.
- [ ] Bluetooth commands and controller responses work.
- [ ] Motor direction is correct with wheels raised.
- [ ] Forward is blocked by close, invalid or stale ultrasonic data.
- [ ] STOP cancels travel and patrol while leaving balance control active.
- [ ] The physical motor disconnect is within reach.

This is experimental hardware, not a certified safety system. Do not test near people, stairs, pets, roads or fragile objects.
