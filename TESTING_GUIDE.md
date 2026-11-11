# PID Patrol Bot: Step-by-Step Testing Guide

This guide explains how to use the two sketches in **BalanceTest/** and **SensorTest/** before installing the production firmware. Follow the stages in order. Do not skip the sensor-only checks.

> **Important:** These are diagnostic/test sketches, not the normal patrol firmware. Uploading a test sketch replaces whatever sketch is currently on that Nano. Upload `SensorTest` to the sensor Nano and `BalanceTest` to the balance Nano only for the tests described here.

## 0. Prepare a safe test setup

1. Put the robot on a stable workbench or floor area with plenty of clear space.
2. **Disconnect the motor battery / motor supply physically.** Do not rely only on a software STOP command. Keep the motor supply disconnected during sensor tests, wiring checks, and all initial balance tests.
3. Keep the wheels clear of the floor and the chassis securely restrained whenever motor power is later connected.
4. Check that both Nanos share GND and that the sensor modules have the correct supply voltage and common ground.
5. Install the required Arduino libraries:
   - DFRobot_BMI160
   - Pololu VL53L0X (the setup notes specify version 1.3.1)
6. Select **Arduino Nano** and the correct processor/bootloader option for your board. Many classic Nano clones use **ATmega328P (Old Bootloader)**; if upload fails, try that processor option.
7. Close Serial Monitor / Bluetooth terminal before uploading if it is holding the serial connection.

**Never connect or disconnect the D0/D1 inter-Nano wires while uploading.** Those pins are also used by the USB bootloader. Disconnect both UART link wires from the Nano being programmed, upload, then reconnect them after upload and power is off.

## 1. Test the Sensor Nano by itself

Purpose: confirm the two VL53L0X floor sensors, RCWL-9610 distance sensor, and HC-05 Bluetooth telemetry before involving the balance controller.

### Upload

1. Leave motor power disconnected.
2. Disconnect the D0/D1 UART link between the two Nanos.
3. Connect the **sensor Nano** to USB.
4. Open `SensorTest/SensorTest.ino` in Arduino IDE and upload it to the sensor Nano.
5. Disconnect USB if needed, reconnect the HC-05 and sensor wiring, and power the sensor Nano.
6. Pair the phone with the HC-05 using the phone's Bluetooth settings.
7. Open a Bluetooth Classic serial-terminal app, connect to the HC-05, set **9600 baud**, and select line ending **Newline** or **CRLF**.

### Check the floor sensors

The terminal should repeatedly show lines similar to:

- `FLOOR_F,OK:xxxmm`
- `FLOOR_R,OK:xxxmm`

The exact distances depend on sensor height and the surface. Keep a flat, non-reflective target beneath each sensor at its normal mounting distance.

1. Hold a flat target under the front sensor. Move it closer and farther; the `FLOOR_F` value should change.
2. Repeat under the rear sensor and watch `FLOOR_R`.
3. Cover or point one sensor away from a target briefly. Its line may show `ERR,NA` if the reading times out.
4. Check that the front and rear readings respond independently.

At boot, `ERR,FRONT TOF INIT` or `ERR,REAR TOF INIT` means that sensor did not initialize. Check its power, GND, SDA/SCL, XSHUT wire, and module voltage compatibility. The two VL53L0X modules must have separate XSHUT wires as shown in the wiring guide.

### Check the front distance sensor

1. Keep a flat target in front of the RCWL-9610.
2. Move it to several measured distances within the module's usable range.
3. Watch for `ULTRA,OK:xxcm`; the displayed distance should change as the target moves.
4. Remove the target or move it outside the sensor's reliable range. The diagnostic sketch should report `ULTRA,INVALID,RAW:...` when the I2C read succeeds but the returned distance is outside the accepted range. It reports `ULTRA,I2C_TRIGGER_ERROR` or `ULTRA,I2C_READ_ERROR` for communication failures.

If the ultrasonic line remains invalid, verify that the RCWL-9610 is configured for I2C (M1 open, M2 closed), is at address `0x57`, and shares the sensor Nano's A4/A5 bus. Check the module's voltage and pull-up requirements.

### Understand the other telemetry

- `DATA,LINK:DOWN` is expected while the balance Nano is disconnected.
- `IMU:ERR` and `ARM:0` are expected without a working BalanceTest controller.
- `GYRO,...` values are controller telemetry and will not be meaningful until the balance Nano is connected.
- The sensor test sends readings to the balance Nano over D0/D1 at 38400 baud.

**Pass condition:** both floor readings respond independently, the front distance reading responds to a target, and the sensor Nano remains running without repeated initialization errors.

## 2. Connect both Nanos and test the sensor-to-controller link

Purpose: verify that SensorTest readings reach BalanceTest and that Bluetooth can carry controller status back to the phone.

1. Turn all power off.
2. Upload `BalanceTest/BalanceTest.ino` to the **balance Nano**. Disconnect its D0/D1 link wires during upload.
3. Upload `SensorTest/SensorTest.ino` to the **sensor Nano** if it is not already installed. Disconnect its D0/D1 link wires during upload.
4. Turn power off again. Connect the UART wires crossed:
   - Balance Nano D1 (TX) → Sensor Nano D0 (RX)
   - Sensor Nano D1 (TX) → Balance Nano D0 (RX)
   - Balance Nano GND ↔ Sensor Nano GND
5. Keep motor power disconnected. Power both logic boards.
6. Connect the phone's Bluetooth terminal to the HC-05 at **9600 baud**.

You should now see `DATA,LINK:UP` and controller status/gyro telemetry. The balance test also sends heartbeat messages internally at 38400 baud. If the phone continues to show `LINK:DOWN`, power off and check crossed TX/RX, common GND, and the D0/D1 connections.

**Do not connect the two Nano TX pins together.** TX from each board goes to RX on the other.

## 3. Test the BMI160 and balance controller with motors disabled

Purpose: verify that the balance Nano can initialize the BMI160 and receive fresh sensor packets without moving the robot.

1. Keep the motor supply physically disconnected and wheels clear.
2. Place the robot/chassis upright and hold it completely still during startup calibration.
3. Power-cycle both Nanos.
4. Look for `TEST IMU OK`, followed by `TEST DISARMED`.
5. Gently tilt the restrained chassis by hand and watch the `GYRO,P:...` telemetry change. Return it upright and keep it still.
6. Confirm `DATA,LINK:UP` continues.

If you see `TEST ERR IMU`, check BMI160 power, GND, SDA/SCL on A4/A5, and the SAO address strap (GND for address `0x68` on the specified breakout). Do not proceed to motor tests until the IMU initializes.

The BalanceTest calibration is a basic diagnostic calibration. It is not a substitute for the stricter startup checks in BalanceNano production firmware.

## 4. Check sensor safety indications

Keep motor power disconnected.

1. With a normal floor target under both downward sensors and a clear front path, check that both floor readings are within the test's `220 mm` floor limit and the ultrasonic reading is greater than the default `35 cm` obstacle limit.
2. Move a target in front of the ultrasonic module to within 35 cm. The terminal should show `EVENT,OBSTACLE`.
3. Move the target away again.
4. Briefly remove the target under the front floor sensor (do not move the robot toward an actual edge). Watch for `EVENT,FRONT EDGE`.
5. Repeat for the rear sensor and look for `EVENT,REAR EDGE`.

Restore both floor targets and a clear path before attempting to arm. Do not use a real table edge or staircase to test edge detection.

## 5. Test motor direction with the wheels raised

Only proceed after all previous checks pass.

1. Keep the robot securely restrained with **both drive wheels completely clear of the floor**.
2. Keep a hand on the physical motor-power disconnect. Connect motor power only for this short test.
3. Send `STATUS` if needed; the test firmware continuously reports heartbeat and gyro telemetry.
4. Send `ARM`. Expected reply: `ACK,ARMED`. If it replies `ERR ARM CHECK SENSORS`, do not bypass it; restore valid front/rear floor readings and a fresh sensor link.
5. Send `MANUAL`, then test one command at a time:
   - `FWD` — brief forward pulse (350 ms); needs front floor and clear valid ultrasonic reading beyond 35 cm.
   - `REV` — brief reverse pulse (350 ms); needs rear floor reading.
   - `LEFT` / `RIGHT` — short forward arcs; need the forward safety checks.
   - `PIVOT_L` / `PIVOT_R` — short opposite-wheel pivots; need both floor sensors.
   - `UTURN_L` / `UTURN_R` — longer 900 ms forward arcs; do not use until the short tests are correct.
6. Send `STOP` between tests. Send `DISARM` when finished.

If a motor turns opposite to the intended direction, disconnect motor power and correct the motor wiring or motor-direction mapping deliberately; do not compensate by swapping random control pins. Verify each wheel independently before attempting to balance on the floor.

### Verify that forward movement is blocked safely

With wheels still raised and the robot restrained:

1. Arm with both floor readings valid and a clear ultrasonic reading.
2. Place a target within 35 cm of the front distance sensor.
3. Send `FWD`. Expected response: `BLOCKED,SENSOR LIMIT`; the wheels must not drive forward.
4. Remove the target and restore a valid clear distance reading.
5. Send `FWD` again; it should be accepted briefly.
6. If practical, repeat with the ultrasonic sensor disconnected or invalid. Forward movement must remain blocked.
7. Send `DISARM` and disconnect motor power.

Do not intentionally disconnect a sensor while the wheels are touching the floor.

## 6. Test the automatic diagnostic mode

This is a short diagnostic behavior test, not the production patrol algorithm.

1. Keep the robot restrained with wheels raised and motor power disconnectable.
2. Confirm both floor readings and ultrasonic data are valid.
3. Send `ARM`, then `AUTO`.
4. Observe only briefly. The test may issue short forward pulses when the front floor is present and the path is clear, reverse/turn behavior for a front-edge indication, or a turn request for an obstacle.
5. Send `STOP`, then `DISARM`.

Do not leave AUTO running unattended. Do not test it near a real drop, person, pet, road, or fragile object.

## 7. Return to production firmware

When diagnostics are complete:

1. Disconnect motor power and switch off both Nanos.
2. Upload `SensorNano/SensorNano.ino` to the sensor Nano.
3. Upload `BalanceNano/BalanceNano.ino` to the balance Nano.
4. Disconnect the D0/D1 link wires from the Nano currently being uploaded, then reconnect them after upload with power off.
5. Recheck the production startup sequence and IMU calibration with the wheels raised.
6. Only attempt a low-speed floor test after restrained tests pass, in a clear padded area with a person at the physical power disconnect.

The test sketches and production sketches use the same pin assignments, but they do not have identical behavior. Do not leave a test sketch installed for normal operation.

## Quick troubleshooting

| Symptom | First checks |
| --- | --- |
| `ERR,FRONT TOF INIT` / `ERR,REAR TOF INIT` | Sensor power, GND, A4/A5, correct XSHUT pin, module voltage |
| `ULTRA,INVALID` | Target is not returning a usable echo or is outside the accepted range; compare RAW values and target material/angle.
| `ULTRA,I2C_TRIGGER_ERROR` / `ULTRA,I2C_READ_ERROR` | RCWL-9610 I2C mode, address 0x57, A4/A5, supply and wiring |
| `DATA,LINK:DOWN` | Both Nanos powered, crossed D0/D1, common GND, 38400 baud inter-Nano link |
| `TEST ERR IMU` | BMI160 supply, GND, A4/A5, SAO address strap |
| `ERR ARM CHECK SENSORS` | Fresh link, both floor readings valid and <= 220 mm |
| `BLOCKED,SENSOR LIMIT` on FWD | Front floor must be valid and ultrasonic must be valid and farther than obstacle limit |
| Upload fails | Disconnect D0/D1 wires from the Nano being uploaded; check board/processor/COM port |

## Test record

Record each stage before moving to the next:

- [ ] Sensor Nano boots; front and rear ToF initialize.
- [ ] Front and rear floor readings change independently.
- [ ] Ultrasonic distance responds to a target.
- [ ] Both Nanos communicate; Bluetooth reports `LINK:UP`.
- [ ] BMI160 initializes and gyro/pitch telemetry responds.
- [ ] Obstacle and front/rear floor diagnostics respond.
- [ ] ARM is accepted only when sensor checks pass.
- [ ] Wheel direction and short manual pulses are correct with wheels raised.
- [ ] Forward command is blocked by a close or invalid ultrasonic reading.
- [ ] AUTO diagnostic mode stops/turns as expected.
- [ ] Production sketches are reinstalled after testing.
