# PID Patrol Bot: Calibration Guide

This guide assumes both production sketches are installed and the two-Nano link is working. The goal is to keep the firmware stable and tune the physical robot systematically.

## Current starting values

| Setting | Current value |
| --- | ---: |
| Pitch Kp | 35.0 |
| Pitch Ki | 0.8 |
| Pitch Kd | 7.2 |
| Upright clearance | 2.0° |
| Motor minimum PWM | 22 |
| Maximum PWM | 210 |
| Motor slew step | 8.5 |
| Control loop target | 5.5 ms |

The gains are a starting combination, not a claim of perfect tuning. Do not run an auto-tuner. Change one value at a time and keep a note of the result.

## 1. Mechanical and sensor reference

1. Check that the battery, motors, wheels and sensor board are firmly mounted.
2. Check that both wheels turn freely and have similar mechanical resistance.
3. Check that the BMI160 is firmly mounted and its axes match the installed Balance Nano firmware orientation.
4. Put the robot on a level surface and identify its actual upright resting angle.
5. Make sure the chassis is not leaning because of the battery, wiring or sensor bracket.

The startup IMU calibration measures gyro bias and the accelerometer X-axis reference. Hold the chassis still at its true upright reference during startup. It does not compensate for a loose or moving sensor mount.

## 2. Startup calibration

1. Keep motor power disconnected.
2. Place the robot upright and completely still.
3. Power both Nanos.
4. Do not touch or move the chassis while the Balance Nano collects its startup samples.
5. Wait for READY MANUAL.
6. If calibration fails, do not bypass it. Check sensor wiring, mounting vibration and startup stillness, then restart.

## 3. Motor baseline

With the robot restrained and both wheels raised:

1. Verify left and right motor directions.
2. Verify each motor starts reliably in both directions.
3. Check whether one motor needs a higher PWM to start than the other.
4. Keep the configured minimum PWM at 22 initially. Only change it after recording repeatable motor-start tests under the actual battery voltage and load.
5. If the motors behave differently, record the difference before trying to correct it in PID gains.

Do not confuse the controller output threshold with PWM. The production sketch's DEADZONE = 22 is the minimum applied motor PWM; any separate control-output threshold must be treated as a different quantity.

## 4. Balance tuning sequence

Use the same battery charge, surface, payload and starting position for each comparison.

1. Begin with the current 35.0 / 0.8 / 7.2 gains.
2. Make a short, restrained test and record whether it falls forward, falls backward, oscillates, vibrates, or returns toward upright.
3. Change **Kp only** for the next comparison. Record the result.
4. Restore or keep the better Kp, then change **Kd only**. Record the result.
5. Adjust **Ki last**, in small increments, only if a persistent steady bias remains after Kp and Kd are behaving well.
6. Do not change PWM minimum, clearance, slew and PID gains in the same test.
7. Stop immediately if the robot accelerates away, the chassis vibrates violently, or either motor behaves unpredictably.

### Interpreting the response

- **Falls away without correcting strongly:** proportional response may be too weak, motor direction may be wrong, or the motor may not be starting.
- **Fast repeated rocking or vibration:** derivative noise, excessive gain, motor deadzone behavior, mechanical play or electrical noise may be contributing.
- **Overshoots and swings back repeatedly:** compare Kp and Kd separately; do not immediately add integral.
- **Slow steady lean in one direction:** check mechanical balance, sensor mounting and motor matching before using Ki to compensate.
- **Motors run at upright when untouched:** inspect the pitch reference, 2° upright clearance, gyro rate and motor outputs before changing gains.

## 5. Upright clearance

The current PITCH_CLEARANCE_DEG = 2.0 is applied only when the target is upright. Inside the clearance, proportional and integral pitch error are suppressed and the derivative term remains active to damp motion. Outside the boundary, the clearance is subtracted from the error so the response grows from the boundary. Forward and reverse movement targets bypass this upright clearance.

Keep this at 2° while tuning the gains. Change it only after the robot can balance consistently and you have a clear reason to adjust the neutral zone.

## 6. Record every trial

Use a simple log:

| Trial | Kp | Ki | Kd | PWM min | Clearance | Battery | Observed response |
| --- | ---: | ---: | ---: | ---: | ---: | --- | --- |
| Baseline | 35.0 | 0.8 | 7.2 | 22 | 2.0° |  |  |

Only one tuning variable should change between adjacent trials. Keep the best-known combination as the new baseline.

## 7. Movement and sensor calibration

After upright balancing is reliable:

1. Verify that FWD and REV travel in the intended directions.
2. Verify left/right arcs and pivots with the wheels raised first.
3. Check the front and rear floor readings at the actual installed sensor height.
4. The current floor safety limit is 220 mm. Do not change it until the installed sensors' normal floor readings and no-floor readings have been measured.
5. The default obstacle limit is 35 cm. Use DIST=45 to save 45 cm, or another supported value from 15 to 200 cm.
6. Verify that a close, invalid or stale ultrasonic reading blocks forward travel.

Yaw turns are approximate because this robot has no wheel encoders and yaw is integrated from the gyro. Do not expect exact 45°, 90° or 180° turns without additional position feedback.

**Do not attempt an unattended test.** Keep a physical motor-power disconnect within reach and perform trials in a clear padded area.
