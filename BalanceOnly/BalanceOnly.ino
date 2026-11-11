/*
  PIDpatrolbot - Balance Nano only
  Standalone 2WD self-balancing firmware.
  No Sensor Nano, UART protocol, Bluetooth, ultrasonic or manual control.

  Hardware: Arduino Nano + BMI160 + TB6612FNG
*/

#include <Wire.h>
#include <DFRobot_BMI160.h>
#include <math.h>

// ---------- EXISTING PIN CONFIGURATION ----------
const uint8_t PWMA = 5;
const uint8_t AIN2 = 2;
const uint8_t AIN1 = 3;
const uint8_t STBY = 8;
const uint8_t BIN1 = A2;
const uint8_t BIN2 = A1;
const uint8_t PWMB = 6;
const uint8_t BUZZER_PIN = 12;
const uint8_t STATUS_LED_PIN = 13;

// ---------- IMU ----------
const uint8_t BMI160_ADDRESS = 0x68;
DFRobot_BMI160 bmi160;

// ---------- BALANCE TUNING ----------
// Starting values carried over from the existing controller.
// They are starting values, not guaranteed final tuning values.
float Kp = 28.0f;
float Ki = 0.55f;
float Kd = 1.45f;

// Set this to the measured pitch reading when the robot is physically upright.
// Example: if upright pitch reads +1.2 degrees, set this to +1.2.
float BALANCE_ANGLE_TRIM_DEG = 0.0f;

// Keep these at +1 initially to preserve the existing motor wiring logic.
// If a later controlled direction test proves a motor is reversed, change
// only that motor's sign to -1.
const int8_t LEFT_MOTOR_SIGN = 1;
const int8_t RIGHT_MOTOR_SIGN = 1;

const float COMPLEMENTARY_ALPHA = 0.985f;
const float INTEGRAL_LIMIT = 22.0f;
const float MAX_SLEW_PER_LOOP = 8.5f;
const int MAX_PWM = 210;
const int MIN_EFFECTIVE_PWM = 22;

// ---------- SAFETY / TIMING ----------
const unsigned long LOOP_INTERVAL_US = 5500UL; // about 181 Hz
const unsigned long IMU_TIMEOUT_MS = 50UL;
const float FALL_ANGLE_DEG = 35.0f;
const float MAX_VALID_DT = 0.025f;

float pitch = 0.0f;
float pitchRate = 0.0f;
float gyroOffsetY = 0.0f;
float accelOffsetX = 0.0f;

float integralTerm = 0.0f;
float filteredDerivative = 0.0f;
float previousLeft = 0.0f;
float previousRight = 0.0f;

unsigned long lastLoopUs = 0;
unsigned long lastImuMs = 0;
unsigned long lastLedMs = 0;
bool ledState = false;
uint8_t faultCode = 0;
bool controllerReady = false;
bool faultLatched = false;

void stopMotors() {
  analogWrite(PWMA, 0);
  analogWrite(PWMB, 0);
  digitalWrite(AIN1, LOW);
  digitalWrite(AIN2, LOW);
  digitalWrite(BIN1, LOW);
  digitalWrite(BIN2, LOW);
  previousLeft = 0.0f;
  previousRight = 0.0f;
}

void latchFault(uint8_t reason) {
  faultCode = reason;
  faultLatched = true;
  controllerReady = false;
  stopMotors();
  digitalWrite(STBY, LOW);
  integralTerm = 0.0f;
  filteredDerivative = 0.0f;
}

float limitSlew(float target, float current) {
  float change = target - current;
  if (change > MAX_SLEW_PER_LOOP) return current + MAX_SLEW_PER_LOOP;
  if (change < -MAX_SLEW_PER_LOOP) return current - MAX_SLEW_PER_LOOP;
  return target;
}

void driveOneMotor(bool leftMotor, float command) {
  const int8_t sign = leftMotor ? LEFT_MOTOR_SIGN : RIGHT_MOTOR_SIGN;
  command *= sign;

  int pwm = constrain((int)fabs(command), 0, MAX_PWM);
  if (pwm > 0 && pwm < MIN_EFFECTIVE_PWM) pwm = MIN_EFFECTIVE_PWM;

  if (leftMotor) {
    if (command > 0.0f) {
      digitalWrite(AIN1, HIGH);
      digitalWrite(AIN2, LOW);
    } else if (command < 0.0f) {
      digitalWrite(AIN1, LOW);
      digitalWrite(AIN2, HIGH);
    } else {
      digitalWrite(AIN1, LOW);
      digitalWrite(AIN2, LOW);
    }
    analogWrite(PWMA, pwm);
  } else {
    if (command > 0.0f) {
      digitalWrite(BIN1, HIGH);
      digitalWrite(BIN2, LOW);
    } else if (command < 0.0f) {
      digitalWrite(BIN1, LOW);
      digitalWrite(BIN2, HIGH);
    } else {
      digitalWrite(BIN1, LOW);
      digitalWrite(BIN2, LOW);
    }
    analogWrite(PWMB, pwm);
  }
}

void driveMotors(float command) {
  command = constrain(command, -(float)MAX_PWM, (float)MAX_PWM);
  float left = limitSlew(command, previousLeft);
  float right = limitSlew(command, previousRight);

  previousLeft = left;
  previousRight = right;

  driveOneMotor(true, left);
  driveOneMotor(false, right);
}

bool calibrateImu() {
  // Robot must be held completely still and approximately upright.
  const uint16_t samples = 400;
  int64_t gyroSum = 0;
  int64_t gyroSumSquares = 0;
  int32_t accelXSum = 0;
  uint16_t goodSamples = 0;

  for (uint16_t i = 0; i < samples; i++) {
    int16_t data[6];

    if (bmi160.getAccelGyroData(data) == 0) {
      gyroSum += data[1];
      gyroSumSquares += (int32_t)data[1] * data[1];
      accelXSum += data[3];
      goodSamples++;
    }
    delay(3);
  }

  if (goodSamples < (samples * 9U) / 10U) return false;

  const float mean = gyroSum / (float)goodSamples;
  const float variance =
      gyroSumSquares / (float)goodSamples - mean * mean;
  const float deviation = sqrt(variance > 0.0f ? variance : 0.0f);

  if (fabs(mean) > 250.0f || deviation > 80.0f) return false;

  gyroOffsetY = mean * 0.0305f;
  accelOffsetX =
      (accelXSum / (float)goodSamples) / 16384.0f;

  return true;
}

bool readImu(float dt) {
  int16_t data[6];
  if (bmi160.getAccelGyroData(data) != 0) return false;

  // Preserve the axis mapping and scaling used by the previous firmware.
  const float gyroY = data[1] * 0.0305f - gyroOffsetY;
  pitchRate = gyroY;

  const float accX = data[3] / 16384.0f - accelOffsetX;
  const float accY = data[4] / 16384.0f;
  const float accZ = data[5] / 16384.0f;

  const float accelPitch =
      atan2(-accX, sqrt(accY * accY + accZ * accZ)) * 57.2957795f;

  pitch = COMPLEMENTARY_ALPHA * (pitch + gyroY * dt)
        + (1.0f - COMPLEMENTARY_ALPHA) * accelPitch;

  lastImuMs = millis();
  return true;
}

void updateStatusLed() {
  if (faultLatched) {
    // Runtime fault code: number of short flashes, repeated every 2 seconds.
    const unsigned long phase = millis() % 2000UL;
    bool on = false;
    for (uint8_t i = 0; i < faultCode; i++) {
      const unsigned long start = (unsigned long)i * 260UL;
      if (phase >= start && phase < start + 120UL) on = true;
    }
    digitalWrite(STATUS_LED_PIN, on ? HIGH : LOW);
  } else {
    digitalWrite(STATUS_LED_PIN, controllerReady ? HIGH : LOW);
  }
}

void setup() {
  pinMode(STATUS_LED_PIN, OUTPUT);
  pinMode(BUZZER_PIN, OUTPUT);
  pinMode(PWMA, OUTPUT);
  pinMode(AIN1, OUTPUT);
  pinMode(AIN2, OUTPUT);
  pinMode(PWMB, OUTPUT);
  pinMode(BIN1, OUTPUT);
  pinMode(BIN2, OUTPUT);
  pinMode(STBY, OUTPUT);

  digitalWrite(STATUS_LED_PIN, LOW);
  digitalWrite(BUZZER_PIN, LOW);
  digitalWrite(STBY, LOW);
  stopMotors();

  Wire.begin();

  if (bmi160.softReset() != BMI160_OK ||
      bmi160.I2cInit(BMI160_ADDRESS) != BMI160_OK) {
    latchFault(1);
    while (true) {
      digitalWrite(BUZZER_PIN, HIGH);
      delay(150);
      digitalWrite(BUZZER_PIN, LOW);
      delay(350);
    }
  }

  // Calibration requires the robot to be motionless and held upright.
  if (!calibrateImu()) {
    latchFault(2);
    while (true) {
      digitalWrite(BUZZER_PIN, HIGH);
      delay(150);
      digitalWrite(BUZZER_PIN, LOW);
      delay(850);
    }
  }

  // Seed the complementary filter with a real accelerometer angle.
  int16_t data[6];
  if (bmi160.getAccelGyroData(data) != 0) {
    latchFault(3);
    while (true) delay(100);
  }

  const float ax = data[3] / 16384.0f - accelOffsetX;
  const float ay = data[4] / 16384.0f;
  const float az = data[5] / 16384.0f;
  pitch = atan2(-ax, sqrt(ay * ay + az * az)) * 57.2957795f;

  lastImuMs = millis();
  controllerReady = true;
  digitalWrite(STBY, HIGH);

  digitalWrite(BUZZER_PIN, HIGH);
  delay(80);
  digitalWrite(BUZZER_PIN, LOW);

  // Start the control-loop clock AFTER the blocking startup beep.
  // Otherwise the first loop sees the beep delay as a timing fault.
  lastLoopUs = micros();
}

void loop() {
  if (faultLatched) {
    updateStatusLed();
    return;
  }

  const unsigned long nowUs = micros();
  const unsigned long elapsedUs = nowUs - lastLoopUs;
  if (elapsedUs < LOOP_INTERVAL_US) {
    updateStatusLed();
    return;
  }

  lastLoopUs = nowUs;
  const float dt = elapsedUs * 0.000001f;

  // A long loop pause means the controller missed its timing budget.
  if (dt > MAX_VALID_DT || dt <= 0.0f) {
    latchFault(4);
    updateStatusLed();
    return;
  }

  if (!readImu(dt)) {
    if (millis() - lastImuMs > IMU_TIMEOUT_MS) latchFault(5);
    updateStatusLed();
    return;
  }

  if (millis() - lastImuMs > IMU_TIMEOUT_MS ||
      fabs(pitch) >= FALL_ANGLE_DEG) {
    latchFault(6);
    updateStatusLed();
    return;
  }

  // Positive error requests positive wheel correction.
  const float targetPitch = BALANCE_ANGLE_TRIM_DEG;
  const float error = targetPitch - pitch;

  // Derivative on measured gyro rate reduces setpoint kick.
  filteredDerivative =
      0.78f * filteredDerivative - 0.22f * pitchRate;

  // Conditional integration limits wind-up when output saturates.
  const float proposedIntegral =
      constrain(integralTerm + error * dt,
                -INTEGRAL_LIMIT, INTEGRAL_LIMIT);

  float output = Kp * error
               + Ki * proposedIntegral
               + Kd * filteredDerivative;

  if (fabs(output) < MAX_PWM) {
    integralTerm = proposedIntegral;
  } else {
    output = Kp * error + Ki * integralTerm
           + Kd * filteredDerivative;
  }

  output = constrain(output, -(float)MAX_PWM, (float)MAX_PWM);
  driveMotors(output);
  updateStatusLed();
}
