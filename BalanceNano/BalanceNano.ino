#include <Wire.h>
#include <DFRobot_BMI160.h>
#include <math.h>

// =====================================================
// BALANCE NANO: PITCH PID + YAW POSITION PID
// =====================================================

DFRobot_BMI160 bmi160;

#define PWMA 5
#define AIN2 2
#define AIN1 3
#define STBY 8

#define BIN1 A2
#define BIN2 A1
#define PWMB 6

#define BUZZER 12
#define LED 13

#define BMI160_ADDRESS 0x68

// =====================================================
// USER SETTINGS
// =====================================================

const float STARTUP_YAW_TARGET = 30.0f;

const float YAW_POSITION_TOLERANCE = 3.0f;
const float YAW_STOP_RATE = 3.0f;
const unsigned long YAW_SETTLE_TIME_MS = 250;

float yawTarget = 0.0f;

bool yawCommandActive = false;
bool yawCommandCompleted = false;

unsigned long yawSettledSince = 0;

// =====================================================
// MOVEMENT SETTINGS
// =====================================================

const float FORWARD_PITCH_OFFSET = 3.0f;
const float REVERSE_PITCH_OFFSET = -3.0f;
const unsigned long MOVEMENT_DURATION_MS = 1000;

float pitchTarget = 0.0f;
bool pitchMovementActive = false;
unsigned long pitchMovementStarted = 0;

// =====================================================
// BALANCE NANO OPERATING MODE
// =====================================================

enum ControlMode {
  MOTOR_OFF = 0,
  KEEP_PID = 1,
  MOVEMENT = 2
};

ControlMode controlMode = KEEP_PID;

// =====================================================
// MOTOR SETTINGS
// =====================================================

const int MIN_PWM = 22;
const int MAX_PWM = 210;
const float MOTOR_SLEW = 25.0f;

int currentLeft = 0;
int currentRight = 0;

// =====================================================
// PITCH PID GAINS
// =====================================================

const float PITCH_KP = 35.0f;
const float PITCH_KI = 0.8f;
const float PITCH_KD = 7.2f;

const float PITCH_INTEGRAL_LIMIT = 22.0f;
const float PITCH_OUTPUT_LIMIT = 210.0f;

// =====================================================
// YAW PID GAINS
// =====================================================

const float YAW_KP = 0.8f;
const float YAW_KI = 0.02f;
const float YAW_KD = 0.05f;

const float YAW_INTEGRAL_LIMIT = 30.0f;
const float YAW_OUTPUT_LIMIT = 65.0f;

// =====================================================
// IMU SETTINGS
// =====================================================

const int CALIBRATION_SAMPLES = 400;

const float GYRO_SCALE = 0.0305f;
const float ACCEL_SCALE = 16384.0f;
const float ALPHA = 0.985f;

const float PITCH_SIGN = 1.0f;
const float YAW_SIGN = 1.0f;

float gyroXOffset = 0;
float gyroZOffset = 0;

float pitchAngle = 0;
float startupPitch = 0;
float yawAngle = 0;

// =====================================================
// PID STATE
// =====================================================

float pitchIntegral = 0;
float yawIntegral = 0;

// =====================================================
// LOOP TIMING
// =====================================================

const unsigned long LOOP_INTERVAL_US = 10000;
unsigned long lastLoopUs = 0;

// =====================================================
// MOTOR FUNCTIONS
// =====================================================

void setMotor(int leftSpeed, int rightSpeed) {

  leftSpeed = constrain(leftSpeed, -MAX_PWM, MAX_PWM);
  rightSpeed = constrain(rightSpeed, -MAX_PWM, MAX_PWM);

  if (leftSpeed >= 0) {
    digitalWrite(AIN1, HIGH);
    digitalWrite(AIN2, LOW);
  } else {
    digitalWrite(AIN1, LOW);
    digitalWrite(AIN2, HIGH);
  }

  if (rightSpeed >= 0) {
    digitalWrite(BIN1, HIGH);
    digitalWrite(BIN2, LOW);
  } else {
    digitalWrite(BIN1, LOW);
    digitalWrite(BIN2, HIGH);
  }

  analogWrite(PWMA, abs(leftSpeed));
  analogWrite(PWMB, abs(rightSpeed));
}

void stopMotors() {

  analogWrite(PWMA, 0);
  analogWrite(PWMB, 0);

  digitalWrite(AIN1, LOW);
  digitalWrite(AIN2, LOW);

  digitalWrite(BIN1, LOW);
  digitalWrite(BIN2, LOW);

  currentLeft = 0;
  currentRight = 0;
}

// =====================================================
// MOTOR OUTPUT DEADZONE
// =====================================================

int applyMotorDeadzone(float output) {

  float magnitude = fabs(output);

  if (magnitude < 10.0f) {
    return 0;
  }

  magnitude = constrain(magnitude, 10.0f, 210.0f);

  float compensated =
    MIN_PWM +
    ((magnitude - 10.0f) / 205.0f) *
    (MAX_PWM - MIN_PWM);

  int result = (int)(compensated + 0.5f);

  return output > 0 ? result : -result;
}

// =====================================================
// MOTOR SLEW LIMIT
// =====================================================

int slewMotor(int current, int target) {

  if (target > current + MOTOR_SLEW) {
    return current + (int)ceil(MOTOR_SLEW);
  }

  if (target < current - MOTOR_SLEW) {
    return current - (int)ceil(MOTOR_SLEW);
  }

  return target;
}

// =====================================================
// SENSOR FUNCTIONS
// =====================================================

bool readSensor(int16_t *data) {
  return bmi160.getAccelGyroData(data) == 0;
}

float calculatePhysicalPitch(int16_t *data) {

  float accX = data[3] / ACCEL_SCALE;
  float accY = data[4] / ACCEL_SCALE;
  float accZ = data[5] / ACCEL_SCALE;

  float angle = atan2(
    accY,
    sqrt(accX * accX + accZ * accZ)
  ) * 57.2957795f;

  return angle * PITCH_SIGN;
}

float wrapAngle(float angle) {

  while (angle > 180.0f) angle -= 360.0f;
  while (angle < -180.0f) angle += 360.0f;

  return angle;
}

// =====================================================
// STARTUP CALIBRATION
// =====================================================

bool calibrateSensor() {

  long gyroXSum = 0;
  long gyroZSum = 0;

  int validSamples = 0;
  int16_t data[6];

  for (int i = 0; i < CALIBRATION_SAMPLES; i++) {

    if (readSensor(data)) {
      gyroXSum += data[0];
      gyroZSum += data[2];
      validSamples++;
    }

    delay(5);
  }

  if (validSamples < CALIBRATION_SAMPLES * 0.9f) {
    return false;
  }

  gyroXOffset =
    ((float)gyroXSum / validSamples) * GYRO_SCALE;

  gyroZOffset =
    ((float)gyroZSum / validSamples) * GYRO_SCALE;

  if (!readSensor(data)) {
    return false;
  }

  pitchAngle = calculatePhysicalPitch(data);
  startupPitch = pitchAngle;
  pitchTarget = startupPitch;

  yawAngle = 0;

  pitchIntegral = 0;
  yawIntegral = 0;

  return true;
}

// =====================================================
// PITCH PID
// =====================================================

float calculatePitchPID(float dt, float gyroX) {

  float error = pitchAngle - pitchTarget;

  if (fabs(error) < 0.2f) {
    error = 0;
  }

  float candidateIntegral =
    constrain(
      pitchIntegral + error * dt,
      -PITCH_INTEGRAL_LIMIT,
      PITCH_INTEGRAL_LIMIT
    );

  float raw =
    PITCH_KP * error +
    PITCH_KI * candidateIntegral +
    PITCH_KD * gyroX;

  if (fabs(raw) < PITCH_OUTPUT_LIMIT ||
      (raw > PITCH_OUTPUT_LIMIT && error < 0) ||
      (raw < -PITCH_OUTPUT_LIMIT && error > 0)) {

    pitchIntegral = candidateIntegral;
  }

  raw =
    PITCH_KP * error +
    PITCH_KI * pitchIntegral +
    PITCH_KD * gyroX;

  raw = constrain(
    raw,
    -PITCH_OUTPUT_LIMIT,
    PITCH_OUTPUT_LIMIT
  );

  return -raw;
}

// =====================================================
// YAW PID
// =====================================================

float calculateYawPID(float dt, float gyroZ) {

  if (!yawCommandActive) {
    return 0;
  }

  float error = wrapAngle(yawTarget - yawAngle);

  float candidateIntegral =
    constrain(
      yawIntegral + error * dt,
      -YAW_INTEGRAL_LIMIT,
      YAW_INTEGRAL_LIMIT
    );

  float raw =
    YAW_KP * error +
    YAW_KI * candidateIntegral -
    YAW_KD * gyroZ;

  if (fabs(raw) < YAW_OUTPUT_LIMIT ||
      (raw > YAW_OUTPUT_LIMIT && error < 0) ||
      (raw < -YAW_OUTPUT_LIMIT && error > 0)) {

    yawIntegral = candidateIntegral;
  }

  raw =
    YAW_KP * error +
    YAW_KI * yawIntegral -
    YAW_KD * gyroZ;

  raw = constrain(
    raw,
    -YAW_OUTPUT_LIMIT,
    YAW_OUTPUT_LIMIT
  );

  if (fabs(error) <= YAW_POSITION_TOLERANCE &&
      fabs(gyroZ) <= YAW_STOP_RATE) {

    if (yawSettledSince == 0) {
      yawSettledSince = millis();
    }

    if (millis() - yawSettledSince >= YAW_SETTLE_TIME_MS) {

      yawCommandActive = false;
      yawCommandCompleted = true;

      yawIntegral = 0;

      Serial.println("TURN COMPLETE");

      return 0;
    }

  } else {

    yawSettledSince = 0;
  }

  return raw;
}

// =====================================================
// COMBINE PITCH AND YAW
// =====================================================

void controlMotors(float pitchOutput, float yawOutput) {

  float left = pitchOutput + yawOutput;
  float right = pitchOutput - yawOutput;

  int targetLeft = applyMotorDeadzone(left);
  int targetRight = applyMotorDeadzone(right);

  currentLeft = slewMotor(currentLeft, targetLeft);
  currentRight = slewMotor(currentRight, targetRight);

  setMotor(currentLeft, currentRight);
}

// =====================================================
// YAW COMMAND REQUEST
// =====================================================

void requestYawMove(float targetDegrees) {

  yawTarget = wrapAngle(targetDegrees);

  yawIntegral = 0;
  yawSettledSince = 0;
  yawCommandCompleted = false;

  yawCommandActive =
    fabs(wrapAngle(yawTarget - yawAngle)) >
    YAW_POSITION_TOLERANCE;
}

// =====================================================
// MOVEMENT CONTROL
// =====================================================

void cancelMovement() {

  pitchTarget = startupPitch;
  pitchMovementActive = false;

  yawCommandActive = false;
  yawCommandCompleted = false;

  yawTarget = yawAngle;

  pitchIntegral = 0;
  yawIntegral = 0;
  yawSettledSince = 0;
}

void startPitchMovement(float offset) {

  yawCommandActive = false;
  yawCommandCompleted = false;

  yawIntegral = 0;
  yawSettledSince = 0;
  pitchIntegral = 0;

  pitchTarget = startupPitch + offset;
  pitchMovementStarted = millis();
  pitchMovementActive = true;
}

void startYawMovement(float degrees) {

  pitchTarget = startupPitch;
  pitchMovementActive = false;
  pitchIntegral = 0;

  requestYawMove(yawAngle + degrees);
}

// =====================================================
// SERIAL COMMAND HANDLER
// =====================================================

void handleSerialCommands() {

  while (Serial.available() > 0) {

    char command = Serial.read();

    if (command == '\r' || command == '\n') {
      continue;
    }

    if (command >= 'a' && command <= 'z') {
      command -= 32;
    }

    if (command == '0') {

      controlMode = MOTOR_OFF;
      cancelMovement();

      stopMotors();
      digitalWrite(STBY, LOW);

      Serial.println("MODE 0: MOTOR OFF");

    } else if (command == '1') {

      controlMode = KEEP_PID;
      cancelMovement();

      digitalWrite(STBY, HIGH);

      Serial.println("MODE 1: KEEP PID");

    } else if (command == '2') {

      controlMode = MOVEMENT;
      digitalWrite(STBY, HIGH);

      Serial.println("MODE 2: MOVEMENT READY");

    } else if (controlMode == MOVEMENT) {

      switch (command) {

        case 'F':
          startPitchMovement(FORWARD_PITCH_OFFSET);
          Serial.println("FORWARD: 3 DEG, 1000 MS");
          break;

        case 'B':
          startPitchMovement(REVERSE_PITCH_OFFSET);
          Serial.println("REVERSE: 3 DEG, 1000 MS");
          break;

        case 'L':
          startYawMovement(-15.0f);
          Serial.println("LEFT: 15 DEG");
          break;

        case 'R':
          startYawMovement(15.0f);
          Serial.println("RIGHT: 15 DEG");
          break;

        case 'S':
          cancelMovement();
          Serial.println("MOVEMENT CANCELLED");
          break;

        default:
          Serial.println("UNKNOWN COMMAND");
          break;
      }
    }
  }
}

// =====================================================
// SENSOR FAULT
// =====================================================

void sensorFault() {

  stopMotors();
  digitalWrite(STBY, LOW);

  while (true) {

    digitalWrite(LED, HIGH);
    delay(150);

    digitalWrite(LED, LOW);
    delay(150);
  }
}

// =====================================================
// SETUP
// =====================================================

void setup() {

  Serial.begin(9600);
  Serial.println("BALANCE NANO: MOVEMENT READY");

  pinMode(PWMA, OUTPUT);
  pinMode(AIN1, OUTPUT);
  pinMode(AIN2, OUTPUT);

  pinMode(PWMB, OUTPUT);
  pinMode(BIN1, OUTPUT);
  pinMode(BIN2, OUTPUT);

  pinMode(STBY, OUTPUT);
  pinMode(BUZZER, OUTPUT);
  pinMode(LED, OUTPUT);

  digitalWrite(STBY, LOW);
  stopMotors();

  digitalWrite(LED, LOW);

  Wire.begin();
  delay(100);

  if (bmi160.softReset() != 0) {
    sensorFault();
  }

  delay(100);

  if (bmi160.I2cInit(BMI160_ADDRESS) != 0) {
    sensorFault();
  }

  delay(300);

  if (!calibrateSensor()) {
    sensorFault();
  }

  digitalWrite(STBY, HIGH);

  tone(BUZZER, 1800, 100);
  delay(150);

  digitalWrite(LED, LOW);

  lastLoopUs = micros();

  // Startup yaw command remains disabled.
  // requestYawMove(STARTUP_YAW_TARGET);
}

// =====================================================
// MAIN LOOP
// =====================================================

void loop() {

  handleSerialCommands();

  unsigned long now = micros();
  unsigned long elapsed = now - lastLoopUs;

  if (elapsed < LOOP_INTERVAL_US) {
    return;
  }

  lastLoopUs = now;

  float dt = elapsed / 1000000.0f;

  if (dt > 0.050f || dt <= 0) {

    stopMotors();

    pitchIntegral = 0;
    yawIntegral = 0;

    return;
  }

  // ---------------- TIMED PITCH MOVEMENT ----------------

  if (pitchMovementActive &&
      millis() - pitchMovementStarted >= MOVEMENT_DURATION_MS) {

    pitchTarget = startupPitch;
    pitchMovementActive = false;

    pitchIntegral = 0;

    Serial.println("PITCH MOVEMENT COMPLETE");
  }

  int16_t data[6];

  if (!readSensor(data)) {
    sensorFault();
  }

  // ---------------- PITCH ESTIMATION ----------------

  float gyroX =
    data[0] * GYRO_SCALE - gyroXOffset;

  gyroX *= PITCH_SIGN;

  float accelPitch = calculatePhysicalPitch(data);

  pitchAngle =
    ALPHA * (pitchAngle + gyroX * dt) +
    (1.0f - ALPHA) * accelPitch;

  // ---------------- YAW ESTIMATION ----------------

  float gyroZ =
    data[2] * GYRO_SCALE - gyroZOffset;

  gyroZ *= YAW_SIGN;

  yawAngle = wrapAngle(yawAngle + gyroZ * dt);

  // ---------------- PID CONTROL ----------------

  float pitchOutput =
    calculatePitchPID(dt, gyroX);

  float yawOutput =
    calculateYawPID(dt, gyroZ);

  // ---------------- MOTOR CONTROL ----------------

  if (controlMode == MOTOR_OFF) {

    // PID calculations continue.
    // Motor driver remains disabled.

  } else {

    controlMotors(pitchOutput, yawOutput);
  }

  digitalWrite(LED, HIGH);
}
