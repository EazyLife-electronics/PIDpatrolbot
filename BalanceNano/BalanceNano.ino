#include <Wire.h>
#include <EEPROM.h>
#include <DFRobot_BMI160.h>
#include "RobotLink.h"

const uint8_t PWMA = 5;
const uint8_t AIN2 = 2;
const uint8_t AIN1 = 3;
const uint8_t STBY = 8;
const uint8_t BIN1 = A2;
const uint8_t BIN2 = A1;
const uint8_t PWMB = 6;
const uint8_t BUZZER_PIN = 12;
const uint8_t STATUS_LED_PIN = 13;

const uint8_t SENSOR_FLAG_ULTRASONIC = 0x01;
const uint8_t SENSOR_FLAG_FRONT_FLOOR = 0x02;
const uint8_t SENSOR_FLAG_REAR_FLOOR = 0x04;
const uint16_t FLOOR_LIMIT_MM = 220;
const uint16_t SENSOR_STALE_MS = 350;
const uint16_t DEFAULT_OBSTACLE_CM = 35;
const uint16_t CONFIG_MAGIC = 0x5042;
const unsigned long LOOP_US = 5500;
const unsigned long MANUAL_JOG_MS = 650;
const unsigned long TURN_TIMEOUT_MS = 2600;
const unsigned long SURFACE_LOST_CONFIRM_MS = 120;

DFRobot_BMI160 bmi160;
LinkParser linkParser;

float Kp_angle = 28.0f;
float Ki_angle = 0.55f;
float Kd_angle = 1.45f;
float Kp_yaw = 1.9f;
float Kd_yaw = 0.35f;
const float ALPHA = 0.985f;
const int DEADZONE = 22;
const float MAX_SLEW = 8.5f;
const int MAX_PWM = 210;
const float CRUISE_ANGLE = -1.6f;
const float REVERSE_ANGLE = 1.6f;

float pitch = 0.0f;
float yaw = 0.0f;
float pitchRate = 0.0f;
float yawRate = 0.0f;
float targetYaw = 0.0f;
float integralAngle = 0.0f;
float filteredD = 0.0f;
float lastMotorL = 0.0f;
float lastMotorR = 0.0f;
float gyroOffsetY = 0.0f;
float gyroOffsetZ = 0.0f;
float accelOffsetX = 0.0f;

unsigned long lastLoopTime = 0;
unsigned long lastSensorFrameAt = 0;
unsigned long lastImuAt = 0;
unsigned long surfaceLostSince = 0;
unsigned long lastPatrolTurnAt = 0;
unsigned long actionExpiresAt = 0;
unsigned long overAngleSince = 0;
unsigned long buzzerOffAt = 0;

uint8_t sensorFlags = 0;
uint16_t frontDistanceCm = 0xFFFF;
uint16_t frontFloorMm = 0xFFFF;
uint16_t rearFloorMm = 0xFFFF;
uint16_t obstacleLimitCm = DEFAULT_OBSTACLE_CM;
uint8_t txSequence = 0;
bool hasSensorFrame = false;
bool surfaceLost = false;
bool controllerFault = false;
bool buzzerActive = false;

struct StoredConfig {
  uint16_t magic;
  uint16_t obstacleCm;
  uint8_t checksum;
};

struct MotionAction {
  float pitchTarget;
  float yawTarget;
  bool turning;
  bool active;
};

MotionAction action = { 0.0f, 0.0f, false, false };

enum DriveMode : uint8_t { MODE_MANUAL, MODE_PATROL };
DriveMode driveMode = MODE_MANUAL;

void stopMotors();
void setMotor(bool isLeft, float speed);
float slew(float target, float current);
void updateSensorsFromLink();
void handleCommand(const uint8_t *payload, uint8_t length);
void sendStatus(const char *message);
void calibrateImu();
void readImu(float dt);
void updateBehavior(unsigned long now);
void runControl(float dt);
void beginAction(float pitchTarget, float yawDelta, bool turning,
                 unsigned long durationMs);
bool sensorDataFresh(unsigned long now);
bool floorSafeForForward();
bool floorSafeForReverse();
uint8_t configChecksum(const StoredConfig &config);
void loadConfig();
void saveObstacleLimit(uint16_t value);
void beep(uint16_t durationMs);
void updateBuzzer();

void setup() {
  pinMode(STATUS_LED_PIN, OUTPUT);
  digitalWrite(STATUS_LED_PIN, HIGH);
  Serial.begin(38400);
  Wire.begin();

  pinMode(PWMA, OUTPUT);
  pinMode(AIN1, OUTPUT);
  pinMode(AIN2, OUTPUT);
  pinMode(PWMB, OUTPUT);
  pinMode(BIN1, OUTPUT);
  pinMode(BIN2, OUTPUT);
  pinMode(STBY, OUTPUT);
  pinMode(BUZZER_PIN, OUTPUT);
  digitalWrite(BUZZER_PIN, LOW);
  digitalWrite(STBY, LOW);
  stopMotors();
  randomSeed(analogRead(A0));
  loadConfig();

  if (bmi160.softReset() != BMI160_OK || bmi160.I2cInit(0x68) != BMI160_OK) {
    sendStatus("ERR BMI160");
    while (true) {
      digitalWrite(BUZZER_PIN, HIGH);
      delay(200);
      digitalWrite(BUZZER_PIN, LOW);
      delay(400);
    }
  }

  digitalWrite(BUZZER_PIN, HIGH);
  delay(100);
  digitalWrite(BUZZER_PIN, LOW);
  calibrateImu();
  lastImuAt = millis();
  targetYaw = yaw;
  digitalWrite(STBY, HIGH);
  beep(100);
  lastLoopTime = micros();
  lastPatrolTurnAt = millis() + 2500;
  sendStatus("READY MANUAL");
}

void loop() {
  updateSensorsFromLink();
  updateBuzzer();

  const unsigned long nowMicros = micros();
  if (static_cast<unsigned long>(nowMicros - lastLoopTime) < LOOP_US) return;
  const float dt = static_cast<unsigned long>(nowMicros - lastLoopTime) * 1.0e-6f;
  lastLoopTime = nowMicros;
  if (dt > 0.025f) {
    stopMotors();
    return;
  }

  readImu(dt);
  updateBehavior(millis());
  updateBuzzer();
  runControl(dt);
}

void updateSensorsFromLink() {
  LinkPacket packet;
  while (linkParser.poll(Serial, packet)) {
    if (packet.type == LINK_SENSOR_FRAME && packet.length >= 7) {
      sensorFlags = packet.payload[0];
      frontDistanceCm = linkReadU16(&packet.payload[1]);
      frontFloorMm = linkReadU16(&packet.payload[3]);
      rearFloorMm = linkReadU16(&packet.payload[5]);
      lastSensorFrameAt = millis();
      hasSensorFrame = true;
    } else if (packet.type == LINK_COMMAND_FRAME) {
      handleCommand(packet.payload, packet.length);
    }
  }
}

void handleCommand(const uint8_t *payload, uint8_t length) {
  if (length == 0 || length >= LINK_MAX_PAYLOAD) return;

  char command[LINK_MAX_PAYLOAD + 1];
  memcpy(command, payload, length);
  command[length] = '\0';
  for (uint8_t index = 0; index < length; index++) {
    if (command[index] >= 'a' && command[index] <= 'z')
      command[index] = static_cast<char>(command[index] - 'a' + 'A');
  }

  const unsigned long now = millis();
  if (!strcmp(command, "MANUAL") || !strcmp(command, "M")) {
    driveMode = MODE_MANUAL;
    action.active = false;
    targetYaw = yaw;
    beep(70);
    sendStatus("OK MANUAL");
  } else if (!strcmp(command, "PATROL") || !strcmp(command, "ROAM") ||
             !strcmp(command, "A")) {
    driveMode = MODE_PATROL;
    action.active = false;
    targetYaw = yaw;
    lastPatrolTurnAt = now + random(2000, 5000);
    sendStatus("OK PATROL");
    beep(70);
  } else if (!strcmp(command, "STOP") || !strcmp(command, "S")) {
    action.active = false;
    targetYaw = yaw;
    sendStatus("OK STOP");
  } else if (!strcmp(command, "FWD") || !strcmp(command, "F")) {
    beginAction(CRUISE_ANGLE, 0.0f, false, MANUAL_JOG_MS);
    sendStatus("OK FWD");
  } else if (!strcmp(command, "REV") || !strcmp(command, "B")) {
    beginAction(REVERSE_ANGLE, 0.0f, false, MANUAL_JOG_MS);
    sendStatus("OK REV");
  } else if (!strcmp(command, "LEFT") || !strcmp(command, "L")) {
    beginAction(CRUISE_ANGLE, -45.0f, true, 1100);
    sendStatus("OK LEFT ARC");
  } else if (!strcmp(command, "RIGHT") || !strcmp(command, "R")) {
    beginAction(CRUISE_ANGLE, 45.0f, true, 1100);
    sendStatus("OK RIGHT ARC");
  } else if (!strcmp(command, "PIVOT_L") || !strcmp(command, "PL")) {
    beginAction(0.0f, -90.0f, true, TURN_TIMEOUT_MS);
    sendStatus("OK PIVOT LEFT");
  } else if (!strcmp(command, "PIVOT_R") || !strcmp(command, "PR")) {
    beginAction(0.0f, 90.0f, true, TURN_TIMEOUT_MS);
    sendStatus("OK PIVOT RIGHT");
  } else if (!strcmp(command, "UTURN_L") || !strcmp(command, "UL")) {
    beginAction(CRUISE_ANGLE, -180.0f, true, 4000);
    sendStatus("OK UTURN LEFT");
  } else if (!strcmp(command, "UTURN_R") || !strcmp(command, "UR")) {
    beginAction(CRUISE_ANGLE, 180.0f, true, 4000);
    sendStatus("OK UTURN RIGHT");
  } else if (!strncmp(command, "DIST=", 5)) {
    const long requested = atol(command + 5);
    if (requested >= 15 && requested <= 200) {
      saveObstacleLimit(static_cast<uint16_t>(requested));
      sendStatus("OK DIST SAVED");
    } else {
      sendStatus("ERR DIST 15..200 CM");
    }
  } else if (!strcmp(command, "STATUS") || !strcmp(command, "?")) {
    char status[LINK_MAX_PAYLOAD];
    snprintf(status, sizeof(status), "MODE=%s,DIST=%u",
             driveMode == MODE_PATROL ? "PATROL" : "MANUAL",
             obstacleLimitCm);
    sendStatus(status);
  } else {
    sendStatus("ERR UNKNOWN CMD");
  }
}

void beginAction(float pitchTarget, float yawDelta, bool turning,
                 unsigned long durationMs) {
  action.pitchTarget = pitchTarget;
  action.yawTarget = yaw + yawDelta;
  action.turning = turning;
  action.active = true;
  actionExpiresAt = millis() + durationMs;
  if (turning) targetYaw = action.yawTarget;
}

void updateBehavior(unsigned long now) {
  if (controllerFault) {
    action.active = false;
    targetYaw = yaw;
    return;
  }

  if (fabs(pitch) > 46.0f) {
    if (overAngleSince == 0) overAngleSince = now;
    if (now - overAngleSince >= 180) {
      controllerFault = true;
      digitalWrite(STBY, LOW);
      stopMotors();
      sendStatus("FAULT FALLEN");
      beep(500);
      return;
    }
  } else {
    overAngleSince = 0;
  }

  if (now - lastImuAt > 50) {
    controllerFault = true;
    digitalWrite(STBY, LOW);
    stopMotors();
    sendStatus("FAULT IMU STALE");
    beep(500);
    return;
  }

  if (surfaceLost) {
    action.active = false;
    targetYaw = yaw;
    return;
  }

  const bool fresh = sensorDataFresh(now);
  const bool edgeSensorsValid = fresh &&
      (sensorFlags & (SENSOR_FLAG_FRONT_FLOOR | SENSOR_FLAG_REAR_FLOOR)) ==
      (SENSOR_FLAG_FRONT_FLOOR | SENSOR_FLAG_REAR_FLOOR);
  const bool frontFloorPresent = edgeSensorsValid && frontFloorMm <= FLOOR_LIMIT_MM;
  const bool rearFloorPresent = edgeSensorsValid && rearFloorMm <= FLOOR_LIMIT_MM;
  const bool frontBlocked = fresh && (sensorFlags & SENSOR_FLAG_ULTRASONIC) &&
      frontDistanceCm != 0xFFFF && frontDistanceCm <= obstacleLimitCm;
  const bool noFloorUnderEitherEnd = edgeSensorsValid &&
      !frontFloorPresent && !rearFloorPresent;

  if (noFloorUnderEitherEnd) {
    if (surfaceLostSince == 0) surfaceLostSince = now;
    if (now - surfaceLostSince >= SURFACE_LOST_CONFIRM_MS) {
      surfaceLost = true;
      action.active = false;
      targetYaw = yaw;
      digitalWrite(STBY, LOW);
      stopMotors();
      beep(500);
      sendStatus("FAULT NO FLOOR");
      return;
    }
  } else {
    surfaceLostSince = 0;
  }

  if (action.active) {
    const bool turnComplete = action.turning && fabs(action.yawTarget - yaw) < 10.0f;
    const bool timeComplete = static_cast<int32_t>(now - actionExpiresAt) >= 0;
    if (turnComplete || timeComplete) {
      action.active = false;
      targetYaw = yaw;
      if (driveMode == MODE_PATROL) {
        lastPatrolTurnAt = now + random(2500, 6000);
      }
    }
  }

  if (!action.active && driveMode == MODE_PATROL) {
    if (!fresh || !edgeSensorsValid || !(sensorFlags & SENSOR_FLAG_ULTRASONIC)) {
      targetYaw = yaw;
      return;
    }

    if (!frontFloorPresent && rearFloorPresent) {
      const float turn = random(0, 2) ? 90.0f : -90.0f;
      beginAction(REVERSE_ANGLE, turn, true, TURN_TIMEOUT_MS);
      beep(100);
      return;
    }
    if (!rearFloorPresent && frontFloorPresent && !frontBlocked) {
      const float turn = random(0, 2) ? 90.0f : -90.0f;
      beginAction(CRUISE_ANGLE, turn, true, TURN_TIMEOUT_MS);
      beep(100);
      return;
    }
    if (!frontFloorPresent || !rearFloorPresent) {
      targetYaw = yaw;
      return;
    }
    if (frontBlocked) {
      const float turn = random(0, 2) ? 90.0f : -90.0f;
      beginAction(0.0f, turn, true, TURN_TIMEOUT_MS);
      beep(80);
      return;
    }
    if (static_cast<int32_t>(now - lastPatrolTurnAt) >= 0) {
      const float turn = random(0, 2) ? 45.0f : -45.0f;
      beginAction(CRUISE_ANGLE, turn, true, 1200);
      return;
    }
  }

  if (driveMode == MODE_MANUAL && !fresh) {
    action.active = false;
    targetYaw = yaw;
  }
}

void runControl(float dt) {
  float targetPitch = 0.0f;
  float yawCommand = targetYaw;
  const unsigned long now = millis();
  const bool fresh = sensorDataFresh(now);
  const bool edgeSensorsValid = fresh &&
      (sensorFlags & (SENSOR_FLAG_FRONT_FLOOR | SENSOR_FLAG_REAR_FLOOR)) ==
      (SENSOR_FLAG_FRONT_FLOOR | SENSOR_FLAG_REAR_FLOOR);
  const bool frontFloorPresent = edgeSensorsValid && frontFloorMm <= FLOOR_LIMIT_MM;
  const bool rearFloorPresent = edgeSensorsValid && rearFloorMm <= FLOOR_LIMIT_MM;
  const bool frontBlocked = fresh && (sensorFlags & SENSOR_FLAG_ULTRASONIC) &&
      frontDistanceCm != 0xFFFF && frontDistanceCm <= obstacleLimitCm;

  if (action.active) {
    targetPitch = action.pitchTarget;
    if (action.turning) yawCommand = action.yawTarget;
  } else if (driveMode == MODE_PATROL && fresh && edgeSensorsValid &&
             frontFloorPresent && rearFloorPresent &&
             (sensorFlags & SENSOR_FLAG_ULTRASONIC) && !frontBlocked) {
    targetPitch = CRUISE_ANGLE;
  }

  if (!fresh || !edgeSensorsValid || surfaceLost || controllerFault) {
    targetPitch = 0.0f;
    yawCommand = yaw;
  }
  if (controllerFault) {
    stopMotors();
    return;
  }
  if (targetPitch < 0.0f && (!frontFloorPresent || frontBlocked)) {
    targetPitch = 0.0f;
  }
  if (targetPitch > 0.0f && !rearFloorPresent) {
    targetPitch = 0.0f;
  }
  if (fabs(yawCommand - yaw) > 10.0f && targetPitch == 0.0f &&
      (!frontFloorPresent || !rearFloorPresent)) {
    yawCommand = yaw;
  }

  const float error = targetPitch - pitch;
  integralAngle += error * dt;
  integralAngle = constrain(integralAngle, -22.0f, 22.0f);
  filteredD = 0.78f * filteredD - 0.22f * pitchRate;
  const float angleOut = Kp_angle * error + Ki_angle * integralAngle +
                         Kd_angle * filteredD;
  const float yawError = yawCommand - yaw;
  const float turn = constrain(Kp_yaw * yawError - Kd_yaw * yawRate,
                               -52.0f, 52.0f);

  float left = slew(angleOut + turn, lastMotorL);
  float right = slew(angleOut - turn, lastMotorR);
  lastMotorL = left;
  lastMotorR = right;
  setMotor(true, left);
  setMotor(false, right);
}

bool sensorDataFresh(unsigned long now) {
  return hasSensorFrame && now - lastSensorFrameAt <= SENSOR_STALE_MS;
}

bool floorSafeForForward() {
  return sensorDataFresh(millis()) &&
      (sensorFlags & SENSOR_FLAG_FRONT_FLOOR) && frontFloorMm <= FLOOR_LIMIT_MM;
}

bool floorSafeForReverse() {
  return sensorDataFresh(millis()) &&
      (sensorFlags & SENSOR_FLAG_REAR_FLOOR) && rearFloorMm <= FLOOR_LIMIT_MM;
}

void calibrateImu() {
  const uint16_t samples = 400;
  int32_t sumGY = 0;
  int32_t sumGZ = 0;
  int32_t sumAX = 0;
  uint16_t goodSamples = 0;

  for (uint16_t index = 0; index < samples; index++) {
    int16_t data[6];
    if (bmi160.getAccelGyroData(data) == 0) {
      sumGY += data[1];
      sumGZ += data[2];
      sumAX += data[3];
      goodSamples++;
    }
    delay(3);
  }

  if (goodSamples == 0) return;
  gyroOffsetY = (sumGY / static_cast<float>(goodSamples)) * 0.0305f;
  gyroOffsetZ = (sumGZ / static_cast<float>(goodSamples)) * 0.0305f;
  accelOffsetX = (sumAX / static_cast<float>(goodSamples)) / 16384.0f;
}

void readImu(float dt) {
  int16_t data[6];
  if (bmi160.getAccelGyroData(data) != 0) return;

  const float gyroY = data[1] * 0.0305f - gyroOffsetY;
  yawRate = data[2] * 0.0305f - gyroOffsetZ;
  pitchRate = gyroY;
  const float accX = data[3] / 16384.0f - accelOffsetX;
  const float accY = data[4] / 16384.0f;
  const float accZ = data[5] / 16384.0f;
  const float accelPitch = atan2(-accX, sqrt(accY * accY + accZ * accZ)) *
                           57.2957795f;
  pitch = ALPHA * (pitch + gyroY * dt) + (1.0f - ALPHA) * accelPitch;
  yaw += yawRate * dt;
  lastImuAt = millis();
}

void sendStatus(const char *message) {
  const uint8_t length = min(static_cast<size_t>(strlen(message)),
                             static_cast<size_t>(LINK_MAX_PAYLOAD));
  sendLinkPacket(Serial, LINK_STATUS_FRAME, txSequence++,
                 reinterpret_cast<const uint8_t *>(message), length);
}

void beep(uint16_t durationMs) {
  digitalWrite(BUZZER_PIN, HIGH);
  buzzerOffAt = millis() + durationMs;
  buzzerActive = true;
}

void updateBuzzer() {
  if (buzzerActive && static_cast<int32_t>(millis() - buzzerOffAt) >= 0) {
    digitalWrite(BUZZER_PIN, LOW);
    buzzerActive = false;
  }
}

uint8_t configChecksum(const StoredConfig &config) {
  return static_cast<uint8_t>(config.magic ^ config.obstacleCm ^
                              (config.obstacleCm >> 8) ^ 0xA7);
}

void loadConfig() {
  StoredConfig config;
  EEPROM.get(0, config);
  if (config.magic == CONFIG_MAGIC && config.checksum == configChecksum(config) &&
      config.obstacleCm >= 15 && config.obstacleCm <= 200) {
    obstacleLimitCm = config.obstacleCm;
  } else {
    saveObstacleLimit(DEFAULT_OBSTACLE_CM);
  }
}

void saveObstacleLimit(uint16_t value) {
  obstacleLimitCm = value;
  StoredConfig config;
  config.magic = CONFIG_MAGIC;
  config.obstacleCm = value;
  config.checksum = configChecksum(config);
  EEPROM.put(0, config);
}

float slew(float target, float current) {
  const float difference = target - current;
  if (difference > MAX_SLEW) return current + MAX_SLEW;
  if (difference < -MAX_SLEW) return current - MAX_SLEW;
  return target;
}

void setMotor(bool isLeft, float speed) {
  const int pwm = constrain(static_cast<int>(fabs(speed)), 0, MAX_PWM);
  const int appliedPwm = (pwm > 0 && pwm < DEADZONE) ? DEADZONE : pwm;
  if (isLeft) {
    digitalWrite(AIN1, speed > 0 ? HIGH : LOW);
    digitalWrite(AIN2, speed > 0 ? LOW : HIGH);
    if (speed == 0) {
      digitalWrite(AIN1, LOW);
      digitalWrite(AIN2, LOW);
    }
    analogWrite(PWMA, appliedPwm);
  } else {
    digitalWrite(BIN1, speed > 0 ? HIGH : LOW);
    digitalWrite(BIN2, speed > 0 ? LOW : HIGH);
    if (speed == 0) {
      digitalWrite(BIN1, LOW);
      digitalWrite(BIN2, LOW);
    }
    analogWrite(PWMB, appliedPwm);
  }
}

void stopMotors() {
  analogWrite(PWMA, 0);
  analogWrite(PWMB, 0);
  digitalWrite(AIN1, LOW);
  digitalWrite(AIN2, LOW);
  digitalWrite(BIN1, LOW);
  digitalWrite(BIN2, LOW);
  lastMotorL = 0.0f;
  lastMotorR = 0.0f;
}
