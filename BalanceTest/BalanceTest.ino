#include <Wire.h>
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
const uint16_t DEFAULT_OBSTACLE_CM = 35;
const uint16_t SENSOR_STALE_MS = 350;
const uint8_t TEST_PWM = 60;
const unsigned long LOOP_US = 5000;
const unsigned long LINK_REPORT_MS = 250;
const unsigned long TEST_PULSE_MS = 350;

DFRobot_BMI160 bmi160;
LinkParser parser;

uint8_t sensorFlags = 0;
uint8_t txSequence = 0;
uint16_t frontDistanceCm = 0xFFFF;
uint16_t frontFloorMm = 0xFFFF;
uint16_t rearFloorMm = 0xFFFF;
uint16_t obstacleLimitCm = DEFAULT_OBSTACLE_CM;
unsigned long lastSensorFrameAt = 0;
unsigned long lastImuAt = 0;
unsigned long lastLoopAt = 0;
unsigned long lastLinkReportAt = 0;
unsigned long lastActionAt = 0;
unsigned long actionEndsAt = 0;
unsigned long buzzerOffAt = 0;
unsigned long obstacleSince = 0;
unsigned long frontEdgeSince = 0;
unsigned long rearEdgeSince = 0;

float gyroOffsetY = 0.0f;
float gyroOffsetZ = 0.0f;
float accelOffsetX = 0.0f;
float pitch = 0.0f;
float pitchRate = 0.0f;
float yawRate = 0.0f;

int8_t leftCommand = 0;
int8_t rightCommand = 0;
bool imuReady = false;
bool sensorFrameSeen = false;
bool armed = false;
bool autoMode = false;
bool actionActive = false;
bool linkWasUp = false;
bool buzzerOn = false;
bool priorFrontEdge = false;
bool priorRearEdge = false;
bool priorObstacle = false;
uint16_t buzzerDuration = 0;

void readLink();
void handleCommand(const uint8_t *payload, uint8_t length);
void updateImu(float dt);
void calibrateImu();
void updateDiagnostics(unsigned long now);
void updateAutoTest(unsigned long now);
void updateMotorTest(unsigned long now);
void startMotion(int8_t left, int8_t right, uint16_t durationMs);
void stopMotion();
void setMotors(int8_t left, int8_t right);
void updateBuzzer();
void beep(uint16_t durationMs);
void sendStatus(const char *text);
void sendGyroFrame();
void manualMotion(int8_t left, int8_t right, uint16_t durationMs,
                  const char *reply);
void disarm(const char *reason);
bool linkFresh(unsigned long now);
bool floorFrontPresent();
bool floorRearPresent();
bool ultrasonicClear();

void setup() {
  pinMode(STATUS_LED_PIN, OUTPUT);
  digitalWrite(STATUS_LED_PIN, HIGH);
  pinMode(PWMA, OUTPUT);
  pinMode(AIN1, OUTPUT);
  pinMode(AIN2, OUTPUT);
  pinMode(PWMB, OUTPUT);
  pinMode(BIN1, OUTPUT);
  pinMode(BIN2, OUTPUT);
  pinMode(STBY, OUTPUT);
  pinMode(BUZZER_PIN, OUTPUT);
  digitalWrite(STBY, LOW);
  digitalWrite(BUZZER_PIN, LOW);
  stopMotion();

  Serial.begin(38400);
  Wire.begin();
  randomSeed(analogRead(A0));
  imuReady = bmi160.softReset() == BMI160_OK &&
             bmi160.I2cInit(0x68) == BMI160_OK;
  if (imuReady) {
    calibrateImu();
    lastImuAt = millis();
    beep(100);
    sendStatus("TEST IMU OK");
  } else {
    sendStatus("TEST ERR IMU");
  }
  lastLoopAt = micros();
  sendStatus("TEST DISARMED");
}

void loop() {
  readLink();
  updateBuzzer();

  const unsigned long nowMicros = micros();
  if (static_cast<unsigned long>(nowMicros - lastLoopAt) < LOOP_US) return;
  const float dt = static_cast<unsigned long>(nowMicros - lastLoopAt) * 1.0e-6f;
  lastLoopAt = nowMicros;
  if (imuReady) updateImu(dt);

  const unsigned long now = millis();
  updateDiagnostics(now);
  updateAutoTest(now);
  updateMotorTest(now);
  if (now - lastLinkReportAt >= LINK_REPORT_MS) {
    lastLinkReportAt = now;
    sendStatus(linkFresh(now) ? "HB,LINK=UP" : "HB,LINK=DOWN");
    sendGyroFrame();
  }
}

void readLink() {
  LinkPacket packet;
  while (parser.poll(Serial, packet)) {
    if (packet.type == LINK_SENSOR_FRAME && packet.length >= 7) {
      sensorFlags = packet.payload[0];
      frontDistanceCm = linkReadU16(&packet.payload[1]);
      frontFloorMm = linkReadU16(&packet.payload[3]);
      rearFloorMm = linkReadU16(&packet.payload[5]);
      lastSensorFrameAt = millis();
      sensorFrameSeen = true;
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
  for (uint8_t i = 0; i < length; i++) {
    if (command[i] >= 'a' && command[i] <= 'z') command[i] -= 'a' - 'A';
  }

  if (!strcmp(command, "HELLO") || !strcmp(command, "PING")) {
    sendStatus("ACK,BALANCE TEST");
  } else if (!strcmp(command, "ARM")) {
    if (!imuReady || !linkFresh(millis()) || !floorFrontPresent() ||
        !floorRearPresent()) {
      sendStatus("ERR ARM CHECK SENSORS");
      beep(300);
      return;
    }
    armed = true;
    digitalWrite(STBY, HIGH);
    sendStatus("ACK,ARMED");
    beep(80);
  } else if (!strcmp(command, "DISARM")) {
    disarm("ACK,DISARMED");
  } else if (!strcmp(command, "AUTO")) {
    if (!armed) sendStatus("ERR ARM FIRST");
    else {
      autoMode = true;
      stopMotion();
      sendStatus("ACK,AUTO TEST");
    }
  } else if (!strcmp(command, "MANUAL")) {
    autoMode = false;
    stopMotion();
    sendStatus("ACK,MANUAL TEST");
  } else if (!strcmp(command, "STOP")) {
    autoMode = false;
    stopMotion();
    sendStatus("ACK,STOP");
  } else if (!strcmp(command, "FWD")) {
    manualMotion(TEST_PWM, TEST_PWM, TEST_PULSE_MS, "ACK,FWD");
  } else if (!strcmp(command, "REV")) {
    manualMotion(-TEST_PWM, -TEST_PWM, TEST_PULSE_MS, "ACK,REV");
  } else if (!strcmp(command, "LEFT")) {
    manualMotion(TEST_PWM / 2, TEST_PWM, TEST_PULSE_MS, "ACK,LEFT ARC");
  } else if (!strcmp(command, "RIGHT")) {
    manualMotion(TEST_PWM, TEST_PWM / 2, TEST_PULSE_MS, "ACK,RIGHT ARC");
  } else if (!strcmp(command, "PIVOT_L")) {
    manualMotion(-TEST_PWM / 2, TEST_PWM / 2, TEST_PULSE_MS, "ACK,PIVOT L");
  } else if (!strcmp(command, "PIVOT_R")) {
    manualMotion(TEST_PWM / 2, -TEST_PWM / 2, TEST_PULSE_MS, "ACK,PIVOT R");
  } else if (!strcmp(command, "UTURN_L")) {
    manualMotion(TEST_PWM / 3, TEST_PWM, 900, "ACK,UTURN L ARC");
  } else if (!strcmp(command, "UTURN_R")) {
    manualMotion(TEST_PWM, TEST_PWM / 3, 900, "ACK,UTURN R ARC");
  } else if (!strncmp(command, "DIST=", 5)) {
    const long value = atol(command + 5);
    if (value >= 10 && value <= 200) {
      obstacleLimitCm = static_cast<uint16_t>(value);
      sendStatus("ACK,DIST SET");
    } else sendStatus("ERR DIST 10..200");
  } else if (!strcmp(command, "STATUS")) {
    sendStatus("ACK,STATUS STREAM ACTIVE");
  } else {
    sendStatus("ERR UNKNOWN");
  }
}

void disarm(const char *reason) {
  armed = false;
  autoMode = false;
  digitalWrite(STBY, LOW);
  stopMotion();
  sendStatus(reason);
}

void manualMotion(int8_t left, int8_t right, uint16_t duration, const char *reply) {
  autoMode = false;
  if (!armed) {
    sendStatus("ERR ARM FIRST");
    return;
  }
  if (!motionSafe(left, right)) {
    sendStatus("BLOCKED,SENSOR LIMIT");
    beep(250);
    return;
  }
  startMotion(left, right, duration);
  sendStatus(reply);
}

void updateImu(float dt) {
  int16_t data[6];
  if (bmi160.getAccelGyroData(data) != 0) return;
  const float gyroY = data[1] * 0.0305f - gyroOffsetY;
  const float gyroZ = data[2] * 0.0305f - gyroOffsetZ;
  pitchRate = gyroY;
  yawRate = gyroZ;
  const float accX = data[3] / 16384.0f - accelOffsetX;
  const float accY = data[4] / 16384.0f;
  const float accZ = data[5] / 16384.0f;
  const float accPitch = atan2(-accX, sqrt(accY * accY + accZ * accZ)) * 57.29578f;
  pitch = 0.985f * (pitch + gyroY * dt) + 0.015f * accPitch;
  lastImuAt = millis();
}

void calibrateImu() {
  int32_t sumY = 0, sumZ = 0, sumX = 0;
  uint16_t count = 0;
  for (uint16_t i = 0; i < 250; i++) {
    int16_t data[6];
    if (bmi160.getAccelGyroData(data) == 0) {
      sumY += data[1];
      sumZ += data[2];
      sumX += data[3];
      count++;
    }
    delay(3);
  }
  if (count == 0) return;
  gyroOffsetY = (sumY / static_cast<float>(count)) * 0.0305f;
  gyroOffsetZ = (sumZ / static_cast<float>(count)) * 0.0305f;
  accelOffsetX = (sumX / static_cast<float>(count)) / 16384.0f;
}

void updateDiagnostics(unsigned long now) {
  const bool up = linkFresh(now);
  if (up != linkWasUp) {
    linkWasUp = up;
    if (up) {
      sendStatus("EVENT,LINK RESTORED");
      beep(80);
    } else if (sensorFrameSeen) {
      sendStatus("EVENT,LINK LOST");
      armed = false;
      autoMode = false;
      digitalWrite(STBY, LOW);
      stopMotion();
      beep(500);
    }
  }

  const bool frontEdge = linkFresh(now) && (sensorFlags & SENSOR_FLAG_FRONT_FLOOR) &&
                         frontFloorMm > FLOOR_LIMIT_MM;
  const bool rearEdge = linkFresh(now) && (sensorFlags & SENSOR_FLAG_REAR_FLOOR) &&
                        rearFloorMm > FLOOR_LIMIT_MM;
  const bool obstacle = linkFresh(now) && (sensorFlags & SENSOR_FLAG_ULTRASONIC) &&
                        frontDistanceCm != 0xFFFF && frontDistanceCm <= obstacleLimitCm;

  static bool wasFrontEdge = false;
  static bool wasRearEdge = false;
  static bool wasObstacle = false;
  if (frontEdge && !wasFrontEdge) { sendStatus("EVENT,FRONT EDGE"); beep(180); }
  if (rearEdge && !wasRearEdge) { sendStatus("EVENT,REAR EDGE"); beep(350); }
  if (obstacle && !wasObstacle) { sendStatus("EVENT,OBSTACLE"); beep(90); }
  wasFrontEdge = frontEdge;
  wasRearEdge = rearEdge;
  wasObstacle = obstacle;

  if (imuReady && fabs(pitch) > 45.0f && armed) {
    armed = false;
    autoMode = false;
    digitalWrite(STBY, LOW);
    stopMotion();
    sendStatus("EVENT,TILT DISARM");
    beep(500);
  }
}

void updateAutoTest(unsigned long now) {
  if (!armed || !autoMode || actionActive || !linkFresh(now)) return;
  if (!imuReady || now - lastImuAt > 100) {
    disarm("ERR IMU STALE");
    return;
  }

  const bool frontFloor = floorFrontPresent();
  const bool rearFloor = floorRearPresent();
  const bool obstacle = (sensorFlags & SENSOR_FLAG_ULTRASONIC) &&
      frontDistanceCm != 0xFFFF && frontDistanceCm <= obstacleLimitCm;

  if (!frontFloor) {
    if (rearFloor) startMotion(-35, -35, 250);
    else stopMotion();
    sendStatus("AUTO,FRONT EDGE");
    return;
  }
  if (obstacle) {
    if (rearFloor) startMotion(-35, 35, 300);
    else stopMotion();
    sendStatus("AUTO,OBSTACLE TURN");
    return;
  }
  if (!rearFloor) {
    stopMotion();
    sendStatus("AUTO,REAR EDGE HOLD");
    return;
  }

  startMotion(45, 45, 250);
}

void updateMotorTest(unsigned long now) {
  if (!armed) {
    stopMotion();
    return;
  }
  if (!linkFresh(now)) {
    disarm("ERR SENSOR LINK");
    return;
  }
  if (imuReady && now - lastImuAt > 100) {
    disarm("ERR IMU STALE");
    return;
  }
  if (actionActive && static_cast<int32_t>(now - actionEndsAt) >= 0) {
    stopMotion();
  }
  if (!motionSafe(leftCommand, rightCommand)) {
    stopMotion();
    sendStatus("EVENT,MOTION BLOCKED");
  }
  if (actionActive) setMotors(leftCommand, rightCommand);
  else setMotors(0, 0);
}

bool motionSafe(int8_t left, int8_t right) {
  if (!linkFresh(millis())) return false;
  if (left * right < 0)
    return floorFrontPresent() && floorRearPresent();
  if (left > 0 || right > 0) {
    // Forward motion is blocked when the ultrasonic sensor is missing,
    // invalid, stale, or reporting an obstacle.
    if (!floorFrontPresent() ||
        !(sensorFlags & SENSOR_FLAG_ULTRASONIC) ||
        frontDistanceCm == 0xFFFF || ultrasonicBlocked()) return false;
  }
  if (left < 0 || right < 0) {
    if (!floorRearPresent()) return false;
  }
  if (left * right < 0 && (!floorFrontPresent() || !floorRearPresent()))
    return false;
  return true;
}

bool floorFrontPresent() {
  return (sensorFlags & SENSOR_FLAG_FRONT_FLOOR) && frontFloorMm <= FLOOR_LIMIT_MM;
}

bool floorRearPresent() {
  return (sensorFlags & SENSOR_FLAG_REAR_FLOOR) && rearFloorMm <= FLOOR_LIMIT_MM;
}

bool ultrasonicBlocked() {
  return (sensorFlags & SENSOR_FLAG_ULTRASONIC) && frontDistanceCm != 0xFFFF &&
         frontDistanceCm <= obstacleLimitCm;
}

bool linkFresh(unsigned long now) {
  return sensorFrameSeen && now - lastSensorFrameAt <= SENSOR_STALE_MS;
}

void startMotion(int8_t left, int8_t right, uint16_t durationMs) {
  if (!motionSafe(left, right)) {
    stopMotion();
    sendStatus("BLOCKED,SENSOR LIMIT");
    return;
  }
  leftCommand = left;
  rightCommand = right;
  actionEndsAt = millis() + durationMs;
  actionActive = true;
}

void stopMotion() {
  leftCommand = 0;
  rightCommand = 0;
  actionActive = false;
  setMotors(0, 0);
}

void setMotors(int8_t left, int8_t right) {
  if (!armed) {
    analogWrite(PWMA, 0);
    analogWrite(PWMB, 0);
    digitalWrite(AIN1, LOW); digitalWrite(AIN2, LOW);
    digitalWrite(BIN1, LOW); digitalWrite(BIN2, LOW);
    return;
  }
  digitalWrite(AIN1, left > 0 ? HIGH : LOW);
  digitalWrite(AIN2, left < 0 ? HIGH : LOW);
  digitalWrite(BIN1, right > 0 ? HIGH : LOW);
  digitalWrite(BIN2, right < 0 ? HIGH : LOW);
  analogWrite(PWMA, abs(left));
  analogWrite(PWMB, abs(right));
}

void sendStatus(const char *text) {
  sendLinkPacket(Serial, LINK_STATUS_FRAME, txSequence++,
                 reinterpret_cast<const uint8_t *>(text),
                 min(static_cast<size_t>(strlen(text)),
                     static_cast<size_t>(LINK_MAX_PAYLOAD)));
}

void sendGyroFrame() {
  uint8_t payload[9];
  linkWriteU16(&payload[0], static_cast<uint16_t>(static_cast<int16_t>(pitch * 100.0f)));
  linkWriteU16(&payload[2], static_cast<uint16_t>(static_cast<int16_t>(pitchRate * 100.0f)));
  linkWriteU16(&payload[4], static_cast<uint16_t>(static_cast<int16_t>(yawRate * 100.0f)));
  payload[6] = armed ? 1 : 0;
  payload[7] = autoMode ? 1 : 0;
  payload[8] = imuReady ? 1 : 0;
  sendLinkPacket(Serial, TEST_GYRO_FRAME, txSequence++, payload, sizeof(payload));
}

void updateBuzzer() {
  static unsigned long offAt = 0;
  static bool on = false;
  const unsigned long now = millis();
  if (on && static_cast<int32_t>(now - offAt) >= 0) {
    digitalWrite(BUZZER_PIN, LOW);
    on = false;
  }
  if (buzzerOn && !on) {
    digitalWrite(BUZZER_PIN, HIGH);
    on = true;
    offAt = now + buzzerDuration;
    buzzerOn = false;
  }
}

void beep(uint16_t durationMs) {
  buzzerDuration = durationMs;
  buzzerOn = true;
}
