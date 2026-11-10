#include <Wire.h>
#include <SoftwareSerial.h>
#include <VL53L0X.h>
#include "RobotLink.h"

const uint8_t BT_RX_PIN = 3;
const uint8_t BT_TX_PIN = 2;
const uint8_t XSHUT_FRONT_PIN = A2;
const uint8_t XSHUT_REAR_PIN = A1;
const uint8_t STATUS_LED_PIN = 13;
const uint8_t FRONT_TOF_ADDRESS = 0x30;
const uint8_t REAR_TOF_ADDRESS = 0x31;
const uint8_t RCWL_ADDRESS = 0x57;
const uint8_t SENSOR_FLAG_ULTRASONIC = 0x01;
const uint8_t SENSOR_FLAG_FRONT_FLOOR = 0x02;
const uint8_t SENSOR_FLAG_REAR_FLOOR = 0x04;
const unsigned long TOF_SAMPLE_MS = 70;
const unsigned long ULTRASONIC_PERIOD_MS = 250;
const unsigned long ULTRASONIC_CONVERSION_MS = 150;
const unsigned long LINK_PERIOD_MS = 50;
const unsigned long TELEMETRY_PERIOD_MS = 300;
const unsigned long LINK_TIMEOUT_MS = 1000;

SoftwareSerial bluetooth(BT_RX_PIN, BT_TX_PIN);
VL53L0X frontTof;
VL53L0X rearTof;
LinkParser controllerParser;

uint16_t frontFloorMm = 0xFFFF;
uint16_t rearFloorMm = 0xFFFF;
uint16_t frontDistanceCm = 0xFFFF;
bool frontTofReady = false;
bool rearTofReady = false;
bool frontFloorValid = false;
bool rearFloorValid = false;
bool ultrasonicValid = false;
bool ultrasonicPending = false;
bool controllerSeen = false;
bool linkWasUp = false;
bool controllerImuReady = false;
unsigned long ultrasonicReadyAt = 0;
unsigned long lastUltrasonicTriggerAt = 0;
unsigned long lastTofSampleAt = 0;
unsigned long lastLinkAt = 0;
unsigned long lastTelemetryAt = 0;
unsigned long lastControllerFrameAt = 0;
uint8_t linkSequence = 0;
uint8_t commandSequence = 0;
char bluetoothLine[LINK_MAX_PAYLOAD];
uint8_t bluetoothLineLength = 0;
int16_t pitchCentiDegrees = 0;
int16_t pitchRateCentiDps = 0;
int16_t yawRateCentiDps = 0;
bool controllerArmed = false;
bool controllerAuto = false;

void initializeTofSensors();
void sampleFloorSensors(unsigned long now);
void updateUltrasonic(unsigned long now);
void triggerUltrasonic(unsigned long now);
void readUltrasonic();
void readBluetoothCommands();
void readControllerLink();
void sendSensorFrame();
void sendBluetoothTelemetry(unsigned long now);
void sendLinkCommand(const char *command);
void appendDistance(char *line, const char *name, bool valid, uint16_t value,
                    const char *unit);

void setup() {
  pinMode(STATUS_LED_PIN, OUTPUT);
  digitalWrite(STATUS_LED_PIN, HIGH);
  Serial.begin(38400);
  bluetooth.begin(9600);
  Wire.begin();
  randomSeed(analogRead(A0));

  pinMode(XSHUT_FRONT_PIN, OUTPUT);
  pinMode(XSHUT_REAR_PIN, OUTPUT);
  initializeTofSensors();
  bluetooth.println(F("BOOT,SENSOR TEST; send STATUS"));
  if (!frontTofReady) bluetooth.println(F("ERR,FRONT TOF INIT"));
  if (!rearTofReady) bluetooth.println(F("ERR,REAR TOF INIT"));
  sendLinkCommand("PING");
}

void loop() {
  const unsigned long now = millis();
  readBluetoothCommands();
  readControllerLink();
  sampleFloorSensors(now);
  updateUltrasonic(now);

  if (now - lastLinkAt >= LINK_PERIOD_MS) {
    lastLinkAt = now;
    sendSensorFrame();
  }
  if (now - lastTelemetryAt >= TELEMETRY_PERIOD_MS) {
    lastTelemetryAt = now;
    sendBluetoothTelemetry(now);
  }
}

void initializeTofSensors() {
  digitalWrite(XSHUT_FRONT_PIN, LOW);
  digitalWrite(XSHUT_REAR_PIN, LOW);
  delay(15);

  digitalWrite(XSHUT_FRONT_PIN, HIGH);
  delay(15);
  frontTofReady = frontTof.init();
  if (frontTofReady) {
    frontTof.setAddress(FRONT_TOF_ADDRESS);
    frontTof.setTimeout(25);
    frontTof.startContinuous(50);
  } else {
    digitalWrite(XSHUT_FRONT_PIN, LOW);
  }

  digitalWrite(XSHUT_REAR_PIN, HIGH);
  delay(15);
  rearTofReady = rearTof.init();
  if (rearTofReady) {
    rearTof.setAddress(REAR_TOF_ADDRESS);
    rearTof.setTimeout(25);
    rearTof.startContinuous(50);
  }
}

void sampleFloorSensors(unsigned long now) {
  if (now - lastTofSampleAt < TOF_SAMPLE_MS) return;
  lastTofSampleAt = now;

  if (frontTofReady) {
    const uint16_t reading = frontTof.readRangeContinuousMillimeters();
    const bool timedOut = frontTof.timeoutOccurred();
    frontFloorValid = !timedOut;
    frontFloorMm = timedOut ? 0xFFFF : reading;
  } else {
    frontFloorValid = false;
    frontFloorMm = 0xFFFF;
  }

  if (rearTofReady) {
    const uint16_t reading = rearTof.readRangeContinuousMillimeters();
    const bool timedOut = rearTof.timeoutOccurred();
    rearFloorValid = !timedOut;
    rearFloorMm = timedOut ? 0xFFFF : reading;
  } else {
    rearFloorValid = false;
    rearFloorMm = 0xFFFF;
  }
}

void updateUltrasonic(unsigned long now) {
  if (ultrasonicPending && static_cast<int32_t>(now - ultrasonicReadyAt) >= 0) {
    readUltrasonic();
    ultrasonicPending = false;
  }
  if (!ultrasonicPending && now - lastUltrasonicTriggerAt >= ULTRASONIC_PERIOD_MS)
    triggerUltrasonic(now);
}

void triggerUltrasonic(unsigned long now) {
  lastUltrasonicTriggerAt = now;
  Wire.beginTransmission(RCWL_ADDRESS);
  Wire.write(0x01);
  ultrasonicPending = Wire.endTransmission() == 0;
  ultrasonicReadyAt = now + ULTRASONIC_CONVERSION_MS;
  if (!ultrasonicPending) ultrasonicValid = false;
}

void readUltrasonic() {
  const uint8_t received = Wire.requestFrom(RCWL_ADDRESS, static_cast<uint8_t>(3));
  if (received < 3 || Wire.available() < 3) {
    while (Wire.available()) Wire.read();
    ultrasonicValid = false;
    frontDistanceCm = 0xFFFF;
    return;
  }

  const uint32_t highByte = static_cast<uint8_t>(Wire.read());
  const uint32_t middleByte = static_cast<uint8_t>(Wire.read());
  const uint32_t lowByte = static_cast<uint8_t>(Wire.read());
  const uint32_t distanceMicrometers = (highByte << 16) | (middleByte << 8) | lowByte;
  const uint32_t distanceCm = distanceMicrometers / 10000UL;
  ultrasonicValid = distanceCm > 0 && distanceCm < 450;
  frontDistanceCm = ultrasonicValid ? static_cast<uint16_t>(distanceCm) : 0xFFFF;
}

void readBluetoothCommands() {
  while (bluetooth.available() > 0) {
    const char value = static_cast<char>(bluetooth.read());
    if (value == '\r') continue;
    if (value == '\n') {
      if (bluetoothLineLength > 0) {
        sendLinkCommand(bluetoothLine);
        bluetoothLineLength = 0;
      }
      continue;
    }
    if (bluetoothLineLength < sizeof(bluetoothLine) - 1) {
      bluetoothLine[bluetoothLineLength++] = value;
    } else {
      bluetoothLineLength = 0;
      bluetooth.println(F("ERR,COMMAND TOO LONG"));
    }
  }
}

void sendLinkCommand(const char *command) {
  sendLinkPacket(Serial, LINK_COMMAND_FRAME, commandSequence++,
                 reinterpret_cast<const uint8_t *>(command),
                 min(static_cast<size_t>(strlen(command)),
                     static_cast<size_t>(LINK_MAX_PAYLOAD)));
}

void readControllerLink() {
  LinkPacket packet;
  while (controllerParser.poll(Serial, packet)) {
    lastControllerFrameAt = millis();
    controllerSeen = true;
    if (packet.type == LINK_STATUS_FRAME && packet.length > 0) {
      bluetooth.write(packet.payload, packet.length);
      bluetooth.println();
    } else if (packet.type == TEST_GYRO_FRAME && packet.length >= 9) {
      pitchCentiDegrees = static_cast<int16_t>(linkReadU16(&packet.payload[0]));
      pitchRateCentiDps = static_cast<int16_t>(linkReadU16(&packet.payload[2]));
      yawRateCentiDps = static_cast<int16_t>(linkReadU16(&packet.payload[4]));
      controllerArmed = packet.payload[6] != 0;
      controllerAuto = packet.payload[7] != 0;
      controllerImuReady = packet.payload[8] != 0;
    }
  }
}

void sendSensorFrame() {
  uint8_t payload[7];
  payload[0] = (ultrasonicValid ? SENSOR_FLAG_ULTRASONIC : 0) |
               (frontFloorValid ? SENSOR_FLAG_FRONT_FLOOR : 0) |
               (rearFloorValid ? SENSOR_FLAG_REAR_FLOOR : 0);
  linkWriteU16(&payload[1], frontDistanceCm);
  linkWriteU16(&payload[3], frontFloorMm);
  linkWriteU16(&payload[5], rearFloorMm);
  sendLinkPacket(Serial, LINK_SENSOR_FRAME, linkSequence++, payload,
                 sizeof(payload));
}

void sendBluetoothTelemetry(unsigned long now) {
  const bool linkUp = controllerSeen && now - lastControllerFrameAt <= LINK_TIMEOUT_MS;
  if (linkUp != linkWasUp) {
    linkWasUp = linkUp;
    bluetooth.println(linkUp ? F("EVENT,LINK UP") : F("EVENT,LINK LOST"));
  }

  char line[LINK_MAX_PAYLOAD * 5];
  snprintf(line, sizeof(line), "DATA,LINK:%s,IMU:%s,ARM:%u,AUTO:%u",
           linkUp ? "UP" : "DOWN", controllerImuReady ? "OK" : "ERR",
           controllerArmed ? 1 : 0,
           controllerAuto ? 1 : 0);
  bluetooth.println(line);

  appendDistance(line, "ULTRA", ultrasonicValid, frontDistanceCm, "cm");
  bluetooth.println(line);
  appendDistance(line, "FLOOR_F", frontFloorValid, frontFloorMm, "mm");
  appendDistance(line, "FLOOR_R", rearFloorValid, rearFloorMm, "mm");
  bluetooth.println(line);

  snprintf(line, sizeof(line), "GYRO,P:%d.%02u,PR:%d.%02u,YR:%d.%02u",
           pitchCentiDegrees / 100, abs(pitchCentiDegrees % 100),
           pitchRateCentiDps / 100, abs(pitchRateCentiDps % 100),
           yawRateCentiDps / 100, abs(yawRateCentiDps % 100));
  bluetooth.println(line);
}

const char *imuStateLabel() {
  return controllerImuReady ? "OK" : "ERR";
}

void appendDistance(char *line, const char *name, bool valid, uint16_t value,
                    const char *unit) {
  snprintf(line, sizeof(line), "%s,%s:%s", name, valid ? "OK" : "ERR",
           valid ? "" : "NA");
  if (valid) {
    const size_t used = strlen(line);
    snprintf(line + used, sizeof(line) - used, "%u%s", value, unit);
  }
}
