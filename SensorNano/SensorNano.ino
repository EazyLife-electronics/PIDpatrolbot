#include <Wire.h>
#include <SoftwareSerial.h>
#include <VL53L0X.h>
#include "RobotLink.h"

const uint8_t BT_RX_PIN = 2;
const uint8_t BT_TX_PIN = 3;
const uint8_t XSHUT_FRONT_PIN = A2;
const uint8_t XSHUT_REAR_PIN = A1;
const uint8_t STATUS_LED_PIN = 13;
const uint8_t FRONT_TOF_ADDRESS = 0x30;
const uint8_t REAR_TOF_ADDRESS = 0x31;
const uint8_t RCWL_ADDRESS = 0x57;
const uint8_t SENSOR_FLAG_ULTRASONIC = 0x01;
const uint8_t SENSOR_FLAG_FRONT_FLOOR = 0x02;
const uint8_t SENSOR_FLAG_REAR_FLOOR = 0x04;
const uint8_t SENSOR_ULTRA_STATUS_SHIFT = 3;
const uint8_t SENSOR_ULTRA_STATUS_MASK = 0x38;
const uint8_t ULTRA_STATUS_VALID = 0;
const uint8_t ULTRA_STATUS_OUT_OF_RANGE = 1;
const uint8_t ULTRA_STATUS_TRIGGER_ERROR = 2;
const uint8_t ULTRA_STATUS_READ_ERROR = 3;
const uint8_t ULTRA_STATUS_STALE = 4;
const uint16_t ULTRASONIC_MAX_NAV_CM = 90;

const unsigned long TOF_SAMPLE_MS = 70;
const unsigned long ULTRASONIC_PERIOD_MS = 250;
const unsigned long ULTRASONIC_CONVERSION_MS = 150;
const unsigned long ULTRASONIC_STALE_MS = 750;
const unsigned long TOF_RETRY_MS = 3000;
const unsigned long LINK_PERIOD_MS = 50;

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
uint8_t ultrasonicStatus = ULTRA_STATUS_OUT_OF_RANGE;
bool ultrasonicPending = false;
unsigned long ultrasonicReadyAt = 0;
unsigned long lastUltrasonicTriggerAt = 0;
unsigned long lastUltrasonicReadAt = 0;
unsigned long lastTofSampleAt = 0;
unsigned long lastTofRetryAt = 0;
uint8_t frontTofTimeoutCount = 0;
uint8_t rearTofTimeoutCount = 0;
unsigned long lastLinkAt = 0;
uint8_t linkSequence = 0;
uint8_t commandSequence = 0;
char bluetoothLine[LINK_MAX_PAYLOAD];
uint8_t bluetoothLineLength = 0;

void initializeTofSensors();
void sampleFloorSensors(unsigned long now);
void updateUltrasonic(unsigned long now);
void triggerUltrasonic(unsigned long now);
void readUltrasonic();
void readBluetoothCommands();
void forwardControllerMessages();
void sendSensorFrame();
void sendStatusToController(const char *message);

void setup() {
  pinMode(STATUS_LED_PIN, OUTPUT);
  digitalWrite(STATUS_LED_PIN, HIGH);
  Serial.begin(38400);
  bluetooth.begin(9600);
  Wire.begin();

  pinMode(XSHUT_FRONT_PIN, OUTPUT);
  pinMode(XSHUT_REAR_PIN, OUTPUT);
  initializeTofSensors();
  lastTofRetryAt = millis();
  sendStatusToController("SENSOR NODE READY");
}

void loop() {
  const unsigned long now = millis();
  readBluetoothCommands();
  forwardControllerMessages();
  sampleFloorSensors(now);
  if ((!frontTofReady || !rearTofReady) && now - lastTofRetryAt >= TOF_RETRY_MS) {
    lastTofRetryAt = now;
    frontFloorValid = false; rearFloorValid = false;
    frontFloorMm = 0xFFFF; rearFloorMm = 0xFFFF;
    initializeTofSensors();
    frontTofTimeoutCount = 0;
    rearTofTimeoutCount = 0;
  }
  updateUltrasonic(now);

  if (now - lastLinkAt >= LINK_PERIOD_MS) {
    lastLinkAt = now;
    sendSensorFrame();
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
    if (timedOut) { if (frontTofTimeoutCount < 5) frontTofTimeoutCount++; if (frontTofTimeoutCount >= 5) frontTofReady = false; }
    else frontTofTimeoutCount = 0;
    frontFloorValid = !timedOut;
    frontFloorMm = timedOut ? 0xFFFF : reading;
  } else {
    frontFloorValid = false;
    frontFloorMm = 0xFFFF;
  }

  if (rearTofReady) {
    const uint16_t reading = rearTof.readRangeContinuousMillimeters();
    const bool timedOut = rearTof.timeoutOccurred();
    if (timedOut) { if (rearTofTimeoutCount < 5) rearTofTimeoutCount++; if (rearTofTimeoutCount >= 5) rearTofReady = false; }
    else rearTofTimeoutCount = 0;
    rearFloorValid = !timedOut;
    rearFloorMm = timedOut ? 0xFFFF : reading;
  } else {
    rearFloorValid = false;
    rearFloorMm = 0xFFFF;
  }
}

void updateUltrasonic(unsigned long now) {
  if (ultrasonicValid && now - lastUltrasonicReadAt > ULTRASONIC_STALE_MS) {
    ultrasonicValid = false;
    ultrasonicStatus = ULTRA_STATUS_STALE;
    frontDistanceCm = 0xFFFF;
  }
  if (ultrasonicPending && static_cast<int32_t>(now - ultrasonicReadyAt) >= 0) {
    readUltrasonic();
    ultrasonicPending = false;
  }

  if (!ultrasonicPending && now - lastUltrasonicTriggerAt >= ULTRASONIC_PERIOD_MS) {
    triggerUltrasonic(now);
  }
}

void triggerUltrasonic(unsigned long now) {
  lastUltrasonicTriggerAt = now;
  Wire.beginTransmission(RCWL_ADDRESS);
  Wire.write(0x01);
  const uint8_t result = Wire.endTransmission();
  ultrasonicPending = result == 0;
  ultrasonicReadyAt = now + ULTRASONIC_CONVERSION_MS;
  if (!ultrasonicPending) {
    ultrasonicValid = false;
    ultrasonicStatus = ULTRA_STATUS_TRIGGER_ERROR;
    frontDistanceCm = 0xFFFF;
  }
}

void readUltrasonic() {
  const uint8_t received = Wire.requestFrom(RCWL_ADDRESS, static_cast<uint8_t>(3));
  if (received < 3 || Wire.available() < 3) {
    while (Wire.available()) Wire.read();
    ultrasonicValid = false;
    ultrasonicStatus = ULTRA_STATUS_READ_ERROR;
    frontDistanceCm = 0xFFFF;
    return;
  }

  const uint32_t highByte = static_cast<uint8_t>(Wire.read());
  const uint32_t middleByte = static_cast<uint8_t>(Wire.read());
  const uint32_t lowByte = static_cast<uint8_t>(Wire.read());
  const uint32_t distanceMicrometers = (highByte << 16) | (middleByte << 8) | lowByte;
  const uint32_t distanceCm = distanceMicrometers / 10000UL;

  ultrasonicValid = distanceCm > 0 && distanceCm <= ULTRASONIC_MAX_NAV_CM;
  ultrasonicStatus = ultrasonicValid ? ULTRA_STATUS_VALID
                                     : ULTRA_STATUS_OUT_OF_RANGE;
  frontDistanceCm = ultrasonicValid ? static_cast<uint16_t>(distanceCm) : 0xFFFF;
  if (ultrasonicValid) lastUltrasonicReadAt = millis();
}

void readBluetoothCommands() {
  while (bluetooth.available() > 0) {
    const char value = static_cast<char>(bluetooth.read());
    if (value == '\r') continue;
    if (value == '\n') {
      if (bluetoothLineLength > 0) {
        sendLinkPacket(Serial, LINK_COMMAND_FRAME, commandSequence++,
                       reinterpret_cast<const uint8_t *>(bluetoothLine),
                       bluetoothLineLength);
        bluetoothLineLength = 0;
      }
      continue;
    }
    if (bluetoothLineLength < sizeof(bluetoothLine) - 1) {
      bluetoothLine[bluetoothLineLength++] = value;
    } else {
      bluetoothLineLength = 0;
      bluetooth.println(F("ERR COMMAND TOO LONG"));
    }
  }
}

void forwardControllerMessages() {
  LinkPacket packet;
  while (controllerParser.poll(Serial, packet)) {
    if (packet.type == LINK_STATUS_FRAME && packet.length > 0) {
      bluetooth.write(packet.payload, packet.length);
      bluetooth.println();
    }
  }
}

void sendSensorFrame() {
  uint8_t payload[7];
  payload[0] = (ultrasonicValid ? SENSOR_FLAG_ULTRASONIC : 0) |
               (frontFloorValid ? SENSOR_FLAG_FRONT_FLOOR : 0) |
               (rearFloorValid ? SENSOR_FLAG_REAR_FLOOR : 0) |
               ((ultrasonicStatus << SENSOR_ULTRA_STATUS_SHIFT) & SENSOR_ULTRA_STATUS_MASK);
  linkWriteU16(&payload[1], frontDistanceCm);
  linkWriteU16(&payload[3], frontFloorMm);
  linkWriteU16(&payload[5], rearFloorMm);
  sendLinkPacket(Serial, LINK_SENSOR_FRAME, linkSequence++, payload,
                 sizeof(payload));
}

void sendStatusToController(const char *message) {
  const uint8_t length = min(static_cast<size_t>(strlen(message)),
                             static_cast<size_t>(LINK_MAX_PAYLOAD));
  sendLinkPacket(Serial, LINK_STATUS_FRAME, linkSequence++,
                 reinterpret_cast<const uint8_t *>(message), length);
}
