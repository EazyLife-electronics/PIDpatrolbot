#pragma once

#include <Arduino.h>

const uint8_t LINK_SYNC_1 = 0xA5;
const uint8_t LINK_SYNC_2 = 0x5A;
const uint8_t LINK_MAX_PAYLOAD = 20;
const uint8_t LINK_SENSOR_FRAME = 1;
const uint8_t LINK_COMMAND_FRAME = 2;
const uint8_t LINK_STATUS_FRAME = 3;
const uint16_t LINK_INTERBYTE_TIMEOUT_MS = 20;

// CRC-8, polynomial 0x07, initial value 0x00.
// Both Nano sketches must use this exact packet definition.
inline uint8_t linkCrc8Update(uint8_t crc, uint8_t value) {
  crc ^= value;
  for (uint8_t bit = 0; bit < 8; bit++) {
    crc = (crc & 0x80) ? static_cast<uint8_t>((crc << 1) ^ 0x07)
                       : static_cast<uint8_t>(crc << 1);
  }
  return crc;
}

struct LinkPacket {
  uint8_t type;
  uint8_t sequence;
  uint8_t length;
  uint8_t payload[LINK_MAX_PAYLOAD];
};

class LinkParser {
public:
  LinkParser() : state(0), payloadIndex(0), crc(0), lastByteAt(0) {}

  bool poll(Stream &stream, LinkPacket &packet) {
    while (stream.available() > 0) {
      const unsigned long now = millis();
      if (state != 0 &&
          static_cast<unsigned long>(now - lastByteAt) > LINK_INTERBYTE_TIMEOUT_MS) {
        reset();
      }

      const uint8_t value = static_cast<uint8_t>(stream.read());
      lastByteAt = now;

      switch (state) {
        case 0:
          if (value == LINK_SYNC_1) state = 1;
          break;

        case 1:
          if (value == LINK_SYNC_2) state = 2;
          else state = (value == LINK_SYNC_1) ? 1 : 0;
          break;

        case 2:
          if (value < LINK_SENSOR_FRAME || value > LINK_STATUS_FRAME) {
            reset();
          } else {
            packet.type = value;
            crc = linkCrc8Update(0, value);
            state = 3;
          }
          break;

        case 3:
          packet.sequence = value;
          crc = linkCrc8Update(crc, value);
          state = 4;
          break;

        case 4:
          packet.length = value;
          crc = linkCrc8Update(crc, value);
          if (value > LINK_MAX_PAYLOAD) {
            reset();
          } else {
            payloadIndex = 0;
            state = value == 0 ? 6 : 5;
          }
          break;

        case 5:
          packet.payload[payloadIndex++] = value;
          crc = linkCrc8Update(crc, value);
          if (payloadIndex >= packet.length) state = 6;
          break;

        case 6: {
          const bool valid = value == crc;
          reset();
          if (valid) return true;
          break;
        }

        default:
          reset();
          break;
      }
    }
    return false;
  }

private:
  uint8_t state;
  uint8_t payloadIndex;
  uint8_t crc;
  unsigned long lastByteAt;

  void reset() {
    state = 0;
    payloadIndex = 0;
    crc = 0;
    lastByteAt = 0;
  }
};

inline bool sendLinkPacket(Stream &stream, uint8_t type, uint8_t sequence,
                           const uint8_t *payload, uint8_t length) {
  if (type < LINK_SENSOR_FRAME || type > LINK_STATUS_FRAME ||
      length > LINK_MAX_PAYLOAD) return false;

  uint8_t crc = 0;
  crc = linkCrc8Update(crc, type);
  crc = linkCrc8Update(crc, sequence);
  crc = linkCrc8Update(crc, length);

  stream.write(LINK_SYNC_1);
  stream.write(LINK_SYNC_2);
  stream.write(type);
  stream.write(sequence);
  stream.write(length);
  for (uint8_t index = 0; index < length; index++) {
    stream.write(payload[index]);
    crc = linkCrc8Update(crc, payload[index]);
  }
  stream.write(crc);
  return true;
}

inline uint16_t linkReadU16(const uint8_t *data) {
  return static_cast<uint16_t>(data[0]) |
         (static_cast<uint16_t>(data[1]) << 8);
}

inline void linkWriteU16(uint8_t *data, uint16_t value) {
  data[0] = static_cast<uint8_t>(value & 0xFF);
  data[1] = static_cast<uint8_t>(value >> 8);
}
