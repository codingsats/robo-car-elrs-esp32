#include "RcInput.h"
#include "Config.h"
#include "Pins.h"

RcInput::RcInput() {
  // RcData is already value-initialized.
}

void RcInput::begin() {
  // Start CRSF receive on Serial1.
  // RX pin = GPIO26
  // TX pin = GPIO27
  Serial1.begin(Config::kCrsfBaud, SERIAL_8N1, Pins::kRcRx, Pins::kRcTx);
}

bool RcInput::update() {
  return readCrsfFrame();
}

const RcData& RcInput::getData() const {
  return rcData;
}

bool RcInput::isLinkAlive() const {
  if (!rcData.hasValidFrame) {
    return false;
  }

  return (millis() - rcData.lastValidFrameMs) <= Config::kFailsafeTimeoutMs;
}

bool RcInput::readCrsfFrame() {
  // Parser state is static so that incomplete frames can continue
  // across multiple calls to update().
  static uint8_t frame[64];
  static int idx = 0;
  static int expectedLen = -1;

  bool decodedValidRcFrame = false;

  while (Serial1.available()) {
    const uint8_t b = Serial1.read();

    // Wait for the expected CRSF sync/address byte before starting a frame.
    if (idx == 0) {
      if (b != Config::kCrsfSyncByte) {
        continue;
      }
    }

    frame[idx++] = b;

    // Once we have the length byte, compute the total frame size.
    if (idx == 2) {
      // CRSF length excludes the address byte,
      // but includes: type + payload + crc
      expectedLen = frame[1] + 2;

      // Reject obviously invalid lengths.
      if (expectedLen < 5 || expectedLen > static_cast<int>(sizeof(frame))) {
        idx = 0;
        expectedLen = -1;
        continue;
      }
    }

    // If we have received a full frame, inspect it.
    if (expectedLen > 0 && idx == expectedLen) {
      const uint8_t type = frame[2];

      // We currently only care about packed RC channel frames.
      if (type == Config::kCrsfFrameTypeRcChannelsPacked) {
        // Payload starts at frame[3].
        decodeChannels(&frame[3]);

        rcData.lastValidFrameMs = millis();
        rcData.hasValidFrame = true;
        decodedValidRcFrame = true;
      }

      // Reset parser state for the next frame.
      idx = 0;
      expectedLen = -1;
    }

    // Safety reset if parsing gets out of sync.
    if (idx >= static_cast<int>(sizeof(frame))) {
      idx = 0;
      expectedLen = -1;
    }
  }

  return decodedValidRcFrame;
}

void RcInput::decodeChannels(const uint8_t* payload) {
  // Decode all 16 standard CRSF packed 11-bit channels.
  rcData.channels[0]  = ((payload[0]       | payload[1]  << 8) & 0x07FF);
  rcData.channels[1]  = ((payload[1]  >> 3 | payload[2]  << 5) & 0x07FF);
  rcData.channels[2]  = ((payload[2]  >> 6 | payload[3]  << 2 | payload[4]  << 10) & 0x07FF);
  rcData.channels[3]  = ((payload[4]  >> 1 | payload[5]  << 7) & 0x07FF);
  rcData.channels[4]  = ((payload[5]  >> 4 | payload[6]  << 4) & 0x07FF);
  rcData.channels[5]  = ((payload[6]  >> 7 | payload[7]  << 1 | payload[8]  << 9) & 0x07FF);
  rcData.channels[6]  = ((payload[8]  >> 2 | payload[9]  << 6) & 0x07FF);
  rcData.channels[7]  = ((payload[9]  >> 5 | payload[10] << 3) & 0x07FF);
  rcData.channels[8]  = ((payload[11]      | payload[12] << 8) & 0x07FF);
  rcData.channels[9]  = ((payload[12] >> 3 | payload[13] << 5) & 0x07FF);
  rcData.channels[10] = ((payload[13] >> 6 | payload[14] << 2 | payload[15] << 10) & 0x07FF);
  rcData.channels[11] = ((payload[15] >> 1 | payload[16] << 7) & 0x07FF);
  rcData.channels[12] = ((payload[16] >> 4 | payload[17] << 4) & 0x07FF);
  rcData.channels[13] = ((payload[17] >> 7 | payload[18] << 1 | payload[19] << 9) & 0x07FF);
  rcData.channels[14] = ((payload[19] >> 2 | payload[20] << 6) & 0x07FF);
  rcData.channels[15] = ((payload[20] >> 5 | payload[21] << 3) & 0x07FF);
}