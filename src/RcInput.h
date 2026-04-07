#ifndef RC_INPUT_H
#define RC_INPUT_H

#include <Arduino.h>
#include "Types.h"

class RcInput {
public:
  RcInput();

  // Initializes the hardware serial port used for CRSF reception.
  void begin();

  // Poll the serial port and parse any available CRSF frames.
  // Returns true if at least one valid RC channel frame was decoded.
  bool update();

  // Returns the latest RC data.
  const RcData& getData() const;

  // Returns true if a recent valid frame has been received.
  bool isLinkAlive() const;

private:
  RcData rcData;

  // Reads and parses one or more CRSF bytes from Serial2.
  bool readCrsfFrame();

  // Decodes the first four CRSF channels from packed 11-bit payload.
  void decodeChannels(const uint8_t* payload);
};

#endif