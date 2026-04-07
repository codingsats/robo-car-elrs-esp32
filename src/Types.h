#ifndef TYPES_H
#define TYPES_H

#include <Arduino.h>

// Stores the latest decoded CRSF channel values and link timing state.
struct RcData {
  uint16_t channels[16] = {0};
  uint32_t lastValidFrameMs = 0;
  bool hasValidFrame = false;
};

// Represents the driver's requested vehicle motion before motor mixing.
struct DriveCommand {
  int steering = 0;   // -255 .. 255
  int throttle = 0;   // -255 .. 255
  bool armed = false; // true when RC arm switch is ON
  bool lightsOn = false; // true when the light switch is ON
  bool proximityAssistOn = false; // true when SB low enables ultrasonic scan mode
};

// Final mixed motor commands for the left and right side.
struct MotorCommand {
  int left = 0;   // -255 .. 255
  int right = 0;  // -255 .. 255
};

#endif