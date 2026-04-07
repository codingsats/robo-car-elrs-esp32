#ifndef CONTROL_MIXER_H
#define CONTROL_MIXER_H

#include <Arduino.h>
#include "Types.h"

namespace ControlMixer {
  // Converts one centered CRSF channel value into a signed motor-style range.
  int normalizeCenteredChannel(int value);

  // Reads RC channels and creates a driver intent.
  DriveCommand buildDriveCommand(const RcData& rcData);

  // Mixes steering + throttle into left/right differential drive outputs.
  MotorCommand mixDifferential(const DriveCommand& driveCommand);
}

#endif