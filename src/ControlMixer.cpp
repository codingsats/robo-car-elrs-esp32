#include "ControlMixer.h"
#include "Config.h"

namespace ControlMixer {

int normalizeCenteredChannel(int value) {
  // Convert a CRSF-style centered channel:
  // 172 .. 1811
  // into:
  // -255 .. 255
  const long mapped = map(
    value,
    Config::kCrsfMin,
    Config::kCrsfMax,
    Config::kMotorMin,
    Config::kMotorMax
  );

  return constrain(static_cast<int>(mapped), Config::kMotorMin, Config::kMotorMax);
}

DriveCommand buildDriveCommand(const RcData& rcData) {
  DriveCommand cmd;

  // Arm/disarm is controlled by SA switch on CH5.
  cmd.armed = rcData.channels[Config::kArmChannelIndex] >= Config::kArmThreshold;

  // Lights on/off is controlled by SD switch on CH8.
  cmd.lightsOn = rcData.channels[Config::kLightsChannelIndex] >= Config::kLightsOnThreshold;

  // Proximity scan mode is enabled only when SB is in the LOW position.
  cmd.proximityAssistOn =
    rcData.channels[Config::kProximityAssistChannelIndex] <= Config::kProximityAssistLowThreshold;

  // Read steering and throttle according to configuration.
  cmd.steering = normalizeCenteredChannel(
    rcData.channels[Config::kSteeringChannelIndex]
  );

  cmd.throttle = normalizeCenteredChannel(
    rcData.channels[Config::kThrottleChannelIndex]
  );

  if (abs(cmd.steering) < Config::kSteeringDeadband) {
    cmd.steering = 0;
  }

  if (abs(cmd.throttle) < Config::kThrottleDeadband) {
    cmd.throttle = 0;
  }

  if (!cmd.armed) {
    cmd.steering = 0;
    cmd.throttle = 0;
  }

  return cmd;
}

MotorCommand mixDifferential(const DriveCommand& driveCommand) {
  MotorCommand motors;

  // Differential mixing:
  // throttle drives both sides equally
  // steering adds/subtracts between left and right
  motors.left = driveCommand.throttle + driveCommand.steering;
  motors.right = driveCommand.throttle - driveCommand.steering;

  // Clamp to supported motor range.
  motors.left = constrain(motors.left, Config::kMotorMin, Config::kMotorMax);
  motors.right = constrain(motors.right, Config::kMotorMin, Config::kMotorMax);

  return motors;
}

}  // namespace ControlMixer