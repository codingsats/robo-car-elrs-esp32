#ifndef CONFIG_H
#define CONFIG_H

#include <Arduino.h>

namespace Config {
  // ---------------------------------------------------------------------------
  // Serial / RC communication
  // ---------------------------------------------------------------------------

  // CRSF / ELRS serial baud rate.
  constexpr uint32_t kCrsfBaud = 420000;

  // CRSF frame type for packed RC channels.
  constexpr uint8_t kCrsfFrameTypeRcChannelsPacked = 0x16;

  // CRSF sync / device address currently expected from your receiver stream.
  constexpr uint8_t kCrsfSyncByte = 0xC8;

  // ---------------------------------------------------------------------------
  // Control loop timing
  // ---------------------------------------------------------------------------

  // How long we allow no valid RC frame before entering failsafe.
  constexpr uint32_t kFailsafeTimeoutMs = 300;

  // How often motor commands are sent to the chassis.
  constexpr uint32_t kMotorUpdateIntervalMs = 30;

  // How often debug output is printed to USB serial.
  constexpr uint32_t kDebugPrintIntervalMs = 250;

  // ---------------------------------------------------------------------------
  // CRSF channel ranges
  // ---------------------------------------------------------------------------

  // Typical ELRS / CRSF packed channel values.
  constexpr int kCrsfMin = 172;
  constexpr int kCrsfCenter = 992;
  constexpr int kCrsfMax = 1811;

  // ---------------------------------------------------------------------------
  // Drive output ranges
  // ---------------------------------------------------------------------------

  // Final motor command range accepted by the motor wrapper.
  constexpr int kMotorMin = -255;
  constexpr int kMotorMax = 255;

  // Deadband around stick center to prevent drift.
  constexpr int kSteeringDeadband = 20;
  constexpr int kThrottleDeadband = 20;

  // Channel mapping in 0-based array form.
  // CH1 -> 0
  // CH2 -> 1
  // CH5 -> 4
  // CH8 -> 7
  constexpr uint8_t kSteeringChannelIndex = 0;
  constexpr uint8_t kThrottleChannelIndex = 1;
  constexpr uint8_t kArmChannelIndex = 4;
  constexpr uint8_t kLightsChannelIndex = 7;

  // Threshold above which the arm switch is considered ON.
  // This works well for a typical 2-position or 3-position switch.
  constexpr int kArmThreshold = 1400;
  constexpr int kLightsOnThreshold = 1400;

  // ---------------------------------------------------------------------------
  // Future sensor / actuator defaults
  // ---------------------------------------------------------------------------

  // Servo defaults
  constexpr int kServoMinAngle = 0;
  constexpr int kServoCenterAngle = 90;
  constexpr int kServoMaxAngle = 180;

  // Ultrasonic defaults
  constexpr uint32_t kUltrasonicPollIntervalMs = 100;
  constexpr int kObstacleStopDistanceCm = 25;
  constexpr uint32_t kUltrasonicMeasureIntervalMs = 120;

  // Tracking sensor defaults
  constexpr int kTrackingBlackThreshold = 1500;
  constexpr int kTrackingWhiteThreshold = 2500;

  // LED defaults
  constexpr uint8_t kLedMinBrightness = 0;
  constexpr uint8_t kLedMaxBrightness = 255;

  // Buzzer defaults
  constexpr int kBuzzerArmToneHz = 2200;
  constexpr int kBuzzerDisarmToneHz = 1200;
  constexpr int kBuzzerArmDurationMs = 120;
  constexpr int kBuzzerDisarmDurationMs = 250;

    // CH6 -> 5
  constexpr uint8_t kProximityAssistChannelIndex = 5;

  // For a 3-position switch, the low position is near the CRSF minimum.
  // We enable scan mode only when SB is in the LOW position.
  constexpr int kProximityAssistLowThreshold = 600;

  // Servo scan range and behavior.
  constexpr int kScanAngleMin = 30;
  constexpr int kScanAngleMax = 150;
  constexpr int kScanAngleStep = 2;
  constexpr uint32_t kServoStepIntervalMs = 35;
  constexpr uint32_t kServoSettleTimeMs = 80;

  // Ultrasonic measurement behavior.
  constexpr uint32_t kUltrasonicEchoTimeoutUs = 25000; // about 4m max practical
  constexpr int kUltrasonicInvalidDistanceCm = -1;

  // Parking-sensor-style buzzer thresholds.
  constexpr int kParkingWarnDistanceCm = 35;
  constexpr int kParkingFastDistanceCm = 25;
  constexpr int kParkingVeryFastDistanceCm = 15;
  constexpr int kParkingContinuousDistanceCm = 8;

  // Buzzer sound for proximity alert.
  constexpr int kParkingBuzzerFrequencyHz = 2200;

  // Beep intervals for different obstacle distances.
  constexpr uint32_t kParkingSlowBeepIntervalMs = 700;
  constexpr uint32_t kParkingMediumBeepIntervalMs = 350;
  constexpr uint32_t kParkingFastBeepIntervalMs = 150;
  constexpr uint32_t kParkingBeepDurationMs = 60;
}

#endif