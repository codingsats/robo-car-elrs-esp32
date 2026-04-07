#include <Arduino.h>
#include "CarMovement.h"
#include "RcInput.h"
#include "ControlMixer.h"
#include "Config.h"
#include "Pins.h"
#include "Types.h"
#include "Debug.h"
#include <ESP32Servo.h>

CarMovement car;
RcInput rcInput;

// Tracks the previous arm state so that we can detect transitions
// and play a buzzer indication only when the state changes.
bool gWasArmed = false;
bool gWereLightsOn = false;

// Servo object for the ultrasonic pan mechanism.
Servo gPanServo;

// Dedicated LEDC channel for buzzer to avoid conflicts with servo PWM channels.
constexpr uint8_t kBuzzerPwmChannel = 7;
constexpr uint8_t kBuzzerPwmResolutionBits = 8;

// Non-blocking buzzer stop time. 0 means no timed stop is pending.
uint32_t gBuzzerOffAtMs = 0;

// Tracks the current proximity assist state.
bool gWasProximityAssistOn = false;

// Current servo scan state.
int gCurrentScanAngle = Config::kServoCenterAngle;
int gScanDirection = 1; // +1 = moving right, -1 = moving left

// Timing for non-blocking servo scanning.
uint32_t gLastServoMoveMs = 0;
uint32_t gLastMeasurementMs = 0;

// Latest measured distance at the current scan angle.
int gLastDistanceCm = Config::kUltrasonicInvalidDistanceCm;

// Nearest distance seen during the current scan activity.
// This is what drives the parking-sensor buzzer behavior.
int gNearestDistanceCm = Config::kUltrasonicInvalidDistanceCm;

uint32_t gLastUltrasonicMeasureMs = 0;

// Timing for non-blocking buzzer beeps.
uint32_t gLastParkingBeepMs = 0;

// Tracks whether parking-alert logic currently owns buzzer output.
bool gParkingBuzzerActive = false;

// Tracks whether parking mode is currently in continuous-tone state.
bool gParkingContinuousTone = false;

// Plays a short buzzer sound to indicate arm/disarm state.
void startBuzzerTone(uint16_t frequencyHz, uint32_t durationMs = 0) {
  ledcWriteTone(kBuzzerPwmChannel, frequencyHz);

  if (durationMs > 0) {
    gBuzzerOffAtMs = millis() + durationMs;
  } else {
    gBuzzerOffAtMs = 0;
  }
}

void stopBuzzerTone() {
  ledcWriteTone(kBuzzerPwmChannel, 0);
  gBuzzerOffAtMs = 0;
}

void updateTimedBuzzer(uint32_t now) {
  if (gBuzzerOffAtMs > 0 && static_cast<int32_t>(now - gBuzzerOffAtMs) >= 0) {
    stopBuzzerTone();
  }
}

void playArmStateTone(bool armed) {
  if (armed) {
    // Short, higher tone when arming.
    startBuzzerTone(Config::kBuzzerArmToneHz, Config::kBuzzerArmDurationMs);
  } else {
    // Longer, lower tone when disarming.
    startBuzzerTone(Config::kBuzzerDisarmToneHz, Config::kBuzzerDisarmDurationMs);
  }
}

int measureDistanceCm();
void resetProximityScanner();
void updateProximityScanner(uint32_t now);
void updateParkingBuzzer(uint32_t now, bool proximityAssistOn);
void releaseParkingBuzzer();

void releaseParkingBuzzer() {
  if (gParkingBuzzerActive) {
    stopBuzzerTone();
  }

  gParkingBuzzerActive = false;
  gParkingContinuousTone = false;
}

// Measures one ultrasonic distance sample in centimeters.
// Returns -1 if the reading is invalid or timed out.
int measureDistanceCm() {
  // Trigger a short ultrasonic pulse.
  digitalWrite(Pins::kUltrasonicTrig, LOW);
  delayMicroseconds(2);

  digitalWrite(Pins::kUltrasonicTrig, HIGH);
  delayMicroseconds(10);
  digitalWrite(Pins::kUltrasonicTrig, LOW);

  // Read echo pulse width.
  const unsigned long durationUs = pulseIn(
    Pins::kUltrasonicEcho,
    HIGH,
    Config::kUltrasonicEchoTimeoutUs
  );

  // Timeout or invalid reading.
  if (durationUs == 0) {
    return Config::kUltrasonicInvalidDistanceCm;
  }

  // Convert time-of-flight to distance in cm.
  // Standard approximation for HC-SR04 class modules.
  const int distanceCm = static_cast<int>(durationUs / 58UL);

  // Reject obviously invalid very small or huge values.
  if (distanceCm <= 0 || distanceCm > 400) {
    return Config::kUltrasonicInvalidDistanceCm;
  }

  return distanceCm;
}

// Resets scanner state and centers the servo.
void resetProximityScanner() {
  gCurrentScanAngle = Config::kServoCenterAngle;
  gScanDirection = 1;
  gLastServoMoveMs = 0;
  gLastMeasurementMs = 0;
  gLastDistanceCm = Config::kUltrasonicInvalidDistanceCm;
  gNearestDistanceCm = Config::kUltrasonicInvalidDistanceCm;

  gPanServo.write(gCurrentScanAngle);
}

void updateProximityScanner(uint32_t now) {
  // Move servo smoothly using small angle steps.
  if (now - gLastServoMoveMs >= Config::kServoStepIntervalMs) {
    gLastServoMoveMs = now;

    gCurrentScanAngle += gScanDirection * Config::kScanAngleStep;

    // Reverse direction at scan limits.
    if (gCurrentScanAngle >= Config::kScanAngleMax) {
      gCurrentScanAngle = Config::kScanAngleMax;
      gScanDirection = -1;
    } else if (gCurrentScanAngle <= Config::kScanAngleMin) {
      gCurrentScanAngle = Config::kScanAngleMin;
      gScanDirection = 1;
    }

    gPanServo.write(gCurrentScanAngle);
  }

  // Take ultrasonic readings at a slower rate than servo motion.
  // This gives the sensor a better chance to return a stable value.
  if (now - gLastUltrasonicMeasureMs >= Config::kUltrasonicMeasureIntervalMs) {
    gLastUltrasonicMeasureMs = now;

    const int measuredCm = measureDistanceCm();
    gLastDistanceCm = measuredCm;

    // Use the latest valid reading directly for buzzer logic.
    if (measuredCm > 0) {
      gNearestDistanceCm = measuredCm;
    } else {
      gNearestDistanceCm = Config::kUltrasonicInvalidDistanceCm;
    }
  }
}

// Generates a parking-sensor-style buzzer pattern based on nearest obstacle distance.
void updateParkingBuzzer(uint32_t now, bool proximityAssistOn) {
  // If scan mode is not active, do not interfere with arm/disarm tones.
  // Parking tone output is released by the caller when mode turns off.
  if (!proximityAssistOn) {
    return;
  }

  // No valid obstacle reading yet or obstacle is out of warning range.
  if (gNearestDistanceCm < 0 || gNearestDistanceCm > Config::kParkingWarnDistanceCm) {
    releaseParkingBuzzer();
    return;
  }

  // Very near obstacle -> continuous warning.
  if (gNearestDistanceCm <= Config::kParkingContinuousDistanceCm) {
    if (!gParkingContinuousTone) {
      startBuzzerTone(Config::kParkingBuzzerFrequencyHz);
    }

    gParkingBuzzerActive = true;
    gParkingContinuousTone = true;
    return;
  }

  // Leave continuous mode when obstacle is no longer in nearest range.
  gParkingContinuousTone = false;

  uint32_t intervalMs = Config::kParkingSlowBeepIntervalMs;

  if (gNearestDistanceCm <= Config::kParkingVeryFastDistanceCm) {
    intervalMs = Config::kParkingFastBeepIntervalMs;
  } else if (gNearestDistanceCm <= Config::kParkingFastDistanceCm) {
    intervalMs = Config::kParkingMediumBeepIntervalMs;
  } else {
    intervalMs = Config::kParkingSlowBeepIntervalMs;
  }

  if (now - gLastParkingBeepMs >= intervalMs) {
    gLastParkingBeepMs = now;
    startBuzzerTone(Config::kParkingBuzzerFrequencyHz, Config::kParkingBeepDurationMs);
    gParkingBuzzerActive = true;
  }
}

void setup() {
  // Start USB serial for debugging.
  DBG_INIT(115200);

  // Configure buzzer pin.
  pinMode(Pins::kBuzzer, OUTPUT);
  ledcSetup(kBuzzerPwmChannel, 2000, kBuzzerPwmResolutionBits);
  ledcAttachPin(Pins::kBuzzer, kBuzzerPwmChannel);
  stopBuzzerTone();

  // Configure front LED pins and force them off at boot.
  pinMode(Pins::kLeftLed, OUTPUT);
  pinMode(Pins::kRightLed, OUTPUT);
  digitalWrite(Pins::kLeftLed, LOW);
  digitalWrite(Pins::kRightLed, LOW);

  // Initialize the car hardware and force a stopped state at boot.
  car.initialize();
  car.stop();

  // Start CRSF / ELRS reception.
  rcInput.begin();

  // Vehicle starts in a logically disarmed state until valid RC data says otherwise.
  gWasArmed = false;
  gWereLightsOn = false;
  gWasProximityAssistOn = false;

  // Configure ultrasonic sensor pins.
  pinMode(Pins::kUltrasonicTrig, OUTPUT);
  pinMode(Pins::kUltrasonicEcho, INPUT);
  digitalWrite(Pins::kUltrasonicTrig, LOW);

  // Attach servo used to pan the ultrasonic sensor.
  gPanServo.setPeriodHertz(50);
  const int servoChannel = gPanServo.attach(Pins::kServoPan, 500, 2400);
  if (servoChannel < 0) {
    DBG_PRINTLN("ERROR: servo attach failed");
  } else {
    DBG_PRINT("Servo attached on PWM channel ");
    DBG_PRINTLN(servoChannel);
  }
  resetProximityScanner();

  DBG_PRINTLN("ELRS car control started");
  DBG_PRINTLN("Initial state: DISARMED");
}

void loop() {
  // Capture current time once per loop for consistent timing checks.
  const uint32_t now = millis();

  // Always poll the RC input parser.
  // This keeps CRSF frame handling responsive.
  const bool gotNewRcFrame = rcInput.update();

  // If we received a fresh RC frame, we can evaluate arm state
  // and optionally send updated motor commands.
  if (gotNewRcFrame) {
    const RcData& rcData = rcInput.getData();

    // Build the current operator command from RC channel data.
    const DriveCommand driveCommand = ControlMixer::buildDriveCommand(rcData);

    // Detect arm/disarm transitions and play buzzer indication only once.
    if (driveCommand.armed != gWasArmed) {
      gWasArmed = driveCommand.armed;

      if (gWasArmed) {
        DBG_PRINTLN("ARMED");
      } else {
        DBG_PRINTLN("DISARMED");
      }

      playArmStateTone(gWasArmed);
    }

    // Apply front LED state from SD switch on CH8.
    if (driveCommand.lightsOn != gWereLightsOn) {
      gWereLightsOn = driveCommand.lightsOn;

      digitalWrite(Pins::kLeftLed, gWereLightsOn ? HIGH : LOW);
      digitalWrite(Pins::kRightLed, gWereLightsOn ? HIGH : LOW);
    }

    // Detect proximity assist mode transitions from SB switch.
    if (driveCommand.proximityAssistOn != gWasProximityAssistOn) {
      gWasProximityAssistOn = driveCommand.proximityAssistOn;

      if (gWasProximityAssistOn) {
        DBG_PRINTLN("PROXIMITY SCAN ON");
        resetProximityScanner();
      } else {
        DBG_PRINTLN("PROXIMITY SCAN OFF");
        resetProximityScanner();
        releaseParkingBuzzer();
      }
    }

    // Only recompute and send motor commands at a limited rate.
    static uint32_t lastMotorUpdateMs = 0;
    if (now - lastMotorUpdateMs >= Config::kMotorUpdateIntervalMs) {
      // Mix throttle + steering into left/right motor outputs.
      const MotorCommand motorCommand = ControlMixer::mixDifferential(driveCommand);

      // Send motor outputs to the chassis.
      // If disarmed, driveCommand already forces zero output.
      car.moveCustom(motorCommand.left, motorCommand.right);

      lastMotorUpdateMs = now;

      // Print debug output at a reduced rate for easier tuning.
      static uint32_t lastDebugPrintMs = 0;
      if (now - lastDebugPrintMs >= Config::kDebugPrintIntervalMs) {
        lastDebugPrintMs = now;

        DBG_PRINT("CH1=");
        DBG_PRINT(rcData.channels[Config::kSteeringChannelIndex]);

        DBG_PRINT("  CH2=");
        DBG_PRINT(rcData.channels[Config::kThrottleChannelIndex]);

        DBG_PRINT("  CH5=");
        DBG_PRINT(rcData.channels[Config::kArmChannelIndex]);

        DBG_PRINT("  CH8=");
        DBG_PRINT(rcData.channels[Config::kLightsChannelIndex]);

        DBG_PRINT("  CH6=");
        DBG_PRINT(rcData.channels[Config::kProximityAssistChannelIndex]);

        DBG_PRINT("  armed=");
        DBG_PRINT(driveCommand.armed ? "YES" : "NO");

        DBG_PRINT("  lights=");
        DBG_PRINT(driveCommand.lightsOn ? "ON" : "OFF");

        DBG_PRINT("  prox=");
        DBG_PRINT(driveCommand.proximityAssistOn ? "ON" : "OFF");

        DBG_PRINT("  scanAngle=");
        DBG_PRINT(gCurrentScanAngle);

        DBG_PRINT("  distCm=");
        DBG_PRINT(gLastDistanceCm);

        DBG_PRINT("  nearestCm=");
        DBG_PRINT(gNearestDistanceCm);

        DBG_PRINT("  steering=");
        DBG_PRINT(driveCommand.steering);

        DBG_PRINT("  throttle=");
        DBG_PRINT(driveCommand.throttle);

        DBG_PRINT("  left=");
        DBG_PRINT(motorCommand.left);

        DBG_PRINT("  right=");
        DBG_PRINTLN(motorCommand.right);
      }
    }
  }

  const bool linkAlive = rcInput.isLinkAlive();

  // Keep servo scan and parking buzzer running continuously while assist mode is active.
  // This avoids scan pauses when no fresh RC frame was parsed in a given loop cycle.
  if (gWasProximityAssistOn && linkAlive) {
    updateProximityScanner(now);
  } else {
    gNearestDistanceCm = Config::kUltrasonicInvalidDistanceCm;
    gLastDistanceCm = Config::kUltrasonicInvalidDistanceCm;
    releaseParkingBuzzer();
  }

  updateParkingBuzzer(now, gWasProximityAssistOn && linkAlive);

  // Failsafe:
  // if no valid frame has arrived recently, stop the car.
  updateTimedBuzzer(now);

  if (!linkAlive) {
    car.stop();

    if (gWasArmed) {
      gWasArmed = false;
      DBG_PRINTLN("FAILSAFE -> DISARMED");
      playArmStateTone(false);
    }

    if (gWereLightsOn) {
      gWereLightsOn = false;
      digitalWrite(Pins::kLeftLed, LOW);
      digitalWrite(Pins::kRightLed, LOW);
      DBG_PRINTLN("FAILSAFE -> LIGHTS OFF");
    }

    if (gWasProximityAssistOn) {
      gWasProximityAssistOn = false;
      resetProximityScanner();
      releaseParkingBuzzer();
      DBG_PRINTLN("FAILSAFE -> PROXIMITY SCAN OFF");
    }

    static uint32_t lastFailsafePrintMs = 0;
    if (now - lastFailsafePrintMs >= 1000) {
      lastFailsafePrintMs = now;
      DBG_PRINTLN("Failsafe: no recent CRSF frames");
    }
  }
}