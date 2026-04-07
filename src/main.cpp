#include <Arduino.h>
#include "CarMovement.h"
#include "RcInput.h"
#include "ControlMixer.h"
#include "Config.h"
#include "Pins.h"
#include "Types.h"
#include <ESP32Servo.h>

CarMovement car;
RcInput rcInput;

// Tracks the previous arm state so that we can detect transitions
// and play a buzzer indication only when the state changes.
bool gWasArmed = false;
bool gWereLightsOn = false;

// Servo object for the ultrasonic pan mechanism.
Servo gPanServo;

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

// Plays a short buzzer sound to indicate arm/disarm state.
void playArmStateTone(bool armed) {
  if (armed) {
    // Short, higher tone when arming.
    tone(Pins::kBuzzer, Config::kBuzzerArmToneHz, Config::kBuzzerArmDurationMs);
  } else {
    // Longer, lower tone when disarming.
    tone(Pins::kBuzzer, Config::kBuzzerDisarmToneHz, Config::kBuzzerDisarmDurationMs);
  }
}

int measureDistanceCm();
void resetProximityScanner();
void updateProximityScanner(uint32_t now);
void updateParkingBuzzer(uint32_t now, bool proximityAssistOn);

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
  // If scan mode is not active, keep buzzer silent.
  if (!proximityAssistOn) {
    noTone(Pins::kBuzzer);
    return;
  }

  // No valid obstacle reading yet or obstacle is out of warning range.
  if (gNearestDistanceCm < 0 || gNearestDistanceCm > Config::kParkingWarnDistanceCm) {
    noTone(Pins::kBuzzer);
    return;
  }

  // Very near obstacle -> continuous warning.
  if (gNearestDistanceCm <= Config::kParkingContinuousDistanceCm) {
    tone(Pins::kBuzzer, Config::kParkingBuzzerFrequencyHz);
    return;
  }

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
    tone(
      Pins::kBuzzer,
      Config::kParkingBuzzerFrequencyHz,
      Config::kParkingBeepDurationMs
    );
  }
}

void setup() {
  // Start USB serial for debugging.
  Serial.begin(115200);

  // Configure buzzer pin.
  pinMode(Pins::kBuzzer, OUTPUT);
  noTone(Pins::kBuzzer);

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
  gPanServo.attach(Pins::kServoPan);
  resetProximityScanner();

  Serial.println("ELRS car control started");
  Serial.println("Initial state: DISARMED");
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
        Serial.println("ARMED");
      } else {
        Serial.println("DISARMED");
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
        Serial.println("PROXIMITY SCAN ON");
        resetProximityScanner();
      } else {
        Serial.println("PROXIMITY SCAN OFF");
        resetProximityScanner();
        noTone(Pins::kBuzzer);
      }
    }

        // Update servo scan and parking-sensor buzzer behavior when enabled.
    if (driveCommand.proximityAssistOn) {
      updateProximityScanner(now);
    } else {
      // Keep servo centered when scan mode is disabled.
      gNearestDistanceCm = Config::kUltrasonicInvalidDistanceCm;
      gLastDistanceCm = Config::kUltrasonicInvalidDistanceCm;
    }

    updateParkingBuzzer(now, driveCommand.proximityAssistOn);

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

        Serial.print("CH1=");
        Serial.print(rcData.channels[Config::kSteeringChannelIndex]);

        Serial.print("  CH2=");
        Serial.print(rcData.channels[Config::kThrottleChannelIndex]);

        Serial.print("  CH5=");
        Serial.print(rcData.channels[Config::kArmChannelIndex]);

        Serial.print("  CH8=");
        Serial.print(rcData.channels[Config::kLightsChannelIndex]);

        Serial.print("  CH6=");
        Serial.print(rcData.channels[Config::kProximityAssistChannelIndex]);

        Serial.print("  armed=");
        Serial.print(driveCommand.armed ? "YES" : "NO");

        Serial.print("  lights=");
        Serial.print(driveCommand.lightsOn ? "ON" : "OFF");

        Serial.print("  prox=");
        Serial.print(driveCommand.proximityAssistOn ? "ON" : "OFF");

        Serial.print("  scanAngle=");
        Serial.print(gCurrentScanAngle);

        Serial.print("  distCm=");
        Serial.print(gLastDistanceCm);

        Serial.print("  nearestCm=");
        Serial.print(gNearestDistanceCm);

        Serial.print("  steering=");
        Serial.print(driveCommand.steering);

        Serial.print("  throttle=");
        Serial.print(driveCommand.throttle);

        Serial.print("  left=");
        Serial.print(motorCommand.left);

        Serial.print("  right=");
        Serial.println(motorCommand.right);
      }
    }
  }

  // Failsafe:
  // if no valid frame has arrived recently, stop the car.
  if (!rcInput.isLinkAlive()) {
    car.stop();

    if (gWasArmed) {
      gWasArmed = false;
      Serial.println("FAILSAFE -> DISARMED");
      playArmStateTone(false);
    }

    if (gWereLightsOn) {
      gWereLightsOn = false;
      digitalWrite(Pins::kLeftLed, LOW);
      digitalWrite(Pins::kRightLed, LOW);
      Serial.println("FAILSAFE -> LIGHTS OFF");
    }

    if (gWasProximityAssistOn) {
      gWasProximityAssistOn = false;
      resetProximityScanner();
      noTone(Pins::kBuzzer);
      Serial.println("FAILSAFE -> PROXIMITY SCAN OFF");
    }

    static uint32_t lastFailsafePrintMs = 0;
    if (now - lastFailsafePrintMs >= 1000) {
      lastFailsafePrintMs = now;
      Serial.println("Failsafe: no recent CRSF frames");
    }
  }
}