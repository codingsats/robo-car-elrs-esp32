#include "CarMovement.h"
#include "Config.h"

CarMovement::CarMovement() {
  // No custom constructor logic needed.
}

void CarMovement::initialize() {
  // Initialize the vendor smart car driver.
  car.Init();
}

void CarMovement::setMotors(int m1, int m2, int m3, int m4) {
  // Write raw speed values to each motor.
  car.motorControl(1, m1);
  car.motorControl(2, m2);
  car.motorControl(3, m3);
  car.motorControl(4, m4);
}

void CarMovement::move(CarDirection direction, uint8_t speed) {
  // Note:
  // Your chassis currently behaves inverted relative to the vendor examples.
  // We keep your existing orientation so behavior does not change.

  switch (direction) {
    case FORWARD:
      setMotors(-speed, -speed, -speed, -speed);
      break;

    case BACKWARD:
      setMotors(speed, speed, speed, speed);
      break;

    case TURN_LEFT:
      setMotors(speed, speed, -speed, -speed);
      break;

    case TURN_RIGHT:
      setMotors(-speed, -speed, speed, speed);
      break;

    case LATERAL_LEFT:
      setMotors(speed, -speed, -speed, speed);
      break;

    case LATERAL_RIGHT:
      setMotors(-speed, speed, speed, -speed);
      break;

    case STOP:
    default:
      setMotors(0, 0, 0, 0);
      break;
  }
}

void CarMovement::moveCustom(int left, int right) {
  // Clamp to the supported range before sending values to the driver.
  left = constrain(left, Config::kMotorMin, Config::kMotorMax);
  right = constrain(right, Config::kMotorMin, Config::kMotorMax);

  // Left side = motors 1 and 2
  // Right side = motors 3 and 4
  //
  // Forward direction is inverted on this hardware layout,
  // so both sides are negated to preserve the currently working behavior.
  setMotors(-left, -left, -right, -right);
}

void CarMovement::stop() {
  setMotors(0, 0, 0, 0);
}