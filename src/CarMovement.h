#ifndef CAR_MOVEMENT_H
#define CAR_MOVEMENT_H

#include <Arduino.h>
#include <ACB_SmartCar_V2.h>

// High-level movement commands for simple scripted movement.
// We keep this enum because it can still be useful later for autonomous modes.
enum CarDirection {
  STOP,
  FORWARD,
  BACKWARD,
  TURN_LEFT,
  TURN_RIGHT,
  LATERAL_LEFT,
  LATERAL_RIGHT
};

class CarMovement {
public:
  CarMovement();

  // Initializes the underlying smart car driver.
  // Must be called once during setup().
  void initialize();

  // Simple predefined movement helper.
  void move(CarDirection direction, uint8_t speed);

  // Direct differential motor control.
  // left/right range: -255 .. 255
  void moveCustom(int left, int right);

  // Stop all motors immediately.
  void stop();

private:
  ACB_SmartCar_V2 car;

  // Low-level helper that writes raw values to all four motors.
  void setMotors(int m1, int m2, int m3, int m4);
};

#endif