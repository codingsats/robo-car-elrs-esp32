# ELRS Smart Car (ESP32 + Acebott Platform)

## Overview

This project turns the Acebott Smart Car V2 into a modular, remotely controlled robotic platform using:

* ESP32 (WROOM)
* ExpressLRS (CRSF protocol)
* Radiomaster Boxer transmitter

The system supports:

* real-time RC control
* safety (arm/disarm + failsafe)
* LED control
* ultrasonic scanning with servo
* parking-sensor-style obstacle feedback

The codebase is structured for future expansion (autonomous modes, additional sensors, etc.).

---

## Features

### RC Control (ELRS / CRSF)

* Steering + throttle via sticks
* Differential drive mixing
* Robust frame parsing (CRSF)

### Safety

* Arm / disarm via SA switch
* Failsafe when signal is lost
* Motors forced to stop when disarmed

### Lighting

* Front LEDs controlled via SD switch (CH8)

### Ultrasonic Scanning (SB switch)

* Servo-mounted ultrasonic sensor
* Left ↔ Center ↔ Right scanning
* Non-blocking scanning logic

### Parking Sensor (Buzzer)

* Distance-based beeping
* No beep > threshold
* Slow → fast → continuous tone when approaching obstacle

---

## Hardware

### Core Components

* ESP32 WROOM (Acebott board)
* Acebott Smart Car V2 chassis (Mecanum wheels)
* ELRS receiver (CRSF output)
* Radiomaster Boxer transmitter

### Sensors / Actuators

* Ultrasonic sensor (HC-SR04 style)
* SG90 servo (pan mount)
* Passive buzzer
* 2x front LEDs
* Line tracking sensors (not yet used)

---

## Pin Configuration

Defined in `Pins.h`:

```cpp
RC Receiver RX     -> GPIO 26

Left LED           -> GPIO 12
Right LED          -> GPIO 2

Buzzer             -> GPIO 33

Ultrasonic TRIG    -> GPIO 13
Ultrasonic ECHO    -> GPIO 14

Servo (pan)        -> GPIO 25

IR Receiver        -> GPIO 4

Line Sensors:
  Left             -> GPIO 35
  Center           -> GPIO 36
  Right            -> GPIO 39
```

---

## RC Channel Mapping

| Function       | Switch / Control | Channel |
| -------------- | ---------------- | ------- |
| Steering       | Right stick X    | CH1     |
| Throttle       | Right stick Y    | CH2     |
| Arm / Disarm   | SA               | CH5     |
| Proximity Mode | SB (LOW pos)     | CH6     |
| Lights         | SD               | CH8     |

---

## Project Structure

```text
elrs_smart_car/
│
├── elrs_smart_car.ino   # main loop (orchestration)
│
├── Config.h             # all tunable parameters
├── Pins.h               # hardware mapping
├── Types.h              # shared data structures
│
├── RcInput.*            # CRSF parsing + RC data
├── ControlMixer.*       # input → drive commands
├── CarMovement.*        # motor abstraction
│
└── README.md
```

---

## Software Architecture

The system is divided into clear layers:

### 1. Input Layer

* Reads CRSF frames from ELRS receiver
* Extracts channel values

### 2. Command Layer

* Converts raw channels into:

  * steering
  * throttle
  * switches (arm, lights, proximity)

### 3. Control Layer

* Applies:

  * deadbands
  * safety rules (disarm = no movement)

### 4. Output Layer

* Motor control
* LED control
* Servo scanning
* Buzzer signaling

---

## Ultrasonic Scan Logic

When SB (CH6) is in LOW position:

* Servo sweeps between defined angles
* At each step:

  1. move servo
  2. wait for stabilization
  3. take measurement
* Distance is evaluated in real-time

---

## Parking Sensor Logic

Distance → buzzer behavior:

| Distance    | Behavior        |
| ----------- | --------------- |
| > threshold | silent          |
| ~30 cm      | slow beep       |
| ~20 cm      | faster beep     |
| ~10 cm      | continuous tone |

Thresholds configurable in `Config.h`.

---

## Configuration

All tuning parameters are centralized in `Config.h`.

### Important settings

```cpp
// Servo scan
kScanAngleMin
kScanAngleMax
kScanAngleStep

// Ultrasonic
kParkingWarnDistanceCm

// Control
kSteeringDeadband
kThrottleDeadband
```

---

## Build & Upload

### Requirements

* Arduino IDE
* ESP32 board package installed

### Steps

1. Open project folder
2. Select board:
   `ESP32 Dev Module`
3. Select correct COM port
4. Upload

---

## Usage

### Basic Control

1. Power on car
2. Turn on transmitter
3. Arm using SA switch
4. Drive using right stick

### Lights

* Toggle SD switch

### Ultrasonic Mode

* Set SB switch to LOW
* Servo starts scanning
* Buzzer indicates obstacles

---

## Known Issues

* Ultrasonic sensor can produce noisy readings

---

## Next Steps (Planned)

* Mecanum full vector drive (strafe support)
* Autonomous obstacle avoidance
* Line tracking mode
* Battery monitoring
* Telemetry feedback to transmitter
* Non-blocking task scheduler

---

## Notes

* Keep servo mechanically centered at 90°
* Ensure stable power supply (servo spikes can cause resets)
* Keep receiver wiring clean and away from noisy lines

---

## License

MIT
