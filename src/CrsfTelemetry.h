#ifndef CRSF_TELEMETRY_H
#define CRSF_TELEMETRY_H

#include <Arduino.h>

class BatteryManager;

class CrsfTelemetry {
public:
  explicit CrsfTelemetry(HardwareSerial& serialPort);

  // Periodic non-blocking telemetry update.
  void update(uint32_t nowMs, const BatteryManager& batteryManager);

private:
  // Standard CRSF battery sensor frame.
  void sendBatterySensorFrame(float packVoltageVolts);

  // CRC-8 DVB-S2 used by CRSF for frame type + payload.
  static uint8_t crc8DvbS2(const uint8_t* data, size_t length);

  HardwareSerial& serialPort_;
  uint32_t lastBatteryTelemetryMs_;
};

#endif