#include "CrsfTelemetry.h"

#include "BatteryManager.h"
#include "Config.h"

namespace {
  constexpr uint8_t kCrsfAddressFlightController = 0xC8;
  constexpr uint8_t kCrsfFrameTypeBatterySensor = 0x08;

  // Battery sensor payload size:
  // voltage(2) + current(2) + capacity(3) + remaining(1) = 8 bytes
  constexpr uint8_t kBatteryPayloadSize = 8;

  // CRSF length field includes: type + payload + crc
  constexpr uint8_t kBatteryFrameLength = 1 + kBatteryPayloadSize + 1;

  // Full frame bytes:
  // address + length + type + payload + crc
  constexpr uint8_t kBatteryFrameSize = 2 + kBatteryFrameLength;
}

CrsfTelemetry::CrsfTelemetry(HardwareSerial& serialPort)
  : serialPort_(serialPort),
    lastBatteryTelemetryMs_(0) {
}

void CrsfTelemetry::update(uint32_t nowMs, const BatteryManager& batteryManager) {
  if (!batteryManager.hasValidReading()) {
    return;
  }

  if ((nowMs - lastBatteryTelemetryMs_) < Config::kCrsfBatteryTelemetryIntervalMs) {
    return;
  }

  lastBatteryTelemetryMs_ = nowMs;
  sendBatterySensorFrame(batteryManager.getPackVoltage());
}

void CrsfTelemetry::sendBatterySensorFrame(float packVoltageVolts) {
  // EdgeTX / ELRS RxBt is effectively displayed with one decimal place.
  // For example:
  //   8.08V -> 81 -> shown as about 8.1V
  const uint16_t voltageDeciVolts =
    static_cast<uint16_t>((packVoltageVolts * 10.0f) + 0.5f);

  // No current sensor yet.
  const uint16_t currentCentiAmps = 0;

  // No consumed capacity tracking yet.
  const uint32_t consumedMah = 0;

  // No reliable remaining-percent model yet.
  const uint8_t remainingPercent = 0;

  uint8_t frame[kBatteryFrameSize] = {0};

  frame[0] = kCrsfAddressFlightController;
  frame[1] = kBatteryFrameLength;
  frame[2] = kCrsfFrameTypeBatterySensor;

  // Voltage big-endian
  frame[3] = static_cast<uint8_t>((voltageDeciVolts >> 8) & 0xFF);
  frame[4] = static_cast<uint8_t>(voltageDeciVolts & 0xFF);

  // Current big-endian
  frame[5] = static_cast<uint8_t>((currentCentiAmps >> 8) & 0xFF);
  frame[6] = static_cast<uint8_t>(currentCentiAmps & 0xFF);

  // Capacity 24-bit big-endian
  frame[7] = static_cast<uint8_t>((consumedMah >> 16) & 0xFF);
  frame[8] = static_cast<uint8_t>((consumedMah >> 8) & 0xFF);
  frame[9] = static_cast<uint8_t>(consumedMah & 0xFF);

  // Remaining percentage
  frame[10] = remainingPercent;

  // CRC is computed over type + payload, not address/length.
  frame[11] = crc8DvbS2(&frame[2], 1 + kBatteryPayloadSize);

  serialPort_.write(frame, sizeof(frame));
}

uint8_t CrsfTelemetry::crc8DvbS2(const uint8_t* data, size_t length) {
  uint8_t crc = 0;

  for (size_t i = 0; i < length; ++i) {
    crc ^= data[i];

    for (uint8_t bit = 0; bit < 8; ++bit) {
      if (crc & 0x80) {
        crc = static_cast<uint8_t>((crc << 1) ^ 0xD5);
      } else {
        crc <<= 1;
      }
    }
  }

  return crc;
}