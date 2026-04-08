#include "BatteryManager.h"

#include "Config.h"
#include "Pins.h"

BatteryManager::BatteryManager()
  : lastSampleMs_(0),
    filteredPackVoltage_(0.0f),
    lastRawAverage_(0),
    hasValidReading_(false),
    lowState_(false),
    criticalState_(false),
    dividerMultiplier_(0.0f) {
}

void BatteryManager::begin() {
  pinMode(Pins::kBatteryAdc, INPUT);

  analogReadResolution(Config::kBatteryAdcResolutionBits);

  // GPIO27 is ADC2 on classic ESP32. 11 dB gives a wider usable range.
  analogSetPinAttenuation(Pins::kBatteryAdc, ADC_11db);

  // We keep the raw ADC value for debug visibility, but for voltage conversion
  // we now rely on analogReadMilliVolts(), which is better calibrated on ESP32
  // than the simplistic raw * 3.3 / 4095 approach.
  dividerMultiplier_ =
    (Config::kBatteryDividerR1Ohms + Config::kBatteryDividerR2Ohms) /
    Config::kBatteryDividerR2Ohms;
}

void BatteryManager::update(uint32_t nowMs) {
  if ((nowMs - lastSampleMs_) < Config::kBatterySampleIntervalMs) {
    return;
  }

  lastSampleMs_ = nowMs;

  uint16_t rawAverage = 0;
  const float packVoltage = readPackVoltageOnce(rawAverage);
  lastRawAverage_ = rawAverage;

  if (!hasValidReading_) {
    filteredPackVoltage_ = packVoltage;
    hasValidReading_ = true;
  } else {
    filteredPackVoltage_ +=
      Config::kBatteryFilterAlpha * (packVoltage - filteredPackVoltage_);
  }

  updateBatteryStates();
}

float BatteryManager::readPackVoltageOnce(uint16_t& rawAverageOut) const {
  uint32_t rawSum = 0;
  uint32_t milliVoltSum = 0;

  for (uint8_t i = 0; i < Config::kBatterySamplesPerUpdate; ++i) {
    rawSum += static_cast<uint32_t>(analogRead(Pins::kBatteryAdc));
    milliVoltSum += static_cast<uint32_t>(analogReadMilliVolts(Pins::kBatteryAdc));
  }

  const uint16_t rawAverage =
    static_cast<uint16_t>(rawSum / Config::kBatterySamplesPerUpdate);
  rawAverageOut = rawAverage;

  const float pinVoltage =
    (static_cast<float>(milliVoltSum) / static_cast<float>(Config::kBatterySamplesPerUpdate)) /
    1000.0f;

  float packVoltage = pinVoltage * dividerMultiplier_;
  packVoltage *= Config::kBatteryCalibrationFactor;

  return packVoltage;
}

void BatteryManager::updateBatteryStates() {
  const float hysteresis = Config::kBatteryStateHysteresis;

  if (!criticalState_ && filteredPackVoltage_ <= Config::kBatteryCriticalVoltage) {
    criticalState_ = true;
  } else if (
    criticalState_ &&
    filteredPackVoltage_ > (Config::kBatteryCriticalVoltage + hysteresis)
  ) {
    criticalState_ = false;
  }

  if (!lowState_ && filteredPackVoltage_ <= Config::kBatteryLowVoltage) {
    lowState_ = true;
  } else if (
    lowState_ &&
    filteredPackVoltage_ > (Config::kBatteryLowVoltage + hysteresis)
  ) {
    lowState_ = false;
  }
}

float BatteryManager::getPackVoltage() const {
  return filteredPackVoltage_;
}

float BatteryManager::getCellVoltage() const {
  return filteredPackVoltage_ * 0.5f;
}

uint16_t BatteryManager::getRawAdcAverage() const {
  return lastRawAverage_;
}

bool BatteryManager::hasValidReading() const {
  return hasValidReading_;
}

bool BatteryManager::isLow() const {
  return lowState_;
}

bool BatteryManager::isCritical() const {
  return criticalState_;
}