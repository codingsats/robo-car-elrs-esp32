#ifndef BATTERY_MANAGER_H
#define BATTERY_MANAGER_H

#include <Arduino.h>

class BatteryManager {
public:
  BatteryManager();

  // Initializes ADC configuration and precomputes static scale values.
  void begin();

  // Non-blocking periodic update. Call every loop with millis().
  void update(uint32_t nowMs);

  // Latest filtered pack voltage in volts.
  float getPackVoltage() const;

  // Latest filtered single-cell voltage estimate in volts.
  float getCellVoltage() const;

  // Latest averaged raw ADC reading.
  uint16_t getRawAdcAverage() const;

  // Returns true after the first valid measurement has been captured.
  bool hasValidReading() const;

  bool isLow() const;
  bool isCritical() const;

private:
  float readPackVoltageOnce(uint16_t& rawAverageOut) const;
  void updateBatteryStates();

  uint32_t lastSampleMs_;
  float filteredPackVoltage_;
  uint16_t lastRawAverage_;
  bool hasValidReading_;
  bool lowState_;
  bool criticalState_;

  // Precomputed once at init.
  float dividerMultiplier_;
};

#endif