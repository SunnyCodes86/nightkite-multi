#pragma once

#include <stdint.h>

enum BatteryState : uint8_t
{
  BATTERY_STATE_NORMAL = 0,
  BATTERY_STATE_LOW_WARNING,
  BATTERY_STATE_CRITICAL,
  BATTERY_STATE_SOFT_CUTOFF,
  BATTERY_STATE_EMERGENCY_CUTOFF
};

constexpr float BATTERY_LOW_WARNING_VOLTAGE = 3.40f;
constexpr float BATTERY_CRITICAL_VOLTAGE = 3.30f;
constexpr float BATTERY_SOFT_CUTOFF_VOLTAGE = 3.20f;
constexpr float BATTERY_EMERGENCY_CUTOFF_VOLTAGE = 3.08f;
constexpr float BATTERY_STATE_HYSTERESIS_VOLTAGE = 0.05f;
constexpr float BATTERY_MEASUREMENT_HYSTERESIS_VOLTAGE = 0.05f;
constexpr uint32_t BATTERY_LOW_WARNING_DELAY_MS = 15000;
constexpr uint32_t BATTERY_CRITICAL_DELAY_MS = 10000;
constexpr uint32_t BATTERY_SOFT_CUTOFF_DELAY_MS = 15000;

struct BatteryMeasurement
{
  float displayVoltage;
  float protectionVoltage;
};

const char* batteryStateName(BatteryState state);
int estimateBatteryPercent(float voltage);
uint8_t batteryBarsForPercent(int percent);
bool batteryStateCutsOff(BatteryState state);
bool batteryStateCapsBrightness(BatteryState state);
uint8_t batteryLimitedBrightness(uint8_t requested, BatteryState state, uint8_t criticalLimit);
BatteryMeasurement batteryMeasurementFromAverage(float averagedVoltage, float displayedVoltage, bool firstSample);

class BatteryStateTracker
{
public:
  BatteryState update(float voltage, bool usbPowered, uint32_t nowMs);
  BatteryState state() const;
  void reset();

private:
  BatteryState currentState = BATTERY_STATE_NORMAL;
  uint32_t lowSinceMs = 0;
  uint32_t criticalSinceMs = 0;
  uint32_t softSinceMs = 0;

  static bool thresholdActive(float voltage, float threshold, bool wasActive);
  static void updateTimer(bool active, uint32_t nowMs, uint32_t* sinceMs);
  static bool elapsed(uint32_t sinceMs, uint32_t nowMs, uint32_t delayMs);
};
