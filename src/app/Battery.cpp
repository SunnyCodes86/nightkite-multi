#include "app/Battery.h"

#include <stddef.h>

struct BatterySocPoint
{
  float voltage;
  int percent;
};

static const BatterySocPoint BATTERY_SOC_CURVE[] = {
    {4.10f, 100},
    {4.02f, 95},
    {3.97f, 90},
    {3.91f, 80},
    {3.83f, 70},
    {3.78f, 60},
    {3.72f, 50},
    {3.65f, 40},
    {3.57f, 30},
    {3.50f, 20},
    {3.43f, 10},
    {3.40f, 8},
    {3.35f, 5},
    {3.30f, 4},
    {3.20f, 2},
    {3.10f, 1},
    {3.00f, 0},
};

static int clampPercent(int value)
{
  if (value < 0) return 0;
  if (value > 100) return 100;
  return value;
}

const char* batteryStateName(BatteryState state)
{
  switch (state)
  {
    case BATTERY_STATE_LOW_WARNING: return "LOW_WARNING";
    case BATTERY_STATE_CRITICAL: return "CRITICAL";
    case BATTERY_STATE_SOFT_CUTOFF: return "SOFT_CUTOFF";
    case BATTERY_STATE_EMERGENCY_CUTOFF: return "EMERGENCY_CUTOFF";
    case BATTERY_STATE_NORMAL:
    default: return "NORMAL";
  }
}

int estimateBatteryPercent(float voltage)
{
  const size_t count = sizeof(BATTERY_SOC_CURVE) / sizeof(BATTERY_SOC_CURVE[0]);
  if (voltage >= BATTERY_SOC_CURVE[0].voltage)
  {
    return BATTERY_SOC_CURVE[0].percent;
  }
  if (voltage <= BATTERY_SOC_CURVE[count - 1].voltage)
  {
    return BATTERY_SOC_CURVE[count - 1].percent;
  }

  for (size_t i = 0; i + 1 < count; ++i)
  {
    const BatterySocPoint high = BATTERY_SOC_CURVE[i];
    const BatterySocPoint low = BATTERY_SOC_CURVE[i + 1];
    if (voltage <= high.voltage && voltage >= low.voltage)
    {
      const float span = high.voltage - low.voltage;
      const float position = span > 0.0f ? (voltage - low.voltage) / span : 0.0f;
      const float interpolated = low.percent + position * (high.percent - low.percent);
      return clampPercent((int)(interpolated + 0.5f));
    }
  }

  return 0;
}

uint8_t batteryBarsForPercent(int percent)
{
  if (percent >= 80) return 5;
  if (percent >= 60) return 4;
  if (percent >= 40) return 3;
  if (percent >= 20) return 2;
  if (percent >= 8) return 1;
  return 0;
}

bool batteryStateCutsOff(BatteryState state)
{
  return state == BATTERY_STATE_SOFT_CUTOFF || state == BATTERY_STATE_EMERGENCY_CUTOFF;
}

bool batteryStateCapsBrightness(BatteryState state)
{
  return state == BATTERY_STATE_CRITICAL || batteryStateCutsOff(state);
}

BatteryState BatteryStateTracker::update(float voltage, bool usbPowered, uint32_t nowMs)
{
  if (usbPowered)
  {
    reset();
    return currentState;
  }

  if (voltage <= BATTERY_EMERGENCY_CUTOFF_VOLTAGE)
  {
    currentState = BATTERY_STATE_EMERGENCY_CUTOFF;
    return currentState;
  }

  if (batteryStateCutsOff(currentState))
  {
    return currentState;
  }

  updateTimer(
      thresholdActive(voltage, BATTERY_LOW_WARNING_VOLTAGE, lowSinceMs != 0 || currentState >= BATTERY_STATE_LOW_WARNING),
      nowMs,
      &lowSinceMs);
  updateTimer(
      thresholdActive(voltage, BATTERY_CRITICAL_VOLTAGE, criticalSinceMs != 0 || currentState >= BATTERY_STATE_CRITICAL),
      nowMs,
      &criticalSinceMs);
  updateTimer(
      thresholdActive(voltage, BATTERY_SOFT_CUTOFF_VOLTAGE, softSinceMs != 0 || currentState >= BATTERY_STATE_SOFT_CUTOFF),
      nowMs,
      &softSinceMs);

  if (elapsed(softSinceMs, nowMs, BATTERY_SOFT_CUTOFF_DELAY_MS))
  {
    currentState = BATTERY_STATE_SOFT_CUTOFF;
  }
  else if (elapsed(criticalSinceMs, nowMs, BATTERY_CRITICAL_DELAY_MS))
  {
    currentState = BATTERY_STATE_CRITICAL;
  }
  else if (elapsed(lowSinceMs, nowMs, BATTERY_LOW_WARNING_DELAY_MS))
  {
    currentState = BATTERY_STATE_LOW_WARNING;
  }
  else
  {
    currentState = BATTERY_STATE_NORMAL;
  }

  return currentState;
}

BatteryState BatteryStateTracker::state() const
{
  return currentState;
}

void BatteryStateTracker::reset()
{
  currentState = BATTERY_STATE_NORMAL;
  lowSinceMs = 0;
  criticalSinceMs = 0;
  softSinceMs = 0;
}

bool BatteryStateTracker::thresholdActive(float voltage, float threshold, bool wasActive)
{
  return wasActive ? voltage < (threshold + BATTERY_STATE_HYSTERESIS_VOLTAGE) : voltage < threshold;
}

void BatteryStateTracker::updateTimer(bool active, uint32_t nowMs, uint32_t* sinceMs)
{
  if (!active)
  {
    *sinceMs = 0;
    return;
  }
  if (*sinceMs == 0)
  {
    *sinceMs = nowMs == 0 ? 1 : nowMs;
  }
}

bool BatteryStateTracker::elapsed(uint32_t sinceMs, uint32_t nowMs, uint32_t delayMs)
{
  return sinceMs != 0 && (uint32_t)(nowMs - sinceMs) >= delayMs;
}
