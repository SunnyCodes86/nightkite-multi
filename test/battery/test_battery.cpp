#include <assert.h>
#include <stdint.h>

#include "app/Battery.h"

static void assertBetween(int value, int minValue, int maxValue)
{
  assert(value >= minValue);
  assert(value <= maxValue);
}

static void testSocInterpolation()
{
  assert(estimateBatteryPercent(4.20f) == 100);
  assert(estimateBatteryPercent(4.10f) == 100);
  assert(estimateBatteryPercent(4.02f) == 95);
  assertBetween(estimateBatteryPercent(3.995f), 92, 93);
  assert(estimateBatteryPercent(3.40f) == 8);
  assert(estimateBatteryPercent(3.325f) == 5);
  assert(estimateBatteryPercent(3.00f) == 0);
  assert(estimateBatteryPercent(2.90f) == 0);

  assert(batteryBarsForPercent(80) == 5);
  assert(batteryBarsForPercent(60) == 4);
  assert(batteryBarsForPercent(40) == 3);
  assert(batteryBarsForPercent(20) == 2);
  assert(batteryBarsForPercent(8) == 1);
  assert(batteryBarsForPercent(7) == 0);
}

static void testStateTransitions()
{
  BatteryStateTracker tracker;

  assert(tracker.update(3.39f, false, 1000) == BATTERY_STATE_NORMAL);
  assert(tracker.update(3.39f, false, 15999) == BATTERY_STATE_NORMAL);
  assert(tracker.update(3.39f, false, 16000) == BATTERY_STATE_LOW_WARNING);
  assert(tracker.update(3.44f, false, 17000) == BATTERY_STATE_LOW_WARNING);
  assert(tracker.update(3.46f, false, 18000) == BATTERY_STATE_NORMAL);

  tracker.reset();
  assert(tracker.update(3.29f, false, 1000) == BATTERY_STATE_NORMAL);
  assert(tracker.update(3.29f, false, 11000) == BATTERY_STATE_CRITICAL);
  assert(batteryStateCapsBrightness(tracker.state()));

  tracker.reset();
  assert(tracker.update(3.19f, false, 1000) == BATTERY_STATE_NORMAL);
  assert(tracker.update(3.19f, false, 16000) == BATTERY_STATE_SOFT_CUTOFF);
  assert(batteryStateCutsOff(tracker.state()));
  assert(tracker.update(3.50f, false, 17000) == BATTERY_STATE_SOFT_CUTOFF);
  assert(tracker.update(3.50f, true, 18000) == BATTERY_STATE_NORMAL);

  tracker.reset();
  assert(tracker.update(3.07f, false, 1000) == BATTERY_STATE_EMERGENCY_CUTOFF);
  assert(batteryStateCutsOff(tracker.state()));
}

static void testDisplayHysteresisDoesNotDelayProtection()
{
  const BatteryMeasurement measurement = batteryMeasurementFromAverage(3.29f, 3.33f, false);
  assert(measurement.displayVoltage == 3.33f);
  assert(measurement.protectionVoltage == 3.29f);

  BatteryStateTracker tracker;
  assert(tracker.update(measurement.protectionVoltage, false, 1000) == BATTERY_STATE_NORMAL);
  assert(tracker.update(measurement.protectionVoltage, false, 11000) == BATTERY_STATE_CRITICAL);
}

int main()
{
  testSocInterpolation();
  testStateTransitions();
  testDisplayHysteresisDoesNotDelayProtection();
  return 0;
}
