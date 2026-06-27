#include "SyncMath.h"

int32_t syncPhaseDeltaMs(uint32_t remotePhaseMs, uint32_t localPhaseMs, uint16_t beatMs)
{
  if (beatMs == 0)
  {
    return (int32_t)(remotePhaseMs - localPhaseMs);
  }

  const int32_t period = beatMs;
  int32_t delta = (int32_t)(remotePhaseMs % beatMs) - (int32_t)(localPhaseMs % beatMs);
  const int32_t halfPeriod = period / 2;
  if (delta > halfPeriod)
  {
    delta -= period;
  }
  else if (delta < -halfPeriod)
  {
    delta += period;
  }
  return delta;
}
