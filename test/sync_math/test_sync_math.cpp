#include <assert.h>

#include "app/SyncMath.h"

static void testLinearDelta()
{
  assert(syncPhaseDeltaMs(105, 100, 500) == 5);
  assert(syncPhaseDeltaMs(95, 100, 500) == -5);
}

static void testWrappedDelta()
{
  assert(syncPhaseDeltaMs(1, 499, 500) == 2);
  assert(syncPhaseDeltaMs(499, 1, 500) == -2);
  assert(syncPhaseDeltaMs(76, 575, 500) == 1);
}

static void testZeroPeriodFallback()
{
  assert(syncPhaseDeltaMs(105, 100, 0) == 5);
}

int main()
{
  testLinearDelta();
  testWrappedDelta();
  testZeroPeriodFallback();
  return 0;
}
