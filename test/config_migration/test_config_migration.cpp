#include <assert.h>

#include "app/ConfigMigration.h"

int main()
{
  const uint32_t legacyAllEnabled = 0x003FFFFFUL;
  const uint32_t legacyCustom = 0x00155555UL;

  assert(migrateEnabledPatternMask(400, legacyAllEnabled) == 0x07FFFFFFUL);
  assert(migrateEnabledPatternMask(0, legacyAllEnabled) == 0x07FFFFFFUL);
  assert((migrateEnabledPatternMask(400, legacyCustom) & legacyAllEnabled) == legacyCustom);
  assert((migrateEnabledPatternMask(400, legacyCustom) & AUDIO_SYNC_PATTERN_MASK) == AUDIO_SYNC_PATTERN_MASK);
  assert(migrateEnabledPatternMask(401, legacyCustom) == legacyCustom);
  assert(migrateEnabledPatternMask(402, legacyCustom) == legacyCustom);

  assert(shouldPersistConfigRecovery(false, false));
  assert(shouldPersistConfigRecovery(true, true));
  assert(!shouldPersistConfigRecovery(true, false));

  assert(shouldRunBootCalibration(false, true));
  assert(!shouldRunBootCalibration(false, false));
  assert(!shouldRunBootCalibration(true, true));
  return 0;
}
