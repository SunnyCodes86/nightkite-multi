#pragma once

#include <stdint.h>

constexpr int CONFIG_VERSION_4_ALPHA_22_PATTERNS = 400;
constexpr int CONFIG_VERSION_4_ALPHA = 401;
constexpr int CONFIG_VERSION_SHOW_CONTROL = 402;
constexpr int CONFIG_VERSION_CURRENT = CONFIG_VERSION_SHOW_CONTROL;

inline bool supportsExtendedConfigVersion(int version)
{
  return version >= CONFIG_VERSION_4_ALPHA_22_PATTERNS && version <= CONFIG_VERSION_CURRENT;
}

inline int migrateShowControlEnabled(int version, int storedValue)
{
  return version >= CONFIG_VERSION_SHOW_CONTROL ? storedValue : 0;
}
constexpr uint32_t AUDIO_SYNC_PATTERN_MASK = 0x07C00000UL;

inline bool needsAudioSyncPatternMigration(int storedConfigVersion)
{
  return storedConfigVersion <= CONFIG_VERSION_4_ALPHA_22_PATTERNS;
}

inline uint32_t migrateEnabledPatternMask(int storedConfigVersion, uint32_t enabledPatternMask)
{
  return needsAudioSyncPatternMigration(storedConfigVersion)
      ? enabledPatternMask | AUDIO_SYNC_PATTERN_MASK
      : enabledPatternMask;
}

inline bool shouldPersistConfigRecovery(bool loadedValuesSane, bool migrated)
{
  return !loadedValuesSane || migrated;
}

inline bool shouldRunBootCalibration(bool safeBoot, bool quickConfigured)
{
  return !safeBoot && quickConfigured;
}
