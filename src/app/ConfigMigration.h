#pragma once

#include <stdint.h>

constexpr int CONFIG_VERSION_4_ALPHA_22_PATTERNS = 400;
constexpr int CONFIG_VERSION_4_ALPHA = 401;
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
