#include "AudioPatternMath.h"

uint16_t sanitizeAudioPatternBeatMs(uint16_t beatMs)
{
  if (beatMs < AUDIO_PATTERN_MIN_BEAT_MS || beatMs > AUDIO_PATTERN_MAX_BEAT_MS)
  {
    return AUDIO_PATTERN_DEFAULT_BEAT_MS;
  }
  return beatMs;
}

uint8_t audioPatternPhase8(uint32_t phaseMs, uint16_t beatMs)
{
  const uint16_t safeBeatMs = sanitizeAudioPatternBeatMs(beatMs);
  return (uint8_t)(((phaseMs % safeBeatMs) * 255UL) / safeBeatMs);
}

uint8_t audioPatternBeatPulse8(uint32_t phaseMs, uint16_t beatMs)
{
  const uint16_t safeBeatMs = sanitizeAudioPatternBeatMs(beatMs);
  const uint16_t elapsedMs = (uint16_t)(phaseMs % safeBeatMs);
  const uint16_t pulseMs = safeBeatMs / 4;
  if (elapsedMs >= pulseMs)
  {
    return 0;
  }
  return (uint8_t)(255U - ((uint32_t)elapsedMs * 255UL) / pulseMs);
}
