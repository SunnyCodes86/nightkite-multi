#include <assert.h>

#include "app/AudioPatternMath.h"

static void testBeatSanitizing()
{
  assert(sanitizeAudioPatternBeatMs(0) == AUDIO_PATTERN_DEFAULT_BEAT_MS);
  assert(sanitizeAudioPatternBeatMs(249) == AUDIO_PATTERN_DEFAULT_BEAT_MS);
  assert(sanitizeAudioPatternBeatMs(250) == 250);
  assert(sanitizeAudioPatternBeatMs(2000) == 2000);
  assert(sanitizeAudioPatternBeatMs(2001) == AUDIO_PATTERN_DEFAULT_BEAT_MS);
}

static void testPhaseAndPulse()
{
  assert(audioPatternPhase8(0, 500) == 0);
  assert(audioPatternPhase8(250, 500) >= 127);
  assert(audioPatternPhase8(500, 500) == 0);

  assert(audioPatternBeatPulse8(0, 500) == 255);
  assert(audioPatternBeatPulse8(124, 500) > 0);
  assert(audioPatternBeatPulse8(125, 500) == 0);
  assert(audioPatternBeatPulse8(500, 500) == 255);
}

int main()
{
  testBeatSanitizing();
  testPhaseAndPulse();
  return 0;
}
