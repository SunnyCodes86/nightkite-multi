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

static void testVisualBrightnessMapping()
{
  assert(audioToVisualBrightness(0) == 0);
  assert(audioToVisualBrightness(20) == 64);
  assert(audioToVisualBrightness(40) == 104);
  assert(audioToVisualBrightness(64) == 152);
  assert(audioToVisualBrightness(120) == 194);
  assert(audioToVisualBrightness(160) == 224);
  assert(audioToVisualBrightness(255) == 255);
  for (int level = 1; level < 256; ++level)
    assert(audioToVisualBrightness((uint8_t)level) >= audioToVisualBrightness((uint8_t)(level - 1)));
}

static void assertSilent(const AudioPatternFrame& f)
{
  assert(!f.valid && !f.fresh && !f.beatLocked && !f.beat);
  assert(f.phase8 == 0 && f.beatPulse == 0 && f.energy == 0 && f.bass == 0);
  assert(f.mid == 0 && f.treble == 0 && f.confidence == 0);
}

static void testStrictAudioLifetime()
{
  AudioPatternFilter filter;
  AudioSyncState audio; // Also represents V1-only operation: no audio source.
  for (uint32_t now = 0; now < 20000; now += 19) assertSilent(filter.update(audio, now));
  audio.valid = true;
  audio.lastUpdateMs = 100;
  audio.phaseMs = 10;
  audio.beatMs = 500;
  audio.beat = true;
  audio.energy = 220; audio.bass = 190; audio.mid = 80; audio.treble = 44; audio.confidence = 255;
  auto f = filter.update(audio, 100);
  assert(f.valid && f.fresh && f.beat && f.energy == 220 && f.bass == 190);
  assert(!f.beatLocked && f.phase8 == 0 && f.beatPulse == 0);
  audio.beatLocked = true;
  f = filter.update(audio, 100);
  assert(f.beatLocked && f.phase8 > 0 && f.beatPulse > 0);
  audio.energy = 100;
  f = filter.update(audio, 108);
  assert(!f.fresh && f.energy == 190); // Same 64/256 FastLED smoothing.
  assert(filter.update(audio, 600).valid);
  assertSilent(filter.update(audio, 601));
  assertSilent(filter.update(audio, 602));
  audio.lastUpdateMs = 1700;
  audio.energy = 5; audio.bass = 3; audio.mid = 2; audio.treble = 1;
  f = filter.update(audio, 1700);
  assert(f.fresh && f.energy == 5 && f.bass == 3 && f.mid == 2 && f.treble == 1);
  audio.valid = false;
  assertSilent(filter.update(audio, 1701));
  audio.valid = true;
  audio.beatLocked = false;
  audio.lastUpdateMs = 2000;
  assert(filter.update(audio, 2000).fresh);
  // Filter was not rendered while a solid override was active.
  audio.lastUpdateMs = 4000;
  audio.energy = 250;
  assert(filter.update(audio, 4000).energy == 250);
  // Loss and reacquisition may both occur between two render frames at the timeout boundary.
  assert(filter.update(audio, 4500).valid);
  audio.lastUpdateMs = 4501;
  audio.energy = 1;
  f = filter.update(audio, 4501);
  assert(f.fresh && f.energy == 1);
  filter.reset();
  audio.lastUpdateMs = UINT32_MAX - 15;
  assert(filter.update(audio, 15).valid);
  assertSilent(filter.update(audio, 1600));
}

int main()
{
  testStrictAudioLifetime();
  testBeatSanitizing();
  testPhaseAndPulse();
  testVisualBrightnessMapping();
  return 0;
}
