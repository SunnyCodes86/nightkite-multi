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

void AudioPatternFilter::reset()
{
  frame = AudioPatternFrame{};
}

AudioPatternFrame AudioPatternFilter::update(const AudioSyncState& state, uint32_t nowMs)
{
  const uint32_t age = nowMs - state.lastUpdateMs;
  if (!state.valid || age > NK_AUDIO_FRESHNESS_TIMEOUT_MS)
  {
    reset();
    return frame;
  }
  frame.fresh = !frame.valid || nowMs - lastFrameMs > NK_AUDIO_FRESHNESS_TIMEOUT_MS ||
      state.lastUpdateMs - lastAudioUpdateMs > NK_AUDIO_FRESHNESS_TIMEOUT_MS;
  // Equivalent to FastLED lerp8by8(a, b, 64), with SCALE8_FIXED enabled.
  const auto smooth = [this](uint8_t oldValue, uint8_t value) -> uint8_t {
    if (frame.fresh) return value;
    if (value >= oldValue) return oldValue + ((uint16_t)(value - oldValue) * 65U >> 8);
    return oldValue - ((uint16_t)(oldValue - value) * 65U >> 8);
  };
  frame.valid = true;
  frame.energy = smooth(frame.energy, state.energy);
  frame.bass = smooth(frame.bass, state.bass);
  frame.mid = smooth(frame.mid, state.mid);
  frame.treble = smooth(frame.treble, state.treble);
  frame.confidence = smooth(frame.confidence, state.confidence);
  frame.beat = state.beat && age < 180;
  frame.phase8 = audioPatternPhase8(state.phaseMs + age, state.beatMs);
  frame.beatPulse = audioPatternBeatPulse8(state.phaseMs + age, state.beatMs);
  lastFrameMs = nowMs;
  lastAudioUpdateMs = state.lastUpdateMs;
  return frame;
}
