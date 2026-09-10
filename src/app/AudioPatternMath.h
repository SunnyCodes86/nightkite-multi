#pragma once

#include <stdint.h>

struct AudioSyncState
{
  bool valid = false;
  uint32_t lastUpdateMs = 0;
  uint16_t seq = 0;
  uint32_t phaseMs = 0;
  uint16_t beatMs = 0;
  bool beatLocked = false;
  bool beat = false;
  uint8_t energy = 0;
  uint8_t bass = 0;
  uint8_t mid = 0;
  uint8_t treble = 0;
  uint8_t confidence = 0;
};

// 5/10/20 Hz profiles: two missed 200 ms frames plus 100 ms jitter margin.
// Independent of the 1500 ms controller-sync loss timeout.
constexpr uint32_t NK_AUDIO_FRESHNESS_TIMEOUT_MS = 500;

constexpr uint16_t AUDIO_PATTERN_DEFAULT_BEAT_MS = 600;
constexpr uint16_t AUDIO_PATTERN_MIN_BEAT_MS = 250;
constexpr uint16_t AUDIO_PATTERN_MAX_BEAT_MS = 2000;

uint16_t sanitizeAudioPatternBeatMs(uint16_t beatMs);
uint8_t audioPatternPhase8(uint32_t phaseMs, uint16_t beatMs);
uint8_t audioPatternBeatPulse8(uint32_t phaseMs, uint16_t beatMs);
uint8_t audioToVisualBrightness(uint8_t level);

struct AudioPatternFrame
{
  bool valid = false;
  bool fresh = false;
  bool beatLocked = false;
  bool beat = false;
  uint8_t phase8 = 0;
  uint8_t beatPulse = 0;
  uint8_t energy = 0;
  uint8_t bass = 0;
  uint8_t mid = 0;
  uint8_t treble = 0;
  uint8_t confidence = 0;
};

class AudioPatternFilter
{
public:
  AudioPatternFrame update(const AudioSyncState& state, uint32_t nowMs);
  void reset();
private:
  AudioPatternFrame frame;
  uint32_t lastFrameMs = 0;
  uint32_t lastAudioUpdateMs = 0;
};
