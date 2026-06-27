#pragma once

#include <stdint.h>

constexpr uint16_t AUDIO_PATTERN_DEFAULT_BEAT_MS = 600;
constexpr uint16_t AUDIO_PATTERN_MIN_BEAT_MS = 250;
constexpr uint16_t AUDIO_PATTERN_MAX_BEAT_MS = 2000;

uint16_t sanitizeAudioPatternBeatMs(uint16_t beatMs);
uint8_t audioPatternPhase8(uint32_t phaseMs, uint16_t beatMs);
uint8_t audioPatternBeatPulse8(uint32_t phaseMs, uint16_t beatMs);
