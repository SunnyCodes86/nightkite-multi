#pragma once

#include <Arduino.h>

class PatternClock
{
public:
  void begin();
  uint32_t now() const;
  uint32_t phaseMs() const;
  void setPhase(uint32_t phase);
  void syncToBeaconPhase(uint32_t phase);
  void markPatternChange();
  void armStart(uint32_t localStartMs, uint32_t phase);
  void tick();
  bool isRunning() const;
  bool isArmed() const;

private:
  uint32_t baseMs = 0;
  uint32_t currentPhaseMs = 0;
  uint32_t armedStartMs = 0;
  uint32_t armedPhaseMs = 0;
  bool running = false;
  bool armed = false;
};
