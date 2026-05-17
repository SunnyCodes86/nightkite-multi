#include "PatternClock.h"

void PatternClock::begin()
{
  baseMs = millis();
  currentPhaseMs = 0;
  running = true;
  armed = false;
}

uint32_t PatternClock::now() const
{
  if (!running)
  {
    return currentPhaseMs;
  }
  return currentPhaseMs + (millis() - baseMs);
}

uint32_t PatternClock::phaseMs() const
{
  return now();
}

void PatternClock::setPhase(uint32_t phase)
{
  currentPhaseMs = phase;
  baseMs = millis();
  running = true;
  armed = false;
}

void PatternClock::syncToBeaconPhase(uint32_t phase)
{
  setPhase(phase);
}

void PatternClock::markPatternChange()
{
  setPhase(0);
}

void PatternClock::armStart(uint32_t localStartMs, uint32_t phase)
{
  armedStartMs = localStartMs;
  armedPhaseMs = phase;
  armed = true;
  running = false;
}

void PatternClock::tick()
{
  if (armed && (int32_t)(millis() - armedStartMs) >= 0)
  {
    currentPhaseMs = armedPhaseMs;
    baseMs = armedStartMs;
    running = true;
    armed = false;
  }
}

bool PatternClock::isRunning() const
{
  return running;
}

bool PatternClock::isArmed() const
{
  return armed;
}
