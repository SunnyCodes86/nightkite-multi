#include "PatternClock.h"

void PatternClock::begin()
{
  baseMs = millis();
  phaseMs = 0;
  running = true;
  armed = false;
}

uint32_t PatternClock::now() const
{
  if (!running)
  {
    return phaseMs;
  }
  return phaseMs + (millis() - baseMs);
}

void PatternClock::setPhase(uint32_t phase)
{
  phaseMs = phase;
  baseMs = millis();
  running = true;
  armed = false;
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
    phaseMs = armedPhaseMs;
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
