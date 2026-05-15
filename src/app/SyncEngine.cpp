#include "SyncEngine.h"

void SyncEngine::begin()
{
  state = IDLE;
  lastSeq = 0;
  localStartMs = 0;
  locked = false;
  driftMs = 0;
  startEventPending = false;
}

bool SyncEngine::arm(uint8_t group, uint8_t pattern, uint8_t brightness, uint32_t startInMs, uint32_t phase)
{
  if (state == ARMED)
  {
    return false;
  }
  armedGroup = group;
  armedPattern = pattern;
  armedBrightness = brightness;
  armedPhaseMs = phase;
  localStartMs = millis() + startInMs;
  state = ARMED;
  locked = false;
  startEventPending = false;
  lastSeq++;
  return true;
}

void SyncEngine::cancel()
{
  state = IDLE;
  locked = false;
  driftMs = 0;
  startEventPending = false;
}

void SyncEngine::tick()
{
  if (state == ARMED && (int32_t)(millis() - localStartMs) >= 0)
  {
    state = RUNNING;
    locked = true;
    startEventPending = true;
  }
}

bool SyncEngine::consumeStartEvent()
{
  if (!startEventPending)
  {
    return false;
  }
  startEventPending = false;
  return true;
}

const char* SyncEngine::stateName() const
{
  switch (state)
  {
    case IDLE: return "idle";
    case ARMED: return "armed";
    case RUNNING: return "running";
    case LOST: return "lost";
    case ERROR: return "error";
    default: return "error";
  }
}
