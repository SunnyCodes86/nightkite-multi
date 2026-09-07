#pragma once

#include "protocol/SyncBeaconCodec.h"

// Values match the existing persisted play-mode/role contract.
inline bool syncAutoplayAllowed(bool enabled, uint8_t playMode, bool syncEnabled, uint8_t role)
{
  return enabled && (playMode == 1 || (playMode == 2 && syncEnabled && role == 1));
}

class SyncEngine
{
public:
  enum State
  {
    IDLE,
    ARMED,
    RUNNING,
    LOST,
    ERROR
  };

  void begin();
  bool arm(uint8_t group, uint8_t pattern, uint8_t brightness, uint32_t startInMs, uint32_t phase);
  void cancel();
  void tick();
  bool consumeStartEvent();
  bool follow(const NkSyncBeaconV1& beacon, uint8_t group, uint32_t nowMs, uint32_t localPhaseMs);
  const char* stateName() const;

  State state = IDLE;
  uint16_t lastSeq = 0;
  uint32_t localStartMs = 0;
  uint32_t armedPhaseMs = 0;
  uint8_t armedGroup = 1;
  uint8_t armedPattern = 1;
  uint8_t armedBrightness = 95;
  bool locked = false;
  int32_t driftMs = 0;
  bool startEventPending = false;
};
