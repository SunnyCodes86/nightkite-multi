#pragma once

#include <Arduino.h>

struct NkSyncBeaconV1
{
  uint8_t magic0;
  uint8_t magic1;
  uint8_t version;
  uint8_t groupId;
  uint8_t flags;
  uint16_t seq;
  uint8_t pattern;
  uint8_t brightness;
  uint32_t phaseMs;
  uint16_t beatMs;
  uint16_t crc;
} __attribute__((packed));

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
