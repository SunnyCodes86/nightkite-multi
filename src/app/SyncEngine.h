#pragma once

#include <Arduino.h>

// Pattern IDs are part of the sync payload's application-level contract.
// Keep transmitter, decoder, and local registry on the same accepted range.
constexpr uint8_t NK_PATTERN_MIN_ID = 1;
constexpr uint8_t NK_PATTERN_MAX_ID = 27;

// Manufacturer data starts with company ID 0xFFFF in little-endian order,
// followed by one packed beacon. Multi-byte beacon fields are little endian.
// V1 offsets: NK[0..1], version[2], group[3], flags[4], seq[5..6],
// pattern[7], brightness[8], phaseMs[9..12], beatMs[13..14], crc[15..16].
// CRC16-CCITT (init 0xFFFF, polynomial 0x1021) covers all 17 bytes with crc
// set to zero.
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

// V2 preserves V1 offsets through beatMs, uses audio values at offsets 15..19,
// and moves the CRC to offsets 20..21. Flags bit 0 marks an audio beat; all
// other flag bits are reserved.
// At 22 bytes it fits in a 29-byte legacy advertisement including flags,
// manufacturer AD framing, and the 0xFFFF company ID.
struct NkSyncBeaconV2
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
  uint8_t audioEnergy;
  uint8_t audioBass;
  uint8_t audioMid;
  uint8_t audioTreble;
  uint8_t audioConfidence;
  uint16_t crc;
} __attribute__((packed));

struct AudioSyncState
{
  bool valid = false;
  unsigned long lastUpdateMs = 0;
  uint16_t seq = 0;
  uint32_t phaseMs = 0;
  uint16_t beatMs = 0;
  bool beat = false;
  uint8_t energy = 0;
  uint8_t bass = 0;
  uint8_t mid = 0;
  uint8_t treble = 0;
  uint8_t confidence = 0;
};

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
