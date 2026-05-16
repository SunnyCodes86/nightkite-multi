#pragma once

#include <Arduino.h>
#include "app/SyncEngine.h"

enum SyncBeaconRole : uint8_t
{
  SYNC_BEACON_ROLE_STANDALONE = 0,
  SYNC_BEACON_ROLE_MASTER = 1,
  SYNC_BEACON_ROLE_FOLLOWER = 2
};

enum SyncBeaconPlayMode : uint8_t
{
  SYNC_BEACON_PLAY_MANUAL = 0,
  SYNC_BEACON_PLAY_AUTOPLAY = 1,
  SYNC_BEACON_PLAY_SYNC = 2
};

enum SyncBeaconWirelessProfile : uint8_t
{
  SYNC_BEACON_PROFILE_LONG_RANGE = 0,
  SYNC_BEACON_PROFILE_BALANCED = 1,
  SYNC_BEACON_PROFILE_FAST_SYNC = 2
};

struct SyncBeaconRuntime
{
  bool syncEnabled;
  uint8_t playMode;
  uint8_t syncRole;
  uint8_t groupId;
  uint8_t pattern;
  uint8_t brightness;
  uint8_t wirelessProfile;
  uint32_t phaseMs;
  uint16_t beatMs;
};

struct SyncBeaconRadioStatus
{
  bool supported;
  bool active;
  bool beaconTx;
  bool beaconRx;
  bool locked;
  uint16_t beaconSeq;
  unsigned long txCount;
  unsigned long rxCount;
  unsigned long crcErrors;
  unsigned long groupMismatch;
  unsigned long invalidPackets;
  unsigned long lastBeaconMs;
  unsigned long beaconAgeMs;
  const char* mode;
  const char* lastError;
};

enum SyncBeaconDecodeResult : uint8_t
{
  SYNC_BEACON_DECODE_OK = 0,
  SYNC_BEACON_DECODE_TOO_SHORT,
  SYNC_BEACON_DECODE_BAD_MAGIC,
  SYNC_BEACON_DECODE_BAD_VERSION,
  SYNC_BEACON_DECODE_BAD_GROUP,
  SYNC_BEACON_DECODE_BAD_PATTERN,
  SYNC_BEACON_DECODE_BAD_BRIGHTNESS,
  SYNC_BEACON_DECODE_BAD_CRC
};

constexpr uint8_t NK_SYNC_BEACON_VERSION = 1;
constexpr uint16_t NK_SYNC_BEACON_BEAT_MS = 1000;

void syncBeaconRadioBegin();
void syncBeaconRadioTick(const SyncBeaconRuntime& runtime);
void syncBeaconRadioStop();
SyncBeaconRadioStatus syncBeaconRadioStatus();
String syncBeaconRadioBuildStatusFields();
bool syncBeaconRadioConsumeBeacon(NkSyncBeaconV1* beacon);

bool syncBeaconEncode(const NkSyncBeaconV1& beacon, uint8_t* output, size_t outputSize, size_t* outputLen);
SyncBeaconDecodeResult syncBeaconDecode(const uint8_t* data, size_t dataLen, uint8_t expectedGroup, NkSyncBeaconV1* beacon);
const char* syncBeaconDecodeResultName(SyncBeaconDecodeResult result);
