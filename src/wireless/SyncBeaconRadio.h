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
  bool wirelessEnabled;
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
  bool codecSelftest;
  bool scanActive;
  bool advActive;
  bool gattAdvSuppressed;
  bool beaconAdvStarted;
  uint8_t advPayloadLen;
  uint8_t advMfgLen;
  uint16_t advCompany;
  uint16_t advMagic;
  uint8_t advVersion;
  uint8_t advGroup;
  uint16_t advCrc;
  unsigned long advEnableCount;
  unsigned long advDisableCount;
  unsigned long advSetCount;
  unsigned long beaconAdvRefreshes;
  uint16_t beaconSeq;
  unsigned long txCount;
  unsigned long rxCount;
  unsigned long crcErrors;
  unsigned long groupMismatch;
  unsigned long invalidPackets;
  unsigned long scanReports;
  unsigned long scanMfgReports;
  unsigned long scanNkCandidates;
  unsigned long scanDecodeOk;
  unsigned long scanDecodeV1;
  unsigned long scanDecodeV2;
  unsigned long scanDecodeFail;
  unsigned long scanCrcFail;
  unsigned long scanGroupMismatch;
  unsigned long scanRejectCompany;
  unsigned long scanRejectMagic;
  unsigned long scanRejectLen;
  unsigned long scanRejectVersion;
  unsigned long lastBeaconMs;
  unsigned long beaconAgeMs;
  int8_t scanLastRssi;
  uint8_t scanLastLen;
  uint8_t scanLastMfgLen;
  uint8_t scanLastAdType;
  uint16_t scanLastCompany;
  uint8_t scanLastGroup;
  uint8_t scanLastVersion;
  uint8_t lastBeaconVersion;
  AudioSyncState audio;
  unsigned long audioAgeMs;
  const char* advMfgHead;
  const char* advOwner;
  const char* advType;
  const char* mode;
  const char* lastError;
  const char* scanLastError;
  const char* scanLastMfgHead;
  const char* scanLastCandidateReason;
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

constexpr uint8_t NK_SYNC_BEACON_VERSION_V1 = 1;
constexpr uint8_t NK_SYNC_BEACON_VERSION_V2 = 2;
// The firmware's beacon transmitter intentionally remains V1.
constexpr uint8_t NK_SYNC_BEACON_VERSION = NK_SYNC_BEACON_VERSION_V1;
constexpr uint16_t NK_SYNC_BEACON_BEAT_MS = 1000;
constexpr uint8_t NK_SYNC_BEACON_MAGIC0 = 'N';
constexpr uint8_t NK_SYNC_BEACON_MAGIC1 = 'K';
constexpr uint16_t NK_SYNC_BEACON_COMPANY_ID = 0xFFFF;
constexpr uint8_t NK_SYNC_BEACON_FLAG_AUDIO_BEAT = 0x01;
constexpr unsigned long NK_AUDIO_SYNC_TIMEOUT_MS = 1500;
constexpr size_t NK_SYNC_BEACON_V1_PACKET_SIZE = sizeof(NkSyncBeaconV1);
constexpr size_t NK_SYNC_BEACON_V2_PACKET_SIZE = sizeof(NkSyncBeaconV2);
constexpr size_t NK_SYNC_BEACON_PACKET_SIZE = NK_SYNC_BEACON_V1_PACKET_SIZE;
constexpr uint8_t NK_SYNC_BEACON_MFG_PAYLOAD_OFFSET = 2;
constexpr size_t NK_SYNC_BEACON_MFG_LEN = NK_SYNC_BEACON_MFG_PAYLOAD_OFFSET + NK_SYNC_BEACON_PACKET_SIZE;
constexpr size_t NK_SYNC_BEACON_ADV_LEN = 3 + 2 + NK_SYNC_BEACON_MFG_LEN;
constexpr size_t NK_SYNC_BEACON_V2_MFG_LEN = NK_SYNC_BEACON_MFG_PAYLOAD_OFFSET + NK_SYNC_BEACON_V2_PACKET_SIZE;
constexpr size_t NK_SYNC_BEACON_V2_ADV_LEN = 3 + 2 + NK_SYNC_BEACON_V2_MFG_LEN;

static_assert(sizeof(NkSyncBeaconV1) == 17, "Unexpected V1 sync beacon layout");
static_assert(sizeof(NkSyncBeaconV2) == 22, "Unexpected V2 sync beacon layout");
static_assert(NK_SYNC_BEACON_V2_ADV_LEN <= 31, "V2 sync beacon exceeds legacy advertising");

void syncBeaconRadioBegin();
void syncBeaconRadioTick(const SyncBeaconRuntime& runtime);
void syncBeaconRadioStop();
SyncBeaconRadioStatus syncBeaconRadioStatus();
String syncBeaconRadioBuildStatusFields();
String syncBeaconAudioBuildStatusFields();
bool syncBeaconRadioConsumeBeacon(NkSyncBeaconV1* beacon);
AudioSyncState syncBeaconAudioState();

bool syncBeaconEncode(const NkSyncBeaconV1& beacon, uint8_t* output, size_t outputSize, size_t* outputLen);
SyncBeaconDecodeResult syncBeaconDecode(const uint8_t* data, size_t dataLen, uint8_t expectedGroup, NkSyncBeaconV1* beacon);
SyncBeaconDecodeResult syncBeaconDecodeV2(const uint8_t* data, size_t dataLen, uint8_t expectedGroup, NkSyncBeaconV2* beacon);
const char* syncBeaconDecodeResultName(SyncBeaconDecodeResult result);
