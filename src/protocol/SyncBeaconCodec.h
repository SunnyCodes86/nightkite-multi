#pragma once

#include <stdint.h>
#include <stddef.h>

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

bool syncBeaconEncode(const NkSyncBeaconV1& beacon, uint8_t* output, size_t outputSize, size_t* outputLen);
SyncBeaconDecodeResult syncBeaconDecode(const uint8_t* data, size_t dataLen, uint8_t expectedGroup, NkSyncBeaconV1* beacon);
SyncBeaconDecodeResult syncBeaconDecodeV2(const uint8_t* data, size_t dataLen, uint8_t expectedGroup, NkSyncBeaconV2* beacon);
const char* syncBeaconDecodeResultName(SyncBeaconDecodeResult result);

uint16_t nkBeaconCrc16(const uint8_t* data, size_t len);
uint16_t computeBeaconCrc(const NkSyncBeaconV1& beacon);
uint16_t computeBeaconCrc(const NkSyncBeaconV2& beacon);
