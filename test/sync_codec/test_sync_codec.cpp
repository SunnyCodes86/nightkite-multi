#include <assert.h>
#include <string.h>
#include "protocol/SyncBeaconCodec.h"

int main()
{
  // Independent CRC-CCITT vectors, generated with Python binascii.crc_hqx(..., 0xFFFF).
  const uint8_t v1[] = {0x4e,0x4b,0x01,0x07,0x00,0xfa,0xff,0x1b,0x9f,0x4e,0x61,0xbc,0x00,0xe8,0x03,0x70,0xab};
  const uint8_t v2[] = {0x4e,0x4b,0x02,0x07,0x01,0x02,0x00,0x19,0xdf,0x31,0xd4,0x00,0x00,0xf4,0x01,0xc9,0xca,0xcb,0xcc,0xcd,0x70,0x6a};
  assert(NK_SYNC_BEACON_ADV_LEN == 24 && NK_SYNC_BEACON_V2_ADV_LEN == 29);
  assert(NK_SYNC_BEACON_V2_PACKET_SIZE == 22);
  assert((NK_SYNC_BEACON_FLAG_AUDIO_BEAT | NK_SYNC_BEACON_FLAG_AUDIO_SIGNAL_VALID |
          NK_SYNC_BEACON_FLAG_AUDIO_BEAT_LOCKED) == 0x07);
  NkSyncBeaconV1 a;
  NkSyncBeaconV2 b;
  assert(syncBeaconDecode(v1, sizeof(v1), 7, &a) == SYNC_BEACON_DECODE_OK);
  assert(a.seq == 65530 && a.pattern == 27 && a.brightness == 159 && a.phaseMs == 12345678);
  assert(syncBeaconDecodeV2(v2, sizeof(v2), 7, &b) == SYNC_BEACON_DECODE_OK);
  assert(b.seq == 2 && b.pattern == 25 && b.phaseMs == 54321 && b.beatMs == 500);
  assert(b.audioEnergy == 201 && b.audioBass == 202 && b.audioMid == 203 && b.audioTreble == 204 && b.audioConfidence == 205);
  assert(computeBeaconCrc(a) == 0xAB70 && computeBeaconCrc(b) == 0x6A70);
  b.flags = NK_SYNC_BEACON_FLAG_AUDIO_BEAT | NK_SYNC_BEACON_FLAG_AUDIO_SIGNAL_VALID |
            NK_SYNC_BEACON_FLAG_AUDIO_BEAT_LOCKED;
  b.crc = computeBeaconCrc(b);
  NkSyncBeaconV2 flagged;
  assert(syncBeaconDecodeV2((uint8_t*)&b, sizeof(b), 7, &flagged) == SYNC_BEACON_DECODE_OK);
  assert(flagged.flags == 0x07 && computeBeaconCrc(flagged) == b.crc);
  uint8_t encoded[17];
  size_t length = 0;
  assert(syncBeaconEncode(a, encoded, sizeof(encoded), &length) && length == sizeof(v1));
  assert(memcmp(v1, encoded, sizeof(v1)) == 0);
  assert(syncBeaconDecode(v1, 16, 7, &a) == SYNC_BEACON_DECODE_TOO_SHORT);
  assert(syncBeaconDecodeV2(v2, 21, 7, &b) == SYNC_BEACON_DECODE_TOO_SHORT);
  assert(syncBeaconDecode(v1, sizeof(v1), 8, &a) == SYNC_BEACON_DECODE_BAD_GROUP);
  assert(syncBeaconDecodeV2(v2, sizeof(v2), 8, &b) == SYNC_BEACON_DECODE_BAD_GROUP);
  assert(syncBeaconDecode(v2, sizeof(v2), 7, &a) == SYNC_BEACON_DECODE_BAD_VERSION);
  assert(syncBeaconDecodeV2(v1, sizeof(v1), 7, &b) == SYNC_BEACON_DECODE_TOO_SHORT);
  for (uint8_t pattern = 0; pattern <= 28; ++pattern)
  {
    a.pattern = b.pattern = pattern;
    a.crc = computeBeaconCrc(a); b.crc = computeBeaconCrc(b);
    const auto expected = pattern >= 1 && pattern <= 27 ? SYNC_BEACON_DECODE_OK : SYNC_BEACON_DECODE_BAD_PATTERN;
    NkSyncBeaconV1 out1; NkSyncBeaconV2 out2;
    assert(syncBeaconDecode((uint8_t*)&a, sizeof(a), 7, &out1) == expected);
    assert(syncBeaconDecodeV2((uint8_t*)&b, sizeof(b), 7, &out2) == expected);
  }
  uint8_t damaged[sizeof(v2)];
  memcpy(damaged, v2, sizeof(v2)); damaged[15] ^= 1;
  assert(syncBeaconDecodeV2(damaged, sizeof(damaged), 7, &b) == SYNC_BEACON_DECODE_BAD_CRC);
  memcpy(damaged, v1, sizeof(v1)); damaged[9] ^= 1;
  assert(syncBeaconDecode(damaged, sizeof(v1), 7, &a) == SYNC_BEACON_DECODE_BAD_CRC);
}
