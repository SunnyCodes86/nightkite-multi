#include "SyncBeaconCodec.h"
#include <string.h>

bool isValidBeaconPattern(uint8_t pattern)
{
  return pattern >= NK_PATTERN_MIN_ID && pattern <= NK_PATTERN_MAX_ID;
}

bool isValidBeaconBrightness(uint8_t brightness)
{
  switch (brightness)
  {
    case 95:
    case 127:
    case 159:
    case 191:
    case 223:
    case 255:
      return true;
    default:
      return false;
  }
}

uint16_t nkBeaconCrc16(const uint8_t* data, size_t len)
{
  uint16_t crc = 0xFFFF;
  for (size_t i = 0; i < len; i++)
  {
    crc ^= (uint16_t)data[i] << 8;
    for (uint8_t bit = 0; bit < 8; bit++)
    {
      crc = (crc & 0x8000) ? (uint16_t)((crc << 1) ^ 0x1021) : (uint16_t)(crc << 1);
    }
  }
  return crc;
}

uint16_t computeBeaconCrc(const NkSyncBeaconV1& beacon)
{
  uint8_t bytes[NK_SYNC_BEACON_V1_PACKET_SIZE] = {0};
  bytes[0] = beacon.magic0;
  bytes[1] = beacon.magic1;
  bytes[2] = beacon.version;
  bytes[3] = beacon.groupId;
  bytes[4] = beacon.flags;
  bytes[5] = (uint8_t)(beacon.seq & 0xFF);
  bytes[6] = (uint8_t)(beacon.seq >> 8);
  bytes[7] = beacon.pattern;
  bytes[8] = beacon.brightness;
  bytes[9] = (uint8_t)(beacon.phaseMs & 0xFF);
  bytes[10] = (uint8_t)((beacon.phaseMs >> 8) & 0xFF);
  bytes[11] = (uint8_t)((beacon.phaseMs >> 16) & 0xFF);
  bytes[12] = (uint8_t)(beacon.phaseMs >> 24);
  bytes[13] = (uint8_t)(beacon.beatMs & 0xFF);
  bytes[14] = (uint8_t)(beacon.beatMs >> 8);
  return nkBeaconCrc16(bytes, sizeof(bytes));
}

uint16_t computeBeaconCrc(const NkSyncBeaconV2& beacon)
{
  uint8_t bytes[NK_SYNC_BEACON_V2_PACKET_SIZE] = {0};
  bytes[0] = beacon.magic0;
  bytes[1] = beacon.magic1;
  bytes[2] = beacon.version;
  bytes[3] = beacon.groupId;
  bytes[4] = beacon.flags;
  bytes[5] = (uint8_t)(beacon.seq & 0xFF);
  bytes[6] = (uint8_t)(beacon.seq >> 8);
  bytes[7] = beacon.pattern;
  bytes[8] = beacon.brightness;
  bytes[9] = (uint8_t)(beacon.phaseMs & 0xFF);
  bytes[10] = (uint8_t)((beacon.phaseMs >> 8) & 0xFF);
  bytes[11] = (uint8_t)((beacon.phaseMs >> 16) & 0xFF);
  bytes[12] = (uint8_t)(beacon.phaseMs >> 24);
  bytes[13] = (uint8_t)(beacon.beatMs & 0xFF);
  bytes[14] = (uint8_t)(beacon.beatMs >> 8);
  bytes[15] = beacon.audioEnergy;
  bytes[16] = beacon.audioBass;
  bytes[17] = beacon.audioMid;
  bytes[18] = beacon.audioTreble;
  bytes[19] = beacon.audioConfidence;
  return nkBeaconCrc16(bytes, sizeof(bytes));
}

bool syncBeaconEncode(const NkSyncBeaconV1& beacon, uint8_t* output, size_t outputSize, size_t* outputLen)
{
  if (output == nullptr || outputLen == nullptr || outputSize < NK_SYNC_BEACON_PACKET_SIZE)
  {
    return false;
  }
  NkSyncBeaconV1 copy = beacon;
  copy.magic0 = NK_SYNC_BEACON_MAGIC0;
  copy.magic1 = NK_SYNC_BEACON_MAGIC1;
  copy.version = NK_SYNC_BEACON_VERSION;
  copy.crc = computeBeaconCrc(copy);
  memcpy(output, &copy, NK_SYNC_BEACON_PACKET_SIZE);
  *outputLen = NK_SYNC_BEACON_PACKET_SIZE;
  return true;
}

SyncBeaconDecodeResult syncBeaconDecode(const uint8_t* data, size_t dataLen, uint8_t expectedGroup, NkSyncBeaconV1* beacon)
{
  if (data == nullptr || beacon == nullptr || dataLen < NK_SYNC_BEACON_PACKET_SIZE)
  {
    return SYNC_BEACON_DECODE_TOO_SHORT;
  }

  NkSyncBeaconV1 decoded;
  memcpy(&decoded, data, NK_SYNC_BEACON_PACKET_SIZE);
  if (decoded.magic0 != NK_SYNC_BEACON_MAGIC0 || decoded.magic1 != NK_SYNC_BEACON_MAGIC1)
  {
    return SYNC_BEACON_DECODE_BAD_MAGIC;
  }
  if (decoded.version != NK_SYNC_BEACON_VERSION)
  {
    return SYNC_BEACON_DECODE_BAD_VERSION;
  }
  if (expectedGroup != 0 && decoded.groupId != expectedGroup)
  {
    return SYNC_BEACON_DECODE_BAD_GROUP;
  }
  if (!isValidBeaconPattern(decoded.pattern))
  {
    return SYNC_BEACON_DECODE_BAD_PATTERN;
  }
  if (!isValidBeaconBrightness(decoded.brightness))
  {
    return SYNC_BEACON_DECODE_BAD_BRIGHTNESS;
  }
  if (decoded.crc != computeBeaconCrc(decoded))
  {
    return SYNC_BEACON_DECODE_BAD_CRC;
  }

  *beacon = decoded;
  return SYNC_BEACON_DECODE_OK;
}

SyncBeaconDecodeResult syncBeaconDecodeV2(const uint8_t* data, size_t dataLen, uint8_t expectedGroup, NkSyncBeaconV2* beacon)
{
  if (data == nullptr || beacon == nullptr || dataLen < NK_SYNC_BEACON_V2_PACKET_SIZE)
  {
    return SYNC_BEACON_DECODE_TOO_SHORT;
  }

  NkSyncBeaconV2 decoded;
  memcpy(&decoded, data, NK_SYNC_BEACON_V2_PACKET_SIZE);
  if (decoded.magic0 != NK_SYNC_BEACON_MAGIC0 || decoded.magic1 != NK_SYNC_BEACON_MAGIC1)
  {
    return SYNC_BEACON_DECODE_BAD_MAGIC;
  }
  if (decoded.version != NK_SYNC_BEACON_VERSION_V2)
  {
    return SYNC_BEACON_DECODE_BAD_VERSION;
  }
  if (expectedGroup != 0 && decoded.groupId != expectedGroup)
  {
    return SYNC_BEACON_DECODE_BAD_GROUP;
  }
  if (!isValidBeaconPattern(decoded.pattern))
  {
    return SYNC_BEACON_DECODE_BAD_PATTERN;
  }
  if (!isValidBeaconBrightness(decoded.brightness))
  {
    return SYNC_BEACON_DECODE_BAD_BRIGHTNESS;
  }
  if (decoded.crc != computeBeaconCrc(decoded))
  {
    return SYNC_BEACON_DECODE_BAD_CRC;
  }

  *beacon = decoded;
  return SYNC_BEACON_DECODE_OK;
}

const char* syncBeaconDecodeResultName(SyncBeaconDecodeResult result)
{
  switch (result)
  {
    case SYNC_BEACON_DECODE_OK: return "ok";
    case SYNC_BEACON_DECODE_TOO_SHORT: return "too_short";
    case SYNC_BEACON_DECODE_BAD_MAGIC: return "bad_magic";
    case SYNC_BEACON_DECODE_BAD_VERSION: return "bad_version";
    case SYNC_BEACON_DECODE_BAD_GROUP: return "bad_group";
    case SYNC_BEACON_DECODE_BAD_PATTERN: return "bad_pattern";
    case SYNC_BEACON_DECODE_BAD_BRIGHTNESS: return "bad_brightness";
    case SYNC_BEACON_DECODE_BAD_CRC: return "bad_crc";
    default: return "invalid";
  }
}
