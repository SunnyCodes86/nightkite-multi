#include "ShowControlCodec.h"
#include "SyncBeaconCodec.h"
#include <string.h>

int32_t showTimeDelta(uint32_t a, uint32_t b)
{
  const uint32_t delta = a - b;
  return delta <= INT32_MAX ? (int32_t)delta : -1 - (int32_t)(UINT32_MAX - delta);
}

uint32_t showExecuteTime(const ShowPacket& packet)
{
  const uint16_t delta = packet.executeMs - (uint16_t)packet.senderMs;
  const int32_t signedDelta = delta <= INT16_MAX ? delta : (int32_t)delta - 65536;
  return packet.senderMs + signedDelta;
}

ShowDecodeResult validateShowPacket(const ShowPacket& p)
{
  if ((p.targetKind == ShowTarget::ALL && p.target != 0) ||
      (p.targetKind == ShowTarget::GROUP && (p.target == 0 || p.target > 255)) ||
      (p.targetKind == ShowTarget::SINGLE && p.target > 0xFFFFFF) ||
      (uint8_t)p.targetKind > (uint8_t)ShowTarget::SINGLE) return ShowDecodeResult::TARGET;
  const int32_t ahead = showTimeDelta(showExecuteTime(p), p.senderMs);
  if (ahead < -NK_SHOW_MAX_LATE_MS || ahead > NK_SHOW_MAX_AHEAD_MS) return ShowDecodeResult::TIME;
  uint8_t used = 0;
  switch (p.command)
  {
    case ShowCommand::SET_PATTERN:
      if (p.params[0] < NK_PATTERN_MIN_ID || p.params[0] > NK_PATTERN_MAX_ID) return ShowDecodeResult::COMMAND;
      used = 1;
      break;
    case ShowCommand::SET_BRIGHTNESS:
      if (p.params[0] == 0) return ShowDecodeResult::COMMAND;
      used = 1;
      break;
    case ShowCommand::SET_SOLID: used = 4; break; // RGB, brightness (0 = retain).
    case ShowCommand::BLACKOUT:
    case ShowCommand::RELEASE: break;
    case ShowCommand::CLEAR_PENDING:
      if (p.params[1] > 32) return ShowDecodeResult::COMMAND;
      used = 2; // Image ID, number of segments.
      break;
    case ShowCommand::SET_SEGMENT:
      if (p.params[1] >= 32 || p.params[3] == 0 ||
          (uint16_t)p.params[2] + p.params[3] > NK_SHOW_MAX_LEDS) return ShowDecodeResult::COMMAND;
      used = 7; // Image ID, segment index, start, count, RGB.
      break;
    case ShowCommand::APPLY_PENDING: used = 1; break; // Image ID.
    case ShowCommand::CLOCK:
      if (p.eventId != 0 || p.executeMs != (uint16_t)p.senderMs || p.targetKind != ShowTarget::ALL)
        return ShowDecodeResult::COMMAND;
      break;
    default: return ShowDecodeResult::COMMAND;
  }
  for (uint8_t i = used; i < sizeof(p.params); ++i)
    if (p.params[i] != 0) return ShowDecodeResult::COMMAND;
  return ShowDecodeResult::OK;
}

bool showTargetMatches(const ShowPacket& p, uint8_t group, uint32_t shortId)
{
  switch (p.targetKind)
  {
    case ShowTarget::ALL: return p.target == 0;
    case ShowTarget::GROUP: return p.target == group && group != 0;
    case ShowTarget::SINGLE: return p.target == shortId && shortId <= 0xFFFFFF;
    default: return false;
  }
}

bool showControlEncode(const ShowPacket& p, uint8_t* output, size_t size)
{
  if (!output || size < NK_SHOW_PACKET_SIZE || validateShowPacket(p) != ShowDecodeResult::OK) return false;
  uint8_t bytes[NK_SHOW_PACKET_SIZE] = {'N', 'S', NK_SHOW_VERSION};
  bytes[3] = ((uint8_t)p.targetKind << 6) | (uint8_t)p.command;
  for (uint8_t i = 0; i < 3; ++i) bytes[4 + i] = p.target >> (8 * i);
  bytes[7] = p.eventId;
  bytes[8] = p.eventId >> 8;
  for (uint8_t i = 0; i < 4; ++i) bytes[9 + i] = p.senderMs >> (8 * i);
  bytes[13] = p.executeMs;
  bytes[14] = p.executeMs >> 8;
  memcpy(bytes + 15, p.params, sizeof(p.params));
  const uint16_t crc = nkBeaconCrc16(bytes, sizeof(bytes));
  bytes[22] = crc;
  bytes[23] = crc >> 8;
  memcpy(output, bytes, sizeof(bytes));
  return true;
}

ShowDecodeResult showControlDecode(const uint8_t* data, size_t size, ShowPacket* output)
{
  if (!data || !output || size != NK_SHOW_PACKET_SIZE) return ShowDecodeResult::LENGTH;
  if (data[0] != 'N' || data[1] != 'S') return ShowDecodeResult::MAGIC;
  if (data[2] != NK_SHOW_VERSION) return ShowDecodeResult::VERSION;
  uint8_t bytes[NK_SHOW_PACKET_SIZE];
  memcpy(bytes, data, sizeof(bytes));
  bytes[22] = bytes[23] = 0;
  if (nkBeaconCrc16(bytes, sizeof(bytes)) != (uint16_t)(data[22] | (uint16_t)data[23] << 8)) return ShowDecodeResult::CRC;
  ShowPacket p;
  p.targetKind = (ShowTarget)(data[3] >> 6);
  p.command = (ShowCommand)(data[3] & 0x3F);
  for (uint8_t i = 0; i < 3; ++i) p.target |= (uint32_t)data[4 + i] << (8 * i);
  p.eventId = data[7] | (uint16_t)data[8] << 8;
  for (uint8_t i = 0; i < 4; ++i) p.senderMs |= (uint32_t)data[9 + i] << (8 * i);
  p.executeMs = data[13] | (uint16_t)data[14] << 8;
  memcpy(p.params, data + 15, sizeof(p.params));
  const ShowDecodeResult result = validateShowPacket(p);
  if (result == ShowDecodeResult::OK) *output = p;
  return result;
}

bool showControlEncodeAdvertisement(const ShowPacket& packet, uint8_t* output, size_t size)
{
  if (!output || size < NK_SHOW_ADV_SIZE) return false;
  const uint8_t prefix[] = {2, 1, 6, 27, 0xFF, 0xFF, 0xFF};
  if (!showControlEncode(packet, output + sizeof(prefix), size - sizeof(prefix))) return false;
  memcpy(output, prefix, sizeof(prefix));
  return true;
}
