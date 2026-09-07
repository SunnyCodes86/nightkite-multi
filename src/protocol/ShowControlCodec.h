#pragma once

#include <stddef.h>
#include <stdint.h>

// See docs/show-control-v1.md. This contract is independent of NK sync V1/V2.
constexpr size_t NK_SHOW_PACKET_SIZE = 24;
constexpr size_t NK_SHOW_ADV_SIZE = 7 + NK_SHOW_PACKET_SIZE;
constexpr uint8_t NK_SHOW_VERSION = 1;
constexpr uint8_t NK_SHOW_MAX_LEDS = 70;
constexpr int32_t NK_SHOW_MAX_AHEAD_MS = 30000;
constexpr int32_t NK_SHOW_MAX_LATE_MS = 250;
static_assert(NK_SHOW_ADV_SIZE == 31, "Show packet must fit legacy advertising");

enum class ShowTarget : uint8_t { ALL = 0, GROUP = 1, SINGLE = 2 };
enum class ShowCommand : uint8_t
{
  SET_PATTERN = 1, SET_BRIGHTNESS = 2, SET_SOLID = 3, BLACKOUT = 4,
  RELEASE = 5, CLEAR_PENDING = 6, SET_SEGMENT = 7, APPLY_PENDING = 8, CLOCK = 9
};

struct ShowPacket
{
  ShowTarget targetKind = ShowTarget::ALL;
  uint32_t target = 0; // 24-bit short_id, or group 1..255, or zero for ALL.
  ShowCommand command = ShowCommand::BLACKOUT;
  uint16_t eventId = 0;
  uint32_t senderMs = 0;
  uint16_t executeMs = 0; // Low 16 bits of the absolute sender execution time.
  uint8_t params[7] = {};
};

enum class ShowDecodeResult : uint8_t { OK, LENGTH, MAGIC, VERSION, CRC, TARGET, COMMAND, TIME };

int32_t showTimeDelta(uint32_t a, uint32_t b);
uint32_t showExecuteTime(const ShowPacket& packet);
ShowDecodeResult validateShowPacket(const ShowPacket& packet);
bool showTargetMatches(const ShowPacket& packet, uint8_t group, uint32_t shortId);
bool showControlEncode(const ShowPacket& packet, uint8_t* output, size_t size);
ShowDecodeResult showControlDecode(const uint8_t* data, size_t size, ShowPacket* output);
bool showControlEncodeAdvertisement(const ShowPacket& packet, uint8_t* output, size_t size);
