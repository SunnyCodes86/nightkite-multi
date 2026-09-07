#pragma once

#include "protocol/ShowControlCodec.h"

constexpr uint8_t NK_SHOW_QUEUE_CAPACITY = 8;
constexpr uint8_t NK_SHOW_DEDUPE_CAPACITY = 64;
#ifndef NK_SHOW_RX_COMPENSATION_MS
#define NK_SHOW_RX_COMPENSATION_MS 0
#endif

struct ShowScheduledEvent
{
  ShowPacket packet;
  uint32_t dueMs = 0; // Frozen local deadline, independent of later clock samples.
};

struct ShowSchedulerStatus
{
  uint32_t accepted = 0, duplicates = 0, stale = 0, full = 0, late = 0, droppedLate = 0;
  uint32_t clockSamples = 0;
  uint32_t lastDueMs = 0, lastLatenessMs = 0;
};

class ShowScheduler
{
public:
  bool receive(const ShowPacket& packet, uint32_t receiveMs, uint32_t nowMs);
  bool popDue(uint32_t nowMs, ShowScheduledEvent& event);
  void reset();
  uint8_t depth() const { return count; }
  bool clockValid() const { return clockReady; }
  int32_t clockOffsetMs() const { return showTimeDelta(offsetMs, 0); }
  uint32_t clockAgeMs(uint32_t nowMs) const { return clockReady ? nowMs - lastClockRxMs : 0; }
  ShowSchedulerStatus status;
private:
  void sampleClock(uint32_t senderMs, uint32_t receiveMs, bool clockPacket);
  bool seen(uint16_t id) const;
  void remember(uint16_t id);
  ShowScheduledEvent queue[NK_SHOW_QUEUE_CAPACITY];
  uint8_t count = 0;
  bool clockReady = false;
  uint32_t offsetMs = 0, lastSenderMs = 0, lastClockRxMs = 0, sampleWindowMs = 0;
  uint16_t recentIds[NK_SHOW_DEDUPE_CAPACITY] = {};
  uint8_t recentCount = 0, nextRecent = 0;
};

struct ShowRgb { uint8_t r = 0, g = 0, b = 0; };
enum class ShowOutput : uint8_t { UNDERLYING = 0, PATTERN = 1, SOLID = 2, BLACKOUT = 3, BUFFER = 4 };

class ShowState
{
public:
  bool apply(const ShowPacket& packet, uint32_t dueMs, uint8_t localBrightness, uint8_t ledCount);
  void release();
  bool active() const { return output != ShowOutput::UNDERLYING || brightness != 0; }
  bool controlsOutput() const { return output != ShowOutput::UNDERLYING; }
  uint8_t selectedPattern(uint8_t underlying) const { return output == ShowOutput::PATTERN ? pattern : underlying; }
  uint8_t selectedBrightness(uint8_t underlying) const { return brightness ? brightness : underlying; }
  bool pendingComplete() const;
  ShowRgb pixel(uint8_t index) const;
  ShowOutput output = ShowOutput::UNDERLYING;
  uint8_t pattern = 1, brightness = 0;
  uint32_t patternStartMs = 0, revision = 0;
  bool pendingValid = false;
  uint8_t pendingId = 0, pendingSegments = 0, pendingLedCount = 0;
  uint32_t receivedSegments = 0;
  uint32_t executed = 0, rejected = 0;
  uint16_t lastEvent = 0;
  uint8_t lastCommand = 0;
private:
  ShowRgb solid;
  ShowRgb pending[NK_SHOW_MAX_LEDS];
  ShowRgb front[NK_SHOW_MAX_LEDS];
};
