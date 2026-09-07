#include "ShowControl.h"
#include <string.h>

namespace
{
int32_t sequenceDelta(uint16_t a, uint16_t b)
{
  const uint16_t delta = a - b;
  return delta <= INT16_MAX ? delta : (int32_t)delta - 65536;
}
}

void ShowScheduler::reset()
{
  count = 0;
  clockReady = false;
  recentCount = nextRecent = 0;
}

void ShowScheduler::sampleClock(uint32_t senderMs, uint32_t receiveMs, bool clockPacket)
{
  // Repeated controller advertisements carry stale timestamps: never re-sample them.
  if (clockReady && senderMs == lastSenderMs) return;
  // A replacement gateway may restart its clock. Fresh CLOCK warmup can rebase
  // after 2 s without forward samples; queued deadlines remain frozen.
  const bool restart = clockPacket && clockReady && receiveMs - lastClockRxMs >= 2000;
  if (clockReady && showTimeDelta(senderMs, lastSenderMs) <= 0 && !restart) return;
  const uint32_t sample = receiveMs - senderMs - NK_SHOW_RX_COMPENSATION_MS;
  // ponytail: minimum receive latency in a 2 s window; hardware timestamping if measured jitter requires it.
  if (!clockReady || restart || receiveMs - sampleWindowMs >= 2000)
  {
    offsetMs = sample;
    sampleWindowMs = receiveMs;
  }
  else if (showTimeDelta(sample, offsetMs) < 0) offsetMs = sample;
  clockReady = true;
  lastSenderMs = senderMs;
  lastClockRxMs = receiveMs;
  ++status.clockSamples;
}

bool ShowScheduler::seen(uint16_t id) const
{
  // Equality only: IDs are not a monotonic sender identity. Queued events stay
  // protected even if more than 64 other events pass through the recent cache.
  for (uint8_t i = 0; i < count; ++i) if (queue[i].packet.eventId == id) return true;
  for (uint8_t i = 0; i < recentCount; ++i) if (recentIds[i] == id) return true;
  return false;
}

void ShowScheduler::remember(uint16_t id)
{
  recentIds[nextRecent] = id;
  nextRecent = (nextRecent + 1) % NK_SHOW_DEDUPE_CAPACITY;
  if (recentCount < NK_SHOW_DEDUPE_CAPACITY) ++recentCount;
}

bool ShowScheduler::receive(const ShowPacket& packet, uint32_t receiveMs, uint32_t nowMs)
{
  if (validateShowPacket(packet) != ShowDecodeResult::OK) return false;
  sampleClock(packet.senderMs, receiveMs, packet.command == ShowCommand::CLOCK);
  if (packet.command == ShowCommand::CLOCK) return true;
  if (seen(packet.eventId))
  {
    ++status.duplicates;
    return false;
  }
  const uint32_t due = showExecuteTime(packet) + offsetMs;
  const int32_t ahead = showTimeDelta(due, nowMs);
  if (ahead < -NK_SHOW_MAX_LATE_MS)
  {
    remember(packet.eventId);
    ++status.late;
    ++status.droppedLate;
    return false;
  }
  if (ahead > NK_SHOW_MAX_AHEAD_MS)
  {
    ++status.stale; // Timing rejection after clock mapping; not an ID-age rule.
    return false;
  }
  if (count == NK_SHOW_QUEUE_CAPACITY)
  {
    ++status.full; // No eviction, and no dedupe mark: a later retry may fit.
    return false;
  }
  ShowScheduledEvent event;
  event.packet = packet;
  event.dueMs = due;
  uint8_t i = count;
  while (i > 0 && (showTimeDelta(due, queue[i - 1].dueMs) < 0 ||
      (due == queue[i - 1].dueMs && sequenceDelta(packet.eventId, queue[i - 1].packet.eventId) < 0)))
  {
    queue[i] = queue[i - 1];
    --i;
  }
  queue[i] = event;
  ++count;
  remember(packet.eventId);
  ++status.accepted;
  return true;
}

bool ShowScheduler::popDue(uint32_t nowMs, ShowScheduledEvent& event)
{
  while (count && showTimeDelta(nowMs, queue[0].dueMs) >= 0)
  {
    event = queue[0];
    for (uint8_t i = 1; i < count; ++i) queue[i - 1] = queue[i];
    --count;
    status.lastDueMs = event.dueMs;
    status.lastLatenessMs = nowMs - event.dueMs;
    if (status.lastLatenessMs) ++status.late;
    if (status.lastLatenessMs > (uint32_t)NK_SHOW_MAX_LATE_MS)
    {
      ++status.droppedLate;
      continue;
    }
    return true;
  }
  return false;
}

bool ShowState::pendingComplete() const
{
  const uint32_t expected = pendingSegments == 32 ? UINT32_MAX : (uint32_t(1) << pendingSegments) - 1;
  return pendingValid && receivedSegments == expected;
}

void ShowState::release()
{
  output = ShowOutput::UNDERLYING;
  brightness = 0;
  pendingValid = false;
  receivedSegments = 0;
  ++revision;
}

bool ShowState::apply(const ShowPacket& p, uint32_t dueMs, uint8_t localBrightness, uint8_t ledCount)
{
  lastEvent = p.eventId;
  lastCommand = (uint8_t)p.command;
  if (validateShowPacket(p) != ShowDecodeResult::OK || ledCount == 0 || ledCount > NK_SHOW_MAX_LEDS)
  {
    ++rejected;
    return false;
  }
  const auto reject = [this]() { ++rejected; return false; };
  switch (p.command)
  {
    case ShowCommand::SET_PATTERN:
      output = ShowOutput::PATTERN;
      pattern = p.params[0];
      patternStartMs = dueMs;
      ++revision;
      break;
    case ShowCommand::SET_BRIGHTNESS: brightness = p.params[0]; break;
    case ShowCommand::SET_SOLID:
      output = ShowOutput::SOLID;
      solid.r = p.params[0]; solid.g = p.params[1]; solid.b = p.params[2];
      if (p.params[3]) brightness = p.params[3];
      break;
    case ShowCommand::BLACKOUT: output = ShowOutput::BLACKOUT; break;
    case ShowCommand::RELEASE: release(); break;
    case ShowCommand::CLEAR_PENDING:
      pendingValid = true;
      pendingId = p.params[0]; pendingSegments = p.params[1]; pendingLedCount = ledCount;
      receivedSegments = 0;
      for (auto& rgb : pending) rgb = ShowRgb{};
      break;
    case ShowCommand::SET_SEGMENT:
      if (!pendingValid || pendingId != p.params[0] || pendingLedCount != ledCount ||
          p.params[1] >= pendingSegments || (uint16_t)p.params[2] + p.params[3] > ledCount ||
          (receivedSegments & (uint32_t(1) << p.params[1]))) return reject();
      for (uint8_t i = p.params[2]; i < p.params[2] + p.params[3]; ++i)
      {
        pending[i].r = p.params[4]; pending[i].g = p.params[5]; pending[i].b = p.params[6];
      }
      receivedSegments |= uint32_t(1) << p.params[1];
      break;
    case ShowCommand::APPLY_PENDING:
      if (!pendingComplete() || p.params[0] != pendingId || pendingLedCount != ledCount) return reject();
      memcpy(front, pending, sizeof(front));
      output = ShowOutput::BUFFER;
      break;
    case ShowCommand::CLOCK: return true;
  }
  if (controlsOutput() && brightness == 0) brightness = localBrightness;
  ++executed;
  return true;
}

ShowRgb ShowState::pixel(uint8_t index) const
{
  if (output == ShowOutput::SOLID) return solid;
  if (output == ShowOutput::BUFFER && index < NK_SHOW_MAX_LEDS) return front[index];
  return ShowRgb{};
}
