#include <assert.h>
#include <string.h>
#include <initializer_list>
#include "app/ShowControl.h"
#include "app/AudioPatternMath.h"
#include "app/Battery.h"
#include "app/SyncEngine.h"
#include "app/PatternClock.h"

uint32_t hostMillis = 0;

static ShowPacket command(ShowCommand cmd, uint16_t id = 1, uint32_t sender = 1000, uint32_t due = 1100)
{
  ShowPacket p;
  p.command = cmd; p.eventId = id; p.senderMs = sender; p.executeMs = due;
  return p;
}

static void testCodec()
{
  auto p = command(ShowCommand::SET_SEGMENT, 0x1234, 0x12345678, 0x12345A00);
  p.targetKind = ShowTarget::SINGLE; p.target = 0xABC123;
  const uint8_t params[] = {42, 1, 15, 10, 0, 0, 255};
  memcpy(p.params, params, 7);
  uint8_t adv[NK_SHOW_ADV_SIZE];
  assert(showControlEncodeAdvertisement(p, adv, sizeof(adv)));
  assert(sizeof(adv) == 31 && adv[3] == 27 && adv[4] == 255);
  assert(adv[5] == 255 && adv[6] == 255);
  auto* bytes = adv + 7;
  const uint8_t golden[] = {0x4e,0x53,0x01,0x87,0x23,0xc1,0xab,0x34,0x12,0x78,0x56,0x34,0x12,0x00,0x5a,0x2a,0x01,0x0f,0x0a,0x00,0x00,0xff,0x73,0x85};
  assert(memcmp(bytes, golden, sizeof(golden)) == 0);
  assert(bytes[0] == 'N' && bytes[1] == 'S' && bytes[2] == 1 && bytes[3] == 0x87);
  assert(bytes[4] == 0x23 && bytes[5] == 0xC1 && bytes[6] == 0xAB);
  assert(bytes[7] == 0x34 && bytes[8] == 0x12 && bytes[9] == 0x78 && bytes[12] == 0x12);
  ShowPacket decoded;
  assert(showControlDecode(bytes, 24, &decoded) == ShowDecodeResult::OK);
  assert(decoded.target == p.target && decoded.eventId == p.eventId && decoded.senderMs == p.senderMs);
  assert(decoded.executeMs == p.executeMs && memcmp(decoded.params, params, 7) == 0);
  assert(showTargetMatches(decoded, 1, 0xABC123));
  assert(!showTargetMatches(decoded, 1, 0x123ABC));
  assert(showControlDecode(bytes, 23, &decoded) == ShowDecodeResult::LENGTH);
  assert(showControlDecode(bytes, 25, &decoded) == ShowDecodeResult::LENGTH);
  assert(showControlDecode(nullptr, 24, &decoded) == ShowDecodeResult::LENGTH);
  assert(!showControlEncode(p, nullptr, 24));
  assert(!showControlEncodeAdvertisement(p, adv, 30));
  for (size_t i = 3; i < 24; ++i)
  {
    bytes[i] ^= 1;
    assert(showControlDecode(bytes, 24, &decoded) == ShowDecodeResult::CRC);
    bytes[i] ^= 1;
  }
  bytes[0] = 'X';
  assert(showControlDecode(bytes, 24, &decoded) == ShowDecodeResult::MAGIC);
  bytes[0] = 'N'; bytes[2] = 2;
  assert(showControlDecode(bytes, 24, &decoded) == ShowDecodeResult::VERSION);
  p.targetKind = ShowTarget::GROUP; p.target = 7;
  assert(showTargetMatches(p, 7, 0) && !showTargetMatches(p, 8, 0));
  p.target = 256;
  assert(validateShowPacket(p) == ShowDecodeResult::TARGET);
  p.targetKind = ShowTarget::ALL; p.target = 0;
  assert(showTargetMatches(p, 99, 0x123456));
  p.target = 1;
  assert(validateShowPacket(p) == ShowDecodeResult::TARGET);
  p = command(ShowCommand::SET_PATTERN);
  for (uint8_t pattern = 1; pattern <= 27; ++pattern)
  {
    p.params[0] = pattern;
    assert(showControlEncode(p, bytes, 24));
    assert(showControlDecode(bytes, 24, &decoded) == ShowDecodeResult::OK && decoded.params[0] == pattern);
  }
  p.params[0] = 28;
  assert(validateShowPacket(p) == ShowDecodeResult::COMMAND);
  p.params[0] = 0;
  assert(validateShowPacket(p) == ShowDecodeResult::COMMAND);
  p = command(ShowCommand::SET_BRIGHTNESS);
  assert(validateShowPacket(p) == ShowDecodeResult::COMMAND);
  p.params[0] = 1; p.params[6] = 1;
  assert(validateShowPacket(p) == ShowDecodeResult::COMMAND);
  p = command((ShowCommand)63);
  assert(validateShowPacket(p) == ShowDecodeResult::COMMAND);
  p = command(ShowCommand::BLACKOUT, 1, 1000, 31001);
  assert(validateShowPacket(p) == ShowDecodeResult::TIME);
  p.executeMs = 749;
  assert(validateShowPacket(p) == ShowDecodeResult::TIME);
  p = command(ShowCommand::CLOCK, 0, 1234, 1234);
  assert(showControlEncode(p, bytes, 24));
}

static void testScheduler()
{
  ShowScheduler scheduler;
  ShowScheduledEvent event;
  auto p = command(ShowCommand::BLACKOUT);
  assert(scheduler.receive(p, 5000, 5000));
  for (uint32_t ms = 5001; ms < 5100; ms += 7) assert(!scheduler.receive(p, ms, ms));
  assert(scheduler.status.clockSamples == 1);
  assert(!scheduler.popDue(5099, event));
  assert(scheduler.popDue(5100, event) && event.dueMs == 5100);
  assert(!scheduler.popDue(5100, event));
  p.senderMs = 1101; // A refreshed transmission retains the same event/deadline.
  assert(!scheduler.receive(p, 5101, 5101));
  assert(scheduler.status.clockSamples == 2);
  scheduler.reset();
  for (uint16_t i = 1; i <= NK_SHOW_QUEUE_CAPACITY; ++i)
    assert(scheduler.receive(command(ShowCommand::BLACKOUT, i), 1000, 1000));
  assert(!scheduler.receive(command(ShowCommand::BLACKOUT, 9), 1000, 1000));
  assert(scheduler.depth() == 8 && scheduler.status.full == 1);
  assert(scheduler.popDue(1100, event) && event.packet.eventId == 1);
  assert(scheduler.receive(command(ShowCommand::BLACKOUT, 9), 1100, 1100));
  for (uint16_t i = 2; i <= 9; ++i)
    assert(scheduler.popDue(1100, event) && event.packet.eventId == i);
  scheduler.reset();
  assert(scheduler.receive(command(ShowCommand::BLACKOUT, 2), 1000, 1000));
  assert(scheduler.receive(command(ShowCommand::BLACKOUT, 1), 1000, 1000));
  assert(scheduler.popDue(1105, event) && event.packet.eventId == 1);
  assert(scheduler.popDue(1105, event) && event.packet.eventId == 2);
  assert(scheduler.status.lastLatenessMs == 5);
  scheduler.reset();
  assert(scheduler.receive(command(ShowCommand::BLACKOUT, 1), 1000, 1000));
  assert(!scheduler.popDue(1351, event)); // A stalled loop never plays obsolete events.
  assert(scheduler.status.droppedLate == 1);
  assert(!scheduler.receive(command(ShowCommand::BLACKOUT, 2), 1400, 1400));
  scheduler.reset();
  assert(scheduler.receive(command(ShowCommand::BLACKOUT, 65535, UINT32_MAX - 49, 50), 10, 10));
  assert(scheduler.receive(command(ShowCommand::BLACKOUT, 0, UINT32_MAX - 49, 50), 10, 10));
  assert(!scheduler.popDue(109, event));
  assert(scheduler.popDue(110, event) && event.packet.eventId == 65535);
  assert(scheduler.popDue(110, event) && event.packet.eventId == 0);
  assert(!scheduler.receive(command(ShowCommand::BLACKOUT, 65535, UINT32_MAX - 49, 50), 110, 110));
  assert(scheduler.receive(command(ShowCommand::BLACKOUT, 1, 51, 100), 111, 111));
  assert(scheduler.popDue(160, event) && event.packet.eventId == 1);
  scheduler.reset();
  assert(scheduler.receive(command(ShowCommand::BLACKOUT, 1, 1000, 1100), UINT32_MAX - 49, UINT32_MAX - 49));
  assert(!scheduler.popDue(49, event));
  assert(scheduler.popDue(50, event));
  // Only recent equality is deduplicated, never numeric age.
  scheduler.reset();
  for (uint16_t id = 1; id < 100; ++id)
  {
    assert(scheduler.receive(command(ShowCommand::BLACKOUT, id, 1000, 1000), 1000, 1000));
    assert(scheduler.popDue(1000, event));
  }
  assert(scheduler.receive(command(ShowCommand::BLACKOUT, 1, 1000, 1000), 1001, 1001));
  assert(scheduler.popDue(1001, event));
  assert(scheduler.receive(command(ShowCommand::BLACKOUT, 50000, 1002, 1002), 1002, 1002));
  assert(scheduler.popDue(1002, event));
  assert(scheduler.receive(command(ShowCommand::BLACKOUT, 200, 1003, 1003), 1003, 1003));
  assert(scheduler.popDue(1003, event)); // Reboot/replacement with smaller arbitrary ID.
  assert(!scheduler.receive(command(ShowCommand::BLACKOUT, 200, 1003, 1003), 1004, 1004));
  // A long-lookahead event stays deduplicated even after cache eviction.
  scheduler.reset();
  assert(scheduler.receive(command(ShowCommand::BLACKOUT, 500, 1000, 20000), 1000, 1000));
  for (uint16_t id = 1; id <= 65; ++id) {
    assert(scheduler.receive(command(ShowCommand::BLACKOUT, id, 1000, 1000), 1000, 1000));
    assert(scheduler.popDue(1000, event));
  }
  assert(!scheduler.receive(command(ShowCommand::BLACKOUT, 500, 1000, 20000), 1000, 1000));
  assert(scheduler.depth() == 1);
  // Fresh CLOCK warmup recovers from a sender clock restart, no receiver reset.
  scheduler.reset();
  assert(scheduler.receive(command(ShowCommand::CLOCK, 0, 100000, 100000), 1000, 1000));
  assert(scheduler.receive(command(ShowCommand::CLOCK, 0, 100, 100), 1100, 1100));
  assert(scheduler.clockAgeMs(1100) == 100);
  assert(scheduler.receive(command(ShowCommand::CLOCK, 0, 2100, 2100), 3100, 3100));
  assert(scheduler.clockOffsetMs() == 1000);
  assert(scheduler.receive(command(ShowCommand::BLACKOUT, 17, 2200, 2300), 3200, 3200));
  assert(scheduler.popDue(3300, event));
  assert(!scheduler.popDue(3300, event));
}

static void testTimeReconstruction()
{
  const int32_t deltas[] = {-251, -250, -1, 0, 1, 30000, 30001, 32767, -32768};
  for (uint32_t low = 0; low <= 65535; ++low) {
    for (int32_t delta : deltas) {
      const uint32_t sender = 0xFFFF0000UL + low;
      auto p = command(ShowCommand::BLACKOUT, 1, sender, sender + delta);
      assert(showExecuteTime(p) == sender + delta);
      assert((validateShowPacket(p) == ShowDecodeResult::OK) == (delta >= -250 && delta <= 30000));
    }
  }
}

static void testSharedClock()
{
  ShowScheduler a, c;
  auto clock = command(ShowCommand::CLOCK, 0, 1000, 1000);
  assert(a.receive(clock, 10000, 10000));
  assert(c.receive(clock, 20000, 20000));
  auto p = command(ShowCommand::SET_PATTERN, 1, 1050, 1200);
  p.params[0] = 6;
  assert(a.receive(p, 10053, 10053));
  assert(c.receive(p, 20061, 20061)); // Different reception delay, same show deadline.
  ShowScheduledEvent event;
  assert(!a.popDue(10199, event) && !c.popDue(20199, event));
  assert(a.popDue(10200, event) && c.popDue(20200, event));
}

static void testStateAndCoexistence()
{
  ShowState show;
  SyncEngine sync;
  PatternClock baseClock;
  AudioPatternFilter audioFilter;
  NkSyncBeaconV1 beacon = {};
  beacon.groupId = 7; beacon.pattern = 6; beacon.brightness = 127; beacon.seq = 500;
  assert(sync.follow(beacon, 7, 1000, 0));
  auto p = command(ShowCommand::SET_PATTERN);
  p.params[0] = 25;
  assert(show.apply(p, 1100, 127, 50));
  assert(show.active() && show.selectedPattern(sync.armedPattern) == 25 && show.patternStartMs == 1100);
  const uint32_t localMask = 1; // Master's pattern is disabled locally.
  for (uint8_t pattern = 1; pattern <= 27; ++pattern)
  {
    beacon.pattern = pattern; beacon.phaseMs = 80; beacon.seq = pattern;
    assert(sync.follow(beacon, 7, 1200, 0)); // Lower sequence from another master is accepted.
    assert(sync.armedPattern == pattern);
    assert(show.selectedPattern(sync.armedPattern) == 25);
    assert(!syncAutoplayAllowed(true, 2, true, 2));
  }
  assert(localMask == 1);
  assert(syncAutoplayAllowed(true, 2, true, 1)); // Autonomous master autoplay.
  assert(syncAutoplayAllowed(true, 1, false, 0));
  assert(!syncAutoplayAllowed(false, 1, true, 1));
  assert(!syncAutoplayAllowed(true, 0, true, 1));
  beacon.pattern = 28;
  assert(!sync.follow(beacon, 7, 1200, 0));
  beacon.pattern = 0;
  assert(!sync.follow(beacon, 7, 1200, 0));
  beacon.pattern = 12; beacon.brightness = 223; beacon.phaseMs = 7000;
  assert(sync.follow(beacon, 7, 1200, 0));
  baseClock.setPhase(beacon.phaseMs);
  AudioSyncState audio;
  audio.valid = true; audio.lastUpdateMs = 1200; audio.energy = 231; audio.beatMs = 500;
  assert(audioFilter.update(audio, 1200).energy == 231);
  p = command(ShowCommand::SET_SOLID);
  p.params[0] = 255; p.params[1] = 255; p.params[2] = 255; p.params[3] = 240;
  assert(show.apply(p, 1300, 223, 50));
  assert(show.output == ShowOutput::SOLID && show.pixel(49).r == 255);
  assert(batteryLimitedBrightness(show.selectedBrightness(223), BATTERY_STATE_CRITICAL, 95) == 95);
  assert(batteryLimitedBrightness(show.selectedBrightness(223), BATTERY_STATE_SOFT_CUTOFF, 95) == 0);
  audio.lastUpdateMs = 1500; audio.energy = 3; // Audio remains current under solid output.
  audioFilter.update(audio, 1500);
  p = command(ShowCommand::SET_PATTERN); p.params[0] = 23;
  assert(show.apply(p, 1600, 223, 50));
  audioFilter.reset(); // Actual renderer's entry reset.
  assert(audioFilter.update(audio, 1600).energy == 3);
  assert(!audioFilter.update(audio, 3001).valid);
  assert(show.apply(command(ShowCommand::BLACKOUT), 1700, 223, 50));
  assert(show.output == ShowOutput::BLACKOUT && show.pixel(0).r == 0);
  p = command(ShowCommand::SET_BRIGHTNESS); p.params[0] = 1;
  assert(show.apply(p, 1700, 223, 50) && show.selectedBrightness(223) == 1);
  assert(show.output == ShowOutput::BLACKOUT);
  assert(show.apply(command(ShowCommand::RELEASE), 1800, 223, 50));
  assert(!show.active() && show.selectedPattern(sync.armedPattern) == 12);
  assert(show.selectedBrightness(sync.armedBrightness) == 223 && baseClock.phaseMs() == 7000);
  // A local controller uses the same underlying selection without a saved show snapshot.
  p = command(ShowCommand::SET_PATTERN); p.params[0] = 27;
  assert(show.apply(p, 1900, 95, 50));
  show.release();
  assert(show.selectedPattern(4) == 4 && show.selectedBrightness(95) == 95);
}

static void testPending()
{
  ShowState show;
  auto p = command(ShowCommand::CLEAR_PENDING);
  p.params[0] = 42; p.params[1] = 3;
  assert(show.apply(p, 1100, 95, 50) && !show.active());
  auto apply = command(ShowCommand::APPLY_PENDING); apply.params[0] = 42;
  assert(!show.apply(apply, 1100, 95, 50));
  p = command(ShowCommand::SET_SEGMENT);
  p.params[0] = 42; p.params[1] = 0; p.params[2] = 0; p.params[3] = 8; p.params[4] = 255;
  assert(show.apply(p, 1101, 95, 50));
  assert(!show.active() && !show.apply(p, 1101, 95, 50)); // Duplicate segment index cannot overwrite.
  p.params[1] = 1; p.params[2] = 8; p.params[3] = 7; p.params[4] = 0;
  assert(show.apply(p, 1102, 95, 50));
  p.params[1] = 2; p.params[2] = 15; p.params[3] = 10; p.params[6] = 255;
  assert(show.apply(p, 1103, 95, 50) && show.pendingComplete());
  assert(!show.active() && show.pixel(0).r == 0);
  assert(show.apply(apply, 1200, 95, 50));
  assert(show.output == ShowOutput::BUFFER && show.pixel(0).r == 255 && show.pixel(7).r == 255);
  assert(show.pixel(8).r == 0 && show.pixel(14).b == 0 && show.pixel(15).b == 255 && show.pixel(24).b == 255);
  assert(show.pixel(25).b == 0 && show.pixel(49).r == 0);
  p = command(ShowCommand::CLEAR_PENDING); p.params[0] = 43; p.params[1] = 1;
  assert(show.apply(p, 1300, 95, 50));
  assert(show.pixel(0).r == 255); // Preparing another image does not mutate the front buffer.
  assert(!show.apply(apply, 1301, 95, 50)); // Old image ID.
  apply.params[0] = 43;
  assert(!show.apply(apply, 1301, 95, 50)); // Incomplete image.
  p = command(ShowCommand::SET_SEGMENT);
  p.params[0] = 43; p.params[2] = 49; p.params[3] = 2;
  assert(!show.apply(p, 1301, 95, 50)); // Actual configured strip bounds.
  p.params[2] = 24; p.params[3] = 2; p.params[5] = 255;
  assert(show.apply(p, 1302, 95, 50)); // Crosses strip 1 -> strip 2 in logical order.
  assert(!show.apply(apply, 1303, 95, 70)); // Changed strip length invalidates prepared image.
  assert(show.apply(apply, 1303, 95, 50));
  assert(show.pixel(24).g == 255 && show.pixel(25).g == 255);
  // Wide arithmetic rejects overflow and zero-length segments before indexing.
  p.params[2] = 255; p.params[3] = 255;
  assert(!show.apply(p, 1304, 95, 50));
  p.params[2] = 0; p.params[3] = 0;
  assert(!show.apply(p, 1304, 95, 50));
  // CLEAR during preparation discards all earlier indices, preserving active pixels.
  p = command(ShowCommand::CLEAR_PENDING); p.params[0] = 44; p.params[1] = 2;
  assert(show.apply(p, 1305, 95, 50));
  p = command(ShowCommand::SET_SEGMENT); p.params[0] = 44; p.params[3] = 1;
  assert(show.apply(p, 1306, 95, 50));
  p = command(ShowCommand::CLEAR_PENDING); p.params[0] = 45; p.params[1] = 1;
  assert(show.apply(p, 1307, 95, 50));
  assert(show.receivedSegments == 0 && !show.pendingComplete() && show.pixel(24).g == 255);
  p = command(ShowCommand::SET_SEGMENT); p.params[0] = 44; p.params[3] = 1;
  assert(!show.apply(p, 1308, 95, 50));
  show.release();
  assert(!show.pendingValid && !show.active());
  p = command(ShowCommand::CLEAR_PENDING); // Zero segments is a valid prepared black image.
  assert(show.apply(p, 1400, 95, 70) && show.pendingComplete());
  apply.params[0] = 0;
  assert(show.apply(apply, 1500, 95, 70) && show.pixel(69).g == 0);
  p = command(ShowCommand::CLEAR_PENDING); p.params[1] = 32;
  assert(show.apply(p, 1600, 95, 70));
  p = command(ShowCommand::SET_SEGMENT); p.params[3] = 1; p.params[4] = 255;
  for (uint8_t i = 0; i < 32; ++i)
  {
    p.params[1] = i; p.params[2] = 38 + i;
    assert(show.apply(p, 1601, 95, 70));
  }
  assert(show.receivedSegments == UINT32_MAX && show.pendingComplete());
  assert(show.apply(apply, 1700, 95, 70) && show.pixel(69).r == 255);
  for (uint8_t length : {uint8_t(10), uint8_t(25), uint8_t(35)}) {
    const uint8_t total = 2 * length;
    p = command(ShowCommand::CLEAR_PENDING); p.params[1] = 1;
    assert(show.apply(p, 1800, 95, total));
    p = command(ShowCommand::SET_SEGMENT); p.params[2] = length - 1;
    p.params[3] = 2; p.params[4] = 123;
    assert(show.apply(p, 1801, 95, total));
    assert(show.apply(apply, 1802, 95, total));
    assert(show.pixel(length - 1).r == 123 && show.pixel(length).r == 123);
    assert(show.pixel(total - 1).r == 0);
  }
}

int main()
{
  testCodec();
  testScheduler();
  testSharedClock();
  testTimeReconstruction();
  testStateAndCoexistence();
  testPending();
}
