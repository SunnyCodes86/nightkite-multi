// Compiles the actual main.cpp orchestration functions, with hardware-bound calls replaced.
// This checks renderer/sync/show wiring; it cannot validate LEDs, BLE timing or the FSM library.
#include <assert.h>
#include <string.h>
#include "app/ShowControl.h"
#include "app/SyncEngine.h"
#include "app/PatternClock.h"
#include "app/AudioPatternMath.h"
#include "app/Battery.h"

uint32_t hostMillis = 1000;
constexpr int PLAY_MODE_AUTOPLAY = 1, PLAY_MODE_SYNC = 2;
constexpr int SYNC_ROLE_MASTER = 1, SYNC_ROLE_FOLLOWER = 2;
constexpr int MIN_BRIGHTNESS = 95, MAX_TOTAL_LEDS = 70, MAX_LEDS_PER_STRIP = 35;
int TOTAL_LEDS = 50, NUM_LEDS = 25;
int currentPlayMode = PLAY_MODE_SYNC, currentSyncEnabled = 1, currentSyncRole = 2, currentSyncGroupId = 7;
int currentWirelessEnabled = 1, currentPattern = 1, currentBrightness = 95, BRIGHTNESS = 95;
int currentAutoplayEnabled = 1;
uint32_t currentEnabledPatternMask = 1;
bool batteryViewActive = false, UsbConnected = false;
bool showReceiveEnabled = true, syncFallbackPending = false;
uint32_t batteryViewLastInteractionMs = 0, autoplayResets = 0;
ShowState showState;
ShowScheduler scheduler;
PatternClock patternClock, showPatternClock;
SyncEngine syncEngine;
AudioPatternFilter audioPatternFilter;
AudioPatternFrame audioPatternFrame;
uint8_t renderedPattern = 0;
uint32_t renderedShowRevision = 0;
BatteryState currentBatteryState = BATTERY_STATE_NORMAL;
uint8_t lastBeaconPattern, lastBeaconBrightness;
uint16_t lastBeaconSeq, lastAppliedSeq;
uint32_t lastBeaconPhaseMs, syncApplySkipped, syncApplyCount;
const char* syncApplyReason;

struct CRGB {
  uint8_t r, g, b;
  CRGB(uint8_t red = 0, uint8_t green = 0, uint8_t blue = 0) : r(red), g(green), b(blue) {}
  static const CRGB Black;
};
const CRGB CRGB::Black;
CRGB Strip[70], PhysicalStrip[70];
void fill_solid(CRGB* leds, int count, CRGB color) { for (int i = 0; i < count; ++i) leds[i] = color; }
struct { uint8_t value = 95; void setBrightness(uint8_t b) { value = b; } uint8_t getBrightness() { return value; } } FastLED;
void PatternStateEntry();
void PatternStateExit();
PatternClock& renderPatternClock();
void switchToPattern(uint8_t, bool, const char*, uint32_t = 0);
void applyEffectiveBrightness();
void resetAutoplayTimer() { ++autoplayResets; }
bool isAutoplayEnabled() { return currentAutoplayEnabled != 0; }
bool isValidPatternId(int p) { return p >= 1 && p <= 27; }
bool isValidBrightnessLevel(int b) { return b >= 95 && b <= 255 && (b - 95) % 32 == 0; }
void recordPatternChange(uint8_t, uint8_t, const char*, uint32_t) {}
void setPlayMode(int mode) { currentPlayMode = mode; }
int s[3];
struct {
  bool pending = false;
  void setInitialState(int*) {}
  void reset() { pending = true; }
} fsm;
NkSyncBeaconV1 receivedBeacon = {};
bool hasBeacon = false;
bool syncBeaconRadioConsumeBeacon(NkSyncBeaconV1* b) {
  if (!hasBeacon) return false;
  *b = receivedBeacon; hasBeacon = false; return true;
}
struct SyncBeaconRadioStatus { uint32_t lastBeaconMs; };
SyncBeaconRadioStatus syncBeaconRadioStatus() { return {hostMillis}; }
AudioSyncState receivedAudio;
AudioSyncState syncBeaconAudioState() { return receivedAudio; }
bool syncBeaconRadioConsumeShow(ShowScheduledEvent* e) { return scheduler.popDue(hostMillis, *e); }
void syncBeaconRadioResetShow() { scheduler.reset(); }
uint32_t entryCount = 0, frameCount = 0;
void fakeEntry() { ++entryCount; fill_solid(Strip, TOTAL_LEDS, CRGB::Black); }
void fakeRun() {
  ++frameCount;
  if (renderedPattern >= 23) assert(audioPatternFrame.valid);
  fill_solid(Strip, TOTAL_LEDS, CRGB(renderedPattern >= 23 ? audioPatternFrame.energy : renderedPattern, 0, 0));
}
struct PatternDefinition { void (*entry)(); void (*run)(); void (*exit)(); };
const PatternDefinition* getPatternDefinition(uint8_t id) {
  static const PatternDefinition def = {fakeEntry, fakeRun, nullptr};
  return isValidPatternId(id) ? &def : nullptr;
}
#include "runtime_under_test.inc"

void frame() {
  if (fsm.pending) { PatternStateEntry(); fsm.pending = false; }
  PatternStateRunning();
  if (showState.active()) FastLED.setBrightness(showState.selectedBrightness(BRIGHTNESS));
  applyBatteryBrightnessLimit();
  syncLogicalToPhysicalLeds();
}
void event(ShowCommand cmd, uint8_t a = 0, uint8_t b = 0, uint8_t c = 0, uint8_t d = 0) {
  static uint16_t id = 1;
  ShowPacket p;
  p.command = cmd; p.eventId = id++; p.senderMs = hostMillis; p.executeMs = hostMillis;
  p.params[0] = a; p.params[1] = b; p.params[2] = c; p.params[3] = d;
  assert(scheduler.receive(p, hostMillis, hostMillis));
  tickShowControl();
}
int main() {
  patternClock.begin(); PatternStateEntry();
  receivedBeacon.groupId = 7; receivedBeacon.brightness = 223;
  for (int p = 1; p <= 27; ++p) {
    receivedBeacon.pattern = p; receivedBeacon.seq = 300 - p; receivedBeacon.phaseMs = 99;
    hasBeacon = true; applyReceivedSyncBeacon(); frame();
    assert(currentPattern == p && currentBrightness == 223);
    assert(currentEnabledPatternMask == 1 && !shouldRunAutoplayTick());
    assert(patternClock.now() == 99);
    if (p >= 23) assert(Strip[0].r == 0);
  }
  // All five audio patterns are gated at the real shared renderer, including its pixel history.
  for (uint8_t p = 23; p <= 27; ++p) {
    switchToPattern(p, true, "host");
    receivedAudio.valid = true; receivedAudio.lastUpdateMs = hostMillis;
    receivedAudio.energy = 240; receivedAudio.beatMs = 500;
    frame(); assert(Strip[0].r == 240 && audioPatternFrame.fresh);
    hostMillis += NK_AUDIO_FRESHNESS_TIMEOUT_MS + 1;
    const uint32_t before = frameCount;
    frame(); assert(Strip[0].r == 0 && frameCount == before && !audioPatternFrame.valid);
    receivedAudio.lastUpdateMs = hostMillis; receivedAudio.energy = 3;
    frame(); assert(Strip[0].r == 3 && audioPatternFrame.fresh);
  }
  // A show pattern has an independent clock and cannot mutate saved configuration fields.
  const int beforePattern = currentPattern, beforeBrightness = currentBrightness;
  event(ShowCommand::SET_PATTERN, 25); frame();
  assert(currentPattern == beforePattern && currentBrightness == beforeBrightness && renderedPattern == 25);
  assert(showPatternClock.now() == 0);
  const uint32_t entries = entryCount;
  hostMillis += 40;
  receivedBeacon.pattern = 12; receivedBeacon.seq = 1; receivedBeacon.phaseMs = 5555; receivedBeacon.brightness = 159;
  hasBeacon = true; applyReceivedSyncBeacon(); frame();
  assert(currentPattern == 12 && renderedPattern == 25 && entryCount == entries);
  assert(patternClock.now() == 5555 && renderPatternClock().now() == 40);
  event(ShowCommand::SET_SOLID, 255, 255, 255, 250); frame();
  assert(Strip[0].r == 255 && PhysicalStrip[35].g == 255 && currentBrightness == 159);
  assert(FastLED.getBrightness() == 250 && PhysicalStrip[25].r == 0);
  currentBatteryState = BATTERY_STATE_CRITICAL; frame();
  assert(FastLED.getBrightness() == 95 && currentBrightness == 159);
  currentBatteryState = BATTERY_STATE_SOFT_CUTOFF; frame();
  assert(FastLED.getBrightness() == 0);
  currentBatteryState = BATTERY_STATE_NORMAL;
  receivedAudio.lastUpdateMs = hostMillis; receivedAudio.energy = 88;
  event(ShowCommand::SET_PATTERN, 23); frame();
  assert(Strip[0].r == 88 && currentPattern == 12);
  event(ShowCommand::BLACKOUT); frame(); assert(Strip[0].r == 0);
  event(ShowCommand::RELEASE); frame();
  assert(renderedPattern == 12 && renderPatternClock().now() == 5555 && !showState.active());
  assert(FastLED.getBrightness() == 159);
  // Local autoplay resumes at release, with the underlying local selection and brightness intact.
  currentPlayMode = PLAY_MODE_AUTOPLAY; currentSyncEnabled = 0;
  switchToPattern(4, true, "host"); frame();
  event(ShowCommand::SET_PATTERN, 6); frame(); assert(!shouldRunAutoplayTick());
  event(ShowCommand::RELEASE); frame(); assert(renderedPattern == 4 && shouldRunAutoplayTick());
  // Returning from battery view to static output must clear the view flag.
  event(ShowCommand::SET_SOLID, 20, 30, 40); batteryViewActive = true;
  PatternStateEntry(); frame(); assert(!batteryViewActive && Strip[0].r == 20);
  // Underlying sync loss fallback is deferred only for the duration of the override.
  currentPlayMode = PLAY_MODE_SYNC; currentSyncEnabled = 1; syncFallbackPending = true; syncEngine.locked = false;
  event(ShowCommand::RELEASE); assert(currentPlayMode == PLAY_MODE_AUTOPLAY && !syncFallbackPending);
  for (int length = 10; length <= 35; ++length) {
    NUM_LEDS = length; TOTAL_LEDS = 2 * length;
    event(ShowCommand::SET_SOLID, 20, 30, 40); frame();
    for (int i = 0; i < 35; ++i) {
      assert(PhysicalStrip[i].r == (i < length ? 20 : 0));
      assert(PhysicalStrip[35 + i].r == (i < length ? 20 : 0));
    }
  }
  event(ShowCommand::RELEASE);
  event(ShowCommand::CLEAR_PENDING, 5, 1);
  assert(showState.pendingValid && !showState.active());
  currentWirelessEnabled = 0; tickShowControl();
  assert(!showState.pendingValid);
}
