// Actual GAP dispatch with a bounded AD iterator double; no BTstack/hardware emulation.
#include <assert.h>
#include <stdio.h>
#include <string.h>
#include <algorithm>
#include "Arduino.h"
#include "app/ShowControl.h"
#include "app/AudioPatternMath.h"
#include "protocol/SyncBeaconCodec.h"
using std::min;
uint32_t hostMillis = 1000;
constexpr uint8_t BLUETOOTH_DATA_TYPE_MANUFACTURER_SPECIFIC_DATA = 0xFF;
struct ad_context_t { const uint8_t* data; unsigned length, offset; };
void ad_iterator_init(ad_context_t* c, uint8_t len, uint8_t* data) { *c = {data, len, 0}; }
bool ad_iterator_has_more(ad_context_t* c) {
  return c->offset + 1 < c->length && c->data[c->offset] && c->offset + 1 + c->data[c->offset] <= c->length;
}
void ad_iterator_next(ad_context_t* c) { c->offset += 1 + c->data[c->offset]; }
uint8_t ad_iterator_get_data_type(ad_context_t* c) { return c->data[c->offset + 1]; }
uint8_t ad_iterator_get_data_len(ad_context_t* c) { return c->data[c->offset] - 1; }
const uint8_t* ad_iterator_get_data(ad_context_t* c) { return c->data + c->offset + 2; }
uint16_t little_endian_read_16(const uint8_t* d, int i) { return d[i] | uint16_t(d[i + 1]) << 8; }
class String;
#include "wireless/SyncBeaconRadio.h"
#define NIGHTKITE_BLE 1
#define NIGHTKITE_RM2 1
#define PIO_FRAMEWORK_ARDUINO_ENABLE_BLUETOOTH 1
#define PICO_CYW43_SUPPORTED 1
struct BluetoothLock { BluetoothLock() {} ~BluetoothLock() {} };
namespace gatt {
  bool initialized = true, connected = false, advertising = false;
  bool syncAdvertisingOwned = false, gattAdvSuppressed = false;
  struct { uint8_t advertisingLength = 3, scanResponseLength = 3; uint8_t advertising[3], scanResponse[3]; } gattAdvertisingData;
  void setAdvertisingEnabled(bool value) { advertising = value; }
  void configureGattAdvertisingParams() {}
  void gap_advertisements_set_data(uint8_t, const uint8_t*) {}
  void gap_scan_response_set_data(uint8_t, const uint8_t*) {}
  #include "gatt_restore_under_test.inc"
}
struct Rm2BleStatus { bool initialized, supported, connected; };
Rm2BleStatus rm2BleStatus() { return {gatt::initialized, true, gatt::connected}; }
void rm2BleRestoreGattAdvertising() { gatt::rm2BleRestoreGattAdvertising(); }
void rm2BleStopAdvertising() { gatt::syncAdvertisingOwned = true; gatt::advertising = false; }
void syncBeaconRadioBegin() {}
void gap_stop_scan() {}
void gap_start_scan() {}
void gap_set_scan_params(int, int, int, int) {}
void sendMasterBeacon(const SyncBeaconRuntime&);
unsigned long txIntervalForProfile(uint8_t) { return 100; }
#include "radio_under_test.inc"
void sendMasterBeacon(const SyncBeaconRuntime&) { ++txCount; }

static void testRadioTransitions() {
  SyncBeaconRuntime runtime = {};
  runtime.wirelessEnabled = runtime.showReceiveEnabled = true;
  runtime.groupId = 7;
  currentMode = RADIO_MODE_OFF;
  syncBeaconRadioTick(runtime);
  assert(currentMode == RADIO_MODE_SHOW_RECEIVER && scanning && gatt::syncAdvertisingOwned);
  gatt::connected = true;
  syncBeaconRadioTick(runtime);
  assert(currentMode == RADIO_MODE_GATT && !scanning);
  gatt::connected = false;
  syncBeaconRadioTick(runtime);
  assert(currentMode == RADIO_MODE_SHOW_RECEIVER && scanning);
  gatt::connected = true;
  syncBeaconRadioTick(runtime);
  runtime.showReceiveEnabled = false;
  syncBeaconRadioTick(runtime);
  assert(currentMode == RADIO_MODE_OFF && !gatt::syncAdvertisingOwned);
  gatt::connected = false; // Actual disconnect handler follows released GATT ownership.
  runtime.showReceiveEnabled = true;
  syncBeaconRadioTick(runtime);
  runtime.syncEnabled = true; runtime.playMode = SYNC_BEACON_PLAY_SYNC;
  runtime.syncRole = SYNC_BEACON_ROLE_MASTER;
  syncBeaconRadioTick(runtime);
  assert(currentMode == RADIO_MODE_BEACON_MASTER && !scanning && txCount == 1);
  runtime.syncRole = SYNC_BEACON_ROLE_FOLLOWER;
  syncBeaconRadioTick(runtime);
  assert(currentMode == RADIO_MODE_BEACON_FOLLOWER && scanning);
  runtime.wirelessEnabled = false;
  syncBeaconRadioTick(runtime);
  assert(currentMode == RADIO_MODE_OFF && !scanning && showScheduler.depth() == 0);
  assert(!gatt::syncAdvertisingOwned && gatt::advertising);
}

static void syncAdvertisement(uint8_t version, uint8_t* adv, size_t& size) {
  const uint8_t header[] = {2, 1, 6, 0, 255, 255, 255};
  memcpy(adv, header, sizeof(header));
  if (version == 2) {
    NkSyncBeaconV2 b = {};
    b.magic0 = 'N'; b.magic1 = 'K'; b.version = 2; b.groupId = 7;
    b.flags = NK_SYNC_BEACON_FLAG_AUDIO_SIGNAL_VALID;
    b.pattern = 25; b.brightness = 159; b.audioEnergy = 210; b.beatMs = 500; b.phaseMs = 125; b.seq = 2;
    b.crc = computeBeaconCrc(b);
    memcpy(adv + 7, &b, sizeof(b)); size = 7 + sizeof(b);
  } else {
    NkSyncBeaconV1 b = {};
    b.groupId = 7; b.pattern = 6; b.brightness = 223; b.seq = 1;
    size_t packetSize;
    assert(syncBeaconEncode(b, adv + 7, 24, &packetSize)); size = 7 + packetSize;
  }
  adv[3] = size - 4;
}

int main() {
  assert(ADV_TYPE_NONCONNECTABLE == 3 && LEGACY_ADV_MAX_LEN == 31 && FOLLOWER_LOST_MS == 1500);
  NkSyncBeaconV2 semantic = {};
  semantic.flags = NK_SYNC_BEACON_FLAG_AUDIO_BEAT;
  semantic.phaseMs = 123; semantic.beatMs = 500; semantic.audioEnergy = 210;
  updateAudioSyncState(semantic, 10);
  assert(!audioSyncState.valid && !audioSyncState.beat && audioSyncState.phaseMs == 0 && audioSyncState.energy == 0);
  semantic.seq = 1;
  semantic.flags = NK_SYNC_BEACON_FLAG_AUDIO_SIGNAL_VALID;
  updateAudioSyncState(semantic, 11);
  assert(audioSyncState.valid && !audioSyncState.beatLocked && audioSyncState.phaseMs == 0 &&
         audioSyncState.beatMs == 0 && audioSyncState.energy == 210);
  semantic.seq = 2;
  semantic.flags |= NK_SYNC_BEACON_FLAG_AUDIO_BEAT | NK_SYNC_BEACON_FLAG_AUDIO_BEAT_LOCKED;
  updateAudioSyncState(semantic, 12);
  assert(audioSyncState.valid && audioSyncState.beatLocked && audioSyncState.beat &&
         audioSyncState.phaseMs == 123 && audioSyncState.beatMs == 500);
  audioSyncState = AudioSyncState{}; lastAudioBeacon = NkSyncBeaconV2{}; haveAudioBeacon = false;
  currentMode = RADIO_MODE_BEACON_FOLLOWER; activeGroup = 7; activeShortId = 0xABC123;
  uint8_t adv[31]; size_t size;
  syncAdvertisement(2, adv, size);
  handleGapReport(adv, size, -42);
  assert(audioSyncState.valid && !audioSyncState.beatLocked && audioSyncState.phaseMs == 0 &&
         audioSyncState.energy == 210 && pendingBeacon.pattern == 25);
  assert(scanDecodeV2 == 1);
  ShowPacket p;
  p.command = ShowCommand::SET_PATTERN; p.params[0] = 23; p.eventId = 1;
  p.senderMs = hostMillis; p.executeMs = hostMillis + 100;
  assert(showControlEncodeAdvertisement(p, adv, 31));
  handleGapReport(adv, 31, -42);
  assert(showScheduler.depth() == 0); // Show enable is explicit, even on followers.
  showPacketsEnabled = true;
  handleGapReport(adv, 31, -42);
  handleGapReport(adv, 31, -42);
  assert(showScheduler.depth() == 1 && showScheduler.status.duplicates == 1);
  assert(audioSyncState.valid && audioSyncState.energy == 210 && pendingBeacon.pattern == 25);
  hostMillis = 1100;
  ShowScheduledEvent event;
  assert(showScheduler.popDue(hostMillis, event) && event.packet.params[0] == 23);
  assert(!showScheduler.popDue(hostMillis, event));
  assert(scanDecodeV2 == 1 && scanDecodeV1 == 0 && scanDecodeFail == 0);
  adv[22] ^= 1;
  handleGapReport(adv, 31, -42);
  assert(showInvalid == 1 && showCrcErrors == 1 && crcErrors == 0);
  p.targetKind = ShowTarget::SINGLE; p.target = 0xABC124;
  assert(showControlEncodeAdvertisement(p, adv, 31)); handleGapReport(adv, 31, -42);
  assert(showTargetMiss == 1);
  syncAdvertisement(1, adv, size);
  handleGapReport(adv, size, -42);
  assert(pendingBeacon.pattern == 6 && pendingBeacon.seq == 1);
  assert(audioSyncState.valid && audioSyncState.lastUpdateMs == 1000); // V1 does not renew audio.
  hostMillis = 1500;
  syncAdvertisement(2, adv, size); handleGapReport(adv, size, -42);
  assert(audioSyncState.lastUpdateMs == 1000); // Identical V2 repeats cannot refresh.
  expireAudioSyncState(1500);
  assert(audioSyncState.valid);
  expireAudioSyncState(1501);
  assert(1501 - lastBeaconMs < FOLLOWER_LOST_MS); // Sync remains locked independently.
  assert(!audioSyncState.valid && audioSyncState.energy == 0 && !audioSyncState.beat);
  currentMode = RADIO_MODE_SHOW_RECEIVER; hostMillis = 2600;
  syncAdvertisement(2, adv, size); handleGapReport(adv, size, -42);
  assert(!audioSyncState.valid); // Old repeated payload cannot revive expired audio.
  NkSyncBeaconV2 fresh;
  memcpy(&fresh, adv + 7, sizeof(fresh));
  fresh.seq = 1; // Lower sequence is legitimate new audio, no sender pinning.
  fresh.crc = computeBeaconCrc(fresh);
  memcpy(adv + 7, &fresh, sizeof(fresh));
  handleGapReport(adv, size, -42);
  assert(audioSyncState.valid && audioSyncState.energy == 210 && audioSyncState.lastUpdateMs == 2600);
  assert(scanDecodeV2 == 4);
  currentMode = RADIO_MODE_BEACON_MASTER;
  handleGapReport(adv, size, -42);
  assert(scanDecodeV2 == 4); // Separate autonomous TX mode.
  currentMode = RADIO_MODE_GATT;
  handleGapReport(adv, size, -42);
  assert(scanDecodeV2 == 4); // Connected GATT pre-empts receive.
  testRadioTransitions();
}
