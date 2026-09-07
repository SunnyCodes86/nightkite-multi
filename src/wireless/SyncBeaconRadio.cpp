#include "SyncBeaconRadio.h"
#include "wireless/Rm2Ble.h"

#ifndef NIGHTKITE_BLE
#define NIGHTKITE_BLE 0
#endif
#ifndef NIGHTKITE_RM2
#define NIGHTKITE_RM2 0
#endif

#if NIGHTKITE_BLE && NIGHTKITE_RM2 && defined(PIO_FRAMEWORK_ARDUINO_ENABLE_BLUETOOTH) && defined(PICO_CYW43_SUPPORTED)
#include <btstack.h>
#include <BluetoothLock.h>
#endif

#ifndef BLUETOOTH_DATA_TYPE_FLAGS
#define BLUETOOTH_DATA_TYPE_FLAGS 0x01
#endif
#ifndef BLUETOOTH_DATA_TYPE_MANUFACTURER_SPECIFIC_DATA
#define BLUETOOTH_DATA_TYPE_MANUFACTURER_SPECIFIC_DATA 0xFF
#endif

namespace
{
constexpr uint8_t ADV_TYPE_NONCONNECTABLE = 3; // ADV_NONCONN_IND.
constexpr unsigned long FOLLOWER_LOST_MS = 1500;
constexpr uint8_t LEGACY_ADV_MAX_LEN = 31;
constexpr size_t HEX_HEAD_BYTES = 12;
constexpr size_t HEX_HEAD_BUFFER_SIZE = (HEX_HEAD_BYTES * 2) + 1;

enum RadioMode : uint8_t
{
  RADIO_MODE_OFF,
  RADIO_MODE_GATT,
  RADIO_MODE_BEACON_MASTER,
  RADIO_MODE_BEACON_FOLLOWER,
  RADIO_MODE_SHOW_RECEIVER
};

RadioMode currentMode = RADIO_MODE_OFF;
bool beginCalled = false;
bool codecSelftestOk = false;
bool scanning = false;
bool advActive = false;
uint8_t advPayloadLen = 0;
uint8_t advMfgLen = 0;
uint16_t advCompany = NK_SYNC_BEACON_COMPANY_ID;
uint16_t advMagic = ((uint16_t)NK_SYNC_BEACON_MAGIC0 << 8) | NK_SYNC_BEACON_MAGIC1;
uint8_t advVersion = NK_SYNC_BEACON_VERSION;
uint8_t advGroup = 0;
uint16_t advCrc = 0;
unsigned long advSetCount = 0;
uint8_t activeGroup = 1;
uint16_t beaconSeq = 0;
unsigned long nextTxMs = 0;
unsigned long txCount = 0;
unsigned long rxCount = 0;
unsigned long crcErrors = 0;
unsigned long groupMismatch = 0;
unsigned long invalidPackets = 0;
unsigned long scanReports = 0;
unsigned long scanMfgReports = 0;
unsigned long scanNkCandidates = 0;
unsigned long scanDecodeOk = 0;
unsigned long scanDecodeV1 = 0;
unsigned long scanDecodeV2 = 0;
unsigned long scanDecodeFail = 0;
unsigned long scanCrcFail = 0;
unsigned long scanGroupMismatch = 0;
unsigned long scanRejectCompany = 0;
unsigned long scanRejectMagic = 0;
unsigned long scanRejectLen = 0;
unsigned long scanRejectVersion = 0;
unsigned long lastBeaconMs = 0;
int8_t scanLastRssi = 0;
uint8_t scanLastLen = 0;
uint8_t scanLastMfgLen = 0;
uint8_t scanLastAdType = 0;
uint16_t scanLastCompany = 0;
uint8_t scanLastGroup = 0;
uint8_t scanLastVersion = 0;
uint8_t lastBeaconVersion = 0;
const char* lastError = "disabled";
const char* scanLastError = "none";
const char* scanLastCandidateReason = "none";
char advMfgHead[HEX_HEAD_BUFFER_SIZE] = "none";
char scanLastMfgHead[HEX_HEAD_BUFFER_SIZE] = "none";
NkSyncBeaconV1 pendingBeacon;
bool pendingBeaconAvailable = false;
AudioSyncState audioSyncState;
NkSyncBeaconV2 lastAudioBeacon = {};
bool haveAudioBeacon = false;
ShowScheduler showScheduler;
uint32_t activeShortId = 0;
bool showPacketsEnabled = false;
uint32_t showReceived = 0, showInvalid = 0, showCrcErrors = 0, showTargetMiss = 0;

bool isReceiveMode()
{
  return currentMode == RADIO_MODE_BEACON_FOLLOWER || currentMode == RADIO_MODE_SHOW_RECEIVER;
}


void formatHexBytes(const uint8_t* data, size_t len, char* output, size_t outputSize)
{
  if (output == nullptr || outputSize == 0)
  {
    return;
  }
  if (data == nullptr || len == 0)
  {
    strncpy(output, "none", outputSize - 1);
    output[outputSize - 1] = '\0';
    return;
  }

  const size_t bytesToWrite = min(len, (outputSize - 1) / 2);
  size_t pos = 0;
  for (size_t i = 0; i < bytesToWrite && pos + 2 < outputSize; i++)
  {
    snprintf(&output[pos], outputSize - pos, "%02X", data[i]);
    pos += 2;
  }
  output[pos] = '\0';
}

String formatHex16(uint16_t value)
{
  char buffer[7];
  snprintf(buffer, sizeof(buffer), "0x%04X", value);
  return String(buffer);
}

String formatHex8(uint8_t value)
{
  char buffer[5];
  snprintf(buffer, sizeof(buffer), "0x%02X", value);
  return String(buffer);
}

const char* modeName(RadioMode mode)
{
  switch (mode)
  {
    case RADIO_MODE_GATT: return "gatt";
    case RADIO_MODE_BEACON_MASTER: return "beacon_master";
    case RADIO_MODE_BEACON_FOLLOWER: return "beacon_follower";
    case RADIO_MODE_SHOW_RECEIVER: return "show_receiver";
    case RADIO_MODE_OFF:
    default: return "off";
  }
}

void copyV2SyncBasis(const NkSyncBeaconV2& source, NkSyncBeaconV1* target)
{
  target->magic0 = source.magic0;
  target->magic1 = source.magic1;
  target->version = source.version;
  target->groupId = source.groupId;
  target->flags = source.flags;
  target->seq = source.seq;
  target->pattern = source.pattern;
  target->brightness = source.brightness;
  target->phaseMs = source.phaseMs;
  target->beatMs = source.beatMs;
  target->crc = source.crc;
}

void updateAudioSyncState(const NkSyncBeaconV2& beacon, unsigned long nowMs)
{
  // Identical controller-generated repeats are not new audio frames. Keep this
  // identity across expiry so a stalled advertiser cannot revive old audio.
  if (haveAudioBeacon && memcmp(&lastAudioBeacon, &beacon, sizeof(beacon)) == 0) return;
  lastAudioBeacon = beacon;
  haveAudioBeacon = true;
  audioSyncState.valid = true;
  audioSyncState.lastUpdateMs = nowMs;
  audioSyncState.seq = beacon.seq;
  audioSyncState.phaseMs = beacon.phaseMs;
  audioSyncState.beatMs = beacon.beatMs;
  audioSyncState.beat = (beacon.flags & NK_SYNC_BEACON_FLAG_AUDIO_BEAT) != 0;
  audioSyncState.energy = beacon.audioEnergy;
  audioSyncState.bass = beacon.audioBass;
  audioSyncState.mid = beacon.audioMid;
  audioSyncState.treble = beacon.audioTreble;
  audioSyncState.confidence = beacon.audioConfidence;
}

void expireAudioSyncState(unsigned long nowMs)
{
  if (audioSyncState.valid && nowMs - audioSyncState.lastUpdateMs > NK_AUDIO_FRESHNESS_TIMEOUT_MS)
  {
    const uint32_t lastUpdateMs = audioSyncState.lastUpdateMs;
    audioSyncState = AudioSyncState{};
    audioSyncState.lastUpdateMs = lastUpdateMs;
  }
}

unsigned long txIntervalForProfile(uint8_t profile)
{
  switch (profile)
  {
    case SYNC_BEACON_PROFILE_LONG_RANGE: return 200; // 5 Hz.
    case SYNC_BEACON_PROFILE_FAST_SYNC: return 50;   // 20 Hz.
    case SYNC_BEACON_PROFILE_BALANCED:
    default: return 100;                             // 10 Hz.
  }
}

uint16_t advIntervalUnitsForProfile(uint8_t profile)
{
  // BLE advertising intervals are in 0.625 ms units.
  switch (profile)
  {
    case SYNC_BEACON_PROFILE_LONG_RANGE: return 320; // 200 ms.
    case SYNC_BEACON_PROFILE_FAST_SYNC: return 80;   // 50 ms.
    case SYNC_BEACON_PROFILE_BALANCED:
    default: return 160;                             // 100 ms.
  }
}

bool buildBeaconAdvertisingData(const NkSyncBeaconV1& beacon, uint8_t* output, uint8_t outputSize, uint8_t* outputLen)
{
  const uint8_t requiredLen = (uint8_t)NK_SYNC_BEACON_ADV_LEN;
  if (output == nullptr || outputLen == nullptr || outputSize < requiredLen || requiredLen > LEGACY_ADV_MAX_LEN)
  {
    return false;
  }

  uint8_t pos = 0;
  output[pos++] = 2;
  output[pos++] = BLUETOOTH_DATA_TYPE_FLAGS;
  output[pos++] = 0x06;

  output[pos++] = (uint8_t)(NK_SYNC_BEACON_MFG_LEN + 1);
  output[pos++] = BLUETOOTH_DATA_TYPE_MANUFACTURER_SPECIFIC_DATA;
  output[pos++] = (uint8_t)(NK_SYNC_BEACON_COMPANY_ID & 0xFF);
  output[pos++] = (uint8_t)(NK_SYNC_BEACON_COMPANY_ID >> 8);
  memcpy(&output[pos], &beacon, NK_SYNC_BEACON_PACKET_SIZE);
  pos += NK_SYNC_BEACON_PACKET_SIZE;
  *outputLen = pos;
  return true;
}

bool runCodecSelftest()
{
  NkSyncBeaconV1 beacon;
  beacon.magic0 = NK_SYNC_BEACON_MAGIC0;
  beacon.magic1 = NK_SYNC_BEACON_MAGIC1;
  beacon.version = NK_SYNC_BEACON_VERSION;
  beacon.groupId = 1;
  beacon.flags = 0;
  beacon.seq = 42;
  beacon.pattern = 1;
  beacon.brightness = 95;
  beacon.phaseMs = 1234;
  beacon.beatMs = NK_SYNC_BEACON_BEAT_MS;
  beacon.crc = computeBeaconCrc(beacon);

  uint8_t advData[LEGACY_ADV_MAX_LEN];
  uint8_t advLen = 0;
  if (!buildBeaconAdvertisingData(beacon, advData, sizeof(advData), &advLen) || advLen != NK_SYNC_BEACON_ADV_LEN)
  {
    return false;
  }
  if (advData[3] != NK_SYNC_BEACON_MFG_LEN + 1 || advData[4] != BLUETOOTH_DATA_TYPE_MANUFACTURER_SPECIFIC_DATA)
  {
    return false;
  }
  if (advData[5] != (uint8_t)(NK_SYNC_BEACON_COMPANY_ID & 0xFF) ||
      advData[6] != (uint8_t)(NK_SYNC_BEACON_COMPANY_ID >> 8))
  {
    return false;
  }

  NkSyncBeaconV1 decoded;
  const SyncBeaconDecodeResult result = syncBeaconDecode(&advData[7], NK_SYNC_BEACON_PACKET_SIZE, 1, &decoded);
  if (result != SYNC_BEACON_DECODE_OK ||
      decoded.seq != beacon.seq ||
      decoded.groupId != beacon.groupId ||
      decoded.crc != beacon.crc)
  {
    return false;
  }

  NkSyncBeaconV1 invalidV1 = beacon;
  invalidV1.version = 3;
  invalidV1.crc = computeBeaconCrc(invalidV1);
  if (syncBeaconDecode(reinterpret_cast<const uint8_t*>(&invalidV1), sizeof(invalidV1), 1, &decoded) != SYNC_BEACON_DECODE_BAD_VERSION ||
      syncBeaconDecode(reinterpret_cast<const uint8_t*>(&beacon), sizeof(beacon), 2, &decoded) != SYNC_BEACON_DECODE_BAD_GROUP)
  {
    return false;
  }

  NkSyncBeaconV2 audioBeacon;
  audioBeacon.magic0 = NK_SYNC_BEACON_MAGIC0;
  audioBeacon.magic1 = NK_SYNC_BEACON_MAGIC1;
  audioBeacon.version = NK_SYNC_BEACON_VERSION_V2;
  audioBeacon.groupId = 1;
  audioBeacon.flags = NK_SYNC_BEACON_FLAG_AUDIO_BEAT;
  audioBeacon.seq = 43;
  audioBeacon.pattern = NK_PATTERN_MAX_ID;
  audioBeacon.brightness = 159;
  audioBeacon.phaseMs = 4321;
  audioBeacon.beatMs = 500;
  audioBeacon.audioEnergy = 201;
  audioBeacon.audioBass = 202;
  audioBeacon.audioMid = 203;
  audioBeacon.audioTreble = 204;
  audioBeacon.audioConfidence = 205;
  audioBeacon.crc = computeBeaconCrc(audioBeacon);

  NkSyncBeaconV2 decodedAudio;
  if (syncBeaconDecodeV2(reinterpret_cast<const uint8_t*>(&audioBeacon), sizeof(audioBeacon), 1, &decodedAudio) != SYNC_BEACON_DECODE_OK ||
      decodedAudio.seq != audioBeacon.seq ||
      decodedAudio.audioEnergy != 201 ||
      decodedAudio.audioBass != 202 ||
      decodedAudio.audioMid != 203 ||
      decodedAudio.audioTreble != 204 ||
      decodedAudio.audioConfidence != 205)
  {
    return false;
  }

  const AudioSyncState savedAudioState = audioSyncState;
  const NkSyncBeaconV2 savedAudioBeacon = lastAudioBeacon;
  const bool savedHaveAudioBeacon = haveAudioBeacon;
  haveAudioBeacon = false;
  updateAudioSyncState(decodedAudio, 123);
  const bool audioStateOk = audioSyncState.valid &&
      audioSyncState.lastUpdateMs == 123 &&
      audioSyncState.seq == audioBeacon.seq &&
      audioSyncState.phaseMs == audioBeacon.phaseMs &&
      audioSyncState.beatMs == audioBeacon.beatMs &&
      audioSyncState.beat &&
      audioSyncState.energy == audioBeacon.audioEnergy &&
      audioSyncState.bass == audioBeacon.audioBass &&
      audioSyncState.mid == audioBeacon.audioMid &&
      audioSyncState.treble == audioBeacon.audioTreble &&
      audioSyncState.confidence == audioBeacon.audioConfidence;
  expireAudioSyncState(123 + NK_AUDIO_FRESHNESS_TIMEOUT_MS + 1);
  const bool audioTimeoutOk = !audioSyncState.valid && !audioSyncState.beat;
  audioSyncState = savedAudioState;
  lastAudioBeacon = savedAudioBeacon;
  haveAudioBeacon = savedHaveAudioBeacon;
  if (!audioStateOk || !audioTimeoutOk)
  {
    return false;
  }

  if (syncBeaconDecodeV2(reinterpret_cast<const uint8_t*>(&audioBeacon), sizeof(audioBeacon), 2, &decodedAudio) != SYNC_BEACON_DECODE_BAD_GROUP)
  {
    return false;
  }

  NkSyncBeaconV2 badPattern = audioBeacon;
  badPattern.pattern = NK_PATTERN_MAX_ID + 1;
  badPattern.crc = computeBeaconCrc(badPattern);
  if (syncBeaconDecodeV2(reinterpret_cast<const uint8_t*>(&badPattern), sizeof(badPattern), 1, &decodedAudio) != SYNC_BEACON_DECODE_BAD_PATTERN)
  {
    return false;
  }

  NkSyncBeaconV2 badCrc = audioBeacon;
  badCrc.audioEnergy++;
  return syncBeaconDecodeV2(reinterpret_cast<const uint8_t*>(&badCrc), sizeof(badCrc), 1, &decodedAudio) == SYNC_BEACON_DECODE_BAD_CRC;
}

#if NIGHTKITE_BLE && NIGHTKITE_RM2 && defined(PIO_FRAMEWORK_ARDUINO_ENABLE_BLUETOOTH) && defined(PICO_CYW43_SUPPORTED)
void stopScan()
{
  if (scanning)
  {
    gap_stop_scan();
    scanning = false;
  }
}

void startScan()
{
  if (!scanning)
  {
    gap_set_scan_params(0, 96, 48, 0);
    gap_start_scan();
    scanning = true;
  }
}

void handleGapReport(const uint8_t* advData, uint8_t advLen, int8_t rssi)
{
  if (!isReceiveMode())
  {
    return;
  }

  scanReports++;
  scanLastRssi = rssi;
  scanLastLen = advLen;
  scanLastMfgLen = 0;

  ad_context_t context;
  for (ad_iterator_init(&context, advLen, (uint8_t*)advData); ad_iterator_has_more(&context); ad_iterator_next(&context))
  {
    const uint8_t dataType = ad_iterator_get_data_type(&context);
    const uint8_t dataLen = ad_iterator_get_data_len(&context);
    const uint8_t* data = ad_iterator_get_data(&context);
    scanLastAdType = dataType;
    if (dataType != BLUETOOTH_DATA_TYPE_MANUFACTURER_SPECIFIC_DATA)
    {
      continue;
    }
    scanMfgReports++;
    scanLastMfgLen = dataLen;
    formatHexBytes(data, dataLen, scanLastMfgHead, sizeof(scanLastMfgHead));
    if (dataLen < 2)
    {
      scanRejectLen++;
      scanDecodeFail++;
      invalidPackets++;
      scanLastError = "mfg_too_short";
      scanLastCandidateReason = "len";
      lastError = "mfg_too_short";
      continue;
    }
    const uint16_t companyId = little_endian_read_16(data, 0);
    scanLastCompany = companyId;
    if (companyId != NK_SYNC_BEACON_COMPANY_ID)
    {
      scanRejectCompany++;
      scanLastCandidateReason = "company";
      continue;
    }
    if (dataLen >= 4 && data[2] == 'N' && data[3] == 'S')
    {
      if (!showPacketsEnabled) continue;
      ShowPacket packet;
      const ShowDecodeResult decoded = showControlDecode(data + 2, dataLen - 2, &packet);
      if (decoded != ShowDecodeResult::OK)
      {
        ++showInvalid;
        if (decoded == ShowDecodeResult::CRC) ++showCrcErrors;
      }
      else if (!showTargetMatches(packet, activeGroup, activeShortId)) ++showTargetMiss;
      else
      {
        ++showReceived;
        const uint32_t nowMs = millis();
        showScheduler.receive(packet, nowMs, nowMs);
      }
      continue;
    }
    scanNkCandidates++;
    if (dataLen < NK_SYNC_BEACON_MFG_LEN)
    {
      scanRejectLen++;
      scanDecodeFail++;
      invalidPackets++;
      scanLastError = "candidate_too_short";
      scanLastCandidateReason = "len";
      lastError = "candidate_too_short";
      continue;
    }
    scanLastGroup = data[2 + 3];
    scanLastVersion = data[2 + 2];

    NkSyncBeaconV1 beacon;
    SyncBeaconDecodeResult result = SYNC_BEACON_DECODE_BAD_VERSION;
    NkSyncBeaconV2 audioBeacon;
    if (data[2] != NK_SYNC_BEACON_MAGIC0 || data[3] != NK_SYNC_BEACON_MAGIC1)
    {
      result = SYNC_BEACON_DECODE_BAD_MAGIC;
    }
    else if (scanLastVersion == NK_SYNC_BEACON_VERSION_V1)
    {
      result = syncBeaconDecode(&data[2], dataLen - 2, activeGroup, &beacon);
    }
    else if (scanLastVersion == NK_SYNC_BEACON_VERSION_V2)
    {
      result = syncBeaconDecodeV2(&data[2], dataLen - 2, activeGroup, &audioBeacon);
      if (result == SYNC_BEACON_DECODE_OK)
      {
        copyV2SyncBasis(audioBeacon, &beacon);
      }
    }
    if (result == SYNC_BEACON_DECODE_OK)
    {
      const unsigned long receiveMs = millis();
      pendingBeacon = beacon;
      pendingBeaconAvailable = true;
      rxCount++;
      scanDecodeOk++;
      if (beacon.version == NK_SYNC_BEACON_VERSION_V2)
      {
        scanDecodeV2++;
        updateAudioSyncState(audioBeacon, receiveMs);
      }
      else
      {
        scanDecodeV1++;
      }
      lastBeaconVersion = beacon.version;
      lastBeaconMs = receiveMs;
      lastError = "none";
      scanLastError = "ok";
      scanLastCandidateReason = "ok";
    }
    else if (result == SYNC_BEACON_DECODE_BAD_GROUP)
    {
      groupMismatch++;
      scanGroupMismatch++;
      scanDecodeFail++;
      scanLastError = "bad_group";
      scanLastCandidateReason = "group";
      lastError = "bad_group";
    }
    else if (result == SYNC_BEACON_DECODE_BAD_CRC)
    {
      crcErrors++;
      scanCrcFail++;
      scanDecodeFail++;
      lastError = "crc";
      scanLastError = "crc";
      scanLastCandidateReason = "crc";
    }
    else if (result == SYNC_BEACON_DECODE_BAD_MAGIC)
    {
      scanRejectMagic++;
      scanDecodeFail++;
      invalidPackets++;
      lastError = "bad_magic";
      scanLastError = "bad_magic";
      scanLastCandidateReason = "magic";
    }
    else if (result == SYNC_BEACON_DECODE_BAD_VERSION)
    {
      scanRejectVersion++;
      scanDecodeFail++;
      invalidPackets++;
      lastError = "bad_version";
      scanLastError = "bad_version";
      scanLastCandidateReason = "version";
    }
    else if (result == SYNC_BEACON_DECODE_TOO_SHORT)
    {
      scanRejectLen++;
      scanDecodeFail++;
      invalidPackets++;
      lastError = "too_short";
      scanLastError = "too_short";
      scanLastCandidateReason = "len";
    }
    else
    {
      invalidPackets++;
      scanDecodeFail++;
      lastError = syncBeaconDecodeResultName(result);
      scanLastError = lastError;
      scanLastCandidateReason = lastError;
    }
  }
}
#else
void stopScan() {}
void handleGapReport(const uint8_t* advData, uint8_t advLen, int8_t rssi)
{
  (void)advData;
  (void)advLen;
  (void)rssi;
}
#endif

void leaveBeaconMode()
{
#if NIGHTKITE_BLE && NIGHTKITE_RM2 && defined(PIO_FRAMEWORK_ARDUINO_ENABLE_BLUETOOTH) && defined(PICO_CYW43_SUPPORTED)
  stopScan();
  if (currentMode == RADIO_MODE_BEACON_MASTER || isReceiveMode() || currentMode == RADIO_MODE_GATT)
  {
    rm2BleRestoreGattAdvertising();
  }
#endif
  currentMode = RADIO_MODE_OFF;
  advActive = false;
  nextTxMs = 0;
}

void sendMasterBeacon(const SyncBeaconRuntime& runtime)
{
#if NIGHTKITE_BLE && NIGHTKITE_RM2 && defined(PIO_FRAMEWORK_ARDUINO_ENABLE_BLUETOOTH) && defined(PICO_CYW43_SUPPORTED)
  NkSyncBeaconV1 beacon;
  beacon.magic0 = NK_SYNC_BEACON_MAGIC0;
  beacon.magic1 = NK_SYNC_BEACON_MAGIC1;
  beacon.version = NK_SYNC_BEACON_VERSION;
  beacon.groupId = runtime.groupId;
  beacon.flags = 0;
  beacon.seq = ++beaconSeq;
  beacon.pattern = runtime.pattern;
  beacon.brightness = runtime.brightness;
  beacon.phaseMs = runtime.phaseMs;
  beacon.beatMs = runtime.beatMs;
  beacon.crc = computeBeaconCrc(beacon);

  uint8_t advData[31];
  uint8_t advLen = 0;
  if (!buildBeaconAdvertisingData(beacon, advData, sizeof(advData), &advLen))
  {
    advActive = false;
    advPayloadLen = advLen;
    lastError = "adv_payload_too_long";
    return;
  }
  advPayloadLen = advLen;
  advMfgLen = NK_SYNC_BEACON_MFG_LEN;
  advCompany = NK_SYNC_BEACON_COMPANY_ID;
  advMagic = ((uint16_t)beacon.magic0 << 8) | beacon.magic1;
  advVersion = beacon.version;
  advGroup = beacon.groupId;
  advCrc = beacon.crc;
  formatHexBytes(&advData[5], advMfgLen, advMfgHead, sizeof(advMfgHead));
  if (advLen > LEGACY_ADV_MAX_LEN)
  {
    advActive = false;
    lastError = "adv_payload_too_long";
    return;
  }

  const uint16_t interval = advIntervalUnitsForProfile(runtime.wirelessProfile);
  if (!rm2BleUseSyncAdvertising(advData, advLen, interval, (uint16_t)(interval + 16), ADV_TYPE_NONCONNECTABLE))
  {
    advActive = false;
    lastError = "adv_failed";
    return;
  }

  advActive = true;
  advSetCount++;
  txCount++;
  lastBeaconMs = millis();
  lastError = "none";
#else
  (void)runtime;
#endif
}
}

void syncBeaconRadioBegin()
{
  if (beginCalled)
  {
    return;
  }
  beginCalled = true;
  codecSelftestOk = runCodecSelftest();
  lastError = codecSelftestOk ? "none" : "codec_selftest_failed";
  rm2BleSetGapReportHandler(handleGapReport);
}

void syncBeaconRadioTick(const SyncBeaconRuntime& runtime)
{
  syncBeaconRadioBegin();
  const Rm2BleStatus ble = rm2BleStatus();
  if (!ble.initialized)
  {
    leaveBeaconMode();
    lastError = ble.supported ? "ble_not_ready" : "unsupported";
    return;
  }

#if NIGHTKITE_BLE && NIGHTKITE_RM2 && defined(PIO_FRAMEWORK_ARDUINO_ENABLE_BLUETOOTH) && defined(PICO_CYW43_SUPPORTED)
  BluetoothLock lock;
#endif
  const bool groupChanged = activeGroup != runtime.groupId;
  activeGroup = runtime.groupId;
  activeShortId = runtime.shortId;
  showPacketsEnabled = runtime.showReceiveEnabled && runtime.wirelessEnabled;
  if (!showPacketsEnabled) showScheduler.reset();
  expireAudioSyncState(millis());
  if (groupChanged) showScheduler.reset();

  const bool syncActive = runtime.syncEnabled &&
      runtime.wirelessEnabled &&
      runtime.playMode == SYNC_BEACON_PLAY_SYNC &&
      (runtime.syncRole == SYNC_BEACON_ROLE_MASTER || runtime.syncRole == SYNC_BEACON_ROLE_FOLLOWER);
  if (!syncActive && !(runtime.wirelessEnabled && runtime.showReceiveEnabled))
  {
    leaveBeaconMode();
    lastError = runtime.wirelessEnabled ? "none" : "wireless_disabled";
    return;
  }

  if (ble.connected)
  {
#if NIGHTKITE_BLE && NIGHTKITE_RM2 && defined(PIO_FRAMEWORK_ARDUINO_ENABLE_BLUETOOTH) && defined(PICO_CYW43_SUPPORTED)
    stopScan();
#endif
    advActive = false;
    currentMode = RADIO_MODE_GATT;
    lastError = "gatt_connected";
    return;
  }

  if (syncActive && runtime.syncRole == SYNC_BEACON_ROLE_MASTER)
  {
#if NIGHTKITE_BLE && NIGHTKITE_RM2 && defined(PIO_FRAMEWORK_ARDUINO_ENABLE_BLUETOOTH) && defined(PICO_CYW43_SUPPORTED)
    stopScan();
#endif
    if (currentMode != RADIO_MODE_BEACON_MASTER) showScheduler.reset();
    currentMode = RADIO_MODE_BEACON_MASTER;
    const unsigned long nowMs = millis();
    if (nextTxMs == 0 || (int32_t)(nowMs - nextTxMs) >= 0)
    {
      sendMasterBeacon(runtime);
      nextTxMs = nowMs + txIntervalForProfile(runtime.wirelessProfile);
    }
    return;
  }

  if (!isReceiveMode() || groupChanged)
  {
    lastBeaconMs = 0;
    pendingBeaconAvailable = false;
    audioSyncState.valid = false;
    audioSyncState.beat = false;
  }
  currentMode = syncActive ? RADIO_MODE_BEACON_FOLLOWER : RADIO_MODE_SHOW_RECEIVER;
  advActive = false;
  nextTxMs = 0;
#if NIGHTKITE_BLE && NIGHTKITE_RM2 && defined(PIO_FRAMEWORK_ARDUINO_ENABLE_BLUETOOTH) && defined(PICO_CYW43_SUPPORTED)
  rm2BleStopAdvertising();
  startScan();
  lastError = "none";
#else
  lastError = "unsupported";
#endif
}

void syncBeaconRadioStop()
{
#if NIGHTKITE_BLE && NIGHTKITE_RM2 && defined(PIO_FRAMEWORK_ARDUINO_ENABLE_BLUETOOTH) && defined(PICO_CYW43_SUPPORTED)
  if (!rm2BleStatus().initialized) return;
  BluetoothLock lock;
#endif
  leaveBeaconMode();
  pendingBeaconAvailable = false;
  audioSyncState = AudioSyncState{};
  showScheduler.reset();
}

SyncBeaconRadioStatus syncBeaconRadioStatus()
{
  const Rm2BleStatus ble = rm2BleStatus();
  SyncBeaconRadioStatus status;
  status.supported = NIGHTKITE_BLE && NIGHTKITE_RM2;
  status.active = currentMode == RADIO_MODE_BEACON_MASTER || isReceiveMode();
  status.beaconTx = currentMode == RADIO_MODE_BEACON_MASTER;
  status.beaconRx = currentMode == RADIO_MODE_BEACON_FOLLOWER;
  status.locked = status.beaconTx || (status.beaconRx && lastBeaconMs > 0 && (millis() - lastBeaconMs) <= FOLLOWER_LOST_MS);
  status.codecSelftest = codecSelftestOk;
  status.scanActive = scanning;
  status.advActive = advActive;
  status.gattAdvSuppressed = ble.gattAdvSuppressed;
  status.beaconAdvStarted = ble.syncAdvStartCount > 0;
  status.advPayloadLen = advPayloadLen;
  status.advMfgLen = advMfgLen;
  status.advCompany = advCompany;
  status.advMagic = advMagic;
  status.advVersion = advVersion;
  status.advGroup = advGroup;
  status.advCrc = advCrc;
  status.advEnableCount = ble.advEnableCount;
  status.advDisableCount = ble.advDisableCount;
  status.advSetCount = advSetCount;
  status.beaconAdvRefreshes = ble.syncAdvRefreshCount;
  status.beaconSeq = beaconSeq;
  status.txCount = txCount;
  status.rxCount = rxCount;
  status.crcErrors = crcErrors;
  status.groupMismatch = groupMismatch;
  status.invalidPackets = invalidPackets;
  status.scanReports = scanReports;
  status.scanMfgReports = scanMfgReports;
  status.scanNkCandidates = scanNkCandidates;
  status.scanDecodeOk = scanDecodeOk;
  status.scanDecodeV1 = scanDecodeV1;
  status.scanDecodeV2 = scanDecodeV2;
  status.scanDecodeFail = scanDecodeFail;
  status.scanCrcFail = scanCrcFail;
  status.scanGroupMismatch = scanGroupMismatch;
  status.scanRejectCompany = scanRejectCompany;
  status.scanRejectMagic = scanRejectMagic;
  status.scanRejectLen = scanRejectLen;
  status.scanRejectVersion = scanRejectVersion;
  status.lastBeaconMs = lastBeaconMs;
  status.beaconAgeMs = lastBeaconMs == 0 ? 0 : millis() - lastBeaconMs;
  status.scanLastRssi = scanLastRssi;
  status.scanLastLen = scanLastLen;
  status.scanLastMfgLen = scanLastMfgLen;
  status.scanLastAdType = scanLastAdType;
  status.scanLastCompany = scanLastCompany;
  status.scanLastGroup = scanLastGroup;
  status.scanLastVersion = scanLastVersion;
  status.lastBeaconVersion = lastBeaconVersion;
  status.audio = syncBeaconAudioState();
  status.audioAgeMs = status.audio.lastUpdateMs == 0 ? 0 : millis() - status.audio.lastUpdateMs;
  status.advMfgHead = advMfgHead;
  status.advOwner = ble.advOwner;
  status.advType = ble.advType;
  status.mode = modeName(currentMode);
  status.lastError = lastError;
  status.scanLastError = scanLastError;
  status.scanLastMfgHead = scanLastMfgHead;
  status.scanLastCandidateReason = scanLastCandidateReason;
  return status;
}

String syncBeaconRadioBuildStatusFields()
{
  const SyncBeaconRadioStatus status = syncBeaconRadioStatus();
  String fields = "sync_radio=";
  fields += status.supported ? 1 : 0;
  fields += " sync_radio_active=";
  fields += status.active ? 1 : 0;
  fields += " beacon_tx=";
  fields += status.beaconTx ? 1 : 0;
  fields += " beacon_rx=";
  fields += status.beaconRx ? 1 : 0;
  fields += " codec_selftest=";
  fields += status.codecSelftest ? 1 : 0;
  fields += " scan_active=";
  fields += status.scanActive ? 1 : 0;
  fields += " adv_active=";
  fields += status.advActive ? 1 : 0;
  fields += " adv_owner=";
  fields += status.advOwner;
  fields += " adv_type=";
  fields += status.advType;
  fields += " gatt_adv_suppressed=";
  fields += status.gattAdvSuppressed ? 1 : 0;
  fields += " beacon_adv_started=";
  fields += status.beaconAdvStarted ? 1 : 0;
  fields += " adv_payload_len=";
  fields += status.advPayloadLen;
  fields += " adv_mfg_len=";
  fields += status.advMfgLen;
  fields += " adv_company=";
  fields += formatHex16(status.advCompany);
  fields += " adv_magic=";
  fields += formatHex16(status.advMagic);
  fields += " adv_version=";
  fields += status.advVersion;
  fields += " adv_group=";
  fields += status.advGroup;
  fields += " adv_crc=";
  fields += formatHex16(status.advCrc);
  fields += " adv_sig=";
  fields += status.advMfgHead;
  fields += " adv_enable_count=";
  fields += status.advEnableCount;
  fields += " adv_disable_count=";
  fields += status.advDisableCount;
  fields += " adv_set_count=";
  fields += status.advSetCount;
  fields += " beacon_adv_refreshes=";
  fields += status.beaconAdvRefreshes;
  fields += " adv_last_error=";
  fields += status.lastError;
  fields += " beacon_seq=";
  fields += status.beaconSeq;
  fields += " beacon_tx_count=";
  fields += status.txCount;
  fields += " beacon_rx_count=";
  fields += status.rxCount;
  fields += " beacon_crc_errors=";
  fields += status.crcErrors;
  fields += " beacon_group_mismatch=";
  fields += status.groupMismatch;
  fields += " beacon_invalid=";
  fields += status.invalidPackets;
  fields += " scan_reports=";
  fields += status.scanReports;
  fields += " scan_mfg_reports=";
  fields += status.scanMfgReports;
  fields += " scan_nk_candidates=";
  fields += status.scanNkCandidates;
  fields += " scan_decode_ok=";
  fields += status.scanDecodeOk;
  fields += " scan_decode_fail=";
  fields += status.scanDecodeFail;
  fields += " scan_crc_fail=";
  fields += status.scanCrcFail;
  fields += " scan_group_mismatch=";
  fields += status.scanGroupMismatch;
  fields += " scan_reject_company=";
  fields += status.scanRejectCompany;
  fields += " scan_reject_magic=";
  fields += status.scanRejectMagic;
  fields += " scan_reject_len=";
  fields += status.scanRejectLen;
  fields += " scan_reject_version=";
  fields += status.scanRejectVersion;
  fields += " scan_last_rssi=";
  fields += status.scanLastRssi;
  fields += " scan_last_len=";
  fields += status.scanLastLen;
  fields += " scan_last_mfg_len=";
  fields += status.scanLastMfgLen;
  fields += " scan_last_ad_type=";
  fields += formatHex8(status.scanLastAdType);
  fields += " scan_last_company=";
  fields += formatHex16(status.scanLastCompany);
  fields += " scan_last_mfg_head=";
  fields += status.scanLastMfgHead;
  fields += " scan_last_group=";
  fields += status.scanLastGroup;
  fields += " scan_last_version=";
  fields += status.scanLastVersion;
  fields += " scan_last_error=";
  fields += status.scanLastError;
  fields += " scan_last_candidate_reason=";
  fields += status.scanLastCandidateReason;
  fields += " last_beacon_ms=";
  fields += status.lastBeaconMs;
  fields += " beacon_age_ms=";
  fields += status.beaconAgeMs;
  fields += " sync_locked=";
  fields += status.locked ? 1 : 0;
  fields += " radio_mode=";
  fields += status.mode;
  fields += " sync_radio_error=";
  fields += status.lastError;
  fields += " radio_last_error=";
  fields += status.lastError;
  return fields;
}

String syncBeaconAudioBuildStatusFields()
{
  const SyncBeaconRadioStatus status = syncBeaconRadioStatus();
  String fields = "audio_sync=";
  fields += status.supported ? 1 : 0;
  fields += " audio_valid=";
  fields += status.audio.valid ? 1 : 0;
  fields += " last_beacon_version=";
  fields += status.lastBeaconVersion;
  fields += " scan_decode_v1=";
  fields += status.scanDecodeV1;
  fields += " scan_decode_v2=";
  fields += status.scanDecodeV2;
  fields += " audio_seq=";
  fields += status.audio.seq;
  fields += " audio_age_ms=";
  fields += status.audioAgeMs;
  fields += " audio_beat=";
  fields += status.audio.beat ? 1 : 0;
  fields += " audio_energy=";
  fields += status.audio.energy;
  fields += " audio_bass=";
  fields += status.audio.bass;
  fields += " audio_mid=";
  fields += status.audio.mid;
  fields += " audio_treble=";
  fields += status.audio.treble;
  fields += " audio_confidence=";
  fields += status.audio.confidence;
  fields += " audio_phase_ms=";
  fields += status.audio.phaseMs;
  fields += " audio_beat_ms=";
  fields += status.audio.beatMs;
  fields += " audio_timeout_ms=";
  fields += NK_AUDIO_FRESHNESS_TIMEOUT_MS;
  return fields;
}

bool syncBeaconRadioConsumeBeacon(NkSyncBeaconV1* beacon)
{
#if NIGHTKITE_BLE && NIGHTKITE_RM2 && defined(PIO_FRAMEWORK_ARDUINO_ENABLE_BLUETOOTH) && defined(PICO_CYW43_SUPPORTED)
  if (!rm2BleStatus().initialized) return false;
  BluetoothLock lock;
#endif
  if (!pendingBeaconAvailable || beacon == nullptr)
  {
    return false;
  }
  *beacon = pendingBeacon;
  pendingBeaconAvailable = false;
  return true;
}

AudioSyncState syncBeaconAudioState()
{
#if NIGHTKITE_BLE && NIGHTKITE_RM2 && defined(PIO_FRAMEWORK_ARDUINO_ENABLE_BLUETOOTH) && defined(PICO_CYW43_SUPPORTED)
  if (!rm2BleStatus().initialized) return AudioSyncState{};
  BluetoothLock lock;
#endif
  expireAudioSyncState(millis());
  return audioSyncState;
}


bool syncBeaconRadioConsumeShow(ShowScheduledEvent* event)
{
#if NIGHTKITE_BLE && NIGHTKITE_RM2 && defined(PIO_FRAMEWORK_ARDUINO_ENABLE_BLUETOOTH) && defined(PICO_CYW43_SUPPORTED)
  if (!rm2BleStatus().initialized || event == nullptr) return false;
  BluetoothLock lock;
  return showScheduler.popDue(millis(), *event);
#else
  (void)event;
  return false;
#endif
}

void syncBeaconRadioResetShow()
{
#if NIGHTKITE_BLE && NIGHTKITE_RM2 && defined(PIO_FRAMEWORK_ARDUINO_ENABLE_BLUETOOTH) && defined(PICO_CYW43_SUPPORTED)
  if (!rm2BleStatus().initialized) return;
  BluetoothLock lock;
#endif
  showScheduler.reset();
}

ShowRadioStatus syncBeaconShowStatus()
{
  ShowRadioStatus status;
#if NIGHTKITE_BLE && NIGHTKITE_RM2 && defined(PIO_FRAMEWORK_ARDUINO_ENABLE_BLUETOOTH) && defined(PICO_CYW43_SUPPORTED)
  if (!rm2BleStatus().initialized) return status;
  BluetoothLock lock;
#endif
  status.receiving = isReceiveMode() && showPacketsEnabled;
  status.queueDepth = showScheduler.depth();
  status.clockValid = showScheduler.clockValid();
  status.clockOffsetMs = showScheduler.clockOffsetMs();
  status.clockAgeMs = showScheduler.clockAgeMs(millis());
  status.scheduler = showScheduler.status;
  status.received = showReceived;
  status.invalid = showInvalid;
  status.crcErrors = showCrcErrors;
  status.targetMiss = showTargetMiss;
  return status;
}
