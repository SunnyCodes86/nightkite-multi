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
  RADIO_MODE_BEACON_FOLLOWER
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
const char* lastError = "disabled";
const char* scanLastError = "none";
const char* scanLastCandidateReason = "none";
char advMfgHead[HEX_HEAD_BUFFER_SIZE] = "none";
char scanLastMfgHead[HEX_HEAD_BUFFER_SIZE] = "none";
NkSyncBeaconV1 pendingBeacon;
bool pendingBeaconAvailable = false;

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
    case RADIO_MODE_OFF:
    default: return "off";
  }
}

bool isValidBeaconPattern(uint8_t pattern)
{
  return pattern >= 1 && pattern <= 22;
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

uint16_t crc16Ccitt(const uint8_t* data, size_t len)
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
  NkSyncBeaconV1 copy = beacon;
  copy.crc = 0;
  return crc16Ccitt((const uint8_t*)&copy, sizeof(copy));
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
  return result == SYNC_BEACON_DECODE_OK &&
      decoded.seq == beacon.seq &&
      decoded.groupId == beacon.groupId &&
      decoded.crc == beacon.crc;
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
  if (currentMode != RADIO_MODE_BEACON_FOLLOWER)
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
    const SyncBeaconDecodeResult result = syncBeaconDecode(&data[2], dataLen - 2, activeGroup, &beacon);
    if (result == SYNC_BEACON_DECODE_OK)
    {
      pendingBeacon = beacon;
      pendingBeaconAvailable = true;
      rxCount++;
      scanDecodeOk++;
      lastBeaconMs = millis();
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
  if (currentMode == RADIO_MODE_BEACON_MASTER || currentMode == RADIO_MODE_BEACON_FOLLOWER)
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
  activeGroup = runtime.groupId;

  const Rm2BleStatus ble = rm2BleStatus();
  if (!ble.initialized)
  {
    leaveBeaconMode();
    lastError = ble.supported ? "ble_not_ready" : "unsupported";
    return;
  }

  const bool syncActive = runtime.syncEnabled &&
      runtime.wirelessEnabled &&
      runtime.playMode == SYNC_BEACON_PLAY_SYNC &&
      (runtime.syncRole == SYNC_BEACON_ROLE_MASTER || runtime.syncRole == SYNC_BEACON_ROLE_FOLLOWER);
  if (!syncActive)
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

  if (runtime.syncRole == SYNC_BEACON_ROLE_MASTER)
  {
#if NIGHTKITE_BLE && NIGHTKITE_RM2 && defined(PIO_FRAMEWORK_ARDUINO_ENABLE_BLUETOOTH) && defined(PICO_CYW43_SUPPORTED)
    stopScan();
#endif
    currentMode = RADIO_MODE_BEACON_MASTER;
    const unsigned long nowMs = millis();
    if (nextTxMs == 0 || (int32_t)(nowMs - nextTxMs) >= 0)
    {
      sendMasterBeacon(runtime);
      nextTxMs = nowMs + txIntervalForProfile(runtime.wirelessProfile);
    }
    return;
  }

  currentMode = RADIO_MODE_BEACON_FOLLOWER;
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
  leaveBeaconMode();
}

SyncBeaconRadioStatus syncBeaconRadioStatus()
{
  const Rm2BleStatus ble = rm2BleStatus();
  SyncBeaconRadioStatus status;
  status.supported = NIGHTKITE_BLE && NIGHTKITE_RM2;
  status.active = currentMode == RADIO_MODE_BEACON_MASTER || currentMode == RADIO_MODE_BEACON_FOLLOWER;
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

bool syncBeaconRadioConsumeBeacon(NkSyncBeaconV1* beacon)
{
  if (!pendingBeaconAvailable || beacon == nullptr)
  {
    return false;
  }
  *beacon = pendingBeacon;
  pendingBeaconAvailable = false;
  return true;
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
