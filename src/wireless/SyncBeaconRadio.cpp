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
constexpr uint8_t BEACON_MAGIC_0 = 'N';
constexpr uint8_t BEACON_MAGIC_1 = 'K';
constexpr size_t BEACON_PACKET_SIZE = sizeof(NkSyncBeaconV1);
constexpr uint16_t BEACON_COMPANY_ID = 0xFFFF; // Experimental manufacturer-specific payload.
constexpr uint8_t ADV_TYPE_NONCONNECTABLE = 3; // ADV_NONCONN_IND.
constexpr unsigned long FOLLOWER_LOST_MS = 1500;

enum RadioMode : uint8_t
{
  RADIO_MODE_OFF,
  RADIO_MODE_GATT,
  RADIO_MODE_BEACON_MASTER,
  RADIO_MODE_BEACON_FOLLOWER
};

RadioMode currentMode = RADIO_MODE_OFF;
bool beginCalled = false;
bool scanning = false;
uint8_t activeGroup = 1;
uint16_t beaconSeq = 0;
unsigned long nextTxMs = 0;
unsigned long txCount = 0;
unsigned long rxCount = 0;
unsigned long crcErrors = 0;
unsigned long groupMismatch = 0;
unsigned long invalidPackets = 0;
unsigned long lastBeaconMs = 0;
const char* lastError = "disabled";
NkSyncBeaconV1 pendingBeacon;
bool pendingBeaconAvailable = false;

#if NIGHTKITE_BLE && NIGHTKITE_RM2 && defined(PIO_FRAMEWORK_ARDUINO_ENABLE_BLUETOOTH) && defined(PICO_CYW43_SUPPORTED)
btstack_packet_callback_registration_t syncRadioHciCallbackRegistration;
#endif

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
  if (output == nullptr || outputLen == nullptr || outputSize < 24)
  {
    return false;
  }

  uint8_t pos = 0;
  output[pos++] = 2;
  output[pos++] = BLUETOOTH_DATA_TYPE_FLAGS;
  output[pos++] = 0x06;

  output[pos++] = (uint8_t)(BEACON_PACKET_SIZE + 3);
  output[pos++] = BLUETOOTH_DATA_TYPE_MANUFACTURER_SPECIFIC_DATA;
  output[pos++] = (uint8_t)(BEACON_COMPANY_ID & 0xFF);
  output[pos++] = (uint8_t)(BEACON_COMPANY_ID >> 8);
  memcpy(&output[pos], &beacon, BEACON_PACKET_SIZE);
  pos += BEACON_PACKET_SIZE;
  *outputLen = pos;
  return true;
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

void handleAdvertisementData(const uint8_t* advData, uint8_t advLen)
{
  ad_context_t context;
  for (ad_iterator_init(&context, advLen, (uint8_t*)advData); ad_iterator_has_more(&context); ad_iterator_next(&context))
  {
    const uint8_t dataType = ad_iterator_get_data_type(&context);
    const uint8_t dataLen = ad_iterator_get_data_len(&context);
    const uint8_t* data = ad_iterator_get_data(&context);
    if (dataType != BLUETOOTH_DATA_TYPE_MANUFACTURER_SPECIFIC_DATA || dataLen < BEACON_PACKET_SIZE + 2)
    {
      continue;
    }
    const uint16_t companyId = little_endian_read_16(data, 0);
    if (companyId != BEACON_COMPANY_ID)
    {
      continue;
    }

    NkSyncBeaconV1 beacon;
    const SyncBeaconDecodeResult result = syncBeaconDecode(&data[2], dataLen - 2, activeGroup, &beacon);
    if (result == SYNC_BEACON_DECODE_OK)
    {
      pendingBeacon = beacon;
      pendingBeaconAvailable = true;
      rxCount++;
      lastBeaconMs = millis();
      lastError = "none";
    }
    else if (result == SYNC_BEACON_DECODE_BAD_GROUP)
    {
      groupMismatch++;
    }
    else if (result == SYNC_BEACON_DECODE_BAD_CRC)
    {
      crcErrors++;
      lastError = "crc";
    }
    else
    {
      invalidPackets++;
      lastError = syncBeaconDecodeResultName(result);
    }
  }
}

void hciPacketHandler(uint8_t packetType, uint16_t channel, uint8_t* packet, uint16_t size)
{
  (void)channel;
  (void)size;
  if (packetType != HCI_EVENT_PACKET || hci_event_packet_get_type(packet) != GAP_EVENT_ADVERTISING_REPORT)
  {
    return;
  }
  const uint8_t advLen = gap_event_advertising_report_get_data_length(packet);
  const uint8_t* advData = gap_event_advertising_report_get_data(packet);
  handleAdvertisementData(advData, advLen);
}
#else
void stopScan() {}
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
  nextTxMs = 0;
}

void sendMasterBeacon(const SyncBeaconRuntime& runtime)
{
#if NIGHTKITE_BLE && NIGHTKITE_RM2 && defined(PIO_FRAMEWORK_ARDUINO_ENABLE_BLUETOOTH) && defined(PICO_CYW43_SUPPORTED)
  NkSyncBeaconV1 beacon;
  beacon.magic0 = BEACON_MAGIC_0;
  beacon.magic1 = BEACON_MAGIC_1;
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
    lastError = "adv_payload";
    return;
  }

  const uint16_t interval = advIntervalUnitsForProfile(runtime.wirelessProfile);
  if (!rm2BleUseSyncAdvertising(advData, advLen, interval, (uint16_t)(interval + 16), ADV_TYPE_NONCONNECTABLE))
  {
    lastError = "adv_failed";
    return;
  }

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
  lastError = "none";
#if NIGHTKITE_BLE && NIGHTKITE_RM2 && defined(PIO_FRAMEWORK_ARDUINO_ENABLE_BLUETOOTH) && defined(PICO_CYW43_SUPPORTED)
  syncRadioHciCallbackRegistration.callback = &hciPacketHandler;
  hci_add_event_handler(&syncRadioHciCallbackRegistration);
#endif
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
      runtime.playMode == SYNC_BEACON_PLAY_SYNC &&
      (runtime.syncRole == SYNC_BEACON_ROLE_MASTER || runtime.syncRole == SYNC_BEACON_ROLE_FOLLOWER);
  if (!syncActive)
  {
    leaveBeaconMode();
    lastError = "none";
    return;
  }

  if (ble.connected)
  {
#if NIGHTKITE_BLE && NIGHTKITE_RM2 && defined(PIO_FRAMEWORK_ARDUINO_ENABLE_BLUETOOTH) && defined(PICO_CYW43_SUPPORTED)
    stopScan();
#endif
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
  SyncBeaconRadioStatus status;
  status.supported = NIGHTKITE_BLE && NIGHTKITE_RM2;
  status.active = currentMode == RADIO_MODE_BEACON_MASTER || currentMode == RADIO_MODE_BEACON_FOLLOWER;
  status.beaconTx = currentMode == RADIO_MODE_BEACON_MASTER;
  status.beaconRx = currentMode == RADIO_MODE_BEACON_FOLLOWER;
  status.locked = status.beaconTx || (status.beaconRx && lastBeaconMs > 0 && (millis() - lastBeaconMs) <= FOLLOWER_LOST_MS);
  status.beaconSeq = beaconSeq;
  status.txCount = txCount;
  status.rxCount = rxCount;
  status.crcErrors = crcErrors;
  status.groupMismatch = groupMismatch;
  status.invalidPackets = invalidPackets;
  status.lastBeaconMs = lastBeaconMs;
  status.beaconAgeMs = lastBeaconMs == 0 ? 0 : millis() - lastBeaconMs;
  status.mode = modeName(currentMode);
  status.lastError = lastError;
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
  if (output == nullptr || outputLen == nullptr || outputSize < BEACON_PACKET_SIZE)
  {
    return false;
  }
  NkSyncBeaconV1 copy = beacon;
  copy.magic0 = BEACON_MAGIC_0;
  copy.magic1 = BEACON_MAGIC_1;
  copy.version = NK_SYNC_BEACON_VERSION;
  copy.crc = computeBeaconCrc(copy);
  memcpy(output, &copy, BEACON_PACKET_SIZE);
  *outputLen = BEACON_PACKET_SIZE;
  return true;
}

SyncBeaconDecodeResult syncBeaconDecode(const uint8_t* data, size_t dataLen, uint8_t expectedGroup, NkSyncBeaconV1* beacon)
{
  if (data == nullptr || beacon == nullptr || dataLen < BEACON_PACKET_SIZE)
  {
    return SYNC_BEACON_DECODE_TOO_SHORT;
  }

  NkSyncBeaconV1 decoded;
  memcpy(&decoded, data, BEACON_PACKET_SIZE);
  if (decoded.magic0 != BEACON_MAGIC_0 || decoded.magic1 != BEACON_MAGIC_1)
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
