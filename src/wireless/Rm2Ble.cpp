#include "Rm2Ble.h"

#ifndef PIN_RM2_WL_ON
#define PIN_RM2_WL_ON 17
#endif
#ifndef PIN_RM2_BL_ON
#define PIN_RM2_BL_ON PIN_RM2_WL_ON
#endif
#ifndef PIN_RM2_WL_CS
#define PIN_RM2_WL_CS 18
#endif
#ifndef PIN_RM2_WL_CLK
#define PIN_RM2_WL_CLK 19
#endif
#ifndef PIN_RM2_WL_DATA
#define PIN_RM2_WL_DATA 20
#endif
#ifndef PIN_RM2_WL_WAKE
#define PIN_RM2_WL_WAKE PIN_RM2_WL_DATA
#endif
#ifndef NIGHTKITE_BLE
#define NIGHTKITE_BLE 0
#endif
#ifndef NIGHTKITE_RM2
#define NIGHTKITE_RM2 0
#endif

#if NIGHTKITE_BLE && NIGHTKITE_RM2 && defined(PIO_FRAMEWORK_ARDUINO_ENABLE_BLUETOOTH) && defined(PICO_CYW43_SUPPORTED)
#include <btstack.h>
#include <ble/att_db_util.h>
#include <pico/cyw43_arch.h>
#include <pico/cyw43_driver.h>
#endif

namespace
{
constexpr size_t BLE_NAME_MAX = 24;
constexpr size_t BLE_COMMAND_MAX = 192;
constexpr size_t BLE_COMMAND_QUEUE_DEPTH = 4;
constexpr size_t BLE_TX_BUFFER_MAX = 1024;
constexpr size_t BLE_TX_QUEUE_DEPTH = 4;
constexpr size_t BLE_NOTIFY_CHUNK_SIZE = 20;
constexpr unsigned long BLE_NOTIFY_PACE_MS = 5;
constexpr unsigned long BLE_TX_STALL_TIMEOUT_MS = 2000;

bool beginCalled = false;
bool enabled = false;
bool initialized = false;
bool advertising = false;
bool connected = false;
bool gattReady = false;
bool rxReady = false;
bool txReady = false;
char bleName[BLE_NAME_MAX] = "disabled";
const char* lastError = "disabled";
Rm2BleNk4Handler nk4Handler = nullptr;
Rm2BleGapReportHandler gapReportHandler = nullptr;
unsigned long txDroppedCount = 0;
unsigned long txChunksSentCount = 0;

#if NIGHTKITE_BLE && NIGHTKITE_RM2 && defined(PIO_FRAMEWORK_ARDUINO_ENABLE_BLUETOOTH) && defined(PICO_CYW43_SUPPORTED)
btstack_packet_callback_registration_t hciEventCallbackRegistration;
uint8_t advData[31];
uint8_t advDataLen = 0;
hci_con_handle_t connectionHandle = HCI_CON_HANDLE_INVALID;
uint16_t rxValueHandle = 0;
uint16_t txValueHandle = 0;
uint16_t txClientConfigHandle = 0;
bool txNotificationsEnabled = false;
char rxLineBuffer[BLE_COMMAND_MAX];
size_t rxLineLen = 0;
bool rxDroppingLongLine = false;
bool rxRangeErrorPending = false;
bool rxQueueFullErrorPending = false;
char commandQueue[BLE_COMMAND_QUEUE_DEPTH][BLE_COMMAND_MAX];
size_t commandQueueLen[BLE_COMMAND_QUEUE_DEPTH];
uint8_t commandQueueHead = 0;
uint8_t commandQueueTail = 0;
uint8_t commandQueueCount = 0;
char txQueue[BLE_TX_QUEUE_DEPTH][BLE_TX_BUFFER_MAX];
size_t txQueueLen[BLE_TX_QUEUE_DEPTH];
uint8_t txQueueHead = 0;
uint8_t txQueueTail = 0;
uint8_t txQueueCount = 0;
size_t txOffset = 0;
unsigned long lastNotifyChunkMs = 0;
unsigned long lastTxProgressMs = 0;

// 4e4b4000-6e69-6768-746b-000000000001
const uint8_t NIGHTKITE_SERVICE_UUID[16] = {
  0x4e, 0x4b, 0x40, 0x00, 0x6e, 0x69, 0x67, 0x68,
  0x74, 0x6b, 0x00, 0x00, 0x00, 0x00, 0x00, 0x01
};
// 4e4b4000-6e69-6768-746b-000000000002
const uint8_t NIGHTKITE_RX_UUID[16] = {
  0x4e, 0x4b, 0x40, 0x00, 0x6e, 0x69, 0x67, 0x68,
  0x74, 0x6b, 0x00, 0x00, 0x00, 0x00, 0x00, 0x02
};
// 4e4b4000-6e69-6768-746b-000000000003
const uint8_t NIGHTKITE_TX_UUID[16] = {
  0x4e, 0x4b, 0x40, 0x00, 0x6e, 0x69, 0x67, 0x68,
  0x74, 0x6b, 0x00, 0x00, 0x00, 0x00, 0x00, 0x03
};

void setLastError(const char* error)
{
  lastError = error;
}

bool appendAdvField(uint8_t type, const uint8_t* value, uint8_t len)
{
  if ((uint16_t)advDataLen + len + 2 > sizeof(advData))
  {
    return false;
  }
  advData[advDataLen++] = len + 1;
  advData[advDataLen++] = type;
  memcpy(&advData[advDataLen], value, len);
  advDataLen += len;
  return true;
}

void reverseUuid128(const uint8_t* input, uint8_t* output)
{
  for (uint8_t i = 0; i < 16; i++)
  {
    output[i] = input[15 - i];
  }
}

bool buildAdvertisingData(const char* name)
{
  advDataLen = 0;
  const uint8_t flags = 0x06; // LE General Discoverable, BR/EDR not supported.
  if (!appendAdvField(BLUETOOTH_DATA_TYPE_FLAGS, &flags, 1))
  {
    return false;
  }

  const size_t nameLen = strnlen(name, BLE_NAME_MAX - 1);
  if (!appendAdvField(BLUETOOTH_DATA_TYPE_COMPLETE_LOCAL_NAME, (const uint8_t*)name, (uint8_t)nameLen))
  {
    return false;
  }

  uint8_t advServiceUuid[16];
  reverseUuid128(NIGHTKITE_SERVICE_UUID, advServiceUuid);
  appendAdvField(BLUETOOTH_DATA_TYPE_COMPLETE_LIST_OF_128_BIT_SERVICE_CLASS_UUIDS, advServiceUuid, sizeof(advServiceUuid));
  return true;
}

bool enqueueCommandLine(const char* line, size_t lineLen)
{
  if (commandQueueCount >= BLE_COMMAND_QUEUE_DEPTH)
  {
    rxQueueFullErrorPending = true;
    lastError = "rx_queue_full";
    return false;
  }
  if (lineLen >= BLE_COMMAND_MAX)
  {
    rxRangeErrorPending = true;
    lastError = "rx_line_too_long";
    return false;
  }

  memcpy(commandQueue[commandQueueTail], line, lineLen);
  commandQueue[commandQueueTail][lineLen] = '\0';
  commandQueueLen[commandQueueTail] = lineLen;
  commandQueueTail = (uint8_t)((commandQueueTail + 1) % BLE_COMMAND_QUEUE_DEPTH);
  commandQueueCount++;
  return true;
}

bool dequeueCommandLine(String& lineOut)
{
  if (commandQueueCount == 0)
  {
    return false;
  }
  lineOut = String(commandQueue[commandQueueHead]);
  commandQueueHead = (uint8_t)((commandQueueHead + 1) % BLE_COMMAND_QUEUE_DEPTH);
  commandQueueCount--;
  return true;
}

void resetTxQueue()
{
  txQueueHead = 0;
  txQueueTail = 0;
  txQueueCount = 0;
  txOffset = 0;
  lastTxProgressMs = 0;
}

void resetRxQueue()
{
  rxLineLen = 0;
  rxDroppingLongLine = false;
  rxRangeErrorPending = false;
  rxQueueFullErrorPending = false;
  commandQueueHead = 0;
  commandQueueTail = 0;
  commandQueueCount = 0;
}

bool enqueueTxLine(const char* data, size_t dataLen)
{
  if (!txNotificationsEnabled || connectionHandle == HCI_CON_HANDLE_INVALID)
  {
    lastError = "notify_disabled";
    return false;
  }
  if (txQueueCount >= BLE_TX_QUEUE_DEPTH)
  {
    txDroppedCount++;
    lastError = "tx_queue_full";
    return false;
  }

  if (dataLen >= BLE_TX_BUFFER_MAX)
  {
    dataLen = BLE_TX_BUFFER_MAX - 1;
    lastError = "tx_truncated";
  }

  memcpy(txQueue[txQueueTail], data, dataLen);
  txQueue[txQueueTail][dataLen] = '\0';
  txQueueLen[txQueueTail] = dataLen;
  txQueueTail = (uint8_t)((txQueueTail + 1) % BLE_TX_QUEUE_DEPTH);
  txQueueCount++;
  if (txQueueCount == 1)
  {
    lastTxProgressMs = millis();
  }
  return true;
}

void dropCurrentTxLine(const char* error)
{
  if (txQueueCount == 0)
  {
    return;
  }
  txQueueHead = (uint8_t)((txQueueHead + 1) % BLE_TX_QUEUE_DEPTH);
  txQueueCount--;
  txOffset = 0;
  txDroppedCount++;
  lastError = error;
  lastTxProgressMs = txQueueCount > 0 ? millis() : 0;
}

void sendNextTxChunk()
{
  if (txQueueCount == 0 || !txNotificationsEnabled || connectionHandle == HCI_CON_HANDLE_INVALID)
  {
    resetTxQueue();
    return;
  }
  if (millis() - lastNotifyChunkMs < BLE_NOTIFY_PACE_MS)
  {
    return;
  }
  if (!att_server_can_send_packet_now(connectionHandle))
  {
    if (lastTxProgressMs > 0 && millis() - lastTxProgressMs > BLE_TX_STALL_TIMEOUT_MS)
    {
      dropCurrentTxLine("notify_stalled");
    }
    return;
  }

  const size_t txLen = txQueueLen[txQueueHead];
  const size_t remaining = txLen - txOffset;
  const size_t chunkLen = remaining > BLE_NOTIFY_CHUNK_SIZE ? BLE_NOTIFY_CHUNK_SIZE : remaining;
  const uint8_t result = att_server_notify(connectionHandle, txValueHandle, (uint8_t*)&txQueue[txQueueHead][txOffset], (uint16_t)chunkLen);
  if (result != ERROR_CODE_SUCCESS)
  {
    if (result == BTSTACK_ACL_BUFFERS_FULL)
    {
      lastError = "notify_busy";
      return;
    }

    dropCurrentTxLine("notify_failed");
    return;
  }

  lastNotifyChunkMs = millis();
  lastTxProgressMs = lastNotifyChunkMs;
  txChunksSentCount++;
  txOffset += chunkLen;
  if (txOffset >= txLen)
  {
    txQueueHead = (uint8_t)((txQueueHead + 1) % BLE_TX_QUEUE_DEPTH);
    txQueueCount--;
    txOffset = 0;
    lastError = "none";
  }
}

void enqueueNk4ErrorLine(const char* code, const char* msg)
{
  char line[96];
  const int written = snprintf(line, sizeof(line), "NK4 seq=0 err code=%s msg=%s\n", code, msg);
  if (written > 0)
  {
    enqueueTxLine(line, (size_t)written);
  }
}

void consumeRxBytes(const uint8_t* data, uint16_t dataLen)
{
  for (uint16_t i = 0; i < dataLen; i++)
  {
    const char ch = (char)data[i];
    if (rxDroppingLongLine)
    {
      if (ch == '\n')
      {
        rxDroppingLongLine = false;
        rxRangeErrorPending = true;
      }
      continue;
    }

    if (ch == '\r')
    {
      continue;
    }
    if (ch == '\n')
    {
      if (rxLineLen > 0)
      {
        enqueueCommandLine(rxLineBuffer, rxLineLen);
        rxLineLen = 0;
      }
      continue;
    }
    if (rxLineLen >= BLE_COMMAND_MAX - 1)
    {
      rxLineLen = 0;
      rxDroppingLongLine = true;
      lastError = "rx_line_too_long";
      continue;
    }
    rxLineBuffer[rxLineLen++] = ch;
  }
}

uint16_t attReadCallback(hci_con_handle_t conHandle, uint16_t attHandle, uint16_t offset, uint8_t* buffer, uint16_t bufferSize)
{
  (void)conHandle;
  if (attHandle == txValueHandle)
  {
    const char* ready = "NK4 BLE TX notify";
    return att_read_callback_handle_blob((const uint8_t*)ready, (uint16_t)strlen(ready), offset, buffer, bufferSize);
  }
  return 0;
}

int attWriteCallback(hci_con_handle_t conHandle, uint16_t attHandle, uint16_t transactionMode, uint16_t offset, uint8_t* buffer, uint16_t bufferSize)
{
  (void)transactionMode;
  (void)offset;
  if (attHandle == txClientConfigHandle)
  {
    txNotificationsEnabled = little_endian_read_16(buffer, 0) == GATT_CLIENT_CHARACTERISTICS_CONFIGURATION_NOTIFICATION;
    connectionHandle = conHandle;
    return 0;
  }
  if (attHandle == rxValueHandle)
  {
    connectionHandle = conHandle;
    consumeRxBytes(buffer, bufferSize);
    return 0;
  }
  return 0;
}

void hciPacketHandler(uint8_t packetType, uint16_t channel, uint8_t* packet, uint16_t size)
{
  (void)channel;
  (void)size;
  if (packetType != HCI_EVENT_PACKET)
  {
    return;
  }

  if (hci_event_packet_get_type(packet) == BTSTACK_EVENT_STATE)
  {
    if (btstack_event_state_get_state(packet) == HCI_STATE_WORKING)
    {
      initialized = true;
      gap_advertisements_enable(1);
      advertising = true;
      setLastError("none");
    }
    return;
  }

  if (hci_event_packet_get_type(packet) == HCI_EVENT_LE_META)
  {
    if (hci_event_le_meta_get_subevent_code(packet) == HCI_SUBEVENT_LE_CONNECTION_COMPLETE)
    {
      connectionHandle = gap_subevent_le_connection_complete_get_connection_handle(packet);
      connected = true;
      advertising = false;
      return;
    }
  }

  if (hci_event_packet_get_type(packet) == GAP_EVENT_ADVERTISING_REPORT)
  {
    if (gapReportHandler != nullptr)
    {
      gapReportHandler(
          gap_event_advertising_report_get_data(packet),
          gap_event_advertising_report_get_data_length(packet),
          gap_event_advertising_report_get_rssi(packet));
    }
    return;
  }

  if (hci_event_packet_get_type(packet) == HCI_EVENT_DISCONNECTION_COMPLETE)
  {
    connected = false;
    connectionHandle = HCI_CON_HANDLE_INVALID;
    txNotificationsEnabled = false;
    resetTxQueue();
    resetRxQueue();
    gap_advertisements_enable(1);
    advertising = true;
    return;
  }

  if (hci_event_packet_get_type(packet) == ATT_EVENT_CONNECTED)
  {
    connectionHandle = att_event_connected_get_handle(packet);
    connected = true;
    advertising = false;
    return;
  }

  if (hci_event_packet_get_type(packet) == ATT_EVENT_CAN_SEND_NOW)
  {
    // TX progression is driven from rm2BleTick() to keep pacing deterministic.
  }
}

bool configureRm2Pins()
{
  pinMode(PIN_RM2_WL_ON, OUTPUT);
  digitalWrite(PIN_RM2_WL_ON, HIGH);
#if PIN_RM2_BL_ON != PIN_RM2_WL_ON
  pinMode(PIN_RM2_BL_ON, OUTPUT);
  digitalWrite(PIN_RM2_BL_ON, HIGH);
#endif
  delay(25);

  uint pins[CYW43_PIN_INDEX_WL_COUNT] = {
    PIN_RM2_WL_ON,
    PIN_RM2_WL_DATA,
    PIN_RM2_WL_DATA,
    PIN_RM2_WL_WAKE,
    PIN_RM2_WL_CLK,
    PIN_RM2_WL_CS
  };
  const int result = cyw43_set_pins_wl(pins);
  if (result != PICO_OK)
  {
    setLastError("rm2_pin_config_failed");
    return false;
  }
  return true;
}

bool configureGattDatabase()
{
  att_db_util_init();
  att_db_util_add_service_uuid16(GAP_SERVICE_UUID);
  att_db_util_add_characteristic_uuid16(GAP_DEVICE_NAME_UUID, ATT_PROPERTY_READ, ATT_SECURITY_NONE, ATT_SECURITY_NONE, (uint8_t*)bleName, (uint16_t)strlen(bleName));
  att_db_util_add_service_uuid128(NIGHTKITE_SERVICE_UUID);
  rxValueHandle = att_db_util_add_characteristic_uuid128(NIGHTKITE_RX_UUID, ATT_PROPERTY_WRITE | ATT_PROPERTY_WRITE_WITHOUT_RESPONSE | ATT_PROPERTY_DYNAMIC, ATT_SECURITY_NONE, ATT_SECURITY_NONE, NULL, 0);
  txValueHandle = att_db_util_add_characteristic_uuid128(NIGHTKITE_TX_UUID, ATT_PROPERTY_READ | ATT_PROPERTY_NOTIFY | ATT_PROPERTY_DYNAMIC, ATT_SECURITY_NONE, ATT_SECURITY_NONE, NULL, 0);
  txClientConfigHandle = txValueHandle + 1;
  att_server_init(att_db_util_get_address(), attReadCallback, attWriteCallback);
  att_server_register_packet_handler(hciPacketHandler);
  gattReady = true;
  rxReady = rxValueHandle != 0;
  txReady = txValueHandle != 0;
  return rxReady && txReady;
}

void configureGattAdvertisingParams()
{
  bd_addr_t nullAddress;
  memset(nullAddress, 0, sizeof(nullAddress));
  const uint16_t advIntervalMin = 0x00A0; // 100 ms
  const uint16_t advIntervalMax = 0x00F0; // 150 ms
  const uint8_t advType = 0; // ADV_IND: connectable undirected advertising.
  gap_advertisements_set_params(advIntervalMin, advIntervalMax, advType, 0, nullAddress, 0x07, 0x00);
}
#endif

class BleResponseWriter : public IResponseWriter
{
public:
  void print(const char* value) override { append(value != nullptr ? value : ""); }
  void print(const String& value) override { append(value.c_str()); }
  void print(int value) override { append(String(value).c_str()); }
  void print(unsigned int value) override { append(String(value).c_str()); }
  void print(unsigned long value) override { append(String(value).c_str()); }
  void println() override
  {
#if NIGHTKITE_BLE && NIGHTKITE_RM2 && defined(PIO_FRAMEWORK_ARDUINO_ENABLE_BLUETOOTH) && defined(PICO_CYW43_SUPPORTED)
    append("\n");
    enqueueTxLine(buffer, length);
#endif
    reset();
  }

private:
  char buffer[BLE_TX_BUFFER_MAX] = {0};
  size_t length = 0;

  void reset()
  {
    length = 0;
    buffer[0] = '\0';
  }

  void append(const char* value)
  {
    if (value == nullptr)
    {
      return;
    }
    const size_t available = (length < sizeof(buffer)) ? (sizeof(buffer) - length - 1) : 0;
    if (available == 0)
    {
      lastError = "tx_line_too_long";
      return;
    }
    const size_t copyLen = strnlen(value, available);
    memcpy(&buffer[length], value, copyLen);
    length += copyLen;
    buffer[length] = '\0';
    if (value[copyLen] != '\0')
    {
      lastError = "tx_line_too_long";
    }
  }
};
}

void rm2BleSetNk4Handler(Rm2BleNk4Handler handler)
{
  nk4Handler = handler;
}

void rm2BleSetGapReportHandler(Rm2BleGapReportHandler handler)
{
  gapReportHandler = handler;
}

bool rm2BleBegin(const char* advertisedName)
{
  if (beginCalled)
  {
    return initialized;
  }
  beginCalled = true;

  if (advertisedName != nullptr && advertisedName[0] != '\0')
  {
    strncpy(bleName, advertisedName, sizeof(bleName) - 1);
    bleName[sizeof(bleName) - 1] = '\0';
  }
  else
  {
    strncpy(bleName, "NightKite", sizeof(bleName) - 1);
    bleName[sizeof(bleName) - 1] = '\0';
  }

#if NIGHTKITE_BLE && NIGHTKITE_RM2 && defined(PIO_FRAMEWORK_ARDUINO_ENABLE_BLUETOOTH) && defined(PICO_CYW43_SUPPORTED)
  enabled = true;
  initialized = false;
  advertising = false;
  connected = false;
  gattReady = false;
  rxReady = false;
  txReady = false;
  resetTxQueue();
  resetRxQueue();
  lastError = "starting";

  if (!configureRm2Pins())
  {
    return false;
  }

  const int cyw43Result = cyw43_arch_init();
  if (cyw43Result != 0)
  {
    lastError = "cyw43_init_failed";
    return false;
  }

  if (!buildAdvertisingData(bleName))
  {
    lastError = "adv_data_too_long";
    return false;
  }

  l2cap_init();
  sm_init();
  if (!configureGattDatabase())
  {
    lastError = "gatt_init_failed";
    return false;
  }

  hciEventCallbackRegistration.callback = &hciPacketHandler;
  hci_add_event_handler(&hciEventCallbackRegistration);

  configureGattAdvertisingParams();
  gap_advertisements_set_data(advDataLen, advData);
  gap_set_local_name(bleName);

  if (hci_power_control(HCI_POWER_ON) != 0)
  {
    lastError = "hci_power_on_failed";
    return false;
  }
  return true;
#else
  enabled = false;
  initialized = false;
  advertising = false;
  connected = false;
  gattReady = false;
  rxReady = false;
  txReady = false;
#if NIGHTKITE_BLE && NIGHTKITE_RM2
  lastError = "btstack_not_enabled";
#else
  lastError = "disabled";
#endif
  return false;
#endif
}

void rm2BleTick()
{
#if NIGHTKITE_BLE && NIGHTKITE_RM2 && defined(PIO_FRAMEWORK_ARDUINO_ENABLE_BLUETOOTH) && defined(PICO_CYW43_SUPPORTED)
  if (!txNotificationsEnabled)
  {
    return;
  }

  if (rxRangeErrorPending && txQueueCount == 0)
  {
    rxRangeErrorPending = false;
    enqueueNk4ErrorLine("range_error", "line_too_long");
  }
  if (rxQueueFullErrorPending && txQueueCount == 0)
  {
    rxQueueFullErrorPending = false;
    enqueueNk4ErrorLine("busy", "rx_queue_full");
  }

  if (txQueueCount == 0 && nk4Handler != nullptr)
  {
    String line;
    if (dequeueCommandLine(line))
    {
      BleResponseWriter writer;
      nk4Handler(line, writer);
    }
  }

  sendNextTxChunk();
#endif
}

Rm2BleStatus rm2BleStatus()
{
  Rm2BleStatus status;
  status.supported = NIGHTKITE_BLE ? true : false;
  status.enabled = enabled;
  status.rm2Enabled = NIGHTKITE_RM2 ? true : false;
  status.initialized = initialized;
  status.advertising = advertising;
  status.connected = connected;
  status.gatt = gattReady;
  status.rx = rxReady;
  status.tx = txReady;
#if NIGHTKITE_BLE && NIGHTKITE_RM2 && defined(PIO_FRAMEWORK_ARDUINO_ENABLE_BLUETOOTH) && defined(PICO_CYW43_SUPPORTED)
  status.txQueue = txQueueCount;
  status.notifyReady = txNotificationsEnabled && connectionHandle != HCI_CON_HANDLE_INVALID;
  status.txActive = txQueueCount > 0;
  status.txOffset = txOffset > 65535 ? 65535 : (uint16_t)txOffset;
#else
  status.txQueue = 0;
  status.notifyReady = false;
  status.txActive = false;
  status.txOffset = 0;
#endif
  status.txDropped = txDroppedCount;
  status.txChunksSent = txChunksSentCount;
  status.name = bleName;
  status.lastError = lastError;
  return status;
}

String rm2BleBuildStatusFields()
{
  const Rm2BleStatus status = rm2BleStatus();
  String fields = "ble_supported=";
  fields += status.supported ? 1 : 0;
  fields += " ble_enabled=";
  fields += status.enabled ? 1 : 0;
  fields += " rm2_enabled=";
  fields += status.rm2Enabled ? 1 : 0;
  fields += " rm2_pins=";
  fields += PIN_RM2_WL_ON;
  fields += ",";
  fields += PIN_RM2_WL_CS;
  fields += ",";
  fields += PIN_RM2_WL_CLK;
  fields += ",";
  fields += PIN_RM2_WL_DATA;
  fields += " rm2_wake=";
  fields += PIN_RM2_WL_WAKE;
  fields += " ble_initialized=";
  fields += status.initialized ? 1 : 0;
  fields += " ble_advertising=";
  fields += status.advertising ? 1 : 0;
  fields += " ble_connected=";
  fields += status.connected ? 1 : 0;
  fields += " ble_gatt=";
  fields += status.gatt ? 1 : 0;
  fields += " ble_rx=";
  fields += status.rx ? 1 : 0;
  fields += " ble_tx=";
  fields += status.tx ? 1 : 0;
  fields += " ble_tx_queue=";
  fields += status.txQueue;
  fields += " ble_tx_dropped=";
  fields += status.txDropped;
  fields += " ble_notify_ready=";
  fields += status.notifyReady ? 1 : 0;
  fields += " ble_tx_active=";
  fields += status.txActive ? 1 : 0;
  fields += " ble_tx_offset=";
  fields += status.txOffset;
  fields += " ble_tx_chunks_sent=";
  fields += status.txChunksSent;
  fields += " ble_name=";
  fields += status.name;
  fields += " last_error=";
  fields += status.lastError;
  fields += " wifi=0";
  return fields;
}

bool rm2BleUseSyncAdvertising(const uint8_t* data, uint8_t dataLen, uint16_t intervalMin, uint16_t intervalMax, uint8_t advType)
{
#if NIGHTKITE_BLE && NIGHTKITE_RM2 && defined(PIO_FRAMEWORK_ARDUINO_ENABLE_BLUETOOTH) && defined(PICO_CYW43_SUPPORTED)
  if (!initialized || connected || data == nullptr || dataLen == 0 || dataLen > 31)
  {
    lastError = connected ? "gatt_connected" : "sync_adv_unavailable";
    return false;
  }

  bd_addr_t nullAddress;
  memset(nullAddress, 0, sizeof(nullAddress));
  gap_advertisements_enable(0);
  gap_advertisements_set_params(intervalMin, intervalMax, advType, 0, nullAddress, 0x07, 0x00);
  gap_advertisements_set_data(dataLen, (uint8_t*)data);
  gap_advertisements_enable(1);
  advertising = true;
  return true;
#else
  (void)data;
  (void)dataLen;
  (void)intervalMin;
  (void)intervalMax;
  (void)advType;
  return false;
#endif
}

void rm2BleStopAdvertising()
{
#if NIGHTKITE_BLE && NIGHTKITE_RM2 && defined(PIO_FRAMEWORK_ARDUINO_ENABLE_BLUETOOTH) && defined(PICO_CYW43_SUPPORTED)
  if (initialized)
  {
    gap_advertisements_enable(0);
    advertising = false;
  }
#endif
}

void rm2BleRestoreGattAdvertising()
{
#if NIGHTKITE_BLE && NIGHTKITE_RM2 && defined(PIO_FRAMEWORK_ARDUINO_ENABLE_BLUETOOTH) && defined(PICO_CYW43_SUPPORTED)
  if (!initialized || connected || advDataLen == 0)
  {
    return;
  }
  configureGattAdvertisingParams();
  gap_advertisements_set_data(advDataLen, advData);
  gap_advertisements_enable(1);
  advertising = true;
#endif
}
