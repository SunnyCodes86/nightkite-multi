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
#include <pico/cyw43_arch.h>
#include <pico/cyw43_driver.h>
#endif

namespace
{
constexpr size_t BLE_NAME_MAX = 24;

bool beginCalled = false;
bool enabled = false;
bool initialized = false;
bool advertising = false;
char bleName[BLE_NAME_MAX] = "disabled";
const char* lastError = "disabled";

#if NIGHTKITE_BLE && NIGHTKITE_RM2 && defined(PIO_FRAMEWORK_ARDUINO_ENABLE_BLUETOOTH) && defined(PICO_CYW43_SUPPORTED)
btstack_packet_callback_registration_t hciEventCallbackRegistration;
uint8_t advData[31];
uint8_t advDataLen = 0;

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

  // NightKite service UUID placeholder. GATT command characteristics are added in a later firmware step.
  const uint8_t serviceUuid[16] = {
    0x01, 0x40, 0x4b, 0x4e, 0x10, 0x00, 0x40, 0x00,
    0x80, 0x00, 0x4e, 0x69, 0x67, 0x68, 0x74, 0x4b
  };
  appendAdvField(BLUETOOTH_DATA_TYPE_COMPLETE_LIST_OF_128_BIT_SERVICE_CLASS_UUIDS, serviceUuid, sizeof(serviceUuid));
  return true;
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

  if (hci_event_packet_get_type(packet) == HCI_EVENT_DISCONNECTION_COMPLETE)
  {
    gap_advertisements_enable(1);
    advertising = true;
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
#endif
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

  hciEventCallbackRegistration.callback = &hciPacketHandler;
  hci_add_event_handler(&hciEventCallbackRegistration);

  bd_addr_t nullAddress;
  memset(nullAddress, 0, sizeof(nullAddress));
  const uint16_t advIntervalMin = 0x00A0; // 100 ms
  const uint16_t advIntervalMax = 0x00F0; // 150 ms
  const uint8_t advType = 3; // ADV_NONCONN_IND: discoverable, no command transport yet.
  gap_advertisements_set_params(advIntervalMin, advIntervalMax, advType, 0, nullAddress, 0x07, 0x00);
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
  // The Arduino-Pico BTstack port runs from the CYW43 async context.
}

Rm2BleStatus rm2BleStatus()
{
  Rm2BleStatus status;
  status.supported = NIGHTKITE_BLE ? true : false;
  status.enabled = enabled;
  status.rm2Enabled = NIGHTKITE_RM2 ? true : false;
  status.initialized = initialized;
  status.advertising = advertising;
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
  fields += " ble_name=";
  fields += status.name;
  fields += " last_error=";
  fields += status.lastError;
  fields += " wifi=0";
  return fields;
}
