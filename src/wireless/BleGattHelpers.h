#pragma once

#include <stddef.h>
#include <stdint.h>
#include <string.h>

constexpr size_t BLE_LEGACY_ADV_CAPACITY = 31;
constexpr size_t BLE_GATT_COMMAND_CAPACITY = 192;

struct BleGattAdvertisingData
{
  uint8_t advertising[BLE_LEGACY_ADV_CAPACITY];
  uint8_t scanResponse[BLE_LEGACY_ADV_CAPACITY];
  uint8_t advertisingLength;
  uint8_t scanResponseLength;
};

inline bool appendBleAdvertisingField(uint8_t* data, size_t capacity, uint8_t* dataLength,
    uint8_t type, const uint8_t* value, uint8_t valueLength)
{
  if (data == NULL || dataLength == NULL || (value == NULL && valueLength > 0) ||
      static_cast<size_t>(*dataLength) + valueLength + 2 > capacity)
  {
    return false;
  }
  data[(*dataLength)++] = valueLength + 1;
  data[(*dataLength)++] = type;
  if (valueLength > 0)
  {
    memcpy(&data[*dataLength], value, valueLength);
    *dataLength += valueLength;
  }
  return true;
}

inline bool buildBleGattAdvertisingData(const char* name, size_t maxNameLength,
    const uint8_t serviceUuid[16], BleGattAdvertisingData* output)
{
  if (name == NULL || serviceUuid == NULL || output == NULL)
  {
    return false;
  }

  output->advertisingLength = 0;
  output->scanResponseLength = 0;
  const uint8_t flags = 0x06;
  uint8_t wireUuid[16];
  for (uint8_t i = 0; i < sizeof(wireUuid); i++)
  {
    wireUuid[i] = serviceUuid[sizeof(wireUuid) - 1 - i];
  }
  const size_t nameLength = strnlen(name, maxNameLength);
  if (nameLength > UINT8_MAX)
  {
    return false;
  }

  return appendBleAdvertisingField(output->advertising, sizeof(output->advertising),
             &output->advertisingLength, 0x01, &flags, sizeof(flags)) &&
      appendBleAdvertisingField(output->advertising, sizeof(output->advertising),
             &output->advertisingLength, 0x07, wireUuid, sizeof(wireUuid)) &&
      appendBleAdvertisingField(output->scanResponse, sizeof(output->scanResponse),
             &output->scanResponseLength, 0x09, reinterpret_cast<const uint8_t*>(name),
             static_cast<uint8_t>(nameLength));
}

enum BleAttWriteValidation
{
  BLE_ATT_WRITE_VALID,
  BLE_ATT_WRITE_INVALID_OFFSET,
  BLE_ATT_WRITE_INVALID_LENGTH
};

inline BleAttWriteValidation validateBleAttWrite(uint16_t offset, const uint8_t* buffer,
    uint16_t bufferSize, int expectedSize = -1)
{
  if (offset != 0)
  {
    return BLE_ATT_WRITE_INVALID_OFFSET;
  }
  if ((buffer == NULL && bufferSize > 0) ||
      (expectedSize >= 0 && bufferSize != static_cast<uint16_t>(expectedSize)))
  {
    return BLE_ATT_WRITE_INVALID_LENGTH;
  }
  return BLE_ATT_WRITE_VALID;
}

struct BleCommandFramer
{
  char buffer[BLE_GATT_COMMAND_CAPACITY];
  uint16_t length;
  bool droppingLongLine;
  bool overflowPending;
};

inline void resetBleCommandFramer(BleCommandFramer* framer)
{
  framer->length = 0;
  framer->droppingLongLine = false;
  framer->overflowPending = false;
}

// Returns the completed line length, 0 for no event, or -1 when overflow starts.
inline int consumeBleCommandByte(BleCommandFramer* framer, char ch)
{
  if (framer->droppingLongLine)
  {
    if (ch == '\n')
    {
      framer->droppingLongLine = false;
      framer->overflowPending = true;
    }
    return 0;
  }
  if (ch == '\r')
  {
    return 0;
  }
  if (ch == '\n')
  {
    const int lineLength = framer->length;
    framer->length = 0;
    return lineLength;
  }
  framer->buffer[framer->length++] = ch;
  if (framer->length == BLE_GATT_COMMAND_CAPACITY)
  {
    framer->length = 0;
    framer->droppingLongLine = true;
    return -1;
  }
  return 0;
}

inline bool takeBleCommandOverflow(BleCommandFramer* framer)
{
  if (!framer->overflowPending)
  {
    return false;
  }
  framer->overflowPending = false;
  return true;
}
