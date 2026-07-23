#include <assert.h>
#include <string.h>
#include <string>

#include "wireless/BleGattHelpers.h"

static void testGattAdvertisingContainsUuidAndVisibleName()
{
  const uint8_t uuid[16] = {
      0x4e, 0x4b, 0x40, 0x00, 0x6e, 0x69, 0x67, 0x68,
      0x74, 0x6b, 0x00, 0x00, 0x00, 0x00, 0x00, 0x01};
  BleGattAdvertisingData data;

  assert(buildBleGattAdvertisingData("NK-ABC123", 23, uuid, &data));
  assert(data.advertisingLength == 21);
  assert(data.advertising[0] == 2 && data.advertising[1] == 0x01 && data.advertising[2] == 0x06);
  assert(data.advertising[3] == 17 && data.advertising[4] == 0x07);
  for (uint8_t i = 0; i < sizeof(uuid); i++)
  {
    assert(data.advertising[5 + i] == uuid[sizeof(uuid) - 1 - i]);
  }
  assert(data.scanResponseLength == 11);
  assert(data.scanResponse[0] == 10 && data.scanResponse[1] == 0x09);
  assert(memcmp(&data.scanResponse[2], "NK-ABC123", 9) == 0);

  const char oversizedName[] = "1234567890123456789012345678901";
  assert(!buildBleGattAdvertisingData(oversizedName, strlen(oversizedName), uuid, &data));
}

static void testAttWriteValidationRejectsMalformedBuffers()
{
  const uint8_t ccc[] = {1, 0};
  assert(validateBleAttWrite(0, ccc, sizeof(ccc), 2) == BLE_ATT_WRITE_VALID);
  assert(validateBleAttWrite(1, ccc, sizeof(ccc), 2) == BLE_ATT_WRITE_INVALID_OFFSET);
  assert(validateBleAttWrite(0, ccc, 1, 2) == BLE_ATT_WRITE_INVALID_LENGTH);
  assert(validateBleAttWrite(0, NULL, 1) == BLE_ATT_WRITE_INVALID_LENGTH);
  assert(validateBleAttWrite(0, NULL, 0) == BLE_ATT_WRITE_VALID);
}

static void feedBleWrite(BleCommandFramer* framer, const std::string& bytes,
    std::string* overflowPrefix, std::string* completedLine)
{
  for (size_t i = 0; i < bytes.size(); i++)
  {
    const int lineLength = consumeBleCommandByte(framer, bytes[i]);
    if (lineLength < 0)
    {
      overflowPrefix->assign(framer->buffer, sizeof(framer->buffer));
    }
    else if (lineLength > 0)
    {
      completedLine->assign(framer->buffer, (size_t)lineLength);
    }
  }
}

static void testFragmentedOverflowDefersErrorAndRecovers()
{
  BleCommandFramer framer = {};
  resetBleCommandFramer(&framer);
  std::string overflowPrefix;
  std::string completedLine;
  const std::string malformed = "NK4 seq=41 bad seq=99 cmd=set pattern=1 ";
  const std::string oversized = malformed + std::string(BLE_GATT_COMMAND_CAPACITY, 'x');

  // Both ATT write properties feed the same byte framer; split at arbitrary write boundaries.
  feedBleWrite(&framer, oversized.substr(0, 17), &overflowPrefix, &completedLine);
  feedBleWrite(&framer, oversized.substr(17), &overflowPrefix, &completedLine);
  assert(overflowPrefix == oversized.substr(0, BLE_GATT_COMMAND_CAPACITY));
  assert(!takeBleCommandOverflow(&framer));

  feedBleWrite(&framer, "\n", &overflowPrefix, &completedLine);
  assert(takeBleCommandOverflow(&framer));
  assert(!takeBleCommandOverflow(&framer));

  const std::string compact = "NK4seq=42 cmd=status\n";
  feedBleWrite(&framer, compact, &overflowPrefix, &completedLine);
  assert(completedLine == compact.substr(0, compact.size() - 1));

  resetBleCommandFramer(&framer);
  overflowPrefix.clear();
  const std::string missingSequence = "NK4 cmd=set pattern=1 " +
      std::string(BLE_GATT_COMMAND_CAPACITY, 'x') + "\n";
  feedBleWrite(&framer, missingSequence, &overflowPrefix, &completedLine);
  assert(overflowPrefix == missingSequence.substr(0, BLE_GATT_COMMAND_CAPACITY));
  assert(takeBleCommandOverflow(&framer));
}

int main()
{
  testGattAdvertisingContainsUuidAndVisibleName();
  testAttWriteValidationRejectsMalformedBuffers();
  testFragmentedOverflowDefersErrorAndRecovers();
  return 0;
}
