#include <assert.h>
#include <string.h>

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

int main()
{
  testGattAdvertisingContainsUuidAndVisibleName();
  testAttWriteValidationRejectsMalformedBuffers();
  return 0;
}
