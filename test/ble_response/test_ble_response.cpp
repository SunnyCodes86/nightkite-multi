#include <assert.h>
#include <string>

#include "wireless/BleResponseBuffer.h"

static void testLongNk4ResponseRemainsComplete()
{
  char storage[BLE_NK4_RESPONSE_CAPACITY];
  BleResponseBuffer buffer(storage, sizeof(storage));
  const std::string fields(1800, 'x');

  assert(buffer.append("NK4 seq=42 ok fields="));
  assert(buffer.append(fields.c_str()));
  assert(buffer.append(" end=1"));
  assert(buffer.finishLine());

  const std::string expected = "NK4 seq=42 ok fields=" + fields + " end=1\n";
  assert(buffer.size() == expected.size());
  assert(buffer.size() > 1023);
  assert(std::string(storage, buffer.size()) == expected);

  std::string notified;
  for (size_t offset = 0; offset < buffer.size(); offset += 20)
  {
    const size_t remaining = buffer.size() - offset;
    notified.append(&storage[offset], remaining > 20 ? 20 : remaining);
  }
  assert(notified == expected);
}

static void testCapacityAlwaysReservesNewline()
{
  char storage[16];
  BleResponseBuffer buffer(storage, sizeof(storage));

  assert(buffer.append("12345678901234"));
  assert(buffer.finishLine());
  assert(buffer.size() == 15);
  assert(storage[14] == '\n');
  assert(storage[15] == '\0');
}

static void testOversizeResponseIsRejected()
{
  char storage[16];
  BleResponseBuffer buffer(storage, sizeof(storage));

  assert(!buffer.append("123456789012345"));
  assert(buffer.hasOverflowed());
  assert(!buffer.finishLine());
  assert(buffer.size() == 0);
}

int main()
{
  testLongNk4ResponseRemainsComplete();
  testCapacityAlwaysReservesNewline();
  testOversizeResponseIsRejected();
  return 0;
}
