#include <assert.h>
#include <limits.h>

#include "protocol/CommandInput.h"

static void testSignedParsing()
{
  int value = 0;
  assert(parseStrictInt("+27", &value) && value == 27);
  assert(parseStrictInt("-27", &value) && value == -27);
  assert(parseStrictInt("2147483647", &value) && value == INT_MAX);
  assert(parseStrictInt("-2147483648", &value) && value == INT_MIN);
  assert(!parseStrictInt("1junk", &value));
  assert(!parseStrictInt("2147483648", &value));
  assert(!parseStrictInt("-2147483649", &value));
}

static void testUnsignedParsing()
{
  uint32_t value = 0;
  assert(parseStrictUint32("4294967295", &value) && value == UINT32_MAX);
  assert(parseStrictUint32("0x07FFFFFF", &value) && value == 0x07FFFFFFUL);
  assert(!parseStrictUint32("4294967296", &value));
  assert(!parseStrictUint32("0x100000000", &value));
  assert(!parseStrictUint32("-1", &value));
  assert(!parseStrictUint32("1junk", &value));
}

static void testUsbAutoparsePolicy()
{
  assert(shouldAutoParseUsbInput(false, "show"));
  assert(!shouldAutoParseUsbInput(true, "show"));
  assert(!shouldAutoParseUsbInput(false, "N"));
  assert(!shouldAutoParseUsbInput(false, "NK"));
  assert(!shouldAutoParseUsbInput(false, "NK4"));
  assert(!shouldAutoParseUsbInput(false, " NK4 seq=1"));
  assert(shouldAutoParseUsbInput(false, "not-an-nk4-command"));

  assert(USB_COMMAND_MAX == 512);
  assert(usbInputHasCapacity(USB_COMMAND_MAX - 1));
  assert(!usbInputHasCapacity(USB_COMMAND_MAX));
}

int main()
{
  testSignedParsing();
  testUnsignedParsing();
  testUsbAutoparsePolicy();
  return 0;
}
