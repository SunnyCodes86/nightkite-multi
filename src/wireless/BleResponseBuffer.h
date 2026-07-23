#pragma once

#include <stddef.h>
#include <string.h>

constexpr size_t BLE_NK4_RESPONSE_CAPACITY = 4096;

class BleResponseBuffer
{
public:
  BleResponseBuffer(char* storage, size_t capacity)
    : storage(storage), capacity(capacity)
  {
    reset();
  }

  bool append(const char* value)
  {
    if (value == nullptr)
    {
      return true;
    }
    if (overflowed)
    {
      return false;
    }

    const size_t valueLength = strlen(value);
    if (capacity < 2 || valueLength > capacity - length - 2)
    {
      overflowed = true;
      return false;
    }

    memcpy(&storage[length], value, valueLength);
    length += valueLength;
    storage[length] = '\0';
    return true;
  }

  bool finishLine()
  {
    if (overflowed || capacity < 2 || length > capacity - 2)
    {
      return false;
    }

    storage[length++] = '\n';
    storage[length] = '\0';
    return true;
  }

  void reset()
  {
    length = 0;
    overflowed = false;
    if (capacity > 0)
    {
      storage[0] = '\0';
    }
  }

  const char* data() const { return storage; }
  size_t size() const { return length; }
  bool hasOverflowed() const { return overflowed; }

private:
  char* storage;
  size_t capacity;
  size_t length = 0;
  bool overflowed = false;
};
