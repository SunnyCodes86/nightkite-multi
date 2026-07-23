#pragma once

#include <stdint.h>

template <typename FieldHandler>
bool runNk4SetTransaction(uint8_t fieldCount, FieldHandler handleField)
{
  for (uint8_t i = 0; i < fieldCount; i++)
  {
    if (!handleField(i, false))
    {
      return false;
    }
  }
  for (uint8_t i = 0; i < fieldCount; i++)
  {
    if (!handleField(i, true))
    {
      return false;
    }
  }
  return true;
}
