#pragma once

#include <ctype.h>
#include <errno.h>
#include <limits.h>
#include <stddef.h>
#include <stdint.h>
#include <stdlib.h>
#include <string.h>

constexpr size_t USB_COMMAND_MAX = 512;

inline bool usbInputHasCapacity(size_t length)
{
  return length < USB_COMMAND_MAX;
}

inline bool parseStrictInt(const char* value, int* output)
{
  if (value == NULL || output == NULL || value[0] == '\0' || isspace((unsigned char)value[0]))
  {
    return false;
  }

  errno = 0;
  char* end = NULL;
  const long parsed = strtol(value, &end, 10);
  if (errno == ERANGE || end == value || *end != '\0' || parsed < INT_MIN || parsed > INT_MAX)
  {
    return false;
  }

  *output = (int)parsed;
  return true;
}

inline bool parseStrictUint32(const char* value, uint32_t* output)
{
  if (value == NULL || output == NULL || value[0] == '\0' || value[0] == '-' || isspace((unsigned char)value[0]))
  {
    return false;
  }

  errno = 0;
  char* end = NULL;
  const unsigned long parsed = strtoul(value, &end, 0);
  if (errno == ERANGE || end == value || *end != '\0' || parsed > UINT32_MAX)
  {
    return false;
  }

  *output = (uint32_t)parsed;
  return true;
}

inline bool usbInputCouldBeNk4(const char* input)
{
  if (input == NULL)
  {
    return false;
  }
  while (*input == ' ')
  {
    input++;
  }

  const size_t length = strlen(input);
  const size_t prefixLength = 3;
  return length > 0 && strncmp(input, "NK4", length < prefixLength ? length : prefixLength) == 0;
}

inline bool shouldAutoParseUsbInput(bool machineMode, const char* input)
{
  return !machineMode && !usbInputCouldBeNk4(input);
}

inline bool isSupportedCalibrationMode(const char* value)
{
  return value != NULL && (strcmp(value, "quick") == 0 || strcmp(value, "precise") == 0);
}
