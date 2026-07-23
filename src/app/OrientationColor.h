#pragma once

#include <stdint.h>

inline uint8_t orientationHueFromDegrees(int degrees)
{
  return static_cast<uint8_t>((static_cast<int32_t>(degrees) + 180) * 255 / 360);
}
