#include <assert.h>
#include <stdint.h>

#include "app/OrientationColor.h"

int main()
{
  assert(orientationHueFromDegrees(-180) == 0);
  assert(orientationHueFromDegrees(-90) == 63);
  assert(orientationHueFromDegrees(0) == 127);
  assert(orientationHueFromDegrees(90) == 191);
  assert(orientationHueFromDegrees(180) == 255);

  const uint8_t hue = orientationHueFromDegrees(37);
  for (uint8_t frame = 0; frame < 10; ++frame)
  {
    assert(orientationHueFromDegrees(37) == hue);
  }
  assert(orientationHueFromDegrees(-37) == 101);
  return 0;
}
