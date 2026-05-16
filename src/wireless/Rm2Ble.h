#pragma once

#include <Arduino.h>
#include "protocol/NkProtocol.h"

using Rm2BleNk4Handler = bool (*)(const String& line, IResponseWriter& writer);

struct Rm2BleStatus
{
  bool supported;
  bool enabled;
  bool rm2Enabled;
  bool initialized;
  bool advertising;
  bool connected;
  bool gatt;
  bool rx;
  bool tx;
  uint8_t txQueue;
  unsigned long txDropped;
  bool notifyReady;
  const char* name;
  const char* lastError;
};

void rm2BleSetNk4Handler(Rm2BleNk4Handler handler);
bool rm2BleBegin(const char* advertisedName);
void rm2BleTick();
Rm2BleStatus rm2BleStatus();
String rm2BleBuildStatusFields();
