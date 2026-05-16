#pragma once

#include <Arduino.h>

struct Rm2BleStatus
{
  bool supported;
  bool enabled;
  bool rm2Enabled;
  bool initialized;
  bool advertising;
  const char* name;
  const char* lastError;
};

bool rm2BleBegin(const char* advertisedName);
void rm2BleTick();
Rm2BleStatus rm2BleStatus();
String rm2BleBuildStatusFields();

