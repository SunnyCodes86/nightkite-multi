#pragma once

#include <Arduino.h>
#include "protocol/NkProtocol.h"

using Rm2BleNk4Handler = bool (*)(const String& line, IResponseWriter& writer);
using Rm2BleGapReportHandler = void (*)(const uint8_t* advData, uint8_t advLen, int8_t rssi);

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
  bool txActive;
  uint16_t txOffset;
  unsigned long txChunksSent;
  bool gattAdvSuppressed;
  unsigned long advEnableCount;
  unsigned long advDisableCount;
  unsigned long syncAdvStartCount;
  unsigned long syncAdvRefreshCount;
  const char* advOwner;
  const char* advType;
  const char* name;
  const char* lastError;
};

void rm2BleSetNk4Handler(Rm2BleNk4Handler handler);
void rm2BleSetGapReportHandler(Rm2BleGapReportHandler handler);
bool rm2BleBegin(const char* advertisedName);
void rm2BleTick();
Rm2BleStatus rm2BleStatus();
String rm2BleBuildStatusFields();
bool rm2BleUseSyncAdvertising(const uint8_t* data, uint8_t dataLen, uint16_t intervalMin, uint16_t intervalMax, uint8_t advType);
void rm2BleStopAdvertising();
void rm2BleRestoreGattAdvertising();
