#pragma once

#include <Arduino.h>
#include "protocol/SyncBeaconCodec.h"
#include "app/AudioPatternMath.h"
#include "app/ShowControl.h"

enum SyncBeaconRole : uint8_t
{
  SYNC_BEACON_ROLE_STANDALONE = 0,
  SYNC_BEACON_ROLE_MASTER = 1,
  SYNC_BEACON_ROLE_FOLLOWER = 2
};

enum SyncBeaconPlayMode : uint8_t
{
  SYNC_BEACON_PLAY_MANUAL = 0,
  SYNC_BEACON_PLAY_AUTOPLAY = 1,
  SYNC_BEACON_PLAY_SYNC = 2
};

enum SyncBeaconWirelessProfile : uint8_t
{
  SYNC_BEACON_PROFILE_LONG_RANGE = 0,
  SYNC_BEACON_PROFILE_BALANCED = 1,
  SYNC_BEACON_PROFILE_FAST_SYNC = 2
};

struct SyncBeaconRuntime
{
  bool syncEnabled;
  bool wirelessEnabled;
  uint8_t playMode;
  uint8_t syncRole;
  uint8_t groupId;
  uint8_t pattern;
  uint8_t brightness;
  uint8_t wirelessProfile;
  uint32_t phaseMs;
  uint16_t beatMs;
  bool showReceiveEnabled;
  uint32_t shortId;
};

struct ShowRadioStatus
{
  bool receiving = false, clockValid = false;
  uint8_t queueDepth = 0;
  int32_t clockOffsetMs = 0;
  uint32_t clockAgeMs = 0;
  uint32_t received = 0, invalid = 0, crcErrors = 0, targetMiss = 0;
  ShowSchedulerStatus scheduler;
};

struct SyncBeaconRadioStatus
{
  bool supported;
  bool active;
  bool beaconTx;
  bool beaconRx;
  bool locked;
  bool codecSelftest;
  bool scanActive;
  bool advActive;
  bool gattAdvSuppressed;
  bool beaconAdvStarted;
  uint8_t advPayloadLen;
  uint8_t advMfgLen;
  uint16_t advCompany;
  uint16_t advMagic;
  uint8_t advVersion;
  uint8_t advGroup;
  uint16_t advCrc;
  unsigned long advEnableCount;
  unsigned long advDisableCount;
  unsigned long advSetCount;
  unsigned long beaconAdvRefreshes;
  uint16_t beaconSeq;
  unsigned long txCount;
  unsigned long rxCount;
  unsigned long crcErrors;
  unsigned long groupMismatch;
  unsigned long invalidPackets;
  unsigned long scanReports;
  unsigned long scanMfgReports;
  unsigned long scanNkCandidates;
  unsigned long scanDecodeOk;
  unsigned long scanDecodeV1;
  unsigned long scanDecodeV2;
  unsigned long scanDecodeFail;
  unsigned long scanCrcFail;
  unsigned long scanGroupMismatch;
  unsigned long scanRejectCompany;
  unsigned long scanRejectMagic;
  unsigned long scanRejectLen;
  unsigned long scanRejectVersion;
  unsigned long lastBeaconMs;
  unsigned long beaconAgeMs;
  int8_t scanLastRssi;
  uint8_t scanLastLen;
  uint8_t scanLastMfgLen;
  uint8_t scanLastAdType;
  uint16_t scanLastCompany;
  uint8_t scanLastGroup;
  uint8_t scanLastVersion;
  uint8_t lastBeaconVersion;
  AudioSyncState audio;
  unsigned long audioAgeMs;
  const char* advMfgHead;
  const char* advOwner;
  const char* advType;
  const char* mode;
  const char* lastError;
  const char* scanLastError;
  const char* scanLastMfgHead;
  const char* scanLastCandidateReason;
};

void syncBeaconRadioBegin();
void syncBeaconRadioTick(const SyncBeaconRuntime& runtime);
void syncBeaconRadioStop();
SyncBeaconRadioStatus syncBeaconRadioStatus();
String syncBeaconRadioBuildStatusFields();
String syncBeaconAudioBuildStatusFields();
bool syncBeaconRadioConsumeBeacon(NkSyncBeaconV1* beacon);
AudioSyncState syncBeaconAudioState();
bool syncBeaconRadioConsumeShow(ShowScheduledEvent* event);
void syncBeaconRadioResetShow();
ShowRadioStatus syncBeaconShowStatus();
