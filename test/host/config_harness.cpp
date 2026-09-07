// Actual EEPROM field I/O and migration extracted from main.cpp by the host runner.
#include <assert.h>
#include <stdint.h>
#include <string.h>
#include "app/ConfigMigration.h"

struct {
  uint8_t bytes[224];
  template<class T> void put(unsigned address, const T& value) {
    assert(address + sizeof(T) <= sizeof(bytes));
    memcpy(bytes + address, &value, sizeof(T));
  }
  template<class T> void get(unsigned address, T& value) {
    assert(address + sizeof(T) <= sizeof(bytes));
    memcpy(&value, bytes + address, sizeof(T));
  }
} EEPROM;
void writeEEPROMCString(int address, const char* value, size_t length) {
  assert(address + length <= sizeof(EEPROM.bytes));
  memcpy(EEPROM.bytes + address, value, length);
}
void readEEPROMCString(int address, char* value, size_t length) {
  assert(address + length <= sizeof(EEPROM.bytes));
  memcpy(value, EEPROM.bytes + address, length);
}
#include "config_under_test.inc"

int main() {
  static_assert(sizeof(int) == 4 && EEPROM_SIZE == 224, "Existing EEPROM contract");
  static_assert(EEPROM_ADDR_WIRELESS_PROFILE == 184 && EEPROM_ADDR_SHOW_CONTROL_ENABLED == 188,
                "Only append the receive flag after existing fields");
  for (int version = 400; version <= 402; ++version) {
    memset(EEPROM.bytes, 0xFF, sizeof(EEPROM.bytes));
    currentConfigVersion = version;
    currentWirelessEnabled = 1; currentWirelessProfile = 2;
    currentSyncGroupId = 73; currentSyncRole = 2;
    currentPlayMode = 2; currentBootMode = 3;
    currentBrightness = 223; currentPattern = 27;
    currentEnabledPatternMask = 1;
    strcpy(currentDeviceUid, "1234567890ABCDEF");
    strcpy(currentDeviceName, "flight controller");
    showReceiveEnabled = 1;
    writeFields();
    if (version < 402) memset(EEPROM.bytes + 188, 0xFF, 4); // Unallocated legacy bytes.
    currentWirelessEnabled = currentWirelessProfile = currentSyncGroupId = currentSyncRole = 0;
    currentPlayMode = currentBootMode = showReceiveEnabled = 0;
    currentDeviceUid[0] = currentDeviceName[0] = 0;
    int storedVersion = 0; bool migrated = false;
    readFields(storedVersion, migrated);
    assert(storedVersion == version && currentConfigVersion == 402);
    assert(migrated == (version < 402) && showReceiveEnabled == (version == 402 ? 1 : 0));
    assert(currentWirelessEnabled == 1 && currentWirelessProfile == 2);
    assert(currentSyncGroupId == 73 && currentSyncRole == 2 && currentPlayMode == 2 && currentBootMode == 3);
    assert(strcmp(currentDeviceUid, "1234567890ABCDEF") == 0);
    assert(strcmp(currentDeviceName, "flight controller") == 0);
    assert(currentEnabledPatternMask == (version == 400 ? int(1 | AUDIO_SYNC_PATTERN_MASK) : 1));
    uint8_t before[224]; memcpy(before, EEPROM.bytes, sizeof(before));
    writeFields();
    // Migration only changes version, new receive flag, and the v400 audio mask.
    for (unsigned i = 0; i < sizeof(before); ++i) {
      if ((i >= 132 && i < 136) || (i >= 188 && i < 192) ||
          (version == 400 && i >= 56 && i < 60)) continue;
      assert(EEPROM.bytes[i] == before[i]);
    }
    migrated = false; showReceiveEnabled = 0;
    readFields(storedVersion, migrated);
    assert(storedVersion == 402 && !migrated);
    assert(showReceiveEnabled == (version == 402 ? 1 : 0));
  }
}
