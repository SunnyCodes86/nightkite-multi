/*
  NightKite multi-strip controller

  Drives two WS2811 LED strips from one logical LED buffer and uses an MPU6050
  DMP packet stream for motion-reactive effects. The motion pipeline reads
  yaw/pitch/roll for orientation-reactive color and gravity-free world-frame
  acceleration for movement intensity, bursts, and pattern speed.

  Runtime configuration is available through the USB serial CLI and persisted
  to EEPROM. Board-specific pins are supplied through PlatformIO build flags
  for the supported Pico-class targets.

  Hardware overview:
  - MPU6050 on I2C with its INT pin connected to PIN_MPU_INTERRUPT.
  - Two WS2811 data outputs configured by PIN_LED_STRIP_1 and PIN_LED_STRIP_2.
  - Multi-function button, battery ADC input, and USB power sense input.
*/

// ============================================================================
//  INCLUDES
// ============================================================================

#include <Arduino.h> //Arduino.h
#include "MPU6050_6Axis_MotionApps612.h" // MPU6050 DMP library
#include <FastLED.h> // FastLED
#include "SimpleFSM.h" // State Machine
#include <SimpleCLI.h> // Serial command-line interface
#include "avdweb_Switch.h" // Button library
#include <Smoothed.h> // Smoothing library
#include <EEPROM.h> //EEPROM Library
#include <math.h> // Math library
#include <string.h>
#include "app/AudioPatternMath.h"
#include "app/Battery.h"
#include "app/ConfigMigration.h"
#include "app/OrientationColor.h"
#include "app/PatternClock.h"
#include "app/SyncEngine.h"
#include "app/SyncMath.h"
#include "protocol/CommandInput.h"
#include "protocol/NkSetTransaction.h"
#include "protocol/NkProtocol.h"
#include "wireless/Rm2Ble.h"
#include "wireless/SyncBeaconRadio.h"

// ============================================================================
//  MOTION DATA
// ============================================================================

// The LED patterns use yaw/pitch/roll for orientation-reactive color and
// gravity-free world acceleration for motion intensity.

// ============================================================================
//  HARDWARE & LED CONFIG
// ============================================================================

#ifndef PIN_MPU_INTERRUPT
#define PIN_MPU_INTERRUPT 3
#endif
#ifndef PIN_LED_STRIP_1
#define PIN_LED_STRIP_1 12
#endif
#ifndef PIN_LED_STRIP_2
#define PIN_LED_STRIP_2 13
#endif
#ifndef PIN_BUTTON_MULTI
#define PIN_BUTTON_MULTI 23
#endif
#ifndef PIN_BATTERY_ADC
#define PIN_BATTERY_ADC 29
#endif
#ifndef PIN_USB_SENSE
#define PIN_USB_SENSE 24
#endif
#ifndef PIN_RM2_WL_ON
#define PIN_RM2_WL_ON 17
#endif
#ifndef PIN_RM2_BL_ON
#define PIN_RM2_BL_ON PIN_RM2_WL_ON
#endif
#ifndef PIN_RM2_WL_CS
#define PIN_RM2_WL_CS 18
#endif
#ifndef PIN_RM2_WL_CLK
#define PIN_RM2_WL_CLK 19
#endif
#ifndef PIN_RM2_WL_DATA
#define PIN_RM2_WL_DATA 20
#endif
#ifndef PIN_RM2_WL_WAKE
#define PIN_RM2_WL_WAKE PIN_RM2_WL_DATA
#endif
#ifndef NIGHTKITE_BLE
#define NIGHTKITE_BLE 0
#endif
#ifndef NIGHTKITE_RM2
#define NIGHTKITE_RM2 0
#endif
#ifndef NIGHTKITE_HARDWARE
#define NIGHTKITE_HARDWARE "unknown"
#endif
#ifndef NIGHTKITE_BOOT_DEBUG
#define NIGHTKITE_BOOT_DEBUG 1
#endif
#ifndef NIGHTKITE_SAFE_BOOT
#define NIGHTKITE_SAFE_BOOT 0
#endif

constexpr const char* FIRMWARE_VERSION = "4.0.0-alpha.1";
constexpr uint8_t NK4_PROTOCOL_VERSION = 4;

int const INTERRUPT_PIN = PIN_MPU_INTERRUPT; // MPU interrupt input pin

#define PinStrip1 PIN_LED_STRIP_1
#define PinStrip2 PIN_LED_STRIP_2
#define LED_TYPE WS2811
#define COLOR_ORDER GRB

#define MIN_LEDS_PER_STRIP 10
#define MAX_LEDS_PER_STRIP 35
#define DEFAULT_LEDS_PER_STRIP 25
#define MAX_TOTAL_LEDS (MAX_LEDS_PER_STRIP * 2)

constexpr unsigned long BATTERY_SAMPLE_INTERVAL_MS = 1000;
constexpr uint8_t BATTERY_SAMPLE_WINDOW = 15;

int ledsPerStrip = DEFAULT_LEDS_PER_STRIP;
int totalLeds = (DEFAULT_LEDS_PER_STRIP * 2); // total logical LEDs across both strips

#define NUM_LEDS ledsPerStrip
#define TOTAL_LEDS totalLeds
#define NUM_COMETS 4        // comet count


// Logical LED buffer used by all patterns (contiguous: strip1 then strip2).
CRGB Strip[MAX_TOTAL_LEDS];
// Physical LED buffer wired as two fixed hardware segments.
CRGB PhysicalStrip[MAX_TOTAL_LEDS];

int BRIGHTNESS = 95;
#define MIN_BRIGHTNESS 95
#define MAX_BRIGHTNESS 255
#define FRAMES_PER_SECOND 120

uint8_t gHue = 0; // rotating "base color" used by many of the patterns

// ============================================================================
//  EEPROM LAYOUT
// ============================================================================

// Memory layout in emulated EEPROM.
// On Pico-class MCUs an int is 4 bytes, so values are packed sequentially.
#define EEPROM_ADDR_PATTERN         0
#define EEPROM_ADDR_BRIGHTNESS      sizeof(int)
#define EEPROM_ADDR_STRIP_LENGTH    (sizeof(int) * 2)
#define EEPROM_ADDR_SMOOTHING_SIZE  (sizeof(int) * 3)
#define EEPROM_ADDR_ACCEL_RANGE     (sizeof(int) * 4)
#define EEPROM_ADDR_GYRO_RANGE      (sizeof(int) * 5)
#define EEPROM_ADDR_BOOT_CAL_MODE   (sizeof(int) * 6)
#define EEPROM_ADDR_X_ACCEL_OFFSET  (sizeof(int) * 7)
#define EEPROM_ADDR_Y_ACCEL_OFFSET  (sizeof(int) * 8)
#define EEPROM_ADDR_Z_ACCEL_OFFSET  (sizeof(int) * 9)
#define EEPROM_ADDR_X_GYRO_OFFSET   (sizeof(int) * 10)
#define EEPROM_ADDR_Y_GYRO_OFFSET   (sizeof(int) * 11)
#define EEPROM_ADDR_Z_GYRO_OFFSET   (sizeof(int) * 12)
#define EEPROM_ADDR_MAGIC           (sizeof(int) * 13)
#define EEPROM_ADDR_ENABLED_PATTERNS (sizeof(int) * 14)
#define EEPROM_ADDR_INVERTED_PATTERNS (sizeof(int) * 15)
#define EEPROM_ADDR_AUTOPLAY_ENABLED (sizeof(int) * 16)
#define EEPROM_ADDR_AUTOPLAY_INTERVAL_MS (sizeof(int) * 17)
#define EEPROM_ADDR_DEVICE_UID       80
#define EEPROM_ADDR_DEVICE_NAME      100
#define EEPROM_ADDR_CONFIG_VERSION   132
#define EEPROM_ADDR_PLAY_MODE        136
#define EEPROM_ADDR_BOOT_MODE        140
#define EEPROM_ADDR_SYNC_ENABLED     144
#define EEPROM_ADDR_SYNC_GROUP_ID    148
#define EEPROM_ADDR_SYNC_ROLE        152
#define EEPROM_ADDR_SYNC_MASTER_UID  156
#define EEPROM_ADDR_SYNC_LOSS_BEHAVIOR 176
#define EEPROM_ADDR_WIRELESS_ENABLED 180
#define EEPROM_ADDR_WIRELESS_PROFILE 184
const int EEPROM_MAGIC = 0x4E4B3434; // "NK44"

// Size of emulated EEPROM.
// Must cover all persisted ints plus the enabled/inverted pattern bitmasks.
#define EEPROM_SIZE 224

const size_t DEVICE_UID_LENGTH = 16;
const size_t SHORT_ID_LENGTH = 6;
const size_t DEVICE_NAME_LENGTH = 24;

const int PLAY_MODE_MANUAL = 0;
const int PLAY_MODE_AUTOPLAY = 1;
const int PLAY_MODE_SYNC = 2;
const int BOOT_MODE_LAST = 0;
const int BOOT_MODE_MANUAL = 1;
const int BOOT_MODE_AUTOPLAY = 2;
const int BOOT_MODE_SYNC = 3;
const int SYNC_ROLE_STANDALONE = 0;
const int SYNC_ROLE_MASTER = 1;
const int SYNC_ROLE_FOLLOWER = 2;
const int SYNC_LOSS_CONTINUE_LOCAL = 0;
const int SYNC_LOSS_FALLBACK_AUTOPLAY = 1;
const int SYNC_LOSS_WARNING_ONLY = 2;
const int WIRELESS_PROFILE_LONG_RANGE = 0;
const int WIRELESS_PROFILE_BALANCED = 1;
const int WIRELESS_PROFILE_FAST_SYNC = 2;
const int PATTERN_SYNC_UNKNOWN = 0;
const int PATTERN_SYNC_READY = 1;
const int PATTERN_SYNC_PARTIAL = 2;
const int PATTERN_SYNC_LOCAL_REACTIVE = 3;

// Persisted configuration values.
int currentPattern = 1;
int currentBrightness = 95;
int currentStripLength = DEFAULT_LEDS_PER_STRIP;
int currentMotionSmoothingSize = 100;
int currentAccelRange = 2;
int currentGyroRange = 2000;
int currentBootCalibrationMode = 1;
int currentXAccelOffset = -3137;
int currentYAccelOffset = -7;
int currentZAccelOffset = 3687;
int currentXGyroOffset = 111;
int currentYGyroOffset = -6;
int currentZGyroOffset = 34;
uint32_t currentEnabledPatternMask = 0;
uint32_t currentInvertedPatternMask = 0;
int currentAutoplayEnabled = 0;
int currentAutoplayIntervalMs = 20000;
int currentConfigVersion = CONFIG_VERSION_4_ALPHA;
char currentDeviceUid[DEVICE_UID_LENGTH + 1] = "";
char currentShortId[SHORT_ID_LENGTH + 1] = "";
char currentDeviceName[DEVICE_NAME_LENGTH + 1] = "";
int currentPlayMode = PLAY_MODE_MANUAL;
int currentBootMode = BOOT_MODE_LAST;
int currentSyncEnabled = 0;
int currentSyncGroupId = 1;
int currentSyncRole = SYNC_ROLE_STANDALONE;
char currentSyncMasterUid[DEVICE_UID_LENGTH + 1] = "";
int currentSyncLossBehavior = SYNC_LOSS_CONTINUE_LOCAL;
int currentWirelessEnabled = 0;
int currentWirelessProfile = WIRELESS_PROFILE_BALANCED;
uint8_t lastBeaconPattern = 0;
uint8_t lastBeaconBrightness = 0;
uint16_t lastBeaconSeq = 0;
uint16_t lastAppliedSeq = 0;
uint32_t lastBeaconPhaseMs = 0;
unsigned long syncApplyCount = 0;
unsigned long syncApplySkipped = 0;
const char* syncApplyReason = "none";
unsigned long syncLossCount = 0;
unsigned long lastSyncLossMs = 0;
const char* lastSyncLossAction = "none";
unsigned long patternChangeCount = 0;
unsigned long lastPatternChangeMs = 0;
unsigned long lastPatternChangeLatencyMs = 0;
uint8_t lastPatternFrom = 0;
uint8_t lastPatternTo = 0;
const char* lastPatternChangeSource = "none";

// Last values written to EEPROM.
// Used to avoid unnecessary flash writes.
int lastSavedPattern = 1;
int lastSavedBrightness = 95;
int lastSavedStripLength = DEFAULT_LEDS_PER_STRIP;
int lastSavedMotionSmoothingSize = 100;
int lastSavedAccelRange = 2;
int lastSavedGyroRange = 2000;
int lastSavedBootCalibrationMode = 1;
int lastSavedXAccelOffset = -3137;
int lastSavedYAccelOffset = -7;
int lastSavedZAccelOffset = 3687;
int lastSavedXGyroOffset = 111;
int lastSavedYGyroOffset = -6;
int lastSavedZGyroOffset = 34;
uint32_t lastSavedEnabledPatternMask = 0;
uint32_t lastSavedInvertedPatternMask = 0;
int lastSavedAutoplayEnabled = 0;
int lastSavedAutoplayIntervalMs = 20000;
int lastSavedConfigVersion = CONFIG_VERSION_4_ALPHA;
char lastSavedDeviceUid[DEVICE_UID_LENGTH + 1] = "";
char lastSavedDeviceName[DEVICE_NAME_LENGTH + 1] = "";
int lastSavedPlayMode = PLAY_MODE_MANUAL;
int lastSavedBootMode = BOOT_MODE_LAST;
int lastSavedSyncEnabled = 0;
int lastSavedSyncGroupId = 1;
int lastSavedSyncRole = SYNC_ROLE_STANDALONE;
char lastSavedSyncMasterUid[DEVICE_UID_LENGTH + 1] = "";
int lastSavedSyncLossBehavior = SYNC_LOSS_CONTINUE_LOCAL;
int lastSavedWirelessEnabled = 0;
int lastSavedWirelessProfile = WIRELESS_PROFILE_BALANCED;

const int DEFAULT_MOTION_SMOOTHING_SIZE = 100;
const int MIN_MOTION_SMOOTHING_SIZE = 1;
const int MAX_MOTION_SMOOTHING_SIZE = 512;

const int DEFAULT_ACCEL_RANGE = 2;
const int DEFAULT_GYRO_RANGE = 2000;
const int DEFAULT_BOOT_CALIBRATION_MODE = 1;
const int DEFAULT_AUTOPLAY_ENABLED = 0;
const int DEFAULT_AUTOPLAY_INTERVAL_MS = 20000;
const int DEFAULT_PLAY_MODE = PLAY_MODE_MANUAL;
const int DEFAULT_BOOT_MODE = BOOT_MODE_LAST;
const int DEFAULT_SYNC_ENABLED = 0;
const int DEFAULT_SYNC_GROUP_ID = 1;
const int DEFAULT_SYNC_ROLE = SYNC_ROLE_STANDALONE;
const int DEFAULT_SYNC_LOSS_BEHAVIOR = SYNC_LOSS_CONTINUE_LOCAL;
const int DEFAULT_WIRELESS_ENABLED = 0;
const int DEFAULT_WIRELESS_PROFILE = WIRELESS_PROFILE_BALANCED;
const int MIN_AUTOPLAY_INTERVAL_MS = 1000;
const int MAX_AUTOPLAY_INTERVAL_MS = 300000;

const int BOOT_CALIBRATION_MODE_OFF = 0;
const int BOOT_CALIBRATION_MODE_QUICK = 1;

const int DEFAULT_X_ACCEL_OFFSET = -3137;
const int DEFAULT_Y_ACCEL_OFFSET = -7;
const int DEFAULT_Z_ACCEL_OFFSET = 3687;
const int DEFAULT_X_GYRO_OFFSET = 111;
const int DEFAULT_Y_GYRO_OFFSET = -6;
const int DEFAULT_Z_GYRO_OFFSET = 34;
const uint8_t FIRST_PATTERN_ID = NK_PATTERN_MIN_ID;
const uint8_t LAST_PATTERN_ID = NK_PATTERN_MAX_ID;
const uint8_t PATTERN_COUNT = LAST_PATTERN_ID - FIRST_PATTERN_ID + 1;
const uint32_t ALL_ENABLED_PATTERN_MASK = (1ul << PATTERN_COUNT) - 1ul;
const uint32_t ALL_INVERTED_PATTERN_MASK = (1ul << PATTERN_COUNT) - 1ul;
static_assert(
    AUDIO_SYNC_PATTERN_MASK == (ALL_ENABLED_PATTERN_MASK & ~((1ul << 22) - 1ul)),
    "Config migration mask must cover patterns 23 through 27");

int activeMotionSmoothingSize = DEFAULT_MOTION_SMOOTHING_SIZE;

// supported brightness levels for button + CLI
const int BRIGHTNESS_LEVELS[] = {95, 127, 159, 191, 223, 255};
const size_t BRIGHTNESS_LEVEL_COUNT = sizeof(BRIGHTNESS_LEVELS) / sizeof(BRIGHTNESS_LEVELS[0]);

// for timekeeping
unsigned long lastUpdateTime = 0;
const unsigned long UPDATE_INTERVAL = 5 * 60 * 1000; // 5 minutes in milliseconds (5 * 60 * 1000)
const unsigned long BATTERY_VIEW_TIMEOUT_MS = 5000;
unsigned long batteryViewLastInteractionMs = 0;

// ====================================================================
//  USB CLI STATE
// ====================================================================

SimpleCLI cli;
String cliInputBuffer;
bool cliInputOverflow = false;
bool cliPromptShown = false;
unsigned long cliLastInputMs = 0;
const unsigned long CLI_AUTOPARSE_TIMEOUT_MS = 200;
bool cliSessionBannerPending = false;
unsigned long cliSessionBecameActiveMs = 0;
const unsigned long CLI_CONNECT_BANNER_DELAY_MS = 150;
enum UsbProtocolMode
{
  USB_PROTOCOL_HUMAN = 0,
  USB_PROTOCOL_MACHINE = 1
};
UsbProtocolMode usbProtocolMode = USB_PROTOCOL_HUMAN;
bool configValid = false;
bool configRepaired = false;
bool eepromReady = false;
bool safeBootActive = (NIGHTKITE_SAFE_BOOT != 0);
bool imuReady = false;
const char* bootStage = "reset";
bool bootLoopAnnounced = false;

// ============================================================================
//  MPU6050 & MOTION STATE
// ============================================================================
/* MPU6050 default I2C address is 0x68*/
MPU6050 mpu;

/*---MPU6050 Control/Status Variables---*/
bool DMPReady = false;  // Set true if DMP init was successful
uint8_t MPUIntStatus;   // Holds actual interrupt status byte from MPU
uint8_t devStatus;      // Return status after each device operation (0 = success, !0 = error)
uint16_t packetSize;    // Expected DMP packet size (default is 42 bytes)
uint8_t FIFOBuffer[64]; // FIFO storage buffer

/*---Orientation/Motion Variables---*/
Quaternion q;        // [w, x, y, z]         Quaternion container
VectorInt16 aa;      // [x, y, z]            Accel sensor measurements
VectorInt16 gy;      // [x, y, z]            Gyro sensor measurements
VectorInt16 aaReal;  // [x, y, z]            Gravity-free accel sensor measurements
VectorInt16 aaWorld; // [x, y, z]            World-frame accel sensor measurements
VectorFloat gravity; // [x, y, z]            Gravity vector
float ypr[3];        // [yaw, pitch, roll]   Yaw/Pitch/Roll container and gravity vector

volatile bool MPUInterrupt = false; // Indicates whether MPU6050 interrupt pin has gone high
/*------Interrupt detection routine------*/
void DMPDataReady(){
  MPUInterrupt = true;
}

// ============================================================================
//  BUTTON, BATTERY, TIMING
// ============================================================================

// Button handling.
const byte multiresponseButtonpin = PIN_BUTTON_MULTI;
Switch multiresponseButton = Switch(multiresponseButtonpin, INPUT);

// Battery, charging, and serial-session state.
int UsbConnected = 0;
int UsbPowerRaw = 0;
bool SerialSessionActive = false;
int RawVoltage = 0;
float Voltage = 0;
int BatteryPercent = 0;
BatteryState currentBatteryState = BATTERY_STATE_NORMAL;
BatteryStateTracker batteryStateTracker;
float batteryVoltageSamples[BATTERY_SAMPLE_WINDOW] = {0};
uint8_t batteryVoltageSampleCount = 0;
uint8_t batteryVoltageSampleIndex = 0;
float batteryVoltageSampleSum = 0.0f;
unsigned long lastBatterySampleMs = 0;
bool lowPowerCutoffActive = false;
bool lowPowerCutoffSaved = false;

unsigned long previousMillis = 0; // will store last time LED was updated
unsigned long currentMillis = 0;
bool blinkState;
bool blink;
bool batteryViewActive = false;
unsigned long autoplayLastSwitchMs = 0;
bool autoplayWasPaused = false;
uint32_t lastLoopDurationUs = 0;
uint32_t maxLoopDurationUs = 0;
uint32_t lastWorkDurationUs = 0;
uint32_t maxWorkDurationUs = 0;
uint64_t totalLoopDurationUs = 0;
uint64_t totalWorkDurationUs = 0;
uint32_t loopTimingSamples = 0;

// ============================================================================
//  LED EFFECT PARAMETERS & SMOOTHING
// ============================================================================

Smoothed<uint32_t> myAccel;

uint8_t bloodHue = 96;       // Blood color [hue from 0-255]
uint8_t bloodSat = 255;      // Blood staturation [0-255]
int flowDirection = -1;      // Use either 1 or -1 to set flow direction
uint16_t cycleLength = 1300; // Lover values = continuous flow, higher values = distinct pulses.
uint16_t pulseLength = 200;  // How long the pulse takes to fade out.  Higher value is longer.
uint16_t pulseOffset = 250;  // Delay before second pulse.  Higher value is more delay.
uint8_t baseBrightness = 0;  // Brightness of LEDs when not pulsing. Set to 0 for off.

int ledeffect;
int ledeffect2;

// Shared scratch variables used by multiple motion-reactive patterns.
int accel;
uint8_t accelcon;
int fade;

// ============================================================================
//  FSM INSTANCE
// ============================================================================
// FSM is only used for high-level modes: battery view, charging view, and pattern playback.
SimpleFSM fsm;
extern State s[];

// Generic callback type used by pattern entry/run/exit functions.
typedef void (*PatternCallback)();

// Dispatch table entry for one user-selectable LED pattern.
struct PatternDefinition
{
  uint8_t id;
  const char* name;
  PatternCallback entry;
  PatternCallback run;
  PatternCallback exit;
};

PatternClock patternClock;
SyncEngine syncEngine;

// ============================================================================
//  HELPERS
// ============================================================================

bool isValidBrightnessLevel(int value);
bool isValidStripLength(int value);
bool isValidMotionSmoothingSize(int value);
bool isValidAccelRange(int value);
bool isValidGyroRange(int value);
bool isValidBootCalibrationMode(int value);
int parseBootCalibrationMode(String value);
const char* bootCalibrationModeToString(int value);
int accelRangeToRegisterValue(int value);
int accelRegisterValueToRange(int value);
int gyroRangeToRegisterValue(int value);
int gyroRegisterValueToRange(int value);
void applyConfiguredStripLength();
void applyPersistentConfig();
void applyConfiguredMotionSmoothing();
void applyConfiguredSensorRanges();
void applyConfiguredOffsets();
void syncConfiguredOffsetsFromMPU();
void applyDefaultExtendedConfig(bool resetName);
void updateShortIdFromUid();
void ensureDeviceIdentity();
void bootMark(const char* stage);
void copyCString(char* dest, size_t destSize, const char* source);
void writeEEPROMCString(int address, const char* value, size_t maxLength);
void readEEPROMCString(int address, char* value, size_t maxLength);
bool isValidDeviceUid(const char* value);
void generateDeviceUid();
void setDefaultDeviceName();
bool sanitizeDeviceName(String value, char* output, size_t outputSize);
bool sanitizeUidString(String value, char* output, size_t outputSize);
bool readStoredDeviceUid(char* output, size_t outputSize);
bool readStoredDeviceName(char* output, size_t outputSize);
const char* playModeToString(int value);
int parsePlayMode(String value);
const char* bootModeToString(int value);
int parseBootMode(String value);
const char* syncRoleToString(int value);
int parseSyncRole(String value);
const char* syncLossBehaviorToString(int value);
int parseSyncLossBehavior(String value);
const char* wirelessProfileToString(int value);
int parseWirelessProfile(String value);
int sanitizeBinaryFlag(int value);
String formatHex32(uint32_t value);
String buildPatternMaskFields();
int patternSyncClass(uint8_t patternId);
bool isSyncReadyPattern(uint8_t patternId);
bool isPartialSyncPattern(uint8_t patternId);
uint32_t buildPatternClassMask(int syncClass);
bool hasUnsavedConfigChanges();
void markCurrentConfigSaved();
void emitNk4Event(const char* eventName, const String& fields);
int sanitizeAutoplayEnabled(int value);
int sanitizeAutoplayIntervalMs(int value);
void resetAutoplayTimer();
bool isAutoplayEnabled();
bool isSyncMasterAutoplayActive();
bool shouldRunAutoplayTick();
unsigned long autoplayNextDueMs();
void setAutoplayEnabledFlag(int enabled);
const char* autoplayEnabledToString();
int parseOnOffValue(String valueText);
bool parseIntValue(String value, int* output);
bool parseUint32Value(String value, uint32_t* output);
bool parseBinaryValue(String value, int* output);
void setPlayMode(int mode);
void cyclePlayMode();
CRGB playModeIndicatorColor(int mode, int syncRole, bool syncError);
void applySyncStartIfDue();
SyncBeaconRuntime buildSyncBeaconRuntime();
void tickSyncBeaconRadio();
void applyReceivedSyncBeacon();
void announcePatternChange(const char* source);
void printOffsets();
void printOffsetsWithPrefix(const char* prefix);
void printConfigSummaryWithPrefix(const char* prefix);
String buildConfigFields();
String buildSyncFields(bool detailed);
String buildWirelessFields();
String buildBatteryFields();
String buildSensorFields();
String buildTimingFields();
String buildOffsetsFields();
void printEnabledPatternsList();
void printInvertedPatternsList();
void printPatternStates();
bool beginCalibrationSession(bool verbose, bool* restartDMP);
void endCalibrationSession(bool restartDMP);
bool runQuickCalibration(bool verbose);
bool runPreciseCalibration(bool verbose);
void clearInactiveLeds();
void syncLogicalToPhysicalLeds();
void normalizePersistentConfig();
bool isCurrentConfigSane();
bool isValidPatternId(int value);
uint32_t sanitizeEnabledPatternMask(uint32_t mask);
uint32_t sanitizeInvertedPatternMask(uint32_t mask);
bool isPatternEnabled(uint8_t patternId);
bool isPatternInverted(uint8_t patternId);
int getPatternDirectionFactor(uint8_t patternId);
bool setPatternEnabled(uint8_t patternId, bool enabled);
bool parsePatternListMask(String valueText, uint32_t* maskOut);
bool updateEnabledPatternsFromMask(uint32_t mask, bool enabled);
bool updateInvertedPatternsFromMask(uint32_t mask, bool inverted);
uint8_t getNextEnabledPattern(uint8_t currentId);
const PatternDefinition* getPatternDefinition(uint8_t patternId);
void runPatternEntry(uint8_t patternId);
void runPatternFrame(uint8_t patternId);
void runPatternExit(uint8_t patternId);
void switchToPattern(uint8_t patternId, bool activatePatternState, const char* source = NULL, uint32_t latencyMs = 0);
bool batteryViewTimedOut();
bool chargingUsbDisconnected();
int readBatteryRawValue();
float convertBatteryRawToVoltage(int rawValue);
bool isUsbPowered();
void updateBatteryMeasurement(bool force);
void applyEffectiveBrightness();
void applyBatteryBrightnessLimit();
void renderBatteryBar(int batteryBarMax);
void handleBatteryCutoff();
void printBatteryStatus();
void printSensorStatus();
void resetTimingStats();
void printTimingStatus();
bool saveConfigToEEPROM(bool verbose);
void readConfigFromEEPROM(bool verbose);
bool handleNk4Line(const String& line);
bool handleNk4LineWithWriter(const String& line, IResponseWriter& writer);
void handleNk4Command(const NkCommand& command, IResponseWriter& writer);
void showPlayModeIndicatorTest();
void rebootController();
void printCliHelp();
void printCliPrompt();
void setupCLI();
void handleCLI();

void onCliHelp(cmd* cPtr);
void onCliShow(cmd* cPtr);
void onCliGet(cmd* cPtr);
void onCliSet(cmd* cPtr);
void onCliSave(cmd* cPtr);
void onCliLoad(cmd* cPtr);
void onCliDefaults(cmd* cPtr);
void onCliBattery(cmd* cPtr);
void onCliSensor(cmd* cPtr);
void onCliTiming(cmd* cPtr);
void onCliOffsets(cmd* cPtr);
void onCliCalibrate(cmd* cPtr);
void onCliReboot(cmd* cPtr);
void onCliProtocol(cmd* cPtr);
void onCliPatterns(cmd* cPtr);
void onCliEnablePattern(cmd* cPtr);
void onCliDisablePattern(cmd* cPtr);
void onCliInvertPattern(cmd* cPtr);
void onCliNormalPattern(cmd* cPtr);
void onCliError(cmd_error* e);

// Simple triangular pulse used by the heartbeat-style pattern.
uint8_t pulseWave8(uint32_t ms, uint16_t cycleLength, uint16_t pulseLength)
{
  uint16_t T = ms % cycleLength;
  if (T > pulseLength)
    return baseBrightness;
  uint16_t halfPulse = pulseLength / 2;
  if (T <= halfPulse)
  {
    return (T * 255) / halfPulse; // first half = going up
  }
  else
  {
    return ((pulseLength - T) * 255) / halfPulse; // second half = going down
  }
}

int sumPulse(int time_shift)
{
  const uint32_t phase = patternClock.phaseMs();
  int pulse1 = pulseWave8(phase + time_shift, cycleLength, pulseLength);
  int pulse2 = pulseWave8(phase + time_shift + pulseOffset, cycleLength, pulseLength);
  return qadd8(pulse1, pulse2); // Add pulses together without overflow
}

uint16_t clockBeat16(uint16_t bpm)
{
  return (uint16_t)(((uint64_t)patternClock.phaseMs() * (uint64_t)bpm * 65536ULL) / 60000ULL);
}

uint8_t clockBeat8(uint16_t bpm)
{
  return (uint8_t)(clockBeat16(bpm) >> 8);
}

uint16_t clockSin16(uint16_t bpm)
{
  return (uint16_t)((int32_t)sin16(clockBeat16(bpm)) + 32768L);
}

uint8_t clockSin8(uint16_t bpm, uint8_t low, uint8_t high)
{
  return (uint8_t)map(sin8(clockBeat8(bpm)), 0, 255, low, high);
}

inline uint32_t smoothedMotion() {
  // The smoothing window is configurable and reused by many patterns.
  uint32_t s = (uint32_t)abs(aaWorld.x) + (uint32_t)abs(aaWorld.y);
  myAccel.add(s);
  return myAccel.get();
}

struct AudioPatternFrame
{
  bool beat;
  uint8_t phase8;
  uint8_t beatPulse;
  uint8_t energy;
  uint8_t bass;
  uint8_t mid;
  uint8_t treble;
  uint8_t confidence;
};

AudioPatternFrame buildAudioPatternFrame()
{
  static bool initialized = false;
  static uint8_t energy = 0;
  static uint8_t bass = 0;
  static uint8_t mid = 0;
  static uint8_t treble = 0;
  static uint8_t confidence = 0;

  const unsigned long now = millis();
  const AudioSyncState state = syncBeaconAudioState();
  const uint16_t beatMs = state.valid
      ? sanitizeAudioPatternBeatMs(state.beatMs)
      : AUDIO_PATTERN_DEFAULT_BEAT_MS;
  const uint32_t phaseMs = state.valid
      ? state.phaseMs + (uint32_t)(now - state.lastUpdateMs)
      : patternClock.phaseMs();
  const uint8_t phase8 = audioPatternPhase8(phaseMs, beatMs);
  const uint8_t beatPulse = audioPatternBeatPulse8(phaseMs, beatMs);

  // V1 or expired V2 data falls back to a quiet synthetic spectrum. This
  // keeps every audio pattern moving and visible without pretending that
  // audio confidence exists.
  const uint8_t targetEnergy = state.valid
      ? state.energy
      : qadd8(52, scale8(sin8(phase8), 54));
  const uint8_t targetBass = state.valid
      ? state.bass
      : qadd8(40, scale8(beatPulse, 80));
  const uint8_t targetMid = state.valid
      ? state.mid
      : qadd8(48, scale8(sin8(phase8 + 64), 58));
  const uint8_t targetTreble = state.valid
      ? state.treble
      : qadd8(30, scale8(sin8((uint8_t)(phase8 * 2U) + 96), 42));
  const uint8_t targetConfidence = state.valid ? state.confidence : 0;

  if (!initialized)
  {
    energy = targetEnergy;
    bass = targetBass;
    mid = targetMid;
    treble = targetTreble;
    confidence = targetConfidence;
    initialized = true;
  }
  else
  {
    const uint8_t blendAmount = state.valid ? 64 : 18;
    energy = lerp8by8(energy, targetEnergy, blendAmount);
    bass = lerp8by8(bass, targetBass, blendAmount);
    mid = lerp8by8(mid, targetMid, blendAmount);
    treble = lerp8by8(treble, targetTreble, blendAmount);
    confidence = lerp8by8(confidence, targetConfidence, blendAmount);
  }

  AudioPatternFrame frame;
  frame.beat = state.valid && state.beat && (now - state.lastUpdateMs) < 180;
  frame.phase8 = phase8;
  frame.beatPulse = beatPulse;
  frame.energy = energy;
  frame.bass = bass;
  frame.mid = mid;
  frame.treble = treble;
  frame.confidence = confidence;
  return frame;
}

int currentYawDegrees()
{
  return (int)(ypr[0] * 180.0f / M_PI);
}

uint8_t currentYawHue()
{
  return orientationHueFromDegrees(currentYawDegrees());
}

uint8_t audioPatternLocalHue()
{
  const int yawDegrees = constrain((int)(ypr[0] * 180.0f / M_PI), -180, 180);
  return (uint8_t)map(yawDegrees, -180, 180, 0, 255);
}

int audioPatternPitchOffset()
{
  const int pitchDegrees = constrain((int)(ypr[1] * 180.0f / M_PI), -90, 90);
  return map(pitchDegrees, -90, 90, -28, 28);
}

bool isValidBrightnessLevel(int value)
{
  for (size_t i = 0; i < BRIGHTNESS_LEVEL_COUNT; i++)
  {
    if (BRIGHTNESS_LEVELS[i] == value)
    {
      return true;
    }
  }
  return false;
}

bool isValidStripLength(int value)
{
  return value >= MIN_LEDS_PER_STRIP && value <= MAX_LEDS_PER_STRIP;
}

bool isValidMotionSmoothingSize(int value)
{
  return value >= MIN_MOTION_SMOOTHING_SIZE && value <= MAX_MOTION_SMOOTHING_SIZE;
}

bool isValidAccelRange(int value)
{
  return value == 2 || value == 4 || value == 8 || value == 16;
}

bool isValidGyroRange(int value)
{
  return value == 250 || value == 500 || value == 1000 || value == 2000;
}

bool isValidBootCalibrationMode(int value)
{
  return value == BOOT_CALIBRATION_MODE_OFF || value == BOOT_CALIBRATION_MODE_QUICK;
}

int parseBootCalibrationMode(String value)
{
  value.toLowerCase();
  value.trim();
  if (value == "off" || value == "0")
  {
    return BOOT_CALIBRATION_MODE_OFF;
  }
  if (value == "quick" || value == "1")
  {
    return BOOT_CALIBRATION_MODE_QUICK;
  }
  return -1;
}

const char* bootCalibrationModeToString(int value)
{
  return (value == BOOT_CALIBRATION_MODE_QUICK) ? "quick" : "off";
}

int accelRangeToRegisterValue(int value)
{
  switch (value)
  {
    case 2:
      return MPU6050_ACCEL_FS_2;
    case 4:
      return MPU6050_ACCEL_FS_4;
    case 8:
      return MPU6050_ACCEL_FS_8;
    case 16:
      return MPU6050_ACCEL_FS_16;
    default:
      return MPU6050_ACCEL_FS_2;
  }
}

int accelRegisterValueToRange(int value)
{
  switch (value)
  {
    case MPU6050_ACCEL_FS_2:
      return 2;
    case MPU6050_ACCEL_FS_4:
      return 4;
    case MPU6050_ACCEL_FS_8:
      return 8;
    case MPU6050_ACCEL_FS_16:
      return 16;
    default:
      return -1;
  }
}

int gyroRangeToRegisterValue(int value)
{
  switch (value)
  {
    case 250:
      return MPU6050_GYRO_FS_250;
    case 500:
      return MPU6050_GYRO_FS_500;
    case 1000:
      return MPU6050_GYRO_FS_1000;
    case 2000:
      return MPU6050_GYRO_FS_2000;
    default:
      return MPU6050_GYRO_FS_2000;
  }
}

int gyroRegisterValueToRange(int value)
{
  switch (value)
  {
    case MPU6050_GYRO_FS_250:
      return 250;
    case MPU6050_GYRO_FS_500:
      return 500;
    case MPU6050_GYRO_FS_1000:
      return 1000;
    case MPU6050_GYRO_FS_2000:
      return 2000;
    default:
      return -1;
  }
}

void applyConfiguredStripLength()
{
  ledsPerStrip = currentStripLength;
  totalLeds = ledsPerStrip * 2;
}

void applyPersistentConfig()
{
  syncEngine.cancel();
  applyConfiguredStripLength();
  currentAutoplayEnabled = sanitizeAutoplayEnabled(currentAutoplayEnabled);
  currentAutoplayIntervalMs = sanitizeAutoplayIntervalMs(currentAutoplayIntervalMs);
  if (currentBootMode == BOOT_MODE_MANUAL)
  {
    currentPlayMode = PLAY_MODE_MANUAL;
  }
  else if (currentBootMode == BOOT_MODE_AUTOPLAY)
  {
    currentPlayMode = PLAY_MODE_AUTOPLAY;
  }
  else if (currentBootMode == BOOT_MODE_SYNC)
  {
    currentPlayMode = PLAY_MODE_SYNC;
  }
  setPlayMode(currentPlayMode);
  BRIGHTNESS = currentBrightness;
  applyEffectiveBrightness();
  resetAutoplayTimer();
}

int sanitizeAutoplayEnabled(int value)
{
  return value != 0 ? 1 : 0;
}

int sanitizeAutoplayIntervalMs(int value)
{
  return constrain(value, MIN_AUTOPLAY_INTERVAL_MS, MAX_AUTOPLAY_INTERVAL_MS);
}

void resetAutoplayTimer()
{
  autoplayLastSwitchMs = millis();
  autoplayWasPaused = false;
}

bool isAutoplayEnabled()
{
  return currentAutoplayEnabled != 0;
}

bool isSyncMasterAutoplayActive()
{
  return isAutoplayEnabled() &&
      currentPlayMode == PLAY_MODE_SYNC &&
      currentSyncEnabled == 1 &&
      currentSyncRole == SYNC_ROLE_MASTER;
}

bool shouldRunAutoplayTick()
{
  if (!isAutoplayEnabled())
  {
    return false;
  }
  return currentPlayMode == PLAY_MODE_AUTOPLAY || isSyncMasterAutoplayActive();
}

unsigned long autoplayNextDueMs()
{
  if (!shouldRunAutoplayTick())
  {
    return 0;
  }
  const unsigned long elapsedMs = millis() - autoplayLastSwitchMs;
  if (elapsedMs >= (unsigned long)currentAutoplayIntervalMs)
  {
    return 0;
  }
  return (unsigned long)currentAutoplayIntervalMs - elapsedMs;
}

void setAutoplayEnabledFlag(int enabled)
{
  currentAutoplayEnabled = sanitizeAutoplayEnabled(enabled);
  if (currentPlayMode != PLAY_MODE_SYNC)
  {
    currentPlayMode = currentAutoplayEnabled ? PLAY_MODE_AUTOPLAY : PLAY_MODE_MANUAL;
  }
  resetAutoplayTimer();
}

const char* autoplayEnabledToString()
{
  return isAutoplayEnabled() ? "on" : "off";
}

int parseOnOffValue(String valueText)
{
  valueText.trim();
  valueText.toLowerCase();
  if (valueText == "on" || valueText == "1" || valueText == "true")
  {
    return 1;
  }
  if (valueText == "off" || valueText == "0" || valueText == "false")
  {
    return 0;
  }
  return -1;
}

bool parseIntValue(String value, int* output)
{
  value.trim();
  return parseStrictInt(value.c_str(), output);
}

bool parseUint32Value(String value, uint32_t* output)
{
  value.trim();
  return parseStrictUint32(value.c_str(), output);
}

bool parseBinaryValue(String value, int* output)
{
  const int parsed = parseOnOffValue(value);
  if (parsed < 0 || output == NULL)
  {
    return false;
  }
  *output = parsed;
  return true;
}

void bootMark(const char* stage)
{
  bootStage = (stage != NULL) ? stage : "unknown";
  digitalWrite(LED_BUILTIN, !digitalRead(LED_BUILTIN));
#if NIGHTKITE_BOOT_DEBUG
  Serial.print("BOOT ");
  Serial.println(bootStage);
#endif
}

int sanitizeBinaryFlag(int value)
{
  return value != 0 ? 1 : 0;
}

void copyCString(char* dest, size_t destSize, const char* source)
{
  if (dest == NULL || destSize == 0)
  {
    return;
  }
  if (source == NULL)
  {
    dest[0] = '\0';
    return;
  }
  strncpy(dest, source, destSize - 1);
  dest[destSize - 1] = '\0';
}

void writeEEPROMCString(int address, const char* value, size_t maxLength)
{
  for (size_t i = 0; i < maxLength; i++)
  {
    char ch = (value != NULL && value[i] != '\0') ? value[i] : '\0';
    EEPROM.write(address + (int)i, (uint8_t)ch);
    if (ch == '\0')
    {
      for (size_t j = i + 1; j < maxLength; j++)
      {
        EEPROM.write(address + (int)j, 0);
      }
      break;
    }
  }
}

void readEEPROMCString(int address, char* value, size_t maxLength)
{
  if (value == NULL || maxLength == 0)
  {
    return;
  }
  for (size_t i = 0; i < maxLength - 1; i++)
  {
    value[i] = (char)EEPROM.read(address + (int)i);
    if (value[i] == '\0')
    {
      value[i] = '\0';
      return;
    }
  }
  value[maxLength - 1] = '\0';
}

bool isValidDeviceUid(const char* value)
{
  if (value == NULL || strlen(value) != DEVICE_UID_LENGTH)
  {
    return false;
  }
  for (size_t i = 0; i < DEVICE_UID_LENGTH; i++)
  {
    const char ch = value[i];
    if (!((ch >= '0' && ch <= '9') || (ch >= 'A' && ch <= 'F')))
    {
      return false;
    }
  }
  return true;
}

void updateShortIdFromUid()
{
  if (!isValidDeviceUid(currentDeviceUid))
  {
    copyCString(currentShortId, sizeof(currentShortId), "000000");
    return;
  }
  copyCString(currentShortId, sizeof(currentShortId), currentDeviceUid + DEVICE_UID_LENGTH - SHORT_ID_LENGTH);
}

void generateDeviceUid()
{
  const char* hex = "0123456789ABCDEF";
  randomSeed((uint32_t)micros() ^ ((uint32_t)analogRead(PIN_BATTERY_ADC) << 10) ^ ((uint32_t)millis() << 20));
  for (size_t i = 0; i < DEVICE_UID_LENGTH; i++)
  {
    currentDeviceUid[i] = hex[random(0, 16)];
  }
  currentDeviceUid[DEVICE_UID_LENGTH] = '\0';
  updateShortIdFromUid();
}

void setDefaultDeviceName()
{
  String name = "NK-";
  name += currentShortId;
  copyCString(currentDeviceName, sizeof(currentDeviceName), name.c_str());
}

bool sanitizeDeviceName(String value, char* output, size_t outputSize)
{
  value.trim();
  if (value.length() == 0 || output == NULL || outputSize < 2)
  {
    return false;
  }
  if (value.length() >= (int)outputSize)
  {
    return false;
  }
  for (int i = 0; i < value.length(); i++)
  {
    const char ch = value[i];
    if (!isalnum((unsigned char)ch) && ch != '-' && ch != '_' && ch != '.')
    {
      return false;
    }
  }
  copyCString(output, outputSize, value.c_str());
  return true;
}

bool sanitizeUidString(String value, char* output, size_t outputSize)
{
  value.trim();
  value.toUpperCase();
  if (output == NULL || outputSize < DEVICE_UID_LENGTH + 1 || value.length() != (int)DEVICE_UID_LENGTH)
  {
    return false;
  }
  for (int i = 0; i < value.length(); i++)
  {
    const char ch = value[i];
    if (!isxdigit((unsigned char)ch))
    {
      return false;
    }
  }
  copyCString(output, outputSize, value.c_str());
  return true;
}

bool readStoredDeviceUid(char* output, size_t outputSize)
{
  if (output == NULL || outputSize < DEVICE_UID_LENGTH + 1)
  {
    return false;
  }

  char storedUid[DEVICE_UID_LENGTH + 1];
  readEEPROMCString(EEPROM_ADDR_DEVICE_UID, storedUid, sizeof(storedUid));
  if (!isValidDeviceUid(storedUid))
  {
    return false;
  }

  copyCString(output, outputSize, storedUid);
  return true;
}

bool readStoredDeviceName(char* output, size_t outputSize)
{
  char storedName[DEVICE_NAME_LENGTH + 1];
  readEEPROMCString(EEPROM_ADDR_DEVICE_NAME, storedName, sizeof(storedName));
  return sanitizeDeviceName(String(storedName), output, outputSize);
}

void applyDefaultExtendedConfig(bool resetName)
{
  currentConfigVersion = CONFIG_VERSION_4_ALPHA;
  currentPlayMode = DEFAULT_PLAY_MODE;
  currentBootMode = DEFAULT_BOOT_MODE;
  currentSyncEnabled = DEFAULT_SYNC_ENABLED;
  currentSyncGroupId = DEFAULT_SYNC_GROUP_ID;
  currentSyncRole = DEFAULT_SYNC_ROLE;
  currentSyncMasterUid[0] = '\0';
  currentSyncLossBehavior = DEFAULT_SYNC_LOSS_BEHAVIOR;
  currentWirelessEnabled = DEFAULT_WIRELESS_ENABLED;
  currentWirelessProfile = DEFAULT_WIRELESS_PROFILE;
  if (resetName)
  {
    setDefaultDeviceName();
  }
}

void ensureDeviceIdentity()
{
  if (!isValidDeviceUid(currentDeviceUid))
  {
    generateDeviceUid();
  }
  else
  {
    updateShortIdFromUid();
  }

  char sanitizedName[DEVICE_NAME_LENGTH + 1];
  if (!sanitizeDeviceName(String(currentDeviceName), sanitizedName, sizeof(sanitizedName)))
  {
    setDefaultDeviceName();
  }
  else
  {
    copyCString(currentDeviceName, sizeof(currentDeviceName), sanitizedName);
  }
}

const char* playModeToString(int value)
{
  switch (value)
  {
    case PLAY_MODE_AUTOPLAY: return "autoplay";
    case PLAY_MODE_SYNC: return "sync";
    case PLAY_MODE_MANUAL:
    default: return "manual";
  }
}

int parsePlayMode(String value)
{
  value.trim();
  value.toLowerCase();
  if (value == "manual") return PLAY_MODE_MANUAL;
  if (value == "autoplay") return PLAY_MODE_AUTOPLAY;
  if (value == "sync") return PLAY_MODE_SYNC;
  return -1;
}

const char* bootModeToString(int value)
{
  switch (value)
  {
    case BOOT_MODE_MANUAL: return "manual";
    case BOOT_MODE_AUTOPLAY: return "autoplay";
    case BOOT_MODE_SYNC: return "sync";
    case BOOT_MODE_LAST:
    default: return "last";
  }
}

int parseBootMode(String value)
{
  value.trim();
  value.toLowerCase();
  if (value == "last" || value == "0") return BOOT_MODE_LAST;
  if (value == "manual") return BOOT_MODE_MANUAL;
  if (value == "autoplay") return BOOT_MODE_AUTOPLAY;
  if (value == "sync") return BOOT_MODE_SYNC;
  return -1;
}

const char* syncRoleToString(int value)
{
  switch (value)
  {
    case SYNC_ROLE_MASTER: return "master";
    case SYNC_ROLE_FOLLOWER: return "follower";
    case SYNC_ROLE_STANDALONE:
    default: return "standalone";
  }
}

int parseSyncRole(String value)
{
  value.trim();
  value.toLowerCase();
  if (value == "standalone") return SYNC_ROLE_STANDALONE;
  if (value == "master") return SYNC_ROLE_MASTER;
  if (value == "follower") return SYNC_ROLE_FOLLOWER;
  return -1;
}

const char* syncLossBehaviorToString(int value)
{
  switch (value)
  {
    case SYNC_LOSS_FALLBACK_AUTOPLAY: return "fallback_autoplay";
    case SYNC_LOSS_WARNING_ONLY: return "warning_only";
    case SYNC_LOSS_CONTINUE_LOCAL:
    default: return "continue_local";
  }
}

int parseSyncLossBehavior(String value)
{
  value.trim();
  value.toLowerCase();
  if (value == "continue_local") return SYNC_LOSS_CONTINUE_LOCAL;
  if (value == "fallback_autoplay") return SYNC_LOSS_FALLBACK_AUTOPLAY;
  if (value == "warning_only") return SYNC_LOSS_WARNING_ONLY;
  return -1;
}

const char* wirelessProfileToString(int value)
{
  switch (value)
  {
    case WIRELESS_PROFILE_LONG_RANGE: return "long_range";
    case WIRELESS_PROFILE_FAST_SYNC: return "fast_sync";
    case WIRELESS_PROFILE_BALANCED:
    default: return "balanced";
  }
}

int parseWirelessProfile(String value)
{
  value.trim();
  value.toLowerCase();
  if (value == "long_range") return WIRELESS_PROFILE_LONG_RANGE;
  if (value == "balanced") return WIRELESS_PROFILE_BALANCED;
  if (value == "fast_sync") return WIRELESS_PROFILE_FAST_SYNC;
  return -1;
}

String formatHex32(uint32_t value)
{
  char buffer[11];
  snprintf(buffer, sizeof(buffer), "0x%08lX", (unsigned long)value);
  return String(buffer);
}

int patternSyncClass(uint8_t patternId)
{
  switch (patternId)
  {
    case 1:
      return PATTERN_SYNC_READY;
    case 4:
    case 7:
    case 8:
    case 10:
    case 15:
    case 16:
    case 19:
    case 20:
    case 23:
    case 24:
    case 25:
    case 26:
    case 27:
      return PATTERN_SYNC_PARTIAL;
    case 2:
    case 3:
    case 5:
    case 6:
    case 9:
    case 11:
    case 12:
    case 13:
    case 14:
    case 17:
    case 18:
    case 21:
    case 22:
      return PATTERN_SYNC_LOCAL_REACTIVE;
    default:
      return PATTERN_SYNC_UNKNOWN;
  }
}

bool isSyncReadyPattern(uint8_t patternId)
{
  return patternSyncClass(patternId) == PATTERN_SYNC_READY;
}

bool isPartialSyncPattern(uint8_t patternId)
{
  return patternSyncClass(patternId) == PATTERN_SYNC_PARTIAL;
}

uint32_t buildPatternClassMask(int syncClass)
{
  uint32_t mask = 0;
  for (uint8_t patternId = FIRST_PATTERN_ID; patternId <= LAST_PATTERN_ID; ++patternId)
  {
    if (patternSyncClass(patternId) == syncClass)
    {
      mask |= (uint32_t)(1ul << (patternId - FIRST_PATTERN_ID));
    }
  }
  return mask;
}

String buildPatternMaskFields()
{
  String fields = "count=";
  fields += PATTERN_COUNT;
  fields += " active=";
  fields += currentPattern;
  fields += " enabled_mask=";
  fields += formatHex32(currentEnabledPatternMask);
  fields += " inverted_mask=";
  fields += formatHex32(currentInvertedPatternMask);
  fields += " sync_ready_mask=";
  fields += formatHex32(buildPatternClassMask(PATTERN_SYNC_READY));
  fields += " partial_sync_mask=";
  fields += formatHex32(buildPatternClassMask(PATTERN_SYNC_PARTIAL));
  fields += " local_reactive_mask=";
  fields += formatHex32(buildPatternClassMask(PATTERN_SYNC_LOCAL_REACTIVE));
  return fields;
}

String buildConfigFields()
{
  String fields = "pattern=";
  fields += currentPattern;
  fields += " brightness=";
  fields += currentBrightness;
  fields += " strip_length=";
  fields += currentStripLength;
  fields += " smoothing=";
  fields += currentMotionSmoothingSize;
  fields += " accel_range=";
  fields += currentAccelRange;
  fields += " gyro_range=";
  fields += currentGyroRange;
  fields += " boot_calibration=";
  fields += bootCalibrationModeToString(currentBootCalibrationMode);
  fields += " autoplay=";
  fields += currentAutoplayEnabled;
  fields += " autoplay_interval=";
  fields += currentAutoplayIntervalMs / 1000;
  fields += " sync_autoplay=";
  fields += (currentPlayMode == PLAY_MODE_SYNC && isAutoplayEnabled()) ? 1 : 0;
  fields += " master_autoplay=";
  fields += isSyncMasterAutoplayActive() ? 1 : 0;
  fields += " autoplay_next_ms=";
  fields += autoplayNextDueMs();
  fields += " play_mode=";
  fields += playModeToString(currentPlayMode);
  fields += " boot_mode=";
  fields += bootModeToString(currentBootMode);
  fields += " config_valid=";
  fields += configValid ? 1 : 0;
  fields += " config_repaired=";
  fields += configRepaired ? 1 : 0;
  fields += " config_version=";
  fields += currentConfigVersion;
  fields += " safe_boot=";
  fields += safeBootActive ? 1 : 0;
  fields += " ";
  fields += buildPatternMaskFields();
  return fields;
}

String buildSyncFields(bool detailed)
{
  String fields = "sync_enabled=";
  fields += currentSyncEnabled;
  fields += " sync_group=";
  fields += currentSyncGroupId;
  fields += " sync_role=";
  fields += syncRoleToString(currentSyncRole);
  fields += " sync_state=";
  fields += syncEngine.stateName();
  fields += " sync_loss_behavior=";
  fields += syncLossBehaviorToString(currentSyncLossBehavior);
  fields += " sync_loss_count=";
  fields += syncLossCount;
  fields += " last_sync_loss_ms=";
  fields += lastSyncLossMs;
  fields += " last_sync_loss_action=";
  fields += lastSyncLossAction;
  fields += " master_uid=";
  fields += strlen(currentSyncMasterUid) > 0 ? currentSyncMasterUid : "none";
  fields += " last_seq=";
  fields += syncEngine.lastSeq;
  fields += " locked=";
  fields += syncEngine.locked ? 1 : 0;
  fields += " drift_ms=";
  fields += syncEngine.driftMs;
  fields += " sync_pattern=";
  if (syncEngine.state == SyncEngine::ARMED ||
      syncEngine.state == SyncEngine::RUNNING ||
      syncEngine.state == SyncEngine::LOST)
  {
    fields += syncEngine.armedPattern;
  }
  else
  {
    fields += currentPattern;
  }
  fields += " local_pattern=";
  fields += currentPattern;
  fields += " last_beacon_pattern=";
  fields += lastBeaconPattern;
  fields += " last_beacon_brightness=";
  fields += lastBeaconBrightness;
  fields += " last_beacon_seq=";
  fields += lastBeaconSeq;
  fields += " last_applied_seq=";
  fields += lastAppliedSeq;
  fields += " phase_ms=";
  fields += patternClock.phaseMs();
  fields += " beacon_phase_ms=";
  fields += lastBeaconPhaseMs;
  fields += " sync_apply_count=";
  fields += syncApplyCount;
  fields += " sync_apply_skipped=";
  fields += syncApplySkipped;
  fields += " sync_apply_reason=";
  fields += syncApplyReason;
  fields += " pattern_change_count=";
  fields += patternChangeCount;
  fields += " last_pattern_change_ms=";
  fields += lastPatternChangeMs;
  fields += " last_pattern_change_latency_ms=";
  fields += lastPatternChangeLatencyMs;
  fields += " last_pattern_from=";
  fields += lastPatternFrom;
  fields += " last_pattern_to=";
  fields += lastPatternTo;
  fields += " last_pattern_change_source=";
  fields += lastPatternChangeSource;
  fields += " sync_ready_pattern=";
  fields += isSyncReadyPattern((uint8_t)currentPattern) ? 1 : 0;
  fields += " partial_sync_pattern=";
  fields += isPartialSyncPattern((uint8_t)currentPattern) ? 1 : 0;
  fields += " sync_autoplay=";
  fields += (currentPlayMode == PLAY_MODE_SYNC && isAutoplayEnabled()) ? 1 : 0;
  fields += " master_autoplay=";
  fields += isSyncMasterAutoplayActive() ? 1 : 0;
  if (detailed)
  {
    fields += " armed_group=";
    fields += syncEngine.armedGroup;
    fields += " armed_pattern=";
    fields += syncEngine.armedPattern;
    fields += " armed_brightness=";
    fields += syncEngine.armedBrightness;
    fields += " local_start_ms=";
    fields += syncEngine.localStartMs;
    fields += " pattern_time_ms=";
    fields += patternClock.now();
  }
  fields += " ";
  fields += syncBeaconRadioBuildStatusFields();
  return fields;
}

String buildWirelessFields()
{
  String fields = "wireless_enabled=";
  fields += currentWirelessEnabled;
  fields += " wireless_profile=";
  fields += wirelessProfileToString(currentWirelessProfile);
  fields += " ble=";
  fields += (NIGHTKITE_BLE ? 1 : 0);
  fields += " rm2=";
  fields += (NIGHTKITE_RM2 ? 1 : 0);
  fields += " ";
  fields += rm2BleBuildStatusFields();
  fields += " ";
  fields += syncBeaconRadioBuildStatusFields();
  return fields;
}

String buildBatteryFields()
{
  updateBatteryMeasurement(false);
  const int usbSenseRaw = digitalRead(PIN_USB_SENSE);
  String fields = "battery_raw=";
  fields += RawVoltage;
  fields += " battery_voltage=";
  fields += String(Voltage, 3);
  fields += " battery_percent=";
  fields += BatteryPercent;
  fields += " battery_state=";
  fields += batteryStateName(currentBatteryState);
  fields += " usb_power_raw=";
  fields += usbSenseRaw;
  fields += " serial_session_active=";
  fields += SerialSessionActive ? 1 : 0;
  return fields;
}

String buildSensorFields()
{
  const bool mpuConnected = !safeBootActive && (mpu.testConnection() == true);
  const int activeAccelRange = imuReady ? accelRegisterValueToRange(mpu.getFullScaleAccelRange()) : -1;
  const int activeGyroRange = imuReady ? gyroRegisterValueToRange(mpu.getFullScaleGyroRange()) : -1;
  String fields = "mpu_connected=";
  fields += mpuConnected ? 1 : 0;
  fields += " dmp_ready=";
  fields += DMPReady ? 1 : 0;
  fields += " imu=";
  fields += imuReady ? 1 : 0;
  fields += " dev_status=";
  fields += devStatus;
  fields += " packet_size=";
  fields += packetSize;
  fields += " int_status=";
  fields += MPUIntStatus;
  fields += " smoothing_config=";
  fields += currentMotionSmoothingSize;
  fields += " smoothing_active=";
  fields += activeMotionSmoothingSize;
  fields += " accel_range_config=";
  fields += currentAccelRange;
  fields += " accel_range_active=";
  fields += activeAccelRange;
  fields += " gyro_range_config=";
  fields += currentGyroRange;
  fields += " gyro_range_active=";
  fields += activeGyroRange;
  return fields;
}

String buildTimingFields()
{
  uint32_t avgLoopDurationUs = 0;
  uint32_t avgWorkDurationUs = 0;
  if (loopTimingSamples > 0)
  {
    avgLoopDurationUs = (uint32_t)(totalLoopDurationUs / loopTimingSamples);
    avgWorkDurationUs = (uint32_t)(totalWorkDurationUs / loopTimingSamples);
  }

  String fields = "fps=";
  fields += FastLED.getFPS();
  fields += " last_loop_us=";
  fields += lastLoopDurationUs;
  fields += " avg_loop_us=";
  fields += avgLoopDurationUs;
  fields += " max_loop_us=";
  fields += maxLoopDurationUs;
  fields += " last_work_us=";
  fields += lastWorkDurationUs;
  fields += " avg_work_us=";
  fields += avgWorkDurationUs;
  fields += " max_work_us=";
  fields += maxWorkDurationUs;
  fields += " frame_budget_us=";
  fields += 1000000UL / FRAMES_PER_SECOND;
  fields += " samples=";
  fields += loopTimingSamples;
  return fields;
}

String buildOffsetsFields()
{
  String fields = "x_accel_offset=";
  fields += currentXAccelOffset;
  fields += " y_accel_offset=";
  fields += currentYAccelOffset;
  fields += " z_accel_offset=";
  fields += currentZAccelOffset;
  fields += " x_gyro_offset=";
  fields += currentXGyroOffset;
  fields += " y_gyro_offset=";
  fields += currentYGyroOffset;
  fields += " z_gyro_offset=";
  fields += currentZGyroOffset;
  return fields;
}

void setPlayMode(int mode)
{
  const int previousMode = currentPlayMode;
  if (mode < PLAY_MODE_MANUAL || mode > PLAY_MODE_SYNC)
  {
    mode = PLAY_MODE_MANUAL;
  }
  currentPlayMode = mode;
  if (previousMode == PLAY_MODE_SYNC && currentPlayMode != PLAY_MODE_SYNC)
  {
    syncEngine.cancel();
  }
  if (currentPlayMode == PLAY_MODE_AUTOPLAY)
  {
    currentAutoplayEnabled = 1;
  }
  else if (currentPlayMode == PLAY_MODE_MANUAL)
  {
    currentAutoplayEnabled = 0;
  }
  resetAutoplayTimer();
}

void cyclePlayMode()
{
  if (currentPlayMode == PLAY_MODE_MANUAL)
  {
    setPlayMode(PLAY_MODE_AUTOPLAY);
  }
  else if (currentPlayMode == PLAY_MODE_AUTOPLAY)
  {
    setPlayMode(PLAY_MODE_SYNC);
  }
  else
  {
    setPlayMode(PLAY_MODE_MANUAL);
  }
}

CRGB playModeIndicatorColor(int mode, int syncRole, bool syncError)
{
  if (syncError)
  {
    return CRGB::Red;
  }
  if (mode == PLAY_MODE_AUTOPLAY)
  {
    return CRGB::Green;
  }
  if (mode == PLAY_MODE_SYNC)
  {
    return (syncRole == SYNC_ROLE_MASTER) ? CRGB::Magenta : CRGB::Cyan;
  }
  return CRGB::Blue;
}

void applySyncStartIfDue()
{
  if (!syncEngine.consumeStartEvent())
  {
    return;
  }

  currentBrightness = syncEngine.armedBrightness;
  BRIGHTNESS = currentBrightness;
  applyEffectiveBrightness();
  switchToPattern(syncEngine.armedPattern, true, "sync_arm");
  setPlayMode(PLAY_MODE_SYNC);
}

void recordPatternChange(uint8_t fromPattern, uint8_t toPattern, const char* source, uint32_t latencyMs)
{
  patternChangeCount++;
  lastPatternChangeMs = millis();
  lastPatternChangeLatencyMs = latencyMs;
  lastPatternFrom = fromPattern;
  lastPatternTo = toPattern;
  lastPatternChangeSource = source != NULL ? source : "local";
}

SyncBeaconRuntime buildSyncBeaconRuntime()
{
  SyncBeaconRuntime runtime;
  runtime.syncEnabled = currentSyncEnabled == 1;
  runtime.wirelessEnabled = currentWirelessEnabled == 1;
  runtime.playMode = (uint8_t)currentPlayMode;
  runtime.syncRole = (uint8_t)currentSyncRole;
  runtime.groupId = (uint8_t)currentSyncGroupId;
  runtime.pattern = (uint8_t)currentPattern;
  runtime.brightness = (uint8_t)currentBrightness;
  runtime.wirelessProfile = (uint8_t)currentWirelessProfile;
  runtime.phaseMs = patternClock.now();
  runtime.beatMs = NK_SYNC_BEACON_BEAT_MS;
  return runtime;
}

void applyReceivedSyncBeacon()
{
  NkSyncBeaconV1 beacon;
  while (syncBeaconRadioConsumeBeacon(&beacon))
  {
    const SyncBeaconRadioStatus radioStatus = syncBeaconRadioStatus();
    const unsigned long receiveMs = millis();
    lastBeaconPattern = beacon.pattern;
    lastBeaconBrightness = beacon.brightness;
    lastBeaconSeq = beacon.seq;
    lastBeaconPhaseMs = beacon.phaseMs;

    if (currentPlayMode != PLAY_MODE_SYNC || currentSyncEnabled != 1 ||
        currentSyncRole != SYNC_ROLE_FOLLOWER || beacon.groupId != currentSyncGroupId)
    {
      syncApplySkipped++;
      syncApplyReason = "not_follower";
      continue;
    }

    const uint32_t localPhaseBeforeUpdate = patternClock.now();
    syncEngine.state = SyncEngine::RUNNING;
    syncEngine.locked = true;
    syncEngine.lastSeq = beacon.seq;
    syncEngine.armedGroup = beacon.groupId;
    syncEngine.armedPattern = beacon.pattern;
    syncEngine.armedBrightness = beacon.brightness;
    syncEngine.armedPhaseMs = beacon.phaseMs;
    syncEngine.localStartMs = millis();
    syncEngine.driftMs = syncPhaseDeltaMs(beacon.phaseMs, localPhaseBeforeUpdate, beacon.beatMs);
    syncApplyCount++;
    lastAppliedSeq = beacon.seq;
    syncApplyReason = "ok";

    if (isValidBrightnessLevel(beacon.brightness))
    {
      currentBrightness = beacon.brightness;
      BRIGHTNESS = currentBrightness;
      applyEffectiveBrightness();
    }
    else
    {
      syncApplySkipped++;
      syncApplyReason = "bad_brightness";
    }

    if (!isValidPatternId(beacon.pattern))
    {
      syncApplySkipped++;
      syncApplyReason = "bad_pattern";
    }
    else if (!isPatternEnabled(beacon.pattern))
    {
      syncApplySkipped++;
      syncApplyReason = "pattern_disabled";
    }
    else if (currentPattern != beacon.pattern)
    {
      const uint32_t latencyMs = radioStatus.lastBeaconMs == 0 ? 0 : receiveMs - radioStatus.lastBeaconMs;
      switchToPattern(beacon.pattern, true, "sync_beacon", latencyMs);
    }
    patternClock.syncToBeaconPhase(beacon.phaseMs);
  }
}

void tickSyncBeaconRadio()
{
  syncBeaconRadioTick(buildSyncBeaconRuntime());
  applyReceivedSyncBeacon();

  const SyncBeaconRadioStatus radioStatus = syncBeaconRadioStatus();
  if (currentPlayMode == PLAY_MODE_SYNC && currentSyncEnabled == 1 && currentSyncRole == SYNC_ROLE_MASTER && radioStatus.beaconTx)
  {
    syncEngine.state = SyncEngine::RUNNING;
    syncEngine.locked = true;
    syncEngine.lastSeq = radioStatus.beaconSeq;
    syncEngine.armedGroup = currentSyncGroupId;
    syncEngine.armedPattern = currentPattern;
    syncEngine.armedBrightness = currentBrightness;
    syncEngine.armedPhaseMs = patternClock.phaseMs();
  }
  else if (currentPlayMode == PLAY_MODE_SYNC && currentSyncEnabled == 1 && currentSyncRole == SYNC_ROLE_FOLLOWER &&
      syncEngine.locked && !radioStatus.locked)
  {
    syncEngine.state = SyncEngine::LOST;
    syncEngine.locked = false;
    syncLossCount++;
    lastSyncLossMs = millis();
    if (currentSyncLossBehavior == SYNC_LOSS_FALLBACK_AUTOPLAY)
    {
      lastSyncLossAction = "fallback_autoplay";
      currentPlayMode = PLAY_MODE_AUTOPLAY;
      currentAutoplayEnabled = 1;
      resetAutoplayTimer();
    }
    else if (currentSyncLossBehavior == SYNC_LOSS_WARNING_ONLY)
    {
      lastSyncLossAction = "warning_only";
    }
    else
    {
      lastSyncLossAction = "continue_local";
    }
  }
}

void announcePatternChange(const char* source)
{
  if (usbProtocolMode == USB_PROTOCOL_MACHINE)
  {
    String fields = "source=";
    fields += source != NULL ? source : "unknown";
    fields += " pattern=";
    fields += currentPattern;
    emitNk4Event("pattern_changed", fields);
    return;
  }

  Serial.print("INFO pattern_changed source=");
  Serial.print(source != NULL ? source : "unknown");
  Serial.print(" pattern=");
  Serial.println(currentPattern);
}

void applyConfiguredMotionSmoothing()
{
  activeMotionSmoothingSize = currentMotionSmoothingSize;
  myAccel.begin(SMOOTHED_AVERAGE, activeMotionSmoothingSize);
  myAccel.clear();
}

void applyConfiguredSensorRanges()
{
  mpu.setFullScaleAccelRange((uint8_t)accelRangeToRegisterValue(currentAccelRange));
  mpu.setFullScaleGyroRange((uint8_t)gyroRangeToRegisterValue(currentGyroRange));
}

void applyConfiguredOffsets()
{
  mpu.setXAccelOffset(currentXAccelOffset);
  mpu.setYAccelOffset(currentYAccelOffset);
  mpu.setZAccelOffset(currentZAccelOffset);
  mpu.setXGyroOffset(currentXGyroOffset);
  mpu.setYGyroOffset(currentYGyroOffset);
  mpu.setZGyroOffset(currentZGyroOffset);
}

void syncConfiguredOffsetsFromMPU()
{
  currentXAccelOffset = mpu.getXAccelOffset();
  currentYAccelOffset = mpu.getYAccelOffset();
  currentZAccelOffset = mpu.getZAccelOffset();
  currentXGyroOffset = mpu.getXGyroOffset();
  currentYGyroOffset = mpu.getYGyroOffset();
  currentZGyroOffset = mpu.getZGyroOffset();
}

void printOffsets()
{
  printOffsetsWithPrefix("");
}

void printOffsetsWithPrefix(const char* prefix)
{
  Serial.print(prefix);
  Serial.print("x_accel_offset=");
  Serial.print(currentXAccelOffset);
  Serial.print(" y_accel_offset=");
  Serial.print(currentYAccelOffset);
  Serial.print(" z_accel_offset=");
  Serial.print(currentZAccelOffset);
  Serial.print(" x_gyro_offset=");
  Serial.print(currentXGyroOffset);
  Serial.print(" y_gyro_offset=");
  Serial.print(currentYGyroOffset);
  Serial.print(" z_gyro_offset=");
  Serial.println(currentZGyroOffset);
}

void printEnabledPatternsList()
{
  // Output only the enabled IDs as a compact comma-separated list for CLI parsing.
  bool first = true;
  for (uint8_t patternId = FIRST_PATTERN_ID; patternId <= LAST_PATTERN_ID; patternId++)
  {
    if (!isPatternEnabled(patternId))
    {
      continue;
    }

    if (!first)
    {
      Serial.print(",");
    }
    Serial.print(patternId);
    first = false;
  }
}

void printInvertedPatternsList()
{
  bool first = true;
  for (uint8_t patternId = FIRST_PATTERN_ID; patternId <= LAST_PATTERN_ID; patternId++)
  {
    if (!isPatternInverted(patternId))
    {
      continue;
    }

    if (!first)
    {
      Serial.print(",");
    }
    Serial.print(patternId);
    first = false;
  }
}

void printConfigSummaryWithPrefix(const char* prefix)
{
  // Shared one-line config summary used by show/save/load/defaults replies.
  Serial.print(prefix);
  Serial.print("pattern=");
  Serial.print(currentPattern);
  Serial.print(" brightness=");
  Serial.print(currentBrightness);
  Serial.print(" strip_length=");
  Serial.print(currentStripLength);
  Serial.print(" smoothing=");
  Serial.print(currentMotionSmoothingSize);
  Serial.print(" accel_range=");
  Serial.print(currentAccelRange);
  Serial.print(" gyro_range=");
  Serial.print(currentGyroRange);
  Serial.print(" boot_calibration=");
  Serial.print(bootCalibrationModeToString(currentBootCalibrationMode));
  Serial.print(" autoplay=");
  Serial.print(autoplayEnabledToString());
  Serial.print(" autoplay_interval=");
  Serial.print(currentAutoplayIntervalMs / 1000);
  Serial.print(" enabled_patterns=");
  printEnabledPatternsList();
  Serial.print(" inverted_patterns=");
  printInvertedPatternsList();
  Serial.println();
}

void printPatternStates()
{
  // Human-readable overview of all patterns and whether button cycling includes them.
  Serial.print("OK patterns=");
  for (uint8_t patternId = FIRST_PATTERN_ID; patternId <= LAST_PATTERN_ID; patternId++)
  {
    const PatternDefinition* pattern = getPatternDefinition(patternId);
    if (patternId > FIRST_PATTERN_ID)
    {
      Serial.print(",");
    }
    Serial.print(patternId);
    Serial.print(":");
    if (pattern != NULL && pattern->name != NULL)
    {
      Serial.print(pattern->name);
    }
    else
    {
      Serial.print("pattern");
    }
    Serial.print(":");
    Serial.print(isPatternEnabled(patternId) ? "on" : "off");
  }
  Serial.println();
}

bool beginCalibrationSession(bool verbose, bool* restartDMP)
{
  if (devStatus != 0)
  {
    if (verbose)
    {
      Serial.println("ERR calibration unavailable while DMP init failed");
    }
    return false;
  }

  *restartDMP = DMPReady;
  if (*restartDMP)
  {
    if (verbose)
    {
      Serial.println("INFO calibration pausing_dmp=1");
    }
    detachInterrupt(digitalPinToInterrupt(INTERRUPT_PIN));
    mpu.setDMPEnabled(false);
    DMPReady = false;
    delay(50);
  }

  return true;
}

void endCalibrationSession(bool restartDMP)
{
  if (restartDMP)
  {
    mpu.resetFIFO();
    mpu.setDMPEnabled(true);
    attachInterrupt(digitalPinToInterrupt(INTERRUPT_PIN), DMPDataReady, RISING);
    MPUIntStatus = mpu.getIntStatus();
    DMPReady = true;
  }
}

bool runQuickCalibration(bool verbose)
{
  bool restartDMP = false;
  if (!beginCalibrationSession(verbose, &restartDMP))
  {
    return false;
  }

  if (verbose)
  {
    Serial.println("INFO calibration mode=quick phase=start");
  }

  mpu.CalibrateAccel(6);
  mpu.CalibrateGyro(6);
  syncConfiguredOffsetsFromMPU();

  endCalibrationSession(restartDMP);

  if (verbose)
  {
    Serial.println("INFO calibration mode=quick phase=offsets");
    mpu.PrintActiveOffsets();
  }

  return true;
}

bool runPreciseCalibration(bool verbose)
{
  const int axisCount = 6;
  const int iAx = 0;
  const int iAy = 1;
  const int iAz = 2;
  const int iGx = 3;
  const int iGy = 4;
  const int iGz = 5;
  const int sampleDelayUs = 3150;
  const int fastSamples = 1000;
  const int slowSamples = 10000;
  const int linesBetweenHeaders = 5;

  bool restartDMP = false;
  if (!beginCalibrationSession(verbose, &restartDMP))
  {
    return false;
  }

  if (verbose)
  {
    Serial.println("INFO calibration mode=precise phase=start");
    Serial.println("INFO calibration instruction=\"Keep the device still on a flat, level surface.\"");
  }

  int lowValue[axisCount];
  int highValue[axisCount];
  int smoothed[axisCount];
  int lowOffset[axisCount];
  int highOffset[axisCount];
  int target[axisCount];
  int newOffset[axisCount];
  long sums[axisCount];
  int linesOut = 99;
  int sampleCount = fastSamples;

  auto forceHeader = [&]() {
    linesOut = 99;
  };

  auto printHeaderIfNeeded = [&]() {
    if (linesOut >= linesBetweenHeaders)
    {
      Serial.println("\t\tXAccel\t\t\tYAccel\t\t\t\tZAccel\t\t\tXGyro\t\t\tYGyro\t\t\tZGyro");
      linesOut = 0;
    }
  };

  auto showProgress = [&]() {
    printHeaderIfNeeded();
    Serial.print(' ');
    for (int axis = 0; axis < axisCount; axis++)
    {
      Serial.print('[');
      Serial.print(lowOffset[axis]);
      Serial.print(',');
      Serial.print(highOffset[axis]);
      Serial.print("] --> [");
      Serial.print(lowValue[axis]);
      Serial.print(',');
      Serial.print(highValue[axis]);
      if (axis == axisCount - 1)
      {
        Serial.println("]");
      }
      else
      {
        Serial.print("]\t");
      }
    }
    linesOut++;
  };

  auto setOffsets = [&](const int offsets[axisCount]) {
    mpu.setXAccelOffset(offsets[iAx]);
    mpu.setYAccelOffset(offsets[iAy]);
    mpu.setZAccelOffset(offsets[iAz]);
    mpu.setXGyroOffset(offsets[iGx]);
    mpu.setYGyroOffset(offsets[iGy]);
    mpu.setZGyroOffset(offsets[iGz]);
  };

  auto getSmoothed = [&]() {
    int16_t rawValue[axisCount];
    for (int axis = 0; axis < axisCount; axis++)
    {
      sums[axis] = 0;
    }

    for (int sample = 1; sample <= sampleCount; sample++)
    {
      mpu.getMotion6(&rawValue[iAx], &rawValue[iAy], &rawValue[iAz], &rawValue[iGx], &rawValue[iGy], &rawValue[iGz]);
      delayMicroseconds(sampleDelayUs);
      for (int axis = 0; axis < axisCount; axis++)
      {
        sums[axis] += rawValue[axis];
      }
    }

    for (int axis = 0; axis < axisCount; axis++)
    {
      smoothed[axis] = (int)((sums[axis] + sampleCount / 2) / sampleCount);
    }
  };

  for (int axis = 0; axis < axisCount; axis++)
  {
    target[axis] = 0;
    lowOffset[axis] = 0;
    highOffset[axis] = 0;
  }
  target[iAz] = 16384;

  if (verbose)
  {
    Serial.print("INFO calibration averaging_samples=");
    Serial.print(sampleCount);
    Serial.println(" phase=expanding");
  }
  forceHeader();

  bool done = false;
  while (!done)
  {
    done = true;
    int nextLowOffset[axisCount];
    int nextHighOffset[axisCount];

    setOffsets(lowOffset);
    getSmoothed();
    for (int axis = 0; axis < axisCount; axis++)
    {
      lowValue[axis] = smoothed[axis];
      if (lowValue[axis] >= target[axis])
      {
        done = false;
        nextLowOffset[axis] = lowOffset[axis] - 1000;
      }
      else
      {
        nextLowOffset[axis] = lowOffset[axis];
      }
    }

    setOffsets(highOffset);
    getSmoothed();
    for (int axis = 0; axis < axisCount; axis++)
    {
      highValue[axis] = smoothed[axis];
      if (highValue[axis] <= target[axis])
      {
        done = false;
        nextHighOffset[axis] = highOffset[axis] + 1000;
      }
      else
      {
        nextHighOffset[axis] = highOffset[axis];
      }
    }

    if (verbose)
    {
      showProgress();
    }

    for (int axis = 0; axis < axisCount; axis++)
    {
      lowOffset[axis] = nextLowOffset[axis];
      highOffset[axis] = nextHighOffset[axis];
    }
  }

  if (verbose)
  {
    Serial.println();
    Serial.println("INFO calibration phase=closing_in");
  }
  forceHeader();

  bool allBracketsNarrow = false;
  bool stillWorking = true;
  while (stillWorking)
  {
    stillWorking = false;
    if (allBracketsNarrow && sampleCount == fastSamples)
    {
      sampleCount = slowSamples;
      if (verbose)
      {
        Serial.print("INFO calibration averaging_samples=");
        Serial.print(sampleCount);
        Serial.println(" phase=closing_in");
      }
    }
    else
    {
      allBracketsNarrow = true;
    }

    for (int axis = 0; axis < axisCount; axis++)
    {
      if (highOffset[axis] <= (lowOffset[axis] + 1))
      {
        newOffset[axis] = lowOffset[axis];
      }
      else
      {
        stillWorking = true;
        newOffset[axis] = (lowOffset[axis] + highOffset[axis]) / 2;
        if (highOffset[axis] > (lowOffset[axis] + 10))
        {
          allBracketsNarrow = false;
        }
      }
    }

    setOffsets(newOffset);
    getSmoothed();
    for (int axis = 0; axis < axisCount; axis++)
    {
      if (smoothed[axis] > target[axis])
      {
        highOffset[axis] = newOffset[axis];
        highValue[axis] = smoothed[axis];
      }
      else
      {
        lowOffset[axis] = newOffset[axis];
        lowValue[axis] = smoothed[axis];
      }
    }

    if (verbose)
    {
      showProgress();
    }
  }

  setOffsets(lowOffset);
  syncConfiguredOffsetsFromMPU();
  endCalibrationSession(restartDMP);

  if (verbose)
  {
    Serial.println("INFO calibration mode=precise phase=done");
    printOffsetsWithPrefix("INFO ");
  }

  return true;
}

void clearInactiveLeds()
{
  for (int i = TOTAL_LEDS; i < MAX_TOTAL_LEDS; i++)
  {
    Strip[i] = CRGB::Black;
  }
}

void syncLogicalToPhysicalLeds()
{
  fill_solid(PhysicalStrip, MAX_TOTAL_LEDS, CRGB::Black);

  // Copy logical strip1 [0..NUM_LEDS-1] to physical strip1 base 0.
  for (int i = 0; i < NUM_LEDS; i++)
  {
    PhysicalStrip[i] = Strip[i];
  }

  // Copy logical strip2 [NUM_LEDS..2*NUM_LEDS-1] to physical strip2 base MAX_LEDS_PER_STRIP.
  for (int i = 0; i < NUM_LEDS; i++)
  {
    PhysicalStrip[MAX_LEDS_PER_STRIP + i] = Strip[NUM_LEDS + i];
  }
}

void normalizePersistentConfig()
{
  if (!isValidPatternId(currentPattern))
  {
    currentPattern = FIRST_PATTERN_ID;
  }

  currentEnabledPatternMask = sanitizeEnabledPatternMask(currentEnabledPatternMask);
  currentInvertedPatternMask = sanitizeInvertedPatternMask(currentInvertedPatternMask);

  if (!isValidBrightnessLevel(currentBrightness))
  {
    currentBrightness = MIN_BRIGHTNESS;
  }

  if (!isValidStripLength(currentStripLength))
  {
    currentStripLength = DEFAULT_LEDS_PER_STRIP;
  }

  if (!isValidMotionSmoothingSize(currentMotionSmoothingSize))
  {
    currentMotionSmoothingSize = DEFAULT_MOTION_SMOOTHING_SIZE;
  }

  if (!isValidAccelRange(currentAccelRange))
  {
    currentAccelRange = DEFAULT_ACCEL_RANGE;
  }

  if (!isValidGyroRange(currentGyroRange))
  {
    currentGyroRange = DEFAULT_GYRO_RANGE;
  }

  if (!isValidBootCalibrationMode(currentBootCalibrationMode))
  {
    currentBootCalibrationMode = DEFAULT_BOOT_CALIBRATION_MODE;
  }

  currentAutoplayEnabled = sanitizeAutoplayEnabled(currentAutoplayEnabled);
  currentAutoplayIntervalMs = sanitizeAutoplayIntervalMs(currentAutoplayIntervalMs);
  currentConfigVersion = CONFIG_VERSION_4_ALPHA;
  ensureDeviceIdentity();
  if (currentPlayMode < PLAY_MODE_MANUAL || currentPlayMode > PLAY_MODE_SYNC)
  {
    currentPlayMode = DEFAULT_PLAY_MODE;
  }
  if (currentBootMode < BOOT_MODE_LAST || currentBootMode > BOOT_MODE_SYNC)
  {
    currentBootMode = DEFAULT_BOOT_MODE;
  }
  currentSyncEnabled = sanitizeBinaryFlag(currentSyncEnabled);
  if (currentSyncGroupId < 1 || currentSyncGroupId > 255)
  {
    currentSyncGroupId = DEFAULT_SYNC_GROUP_ID;
  }
  if (currentSyncRole < SYNC_ROLE_STANDALONE || currentSyncRole > SYNC_ROLE_FOLLOWER)
  {
    currentSyncRole = DEFAULT_SYNC_ROLE;
  }
  char sanitizedMasterUid[DEVICE_UID_LENGTH + 1];
  if (strlen(currentSyncMasterUid) > 0 && !sanitizeUidString(String(currentSyncMasterUid), sanitizedMasterUid, sizeof(sanitizedMasterUid)))
  {
    currentSyncMasterUid[0] = '\0';
  }
  else if (strlen(currentSyncMasterUid) > 0)
  {
    copyCString(currentSyncMasterUid, sizeof(currentSyncMasterUid), sanitizedMasterUid);
  }
  if (currentSyncLossBehavior < SYNC_LOSS_CONTINUE_LOCAL || currentSyncLossBehavior > SYNC_LOSS_WARNING_ONLY)
  {
    currentSyncLossBehavior = DEFAULT_SYNC_LOSS_BEHAVIOR;
  }
  currentWirelessEnabled = sanitizeBinaryFlag(currentWirelessEnabled);
  if (currentWirelessProfile < WIRELESS_PROFILE_LONG_RANGE || currentWirelessProfile > WIRELESS_PROFILE_FAST_SYNC)
  {
    currentWirelessProfile = DEFAULT_WIRELESS_PROFILE;
  }
}

bool isCurrentConfigSane()
{
  char scratchName[DEVICE_NAME_LENGTH + 1];
  char scratchUid[DEVICE_UID_LENGTH + 1];

  return currentConfigVersion == CONFIG_VERSION_4_ALPHA &&
      isValidPatternId(currentPattern) &&
      isValidBrightnessLevel(currentBrightness) &&
      isValidStripLength(currentStripLength) &&
      isValidMotionSmoothingSize(currentMotionSmoothingSize) &&
      isValidAccelRange(currentAccelRange) &&
      isValidGyroRange(currentGyroRange) &&
      isValidBootCalibrationMode(currentBootCalibrationMode) &&
      currentAutoplayEnabled >= 0 && currentAutoplayEnabled <= 1 &&
      currentAutoplayIntervalMs >= MIN_AUTOPLAY_INTERVAL_MS &&
      currentAutoplayIntervalMs <= MAX_AUTOPLAY_INTERVAL_MS &&
      (currentEnabledPatternMask & ~ALL_ENABLED_PATTERN_MASK) == 0 &&
      (currentEnabledPatternMask & ALL_ENABLED_PATTERN_MASK) != 0 &&
      (currentInvertedPatternMask & ~ALL_INVERTED_PATTERN_MASK) == 0 &&
      isValidDeviceUid(currentDeviceUid) &&
      sanitizeDeviceName(String(currentDeviceName), scratchName, sizeof(scratchName)) &&
      currentPlayMode >= PLAY_MODE_MANUAL && currentPlayMode <= PLAY_MODE_SYNC &&
      currentBootMode >= BOOT_MODE_LAST && currentBootMode <= BOOT_MODE_SYNC &&
      currentSyncEnabled >= 0 && currentSyncEnabled <= 1 &&
      currentSyncGroupId >= 1 && currentSyncGroupId <= 255 &&
      currentSyncRole >= SYNC_ROLE_STANDALONE && currentSyncRole <= SYNC_ROLE_FOLLOWER &&
      (strlen(currentSyncMasterUid) == 0 || sanitizeUidString(String(currentSyncMasterUid), scratchUid, sizeof(scratchUid))) &&
      currentSyncLossBehavior >= SYNC_LOSS_CONTINUE_LOCAL &&
      currentSyncLossBehavior <= SYNC_LOSS_WARNING_ONLY &&
      currentWirelessEnabled >= 0 && currentWirelessEnabled <= 1 &&
      currentWirelessProfile >= WIRELESS_PROFILE_LONG_RANGE &&
      currentWirelessProfile <= WIRELESS_PROFILE_FAST_SYNC;
}

bool isValidPatternId(int value)
{
  return value >= FIRST_PATTERN_ID && value <= LAST_PATTERN_ID;
}

// Clamp the bitmask to the known pattern range and guarantee at least one enabled pattern.
uint32_t sanitizeEnabledPatternMask(uint32_t mask)
{
  mask &= ALL_ENABLED_PATTERN_MASK;
  if (mask == 0)
  {
    return ALL_ENABLED_PATTERN_MASK;
  }
  return mask;
}

uint32_t sanitizeInvertedPatternMask(uint32_t mask)
{
  return mask & ALL_INVERTED_PATTERN_MASK;
}

bool isPatternEnabled(uint8_t patternId)
{
  if (!isValidPatternId(patternId))
  {
    return false;
  }
  const uint8_t bitIndex = (uint8_t)(patternId - FIRST_PATTERN_ID);
  return (currentEnabledPatternMask & (1ul << bitIndex)) != 0;
}

bool isPatternInverted(uint8_t patternId)
{
  if (!isValidPatternId(patternId))
  {
    return false;
  }
  const uint8_t bitIndex = (uint8_t)(patternId - FIRST_PATTERN_ID);
  return (currentInvertedPatternMask & (1ul << bitIndex)) != 0;
}

int getPatternDirectionFactor(uint8_t patternId)
{
  return isPatternInverted(patternId) ? -1 : 1;
}

bool setPatternEnabled(uint8_t patternId, bool enabled)
{
  if (!isValidPatternId(patternId))
  {
    return false;
  }

  const uint32_t bit = (uint32_t)(1ul << (patternId - FIRST_PATTERN_ID));
  uint32_t nextMask = currentEnabledPatternMask;
  if (enabled)
  {
    nextMask |= bit;
  }
  else
  {
    nextMask &= (uint32_t)~bit;
    if (nextMask == 0)
    {
      return false;
    }
  }

  currentEnabledPatternMask = sanitizeEnabledPatternMask(nextMask);
  return true;
}

// Parse a comma-separated pattern list like "1,3,7" into a bitmask.
bool parsePatternListMask(String valueText, uint32_t* maskOut)
{
  if (maskOut == NULL)
  {
    return false;
  }

  valueText.trim();
  if (valueText.length() == 0)
  {
    return false;
  }

  uint32_t mask = 0;
  int start = 0;
  while (start < valueText.length())
  {
    int comma = valueText.indexOf(',', start);
    String token = (comma >= 0) ? valueText.substring(start, comma) : valueText.substring(start);
    token.trim();
    if (token.length() == 0)
    {
      return false;
    }

    int patternId = 0;
    if (!parseIntValue(token, &patternId) || !isValidPatternId(patternId))
    {
      return false;
    }

    mask |= (uint32_t)(1ul << (patternId - FIRST_PATTERN_ID));
    if (comma < 0)
    {
      break;
    }
    start = comma + 1;
  }

  *maskOut = mask;
  return mask != 0;
}

bool updateEnabledPatternsFromMask(uint32_t mask, bool enabled)
{
  mask &= ALL_ENABLED_PATTERN_MASK;
  if (mask == 0)
  {
    return false;
  }

  uint32_t nextMask = currentEnabledPatternMask;
  if (enabled)
  {
    nextMask |= mask;
  }
  else
  {
    nextMask &= (uint32_t)~mask;
    if (nextMask == 0)
    {
      return false;
    }
  }

  currentEnabledPatternMask = sanitizeEnabledPatternMask(nextMask);
  return true;
}

bool updateInvertedPatternsFromMask(uint32_t mask, bool inverted)
{
  mask &= ALL_INVERTED_PATTERN_MASK;
  if (mask == 0)
  {
    return false;
  }

  if (inverted)
  {
    currentInvertedPatternMask |= mask;
  }
  else
  {
    currentInvertedPatternMask &= (uint32_t)~mask;
  }

  currentInvertedPatternMask = sanitizeInvertedPatternMask(currentInvertedPatternMask);
  return true;
}

// Find the next enabled pattern used for button-based cycling.
uint8_t getNextEnabledPattern(uint8_t currentId)
{
  const uint8_t startId = isValidPatternId(currentId) ? currentId : FIRST_PATTERN_ID;
  for (uint8_t offset = 1; offset <= PATTERN_COUNT; offset++)
  {
    uint8_t candidate = (uint8_t)(FIRST_PATTERN_ID + ((startId - FIRST_PATTERN_ID + offset) % PATTERN_COUNT));
    if (isPatternEnabled(candidate))
    {
      return candidate;
    }
  }
  return startId;
}

bool batteryViewTimedOut()
{
  // Battery view times out only while running on battery, not while USB charging is active.
  return batteryViewActive && !UsbConnected && (millis() - batteryViewLastInteractionMs >= BATTERY_VIEW_TIMEOUT_MS);
}

bool chargingUsbDisconnected()
{
  // Charging view ends as soon as USB-only power disappears.
  return !UsbConnected;
}

int readBatteryRawValue()
{
  return analogRead(PIN_BATTERY_ADC);
}

float convertBatteryRawToVoltage(int rawValue)
{
  return rawValue * 3.0f * 3.3f / 4096.0f;
}

bool isUsbPowered()
{
  return digitalRead(PIN_USB_SENSE) == 1;
}

void updateBatteryMeasurement(bool force)
{
  const unsigned long now = millis();
  const bool noSamplesYet = batteryVoltageSampleCount == 0;
  if (!force && !noSamplesYet && (now - lastBatterySampleMs) < BATTERY_SAMPLE_INTERVAL_MS)
  {
    return;
  }

  lastBatterySampleMs = now;
  RawVoltage = readBatteryRawValue();
  const float measuredVoltage = convertBatteryRawToVoltage(RawVoltage);

  if (batteryVoltageSampleCount < BATTERY_SAMPLE_WINDOW)
  {
    batteryVoltageSamples[batteryVoltageSampleIndex] = measuredVoltage;
    batteryVoltageSampleSum += measuredVoltage;
    batteryVoltageSampleCount++;
  }
  else
  {
    batteryVoltageSampleSum -= batteryVoltageSamples[batteryVoltageSampleIndex];
    batteryVoltageSamples[batteryVoltageSampleIndex] = measuredVoltage;
    batteryVoltageSampleSum += measuredVoltage;
  }
  batteryVoltageSampleIndex = (batteryVoltageSampleIndex + 1) % BATTERY_SAMPLE_WINDOW;

  const float averagedVoltage = batteryVoltageSampleSum / batteryVoltageSampleCount;
  const BatteryMeasurement measurement = batteryMeasurementFromAverage(averagedVoltage, Voltage, noSamplesYet);
  Voltage = measurement.displayVoltage;

  BatteryPercent = estimateBatteryPercent(Voltage);
  const BatteryState previousState = currentBatteryState;
  currentBatteryState = batteryStateTracker.update(measurement.protectionVoltage, isUsbPowered(), now);
  if (batteryStateCapsBrightness(previousState) && !batteryStateCapsBrightness(currentBatteryState))
  {
    applyEffectiveBrightness();
  }
}

void applyEffectiveBrightness()
{
  const int effectiveBrightness = batteryStateCapsBrightness(currentBatteryState) ? MIN_BRIGHTNESS : BRIGHTNESS;
  FastLED.setBrightness(effectiveBrightness);
}

void applyBatteryBrightnessLimit()
{
  if (batteryStateCapsBrightness(currentBatteryState) && FastLED.getBrightness() > MIN_BRIGHTNESS)
  {
    FastLED.setBrightness(MIN_BRIGHTNESS);
  }
}

void renderBatteryBar(int batteryBarMax)
{
  const uint8_t bars = batteryBarsForPercent(BatteryPercent);
  const int clampedMax = min(5, batteryBarMax);
  if (bars == 0)
  {
    fill_solid(Strip, min(1, clampedMax), blink ? CRGB::Red : CRGB::Black);
    return;
  }

  CRGB color = CRGB::Yellow;
  if (bars >= 5)
  {
    color = CRGB::Blue;
  }
  else if (bars >= 3)
  {
    color = CRGB::Green;
  }

  fill_solid(Strip, min((int)bars, clampedMax), color);
}

void handleBatteryCutoff()
{
  if (!batteryStateCutsOff(currentBatteryState))
  {
    if (lowPowerCutoffActive && isUsbPowered())
    {
      lowPowerCutoffActive = false;
      lowPowerCutoffSaved = false;
      rm2BleRestoreGattAdvertising();
      applyEffectiveBrightness();
    }
    return;
  }

  if (!lowPowerCutoffActive)
  {
    lowPowerCutoffActive = true;
    syncBeaconRadioStop();
    rm2BleStopAdvertising();
    fill_solid(Strip, TOTAL_LEDS, CRGB::Black);
    fill_solid(PhysicalStrip, MAX_TOTAL_LEDS, CRGB::Black);
    FastLED.setBrightness(0);
    FastLED.show();
  }

  if (!lowPowerCutoffSaved)
  {
    saveConfigToEEPROM(false);
    lowPowerCutoffSaved = true;
  }
}

void printBatteryStatus()
{
  updateBatteryMeasurement(false);
  const int usbSenseRaw = digitalRead(PIN_USB_SENSE);

  Serial.print("OK battery_raw=");
  Serial.print(RawVoltage);
  Serial.print(" battery_voltage=");
  Serial.print(Voltage, 3);
  Serial.print(" battery_percent=");
  Serial.print(BatteryPercent);
  Serial.print(" battery_state=");
  Serial.print(batteryStateName(currentBatteryState));
  Serial.print(" usb_power_raw=");
  Serial.print(usbSenseRaw);
  Serial.print(" serial_session_active=");
  Serial.println(SerialSessionActive ? 1 : 0);
}

void printSensorStatus()
{
  const bool mpuConnected = (mpu.testConnection() == true);
  const int activeAccelRange = accelRegisterValueToRange(mpu.getFullScaleAccelRange());
  const int activeGyroRange = gyroRegisterValueToRange(mpu.getFullScaleGyroRange());
  const int usbSenseRaw = digitalRead(PIN_USB_SENSE);

  Serial.print("OK mpu_connected=");
  Serial.print(mpuConnected ? 1 : 0);
  Serial.print(" dmp_ready=");
  Serial.print(DMPReady ? 1 : 0);
  Serial.print(" dev_status=");
  Serial.print(devStatus);
  Serial.print(" packet_size=");
  Serial.print(packetSize);
  Serial.print(" int_status=");
  Serial.print(MPUIntStatus);
  Serial.print(" smoothing_config=");
  Serial.print(currentMotionSmoothingSize);
  Serial.print(" smoothing_active=");
  Serial.print(activeMotionSmoothingSize);
  Serial.print(" accel_range_config=");
  Serial.print(currentAccelRange);
  Serial.print(" accel_range_active=");
  Serial.print(activeAccelRange);
  Serial.print(" gyro_range_config=");
  Serial.print(currentGyroRange);
  Serial.print(" gyro_range_active=");
  Serial.print(activeGyroRange);
  Serial.print(" usb_power_raw=");
  Serial.print(usbSenseRaw);
  Serial.print(" serial_session_active=");
  Serial.println(SerialSessionActive ? 1 : 0);
}

void printTimingStatus()
{
  uint32_t avgLoopDurationUs = 0;
  uint32_t avgWorkDurationUs = 0;
  if (loopTimingSamples > 0)
  {
    avgLoopDurationUs = (uint32_t)(totalLoopDurationUs / loopTimingSamples);
    avgWorkDurationUs = (uint32_t)(totalWorkDurationUs / loopTimingSamples);
  }

  Serial.print("OK fps=");
  Serial.print(FastLED.getFPS());
  Serial.print(" last_loop_us=");
  Serial.print(lastLoopDurationUs);
  Serial.print(" avg_loop_us=");
  Serial.print(avgLoopDurationUs);
  Serial.print(" max_loop_us=");
  Serial.print(maxLoopDurationUs);
  Serial.print(" last_work_us=");
  Serial.print(lastWorkDurationUs);
  Serial.print(" avg_work_us=");
  Serial.print(avgWorkDurationUs);
  Serial.print(" max_work_us=");
  Serial.print(maxWorkDurationUs);
  Serial.print(" frame_budget_us=");
  Serial.print(1000000UL / FRAMES_PER_SECOND);
  Serial.print(" samples=");
  Serial.println(loopTimingSamples);
}

void resetTimingStats()
{
  lastLoopDurationUs = 0;
  maxLoopDurationUs = 0;
  lastWorkDurationUs = 0;
  maxWorkDurationUs = 0;
  totalLoopDurationUs = 0;
  totalWorkDurationUs = 0;
  loopTimingSamples = 0;
}

void markCurrentConfigSaved()
{
  lastSavedPattern = currentPattern;
  lastSavedBrightness = currentBrightness;
  lastSavedStripLength = currentStripLength;
  lastSavedMotionSmoothingSize = currentMotionSmoothingSize;
  lastSavedAccelRange = currentAccelRange;
  lastSavedGyroRange = currentGyroRange;
  lastSavedBootCalibrationMode = currentBootCalibrationMode;
  lastSavedXAccelOffset = currentXAccelOffset;
  lastSavedYAccelOffset = currentYAccelOffset;
  lastSavedZAccelOffset = currentZAccelOffset;
  lastSavedXGyroOffset = currentXGyroOffset;
  lastSavedYGyroOffset = currentYGyroOffset;
  lastSavedZGyroOffset = currentZGyroOffset;
  lastSavedEnabledPatternMask = currentEnabledPatternMask;
  lastSavedInvertedPatternMask = currentInvertedPatternMask;
  lastSavedAutoplayEnabled = currentAutoplayEnabled;
  lastSavedAutoplayIntervalMs = currentAutoplayIntervalMs;
  lastSavedConfigVersion = currentConfigVersion;
  copyCString(lastSavedDeviceUid, sizeof(lastSavedDeviceUid), currentDeviceUid);
  copyCString(lastSavedDeviceName, sizeof(lastSavedDeviceName), currentDeviceName);
  lastSavedPlayMode = currentPlayMode;
  lastSavedBootMode = currentBootMode;
  lastSavedSyncEnabled = currentSyncEnabled;
  lastSavedSyncGroupId = currentSyncGroupId;
  lastSavedSyncRole = currentSyncRole;
  copyCString(lastSavedSyncMasterUid, sizeof(lastSavedSyncMasterUid), currentSyncMasterUid);
  lastSavedSyncLossBehavior = currentSyncLossBehavior;
  lastSavedWirelessEnabled = currentWirelessEnabled;
  lastSavedWirelessProfile = currentWirelessProfile;
}

bool hasUnsavedConfigChanges()
{
  return currentPattern != lastSavedPattern ||
      currentBrightness != lastSavedBrightness ||
      currentStripLength != lastSavedStripLength ||
      currentMotionSmoothingSize != lastSavedMotionSmoothingSize ||
      currentAccelRange != lastSavedAccelRange ||
      currentGyroRange != lastSavedGyroRange ||
      currentBootCalibrationMode != lastSavedBootCalibrationMode ||
      currentXAccelOffset != lastSavedXAccelOffset ||
      currentYAccelOffset != lastSavedYAccelOffset ||
      currentZAccelOffset != lastSavedZAccelOffset ||
      currentXGyroOffset != lastSavedXGyroOffset ||
      currentYGyroOffset != lastSavedYGyroOffset ||
      currentZGyroOffset != lastSavedZGyroOffset ||
      currentEnabledPatternMask != lastSavedEnabledPatternMask ||
      currentInvertedPatternMask != lastSavedInvertedPatternMask ||
      currentAutoplayEnabled != lastSavedAutoplayEnabled ||
      currentAutoplayIntervalMs != lastSavedAutoplayIntervalMs ||
      currentConfigVersion != lastSavedConfigVersion ||
      strcmp(currentDeviceUid, lastSavedDeviceUid) != 0 ||
      strcmp(currentDeviceName, lastSavedDeviceName) != 0 ||
      currentPlayMode != lastSavedPlayMode ||
      currentBootMode != lastSavedBootMode ||
      currentSyncEnabled != lastSavedSyncEnabled ||
      currentSyncGroupId != lastSavedSyncGroupId ||
      currentSyncRole != lastSavedSyncRole ||
      strcmp(currentSyncMasterUid, lastSavedSyncMasterUid) != 0 ||
      currentSyncLossBehavior != lastSavedSyncLossBehavior ||
      currentWirelessEnabled != lastSavedWirelessEnabled ||
      currentWirelessProfile != lastSavedWirelessProfile;
}

bool saveConfigToEEPROM(bool verbose)
{
  normalizePersistentConfig();
  EEPROM.put(EEPROM_ADDR_PATTERN, currentPattern);
  EEPROM.put(EEPROM_ADDR_BRIGHTNESS, currentBrightness);
  EEPROM.put(EEPROM_ADDR_STRIP_LENGTH, currentStripLength);
  EEPROM.put(EEPROM_ADDR_SMOOTHING_SIZE, currentMotionSmoothingSize);
  EEPROM.put(EEPROM_ADDR_ACCEL_RANGE, currentAccelRange);
  EEPROM.put(EEPROM_ADDR_GYRO_RANGE, currentGyroRange);
  EEPROM.put(EEPROM_ADDR_BOOT_CAL_MODE, currentBootCalibrationMode);
  EEPROM.put(EEPROM_ADDR_X_ACCEL_OFFSET, currentXAccelOffset);
  EEPROM.put(EEPROM_ADDR_Y_ACCEL_OFFSET, currentYAccelOffset);
  EEPROM.put(EEPROM_ADDR_Z_ACCEL_OFFSET, currentZAccelOffset);
  EEPROM.put(EEPROM_ADDR_X_GYRO_OFFSET, currentXGyroOffset);
  EEPROM.put(EEPROM_ADDR_Y_GYRO_OFFSET, currentYGyroOffset);
  EEPROM.put(EEPROM_ADDR_Z_GYRO_OFFSET, currentZGyroOffset);
  EEPROM.put(EEPROM_ADDR_MAGIC, EEPROM_MAGIC);
  EEPROM.put(EEPROM_ADDR_ENABLED_PATTERNS, currentEnabledPatternMask);
  EEPROM.put(EEPROM_ADDR_INVERTED_PATTERNS, currentInvertedPatternMask);
  EEPROM.put(EEPROM_ADDR_AUTOPLAY_ENABLED, currentAutoplayEnabled);
  EEPROM.put(EEPROM_ADDR_AUTOPLAY_INTERVAL_MS, currentAutoplayIntervalMs);
  writeEEPROMCString(EEPROM_ADDR_DEVICE_UID, currentDeviceUid, DEVICE_UID_LENGTH + 1);
  writeEEPROMCString(EEPROM_ADDR_DEVICE_NAME, currentDeviceName, DEVICE_NAME_LENGTH + 1);
  EEPROM.put(EEPROM_ADDR_CONFIG_VERSION, currentConfigVersion);
  EEPROM.put(EEPROM_ADDR_PLAY_MODE, currentPlayMode);
  EEPROM.put(EEPROM_ADDR_BOOT_MODE, currentBootMode);
  EEPROM.put(EEPROM_ADDR_SYNC_ENABLED, currentSyncEnabled);
  EEPROM.put(EEPROM_ADDR_SYNC_GROUP_ID, currentSyncGroupId);
  EEPROM.put(EEPROM_ADDR_SYNC_ROLE, currentSyncRole);
  writeEEPROMCString(EEPROM_ADDR_SYNC_MASTER_UID, currentSyncMasterUid, DEVICE_UID_LENGTH + 1);
  EEPROM.put(EEPROM_ADDR_SYNC_LOSS_BEHAVIOR, currentSyncLossBehavior);
  EEPROM.put(EEPROM_ADDR_WIRELESS_ENABLED, currentWirelessEnabled);
  EEPROM.put(EEPROM_ADDR_WIRELESS_PROFILE, currentWirelessProfile);

  if (verbose)
  {
    Serial.print("Pattern: ");
    Serial.println(currentPattern);
    Serial.print("Brightness: ");
    Serial.println(currentBrightness);
    Serial.print("Strip length per side: ");
    Serial.println(currentStripLength);
    Serial.print("Motion smoothing: ");
    Serial.println(currentMotionSmoothingSize);
    Serial.print("Accel range (g): ");
    Serial.println(currentAccelRange);
    Serial.print("Gyro range (dps): ");
    Serial.println(currentGyroRange);
    Serial.print("Boot calibration: ");
    Serial.println(bootCalibrationModeToString(currentBootCalibrationMode));
    Serial.print("Autoplay: ");
    Serial.println(autoplayEnabledToString());
    Serial.print("Autoplay interval (s): ");
    Serial.println(currentAutoplayIntervalMs / 1000);
    Serial.print("Firmware version: ");
    Serial.println(FIRMWARE_VERSION);
    Serial.print("Device UID: ");
    Serial.println(currentDeviceUid);
    Serial.print("Device name: ");
    Serial.println(currentDeviceName);
    Serial.print("Play mode: ");
    Serial.println(playModeToString(currentPlayMode));
    Serial.print("Sync role: ");
    Serial.println(syncRoleToString(currentSyncRole));
    Serial.print("Wireless profile: ");
    Serial.println(wirelessProfileToString(currentWirelessProfile));
    Serial.print("Enabled patterns: ");
    printEnabledPatternsList();
    Serial.println();
    Serial.print("Inverted patterns: ");
    printInvertedPatternsList();
    Serial.println();
    printOffsets();
  }

  if (EEPROM.commit())
  {
    configValid = true;
    configRepaired = false;
    markCurrentConfigSaved();
    if (verbose)
    {
      Serial.println("New values successfully saved to EEPROM.");
    }
    return true;
  }

  if (verbose)
  {
    Serial.println("ERROR: Could not save values to EEPROM.");
  }
  return false;
}

void readConfigFromEEPROM(bool verbose)
{
  int magic = 0;
  int storedConfigVersion = 0;
  bool loadedExtendedConfig = false;
  bool loadedLegacyConfig = false;
  bool migratedAudioPatterns = false;
  configValid = false;
  configRepaired = false;

  currentPattern = FIRST_PATTERN_ID;
  currentBrightness = MIN_BRIGHTNESS;
  currentStripLength = DEFAULT_LEDS_PER_STRIP;
  currentMotionSmoothingSize = DEFAULT_MOTION_SMOOTHING_SIZE;
  currentAccelRange = DEFAULT_ACCEL_RANGE;
  currentGyroRange = DEFAULT_GYRO_RANGE;
  currentBootCalibrationMode = DEFAULT_BOOT_CALIBRATION_MODE;
  currentXAccelOffset = DEFAULT_X_ACCEL_OFFSET;
  currentYAccelOffset = DEFAULT_Y_ACCEL_OFFSET;
  currentZAccelOffset = DEFAULT_Z_ACCEL_OFFSET;
  currentXGyroOffset = DEFAULT_X_GYRO_OFFSET;
  currentYGyroOffset = DEFAULT_Y_GYRO_OFFSET;
  currentZGyroOffset = DEFAULT_Z_GYRO_OFFSET;
  currentEnabledPatternMask = ALL_ENABLED_PATTERN_MASK;
  currentInvertedPatternMask = 0;
  currentAutoplayEnabled = DEFAULT_AUTOPLAY_ENABLED;
  currentAutoplayIntervalMs = DEFAULT_AUTOPLAY_INTERVAL_MS;
  currentDeviceUid[0] = '\0';
  currentDeviceName[0] = '\0';
  applyDefaultExtendedConfig(false);
  if (readStoredDeviceUid(currentDeviceUid, sizeof(currentDeviceUid)))
  {
    readStoredDeviceName(currentDeviceName, sizeof(currentDeviceName));
  }

  if (safeBootActive)
  {
    ensureDeviceIdentity();
    normalizePersistentConfig();
    configRepaired = true;
    if (verbose)
    {
      Serial.println("INFO safe_boot=1 config_ignored=1");
    }
    return;
  }

  EEPROM.get(EEPROM_ADDR_MAGIC, magic);
  if (magic == EEPROM_MAGIC)
  {
    loadedLegacyConfig = true;
    EEPROM.get(EEPROM_ADDR_PATTERN, currentPattern);
    EEPROM.get(EEPROM_ADDR_BRIGHTNESS, currentBrightness);
    EEPROM.get(EEPROM_ADDR_STRIP_LENGTH, currentStripLength);
    EEPROM.get(EEPROM_ADDR_SMOOTHING_SIZE, currentMotionSmoothingSize);
    EEPROM.get(EEPROM_ADDR_ACCEL_RANGE, currentAccelRange);
    EEPROM.get(EEPROM_ADDR_GYRO_RANGE, currentGyroRange);
    EEPROM.get(EEPROM_ADDR_BOOT_CAL_MODE, currentBootCalibrationMode);
    EEPROM.get(EEPROM_ADDR_X_ACCEL_OFFSET, currentXAccelOffset);
    EEPROM.get(EEPROM_ADDR_Y_ACCEL_OFFSET, currentYAccelOffset);
    EEPROM.get(EEPROM_ADDR_Z_ACCEL_OFFSET, currentZAccelOffset);
    EEPROM.get(EEPROM_ADDR_X_GYRO_OFFSET, currentXGyroOffset);
    EEPROM.get(EEPROM_ADDR_Y_GYRO_OFFSET, currentYGyroOffset);
    EEPROM.get(EEPROM_ADDR_Z_GYRO_OFFSET, currentZGyroOffset);
    EEPROM.get(EEPROM_ADDR_ENABLED_PATTERNS, currentEnabledPatternMask);
    EEPROM.get(EEPROM_ADDR_INVERTED_PATTERNS, currentInvertedPatternMask);
    EEPROM.get(EEPROM_ADDR_AUTOPLAY_ENABLED, currentAutoplayEnabled);
    EEPROM.get(EEPROM_ADDR_AUTOPLAY_INTERVAL_MS, currentAutoplayIntervalMs);
    EEPROM.get(EEPROM_ADDR_CONFIG_VERSION, storedConfigVersion);
    if (storedConfigVersion == CONFIG_VERSION_4_ALPHA_22_PATTERNS ||
        storedConfigVersion == CONFIG_VERSION_4_ALPHA)
    {
      loadedExtendedConfig = true;
      currentConfigVersion = storedConfigVersion;
      readEEPROMCString(EEPROM_ADDR_DEVICE_UID, currentDeviceUid, DEVICE_UID_LENGTH + 1);
      readEEPROMCString(EEPROM_ADDR_DEVICE_NAME, currentDeviceName, DEVICE_NAME_LENGTH + 1);
      EEPROM.get(EEPROM_ADDR_PLAY_MODE, currentPlayMode);
      EEPROM.get(EEPROM_ADDR_BOOT_MODE, currentBootMode);
      EEPROM.get(EEPROM_ADDR_SYNC_ENABLED, currentSyncEnabled);
      EEPROM.get(EEPROM_ADDR_SYNC_GROUP_ID, currentSyncGroupId);
      EEPROM.get(EEPROM_ADDR_SYNC_ROLE, currentSyncRole);
      readEEPROMCString(EEPROM_ADDR_SYNC_MASTER_UID, currentSyncMasterUid, DEVICE_UID_LENGTH + 1);
      EEPROM.get(EEPROM_ADDR_SYNC_LOSS_BEHAVIOR, currentSyncLossBehavior);
      EEPROM.get(EEPROM_ADDR_WIRELESS_ENABLED, currentWirelessEnabled);
      EEPROM.get(EEPROM_ADDR_WIRELESS_PROFILE, currentWirelessProfile);
    }
    if (needsAudioSyncPatternMigration(storedConfigVersion))
    {
      currentEnabledPatternMask = migrateEnabledPatternMask(storedConfigVersion, currentEnabledPatternMask);
      currentConfigVersion = CONFIG_VERSION_4_ALPHA;
      migratedAudioPatterns = true;
    }
  }
  if (!loadedExtendedConfig)
  {
    currentPlayMode = currentAutoplayEnabled ? PLAY_MODE_AUTOPLAY : PLAY_MODE_MANUAL;
  }
  const bool loadedValuesSane = loadedExtendedConfig && isCurrentConfigSane();
  normalizePersistentConfig();
  configValid = loadedValuesSane;
  configRepaired = !loadedValuesSane || migratedAudioPatterns;
  if (shouldPersistConfigRecovery(loadedValuesSane, migratedAudioPatterns))
  {
    if (!saveConfigToEEPROM(false) && verbose)
    {
      Serial.println("ERROR config_recovery_save_failed=1");
    }
    configRepaired = true;
  }
  else if (loadedValuesSane)
  {
    markCurrentConfigSaved();
  }

  if (verbose && !loadedLegacyConfig)
  {
    Serial.println("INFO config_repaired=1 reason=bad_magic defaults=1");
  }
  else if (verbose && !loadedExtendedConfig)
  {
    Serial.println("INFO config_repaired=1 reason=legacy_or_incomplete defaults_extended=1");
  }
  else if (verbose && !loadedValuesSane)
  {
    Serial.println("INFO config_repaired=1 reason=invalid_values normalized=1");
  }
  if (verbose && migratedAudioPatterns)
  {
    Serial.println("INFO config_migrated=1 enabled_audio_patterns=23,24,25,26,27");
  }

  if (verbose)
  {
    Serial.println("Current values:");
    Serial.print("Pattern: ");
    Serial.println(currentPattern);
    Serial.print("Brightness: ");
    Serial.println(currentBrightness);
    Serial.print("Strip length per side: ");
    Serial.println(currentStripLength);
    Serial.print("Motion smoothing: ");
    Serial.println(currentMotionSmoothingSize);
    Serial.print("Accel range (g): ");
    Serial.println(currentAccelRange);
    Serial.print("Gyro range (dps): ");
    Serial.println(currentGyroRange);
    Serial.print("Boot calibration: ");
    Serial.println(bootCalibrationModeToString(currentBootCalibrationMode));
    Serial.print("Autoplay: ");
    Serial.println(autoplayEnabledToString());
    Serial.print("Autoplay interval (s): ");
    Serial.println(currentAutoplayIntervalMs / 1000);
    Serial.print("Firmware version: ");
    Serial.println(FIRMWARE_VERSION);
    Serial.print("Device UID: ");
    Serial.println(currentDeviceUid);
    Serial.print("Device name: ");
    Serial.println(currentDeviceName);
    Serial.print("Play mode: ");
    Serial.println(playModeToString(currentPlayMode));
    Serial.print("Sync role: ");
    Serial.println(syncRoleToString(currentSyncRole));
    Serial.print("Wireless profile: ");
    Serial.println(wirelessProfileToString(currentWirelessProfile));
    Serial.print("Enabled patterns: ");
    printEnabledPatternsList();
    Serial.println();
    Serial.print("Inverted patterns: ");
    printInvertedPatternsList();
    Serial.println();
    printOffsets();
  }
}

void rebootController()
{
  Serial.flush();
  delay(100);
  rp2040.reboot();
}

void emitNk4Event(const char* eventName, const String& fields)
{
  if (!SerialSessionActive || usbProtocolMode != USB_PROTOCOL_MACHINE)
  {
    return;
  }
  Serial.print("NK4 event=");
  Serial.print(eventName != NULL ? eventName : "event");
  if (fields.length() > 0)
  {
    Serial.print(" ");
    Serial.print(fields);
  }
  Serial.println();
}

void showPlayModeIndicatorTest()
{
  const CRGB colors[] = {
      playModeIndicatorColor(PLAY_MODE_MANUAL, SYNC_ROLE_STANDALONE, false),
      playModeIndicatorColor(PLAY_MODE_AUTOPLAY, SYNC_ROLE_STANDALONE, false),
      playModeIndicatorColor(PLAY_MODE_SYNC, SYNC_ROLE_FOLLOWER, false),
      playModeIndicatorColor(PLAY_MODE_SYNC, SYNC_ROLE_MASTER, false),
      CRGB::Red};
  const size_t colorCount = sizeof(colors) / sizeof(colors[0]);
  applyEffectiveBrightness();
  for (size_t i = 0; i < colorCount; i++)
  {
    fill_solid(Strip, TOTAL_LEDS, colors[i]);
    clearInactiveLeds();
    syncLogicalToPhysicalLeds();
    FastLED.show();
    delay(220);
    if (i == colorCount - 1)
    {
      fill_solid(Strip, TOTAL_LEDS, CRGB::Black);
      clearInactiveLeds();
      syncLogicalToPhysicalLeds();
      FastLED.show();
      delay(120);
      fill_solid(Strip, TOTAL_LEDS, colors[i]);
      clearInactiveLeds();
      syncLogicalToPhysicalLeds();
      FastLED.show();
      delay(220);
    }
  }
  fill_solid(Strip, TOTAL_LEDS, CRGB::Black);
  clearInactiveLeds();
  syncLogicalToPhysicalLeds();
  FastLED.show();
  applyEffectiveBrightness();
}

void handleNk4Command(const NkCommand& command, IResponseWriter& writer)
{
  const String& seq = command.seq;

  if (command.command == "hello")
  {
    String fields = "proto=";
    fields += NK4_PROTOCOL_VERSION;
    fields += " fw=";
    fields += FIRMWARE_VERSION;
    fields += " name=";
    fields += currentDeviceName;
    fields += " uid=";
    fields += currentDeviceUid;
    fields += " hw=";
    fields += NIGHTKITE_HARDWARE;
    nk4WriteOk(writer, seq, fields);
    return;
  }

  if (command.command == "info")
  {
    String fields = "uid=";
    fields += currentDeviceUid;
    fields += " short_id=";
    fields += currentShortId;
    fields += " name=";
    fields += currentDeviceName;
    fields += " fw=";
    fields += FIRMWARE_VERSION;
    fields += " proto=";
    fields += NK4_PROTOCOL_VERSION;
    fields += " hw=";
    fields += NIGHTKITE_HARDWARE;
    fields += " ble_supported=";
    fields += (NIGHTKITE_BLE ? 1 : 0);
    fields += " rm2_enabled=";
    fields += rm2BleStatus().rm2Enabled ? 1 : 0;
    fields += " ble_initialized=";
    fields += rm2BleStatus().initialized ? 1 : 0;
    fields += " ble_advertising=";
    fields += rm2BleStatus().advertising ? 1 : 0;
    fields += " sync_supported=1 patterns=";
    fields += PATTERN_COUNT;
    fields += " config_valid=";
    fields += configValid ? 1 : 0;
    fields += " config_repaired=";
    fields += configRepaired ? 1 : 0;
    fields += " config_version=";
    fields += currentConfigVersion;
    fields += " safe_boot=";
    fields += safeBootActive ? 1 : 0;
    fields += " imu=";
    fields += imuReady ? 1 : 0;
    fields += " boot_stage=";
    fields += bootStage;
    nk4WriteOk(writer, seq, fields);
    return;
  }

  if (command.command == "caps")
  {
    String fields = "pattern_count=";
    fields += PATTERN_COUNT;
    fields += " brightness_levels=95,127,159,191,223,255 battery=1 imu=1 autoplay=1 sync=1 ble=";
    fields += (NIGHTKITE_BLE ? 1 : 0);
    fields += " wireless_profiles=long_range,balanced,fast_sync";
    nk4WriteOk(writer, seq, fields);
    return;
  }

  if (command.command == "status")
  {
    updateBatteryMeasurement(false);
    String fields = "pattern=";
    fields += currentPattern;
    fields += " brightness=";
    fields += currentBrightness;
    fields += " battery_percent=";
    fields += BatteryPercent;
    fields += " battery_voltage=";
    fields += String(Voltage, 3);
    fields += " battery_state=";
    fields += batteryStateName(currentBatteryState);
    fields += " usb=";
    fields += (UsbPowerRaw == 1 ? 1 : 0);
    fields += " imu=";
    fields += (imuReady ? 1 : 0);
    fields += " fps=";
    fields += FastLED.getFPS();
    fields += " play_mode=";
    fields += playModeToString(currentPlayMode);
    fields += " autoplay=";
    fields += currentAutoplayEnabled;
    fields += " sync_autoplay=";
    fields += (currentPlayMode == PLAY_MODE_SYNC && isAutoplayEnabled()) ? 1 : 0;
    fields += " master_autoplay=";
    fields += isSyncMasterAutoplayActive() ? 1 : 0;
    fields += " autoplay_next_ms=";
    fields += autoplayNextDueMs();
    fields += " sync_state=";
    fields += syncEngine.stateName();
    fields += " sync_role=";
    fields += syncRoleToString(currentSyncRole);
    fields += " sync_group=";
    fields += currentSyncGroupId;
    fields += " pattern_time_ms=";
    fields += patternClock.now();
    fields += " config_valid=";
    fields += configValid ? 1 : 0;
    fields += " config_repaired=";
    fields += configRepaired ? 1 : 0;
    fields += " config_version=";
    fields += currentConfigVersion;
    fields += " safe_boot=";
    fields += safeBootActive ? 1 : 0;
    fields += " boot_stage=";
    fields += bootStage;
    fields += " ble_initialized=";
    fields += rm2BleStatus().initialized ? 1 : 0;
    fields += " ble_advertising=";
    fields += rm2BleStatus().advertising ? 1 : 0;
    fields += " radio_mode=";
    fields += syncBeaconRadioStatus().mode;
    fields += " sync_locked=";
    fields += syncEngine.locked ? 1 : 0;
    nk4WriteOk(writer, seq, fields);
    return;
  }

  if (command.command == "patterns")
  {
    nk4WriteOk(writer, seq, buildPatternMaskFields());
    return;
  }

  if (command.command == "enable_pattern" || command.command == "disable_pattern" ||
      command.command == "invert_pattern" || command.command == "normal_pattern")
  {
    uint32_t mask = 0;
    String patternValue = nk4GetValue(command, "pattern");
    if (patternValue.length() == 0)
    {
      patternValue = nk4GetValue(command, "patterns");
    }
    if (!parsePatternListMask(patternValue, &mask))
    {
      nk4WriteError(writer, seq, "invalid_value", "bad_pattern_list");
      return;
    }

    if (command.command == "enable_pattern")
    {
      updateEnabledPatternsFromMask(mask, true);
    }
    else if (command.command == "disable_pattern")
    {
      if (!updateEnabledPatternsFromMask(mask, false))
      {
        nk4WriteError(writer, seq, "locked", "last_pattern");
        return;
      }
      if (!isPatternEnabled((uint8_t)currentPattern))
      {
        switchToPattern(getNextEnabledPattern((uint8_t)currentPattern), true, "pattern_mask");
      }
    }
    else if (command.command == "invert_pattern")
    {
      updateInvertedPatternsFromMask(mask, true);
    }
    else
    {
      updateInvertedPatternsFromMask(mask, false);
    }

    nk4WriteOk(writer, seq, buildPatternMaskFields());
    return;
  }

  if (command.command == "battery")
  {
    nk4WriteOk(writer, seq, buildBatteryFields());
    return;
  }

  if (command.command == "sensor")
  {
    nk4WriteOk(writer, seq, buildSensorFields());
    return;
  }

  if (command.command == "timing")
  {
    if (nk4GetValue(command, "reset") == "1")
    {
      resetTimingStats();
    }
    nk4WriteOk(writer, seq, buildTimingFields());
    return;
  }

  if (command.command == "offsets")
  {
    nk4WriteOk(writer, seq, buildOffsetsFields());
    return;
  }

  if (command.command == "cycle_play_mode")
  {
    cyclePlayMode();
    String fields = "play_mode=";
    fields += playModeToString(currentPlayMode);
    fields += " autoplay=";
    fields += currentAutoplayEnabled;
    nk4WriteOk(writer, seq, fields);
    return;
  }

  if (command.command == "config" || command.command == "boot")
  {
    nk4WriteOk(writer, seq, buildConfigFields());
    return;
  }

  if (command.command == "ble_status")
  {
    nk4WriteOk(writer, seq, buildWirelessFields());
    return;
  }

  if (command.command == "get")
  {
    String section = nk4GetValue(command, "section");
    section.toLowerCase();
    if (section == "sync")
    {
      nk4WriteOk(writer, seq, buildSyncFields(false));
      return;
    }
    if (section == "wireless")
    {
      nk4WriteOk(writer, seq, buildWirelessFields());
      return;
    }
    if (section == "play")
    {
      String fields = "play_mode=";
      fields += playModeToString(currentPlayMode);
      fields += " boot_mode=";
      fields += bootModeToString(currentBootMode);
      fields += " autoplay_enabled=";
      fields += currentAutoplayEnabled;
      fields += " autoplay_interval=";
      fields += currentAutoplayIntervalMs / 1000;
      fields += " sync_autoplay=";
      fields += (currentPlayMode == PLAY_MODE_SYNC && isAutoplayEnabled()) ? 1 : 0;
      fields += " master_autoplay=";
      fields += isSyncMasterAutoplayActive() ? 1 : 0;
      fields += " autoplay_next_ms=";
      fields += autoplayNextDueMs();
      fields += " pattern=";
      fields += currentPattern;
      fields += " brightness=";
      fields += currentBrightness;
      nk4WriteOk(writer, seq, fields);
      return;
    }
    if (section == "patterns")
    {
      nk4WriteOk(writer, seq, buildPatternMaskFields());
      return;
    }
    if (section == "config" || section == "show")
    {
      nk4WriteOk(writer, seq, buildConfigFields());
      return;
    }
    if (section == "battery")
    {
      nk4WriteOk(writer, seq, buildBatteryFields());
      return;
    }
    if (section == "sensor")
    {
      nk4WriteOk(writer, seq, buildSensorFields());
      return;
    }
    if (section == "timing")
    {
      nk4WriteOk(writer, seq, buildTimingFields());
      return;
    }
    if (section == "offsets")
    {
      nk4WriteOk(writer, seq, buildOffsetsFields());
      return;
    }
    nk4WriteError(writer, seq, "invalid_value", "bad_section");
    return;
  }

  if (command.command == "set")
  {
    uint32_t nextEnabledPatternMask = currentEnabledPatternMask;
    // Validate the complete request before replaying its fields with side effects enabled.
    if (!runNk4SetTransaction(command.pairCount, [&](uint8_t i, bool applyField) {
      if (i == 0)
      {
        nextEnabledPatternMask = currentEnabledPatternMask;
      }
      const String key = command.pairs[i].key;
      const String value = command.pairs[i].value;
      if (key == "name")
      {
        char sanitized[DEVICE_NAME_LENGTH + 1];
        if (!sanitizeDeviceName(value, sanitized, sizeof(sanitized)))
        {
          nk4WriteError(writer, seq, "invalid_value", "bad_name");
          return false;
        }
        if (applyField)
        {
          copyCString(currentDeviceName, sizeof(currentDeviceName), sanitized);
        }
      }
      else if (key == "uid" || key == "device_uid")
      {
        nk4WriteError(writer, seq, "locked", "uid_locked");
        return false;
      }
      else if (key == "pattern")
      {
        int valueInt = 0;
        if (!parseIntValue(value, &valueInt))
        {
          nk4WriteError(writer, seq, "invalid_value", "bad_pattern");
          return false;
        }
        if (!isValidPatternId(valueInt))
        {
          nk4WriteError(writer, seq, "range_error", "bad_pattern");
          return false;
        }
        if (applyField)
        {
          switchToPattern((uint8_t)valueInt, true, "nk4_set");
        }
      }
      else if (key == "brightness")
      {
        int valueInt = 0;
        if (!parseIntValue(value, &valueInt))
        {
          nk4WriteError(writer, seq, "invalid_value", "bad_brightness");
          return false;
        }
        if (!isValidBrightnessLevel(valueInt))
        {
          nk4WriteError(writer, seq, "range_error", "bad_brightness");
          return false;
        }
        if (applyField)
        {
          currentBrightness = valueInt;
          BRIGHTNESS = currentBrightness;
          applyEffectiveBrightness();
        }
      }
      else if (key == "strip_length")
      {
        int valueInt = 0;
        if (!parseIntValue(value, &valueInt))
        {
          nk4WriteError(writer, seq, "invalid_value", "bad_strip_length");
          return false;
        }
        if (!isValidStripLength(valueInt))
        {
          nk4WriteError(writer, seq, "range_error", "bad_strip_length");
          return false;
        }
        if (applyField)
        {
          currentStripLength = valueInt;
          applyConfiguredStripLength();
        }
      }
      else if (key == "smoothing")
      {
        int valueInt = 0;
        if (!parseIntValue(value, &valueInt))
        {
          nk4WriteError(writer, seq, "invalid_value", "bad_smoothing");
          return false;
        }
        if (!isValidMotionSmoothingSize(valueInt))
        {
          nk4WriteError(writer, seq, "range_error", "bad_smoothing");
          return false;
        }
        if (applyField)
        {
          currentMotionSmoothingSize = valueInt;
          applyConfiguredMotionSmoothing();
        }
      }
      else if (key == "accel_range")
      {
        int valueInt = 0;
        if (!parseIntValue(value, &valueInt))
        {
          nk4WriteError(writer, seq, "invalid_value", "bad_accel_range");
          return false;
        }
        if (!isValidAccelRange(valueInt))
        {
          nk4WriteError(writer, seq, "range_error", "bad_accel_range");
          return false;
        }
        if (applyField)
        {
          currentAccelRange = valueInt;
          if (imuReady)
          {
            applyConfiguredSensorRanges();
          }
        }
      }
      else if (key == "gyro_range")
      {
        int valueInt = 0;
        if (!parseIntValue(value, &valueInt))
        {
          nk4WriteError(writer, seq, "invalid_value", "bad_gyro_range");
          return false;
        }
        if (!isValidGyroRange(valueInt))
        {
          nk4WriteError(writer, seq, "range_error", "bad_gyro_range");
          return false;
        }
        if (applyField)
        {
          currentGyroRange = valueInt;
          if (imuReady)
          {
            applyConfiguredSensorRanges();
          }
        }
      }
      else if (key == "boot_calibration")
      {
        int mode = parseBootCalibrationMode(value);
        if (!isValidBootCalibrationMode(mode))
        {
          nk4WriteError(writer, seq, "invalid_value", "bad_boot_calibration");
          return false;
        }
        if (applyField)
        {
          currentBootCalibrationMode = mode;
        }
      }
      else if (key == "enabled_mask")
      {
        uint32_t mask = 0;
        if (!parseUint32Value(value, &mask))
        {
          nk4WriteError(writer, seq, "invalid_value", "bad_enabled_mask");
          return false;
        }
        mask &= ALL_ENABLED_PATTERN_MASK;
        if (mask == 0)
        {
          nk4WriteError(writer, seq, "range_error", "empty_enabled_mask");
          return false;
        }
        nextEnabledPatternMask = mask;
        if (applyField)
        {
          currentEnabledPatternMask = mask;
          if (!isPatternEnabled((uint8_t)currentPattern))
          {
            switchToPattern(getNextEnabledPattern((uint8_t)currentPattern), true, "pattern_mask");
          }
        }
      }
      else if (key == "inverted_mask")
      {
        uint32_t mask = 0;
        if (!parseUint32Value(value, &mask))
        {
          nk4WriteError(writer, seq, "invalid_value", "bad_inverted_mask");
          return false;
        }
        if (applyField)
        {
          currentInvertedPatternMask = sanitizeInvertedPatternMask(mask);
        }
      }
      else if (key == "enable_pattern")
      {
        uint32_t mask = 0;
        if (!parsePatternListMask(value, &mask))
        {
          nk4WriteError(writer, seq, "invalid_value", "bad_pattern_list");
          return false;
        }
        nextEnabledPatternMask = sanitizeEnabledPatternMask(nextEnabledPatternMask | mask);
        if (applyField)
        {
          updateEnabledPatternsFromMask(mask, true);
        }
      }
      else if (key == "disable_pattern")
      {
        uint32_t mask = 0;
        if (!parsePatternListMask(value, &mask))
        {
          nk4WriteError(writer, seq, "invalid_value", "bad_pattern_list");
          return false;
        }
        const uint32_t remainingMask = nextEnabledPatternMask & (uint32_t)~mask;
        if (remainingMask == 0)
        {
          nk4WriteError(writer, seq, "locked", "last_pattern");
          return false;
        }
        nextEnabledPatternMask = sanitizeEnabledPatternMask(remainingMask);
        if (applyField)
        {
          updateEnabledPatternsFromMask(mask, false);
          if (!isPatternEnabled((uint8_t)currentPattern))
          {
            switchToPattern(getNextEnabledPattern((uint8_t)currentPattern), true, "pattern_mask");
          }
        }
      }
      else if (key == "invert_pattern")
      {
        uint32_t mask = 0;
        if (!parsePatternListMask(value, &mask))
        {
          nk4WriteError(writer, seq, "invalid_value", "bad_pattern_list");
          return false;
        }
        if (applyField)
        {
          updateInvertedPatternsFromMask(mask, true);
        }
      }
      else if (key == "normal_pattern")
      {
        uint32_t mask = 0;
        if (!parsePatternListMask(value, &mask))
        {
          nk4WriteError(writer, seq, "invalid_value", "bad_pattern_list");
          return false;
        }
        if (applyField)
        {
          updateInvertedPatternsFromMask(mask, false);
        }
      }
      else if (key == "sync_enabled")
      {
        int flag = 0;
        if (!parseBinaryValue(value, &flag))
        {
          nk4WriteError(writer, seq, "invalid_value", "bad_sync_enabled");
          return false;
        }
        if (applyField && currentSyncEnabled != flag)
        {
          currentSyncEnabled = flag;
          syncEngine.cancel();
        }
      }
      else if (key == "sync_group" || key == "sync_group_id")
      {
        int valueInt = 0;
        if (!parseIntValue(value, &valueInt))
        {
          nk4WriteError(writer, seq, "invalid_value", "bad_sync_group");
          return false;
        }
        if (valueInt < 1 || valueInt > 255)
        {
          nk4WriteError(writer, seq, "range_error", "bad_sync_group");
          return false;
        }
        if (applyField && currentSyncGroupId != valueInt)
        {
          currentSyncGroupId = valueInt;
          syncEngine.cancel();
        }
      }
      else if (key == "sync_role")
      {
        int role = parseSyncRole(value);
        if (role < 0)
        {
          nk4WriteError(writer, seq, "invalid_value", "bad_sync_role");
          return false;
        }
        if (applyField && currentSyncRole != role)
        {
          currentSyncRole = role;
          syncEngine.cancel();
        }
      }
      else if (key == "sync_master_uid" || key == "master_uid")
      {
        if (value == "none" || value == "0")
        {
          if (applyField)
          {
            currentSyncMasterUid[0] = '\0';
          }
        }
        else
        {
          char sanitized[DEVICE_UID_LENGTH + 1];
          if (!sanitizeUidString(value, sanitized, sizeof(sanitized)))
          {
            nk4WriteError(writer, seq, "invalid_value", "bad_master_uid");
            return false;
          }
          if (applyField)
          {
            copyCString(currentSyncMasterUid, sizeof(currentSyncMasterUid), sanitized);
          }
        }
      }
      else if (key == "sync_loss_behavior")
      {
        int behavior = parseSyncLossBehavior(value);
        if (behavior < 0)
        {
          nk4WriteError(writer, seq, "invalid_value", "bad_sync_loss");
          return false;
        }
        if (applyField)
        {
          currentSyncLossBehavior = behavior;
        }
      }
      else if (key == "wireless_enabled")
      {
        int flag = 0;
        if (!parseBinaryValue(value, &flag))
        {
          nk4WriteError(writer, seq, "invalid_value", "bad_wireless_enabled");
          return false;
        }
        if (applyField && currentWirelessEnabled != flag)
        {
          currentWirelessEnabled = flag;
          syncEngine.cancel();
        }
      }
      else if (key == "wireless_profile")
      {
        int profile = parseWirelessProfile(value);
        if (profile < 0)
        {
          nk4WriteError(writer, seq, "invalid_value", "bad_wireless_profile");
          return false;
        }
        if (applyField)
        {
          currentWirelessProfile = profile;
        }
      }
      else if (key == "play_mode")
      {
        int mode = parsePlayMode(value);
        if (mode < 0)
        {
          nk4WriteError(writer, seq, "invalid_value", "bad_play_mode");
          return false;
        }
        if (applyField)
        {
          setPlayMode(mode);
        }
      }
      else if (key == "boot_mode")
      {
        int mode = parseBootMode(value);
        if (mode < 0)
        {
          nk4WriteError(writer, seq, "invalid_value", "bad_boot_mode");
          return false;
        }
        if (applyField)
        {
          currentBootMode = mode;
        }
      }
      else if (key == "autoplay" || key == "autoplay_enabled")
      {
        int flag = 0;
        if (!parseBinaryValue(value, &flag))
        {
          nk4WriteError(writer, seq, "invalid_value", "bad_autoplay");
          return false;
        }
        if (applyField)
        {
          setAutoplayEnabledFlag(flag);
        }
      }
      else if (key == "autoplay_interval")
      {
        int intervalSec = 0;
        if (!parseIntValue(value, &intervalSec))
        {
          nk4WriteError(writer, seq, "invalid_value", "bad_autoplay_interval");
          return false;
        }
        if (intervalSec < 1 || intervalSec > 300)
        {
          nk4WriteError(writer, seq, "range_error", "bad_autoplay_interval");
          return false;
        }
        if (applyField)
        {
          int intervalMs = intervalSec * 1000;
          currentAutoplayIntervalMs = sanitizeAutoplayIntervalMs(intervalMs);
          resetAutoplayTimer();
        }
      }
      else if (key == "usb_mode" || key == "protocol")
      {
        String mode = value;
        mode.toLowerCase();
        if (mode == "human" || mode == "legacy")
        {
          if (applyField)
          {
            usbProtocolMode = USB_PROTOCOL_HUMAN;
          }
        }
        else if (mode == "machine" || mode == "nk4")
        {
          if (applyField)
          {
            usbProtocolMode = USB_PROTOCOL_MACHINE;
          }
        }
        else
        {
          nk4WriteError(writer, seq, "invalid_value", "bad_usb_mode");
          return false;
        }
      }
      else
      {
        nk4WriteError(writer, seq, "invalid_key", "unknown_key");
        return false;
      }
      return true;
    }))
    {
      return;
    }
    normalizePersistentConfig();
    String fields = "updated=1 play_mode=";
    fields += playModeToString(currentPlayMode);
    fields += " pattern=";
    fields += currentPattern;
    fields += " brightness=";
    fields += currentBrightness;
    fields += " enabled_mask=";
    fields += formatHex32(currentEnabledPatternMask);
    fields += " inverted_mask=";
    fields += formatHex32(currentInvertedPatternMask);
    nk4WriteOk(writer, seq, fields);
    return;
  }

  if (command.command == "save")
  {
    if (!saveConfigToEEPROM(false))
    {
      nk4WriteError(writer, seq, "save_failed", "save_failed");
      return;
    }
    nk4WriteOk(writer, seq, "saved=1");
    return;
  }

  if (command.command == "load")
  {
    readConfigFromEEPROM(false);
    applyPersistentConfig();
    nk4WriteOk(writer, seq, "loaded=1");
    return;
  }

  if (command.command == "calibrate")
  {
    const String mode = nk4GetValue(command, "mode");
    if (!isSupportedCalibrationMode(mode.c_str()))
    {
      nk4WriteError(writer, seq, "invalid_value", "bad_calibration_mode");
      return;
    }
    if (rm2BleStatus().connected)
    {
      nk4WriteError(writer, seq, "unsupported", "usb_only");
      return;
    }
    const bool calibrated = mode == "quick" ? runQuickCalibration(false) : runPreciseCalibration(false);
    if (!calibrated)
    {
      nk4WriteError(writer, seq, "not_ready", "calibration_unavailable");
      return;
    }
    if (!saveConfigToEEPROM(false))
    {
      nk4WriteError(writer, seq, "save_failed", "save_failed");
      return;
    }
    String fields = "calibrate_finished=1 mode=";
    fields += mode;
    fields += " saved=1 ";
    fields += buildOffsetsFields();
    nk4WriteOk(writer, seq, fields);
    return;
  }

  if (command.command == "defaults")
  {
    if (nk4GetValue(command, "confirm") != "1")
    {
      nk4WriteError(writer, seq, "locked", "confirm_required");
      return;
    }
    currentPattern = FIRST_PATTERN_ID;
    currentBrightness = MIN_BRIGHTNESS;
    currentStripLength = DEFAULT_LEDS_PER_STRIP;
    currentMotionSmoothingSize = DEFAULT_MOTION_SMOOTHING_SIZE;
    currentAccelRange = DEFAULT_ACCEL_RANGE;
    currentGyroRange = DEFAULT_GYRO_RANGE;
    currentBootCalibrationMode = DEFAULT_BOOT_CALIBRATION_MODE;
    currentXAccelOffset = DEFAULT_X_ACCEL_OFFSET;
    currentYAccelOffset = DEFAULT_Y_ACCEL_OFFSET;
    currentZAccelOffset = DEFAULT_Z_ACCEL_OFFSET;
    currentXGyroOffset = DEFAULT_X_GYRO_OFFSET;
    currentYGyroOffset = DEFAULT_Y_GYRO_OFFSET;
    currentZGyroOffset = DEFAULT_Z_GYRO_OFFSET;
    currentEnabledPatternMask = ALL_ENABLED_PATTERN_MASK;
    currentInvertedPatternMask = 0;
    currentAutoplayEnabled = DEFAULT_AUTOPLAY_ENABLED;
    currentAutoplayIntervalMs = DEFAULT_AUTOPLAY_INTERVAL_MS;
    applyDefaultExtendedConfig(true);
    configValid = false;
    configRepaired = true;
    applyPersistentConfig();
    nk4WriteOk(writer, seq, "defaults=1 saved=0");
    return;
  }

  if (command.command == "reboot")
  {
    if (nk4GetValue(command, "confirm") != "1")
    {
      nk4WriteError(writer, seq, "locked", "confirm_required");
      return;
    }
    nk4WriteOk(writer, seq, "rebooting=1");
    rebootController();
    return;
  }

  if (command.command == "sync_status")
  {
    nk4WriteOk(writer, seq, buildSyncFields(true));
    return;
  }

  if (command.command == "sync_radio_status")
  {
    nk4WriteOk(writer, seq, syncBeaconRadioBuildStatusFields());
    return;
  }

  if (command.command == "audio_sync_status")
  {
    nk4WriteOk(writer, seq, syncBeaconAudioBuildStatusFields());
    return;
  }

  if (command.command == "sync_arm")
  {
    int group = currentSyncGroupId;
    int pattern = currentPattern;
    int brightness = currentBrightness;
    int startIn = 0;
    int phase = 0;
    if ((nk4HasKey(command, "group") && !parseIntValue(nk4GetValue(command, "group"), &group)) ||
        (nk4HasKey(command, "pattern") && !parseIntValue(nk4GetValue(command, "pattern"), &pattern)) ||
        (nk4HasKey(command, "brightness") && !parseIntValue(nk4GetValue(command, "brightness"), &brightness)) ||
        (nk4HasKey(command, "start_in") && !parseIntValue(nk4GetValue(command, "start_in"), &startIn)) ||
        (nk4HasKey(command, "phase") && !parseIntValue(nk4GetValue(command, "phase"), &phase)))
    {
      nk4WriteError(writer, seq, "invalid_value", "bad_sync_arm");
      return;
    }
    if (group < 1 || group > 255 || !isValidPatternId(pattern) || !isValidBrightnessLevel(brightness) || startIn < 0 || startIn > 60000 || phase < 0)
    {
      nk4WriteError(writer, seq, "range_error", "bad_sync_arm");
      return;
    }
    if (!syncEngine.arm((uint8_t)group, (uint8_t)pattern, (uint8_t)brightness, (uint32_t)startIn, (uint32_t)phase))
    {
      nk4WriteError(writer, seq, "sync_busy", "already_armed");
      return;
    }
    patternClock.armStart(syncEngine.localStartMs, (uint32_t)phase);
    String fields = "sync=armed group=";
    fields += group;
    fields += " pattern=";
    fields += pattern;
    fields += " brightness=";
    fields += brightness;
    fields += " phase=";
    fields += phase;
    fields += " local_start_ms=";
    fields += syncEngine.localStartMs;
    nk4WriteOk(writer, seq, fields);
    return;
  }

  if (command.command == "sync_cancel")
  {
    syncEngine.cancel();
    patternClock.begin();
    nk4WriteOk(writer, seq, "sync=idle");
    return;
  }

  if (command.command == "test")
  {
    String indicator = nk4GetValue(command, "indicator");
    indicator.toLowerCase();
    if (indicator != "play_modes")
    {
      nk4WriteError(writer, seq, "invalid_value", "bad_indicator");
      return;
    }
    showPlayModeIndicatorTest();
    nk4WriteOk(writer, seq, "test=play_modes");
    return;
  }

  nk4WriteError(writer, seq, "invalid_command", "unknown_cmd");
}

bool handleNk4Line(const String& line)
{
  SerialResponseWriter writer;
  return handleNk4LineWithWriter(line, writer);
}

bool handleNk4LineWithWriter(const String& line, IResponseWriter& writer)
{
  NkCommand command;
  String errorCode;
  String errorMessage;
  if (!parseNk4Line(line, &command, &errorCode, &errorMessage))
  {
    nk4WriteError(writer, command.seq, errorCode.c_str(), errorMessage.c_str());
    return false;
  }
  handleNk4Command(command, writer);
  return true;
}

void printCliHelp()
{
  Serial.println("Commands:");
  Serial.println("  help");
  Serial.println("  show");
  Serial.println("  get <pattern|brightness|strip_length|smoothing|accel_range|gyro_range|boot_calibration|autoplay|autoplay_interval|enabled_patterns|inverted_patterns>");
  Serial.println("  set pattern <1..27>");
  Serial.println("  set brightness <95|127|159|191|223|255>");
  Serial.println("  set strip_length <10..35>");
  Serial.println("  set smoothing <1..512>           (takes effect after reboot)");
  Serial.println("  set accel_range <2|4|8|16>       (takes effect after reboot)");
  Serial.println("  set gyro_range <250|500|1000|2000> (takes effect after reboot)");
  Serial.println("  set boot_calibration <off|quick>");
  Serial.println("  set autoplay <on|off>");
  Serial.println("  set autoplay_interval <1..300>");
  Serial.println("  patterns");
  Serial.println("  enable_pattern <1..27[,id...]>");
  Serial.println("  disable_pattern <1..27[,id...]>");
  Serial.println("  invert_pattern <1..27[,id...]>");
  Serial.println("  normal_pattern <1..27[,id...]>");
  Serial.println("  battery");
  Serial.println("  sensor");
  Serial.println("  timing [reset]");
  Serial.println("  offsets");
  Serial.println("  calibrate quick");
  Serial.println("  calibrate precise");
  Serial.println("  protocol machine");
  Serial.println("  reboot");
  Serial.println("  save");
  Serial.println("  load");
  Serial.println("  defaults");
}

void printCliPrompt()
{
  Serial.print("nk> ");
}

void onCliHelp(cmd* cPtr)
{
  (void)cPtr;
  printCliHelp();
}

void onCliShow(cmd* cPtr)
{
  (void)cPtr;
  printConfigSummaryWithPrefix("OK ");
}

void onCliGet(cmd* cPtr)
{
  Command cmd(cPtr);
  String key = cmd.getArgument("key").getValue();
  key.toLowerCase();

  if (key == "pattern")
  {
    Serial.print("OK pattern=");
    Serial.println(currentPattern);
    return;
  }
  if (key == "brightness")
  {
    Serial.print("OK brightness=");
    Serial.println(currentBrightness);
    return;
  }
  if (key == "strip_length")
  {
    Serial.print("OK strip_length=");
    Serial.println(currentStripLength);
    return;
  }
  if (key == "smoothing")
  {
    Serial.print("OK smoothing=");
    Serial.println(currentMotionSmoothingSize);
    return;
  }
  if (key == "accel_range")
  {
    Serial.print("OK accel_range=");
    Serial.println(currentAccelRange);
    return;
  }
  if (key == "gyro_range")
  {
    Serial.print("OK gyro_range=");
    Serial.println(currentGyroRange);
    return;
  }
  if (key == "boot_calibration")
  {
    Serial.print("OK boot_calibration=");
    Serial.println(bootCalibrationModeToString(currentBootCalibrationMode));
    return;
  }
  if (key == "autoplay")
  {
    Serial.print("OK autoplay=");
    Serial.println(autoplayEnabledToString());
    return;
  }
  if (key == "autoplay_interval")
  {
    Serial.print("OK autoplay_interval=");
    Serial.println(currentAutoplayIntervalMs / 1000);
    return;
  }
  if (key == "enabled_patterns")
  {
    Serial.print("OK enabled_patterns=");
    printEnabledPatternsList();
    Serial.println();
    return;
  }
  if (key == "inverted_patterns")
  {
    Serial.print("OK inverted_patterns=");
    printInvertedPatternsList();
    Serial.println();
    return;
  }

  Serial.println("ERR unknown key");
}

void onCliSet(cmd* cPtr)
{
  Command cmd(cPtr);
  String key = cmd.getArgument("key").getValue();
  String valueText = cmd.getArgument("value").getValue();
  int value = 0;
  const bool numericValueValid = parseIntValue(valueText, &value);
  key.toLowerCase();

  if (key == "pattern")
  {
    if (!numericValueValid || !isValidPatternId(value))
    {
      Serial.println("ERR pattern range 1..27");
      return;
    }
    switchToPattern((uint8_t)value, true, "cli");
    announcePatternChange("cli");
    Serial.print("OK pattern=");
    Serial.println(currentPattern);
    return;
  }

  if (key == "brightness")
  {
    if (!numericValueValid || !isValidBrightnessLevel(value))
    {
      Serial.println("ERR brightness must be one of 95,127,159,191,223,255");
      return;
    }

    currentBrightness = value;
    BRIGHTNESS = currentBrightness;
    applyEffectiveBrightness();
    batteryViewLastInteractionMs = millis();

    Serial.print("OK brightness=");
    Serial.println(currentBrightness);
    return;
  }
  if (key == "strip_length")
  {
    if (!numericValueValid || !isValidStripLength(value))
    {
      Serial.print("ERR strip_length range ");
      Serial.print(MIN_LEDS_PER_STRIP);
      Serial.print("..");
      Serial.println(MAX_LEDS_PER_STRIP);
      return;
    }

    currentStripLength = value;
    applyConfiguredStripLength();
    Serial.print("OK strip_length=");
    Serial.println(currentStripLength);
    return;
  }
  if (key == "smoothing")
  {
    if (!numericValueValid || !isValidMotionSmoothingSize(value))
    {
      Serial.print("ERR smoothing range ");
      Serial.print(MIN_MOTION_SMOOTHING_SIZE);
      Serial.print("..");
      Serial.println(MAX_MOTION_SMOOTHING_SIZE);
      return;
    }

    currentMotionSmoothingSize = value;
    Serial.print("OK smoothing=");
    Serial.print(currentMotionSmoothingSize);
    Serial.println(" (applies after reboot)");
    return;
  }
  if (key == "accel_range")
  {
    if (!numericValueValid || !isValidAccelRange(value))
    {
      Serial.println("ERR accel_range must be one of 2,4,8,16");
      return;
    }

    currentAccelRange = value;
    Serial.print("OK accel_range=");
    Serial.print(currentAccelRange);
    Serial.println(" (applies after reboot)");
    return;
  }
  if (key == "gyro_range")
  {
    if (!numericValueValid || !isValidGyroRange(value))
    {
      Serial.println("ERR gyro_range must be one of 250,500,1000,2000");
      return;
    }

    currentGyroRange = value;
    Serial.print("OK gyro_range=");
    Serial.print(currentGyroRange);
    Serial.println(" (applies after reboot)");
    return;
  }
  if (key == "boot_calibration")
  {
    int mode = parseBootCalibrationMode(valueText);
    if (!isValidBootCalibrationMode(mode))
    {
      Serial.println("ERR boot_calibration must be 'off' or 'quick'");
      return;
    }

    currentBootCalibrationMode = mode;
    Serial.print("OK boot_calibration=");
    Serial.println(bootCalibrationModeToString(currentBootCalibrationMode));
    return;
  }
  if (key == "autoplay")
  {
    int autoplayValue = parseOnOffValue(valueText);
    if (autoplayValue < 0)
    {
      Serial.println("ERR autoplay must be 'on' or 'off'");
      return;
    }

    setAutoplayEnabledFlag(autoplayValue);
    Serial.print("OK autoplay=");
    Serial.println(autoplayEnabledToString());
    return;
  }
  if (key == "autoplay_interval")
  {
    if (!numericValueValid || value < 1 || value > 300)
    {
      Serial.println("ERR autoplay_interval range 1..300");
      return;
    }

    int intervalMs = value * 1000;
    currentAutoplayIntervalMs = sanitizeAutoplayIntervalMs(intervalMs);
    resetAutoplayTimer();
    Serial.print("OK autoplay_interval=");
    Serial.println(currentAutoplayIntervalMs / 1000);
    return;
  }

  Serial.println("ERR unknown key");
}

void onCliSave(cmd* cPtr)
{
  (void)cPtr;
  if (!saveConfigToEEPROM(false))
  {
    Serial.println("ERR save failed");
    return;
  }
  printConfigSummaryWithPrefix("OK saved=1 ");
}

void onCliLoad(cmd* cPtr)
{
  (void)cPtr;
  readConfigFromEEPROM(false);
  applyPersistentConfig();
  printConfigSummaryWithPrefix("OK loaded=1 ");
}

void onCliDefaults(cmd* cPtr)
{
  (void)cPtr;
  currentPattern = FIRST_PATTERN_ID;
  currentBrightness = MIN_BRIGHTNESS;
  currentStripLength = DEFAULT_LEDS_PER_STRIP;
  currentMotionSmoothingSize = DEFAULT_MOTION_SMOOTHING_SIZE;
  currentAccelRange = DEFAULT_ACCEL_RANGE;
  currentGyroRange = DEFAULT_GYRO_RANGE;
  currentBootCalibrationMode = DEFAULT_BOOT_CALIBRATION_MODE;
  currentXAccelOffset = DEFAULT_X_ACCEL_OFFSET;
  currentYAccelOffset = DEFAULT_Y_ACCEL_OFFSET;
  currentZAccelOffset = DEFAULT_Z_ACCEL_OFFSET;
  currentXGyroOffset = DEFAULT_X_GYRO_OFFSET;
  currentYGyroOffset = DEFAULT_Y_GYRO_OFFSET;
  currentZGyroOffset = DEFAULT_Z_GYRO_OFFSET;
  currentEnabledPatternMask = ALL_ENABLED_PATTERN_MASK;
  currentInvertedPatternMask = 0;
  currentAutoplayEnabled = DEFAULT_AUTOPLAY_ENABLED;
  currentAutoplayIntervalMs = DEFAULT_AUTOPLAY_INTERVAL_MS;
  applyDefaultExtendedConfig(true);
  configValid = false;
  configRepaired = true;
  applyPersistentConfig();
  resetAutoplayTimer();
  batteryViewLastInteractionMs = millis();
  printConfigSummaryWithPrefix("OK defaults=1 saved=0 reboot_required=1 ");
}

void onCliBattery(cmd* cPtr)
{
  (void)cPtr;
  printBatteryStatus();
}

void onCliSensor(cmd* cPtr)
{
  (void)cPtr;
  printSensorStatus();
}

void onCliTiming(cmd* cPtr)
{
  Command cmd(cPtr);
  String action = cmd.getArgument("action").getValue();
  action.toLowerCase();
  action.trim();

  if (action == "reset")
  {
    resetTimingStats();
    Serial.println("OK timing_reset=1");
    return;
  }

  if (action.length() > 0)
  {
    Serial.println("ERR timing action must be 'reset'");
    return;
  }

  printTimingStatus();
}

void onCliOffsets(cmd* cPtr)
{
  (void)cPtr;
  printOffsetsWithPrefix("OK ");
}

void onCliCalibrate(cmd* cPtr)
{
  Command cmd(cPtr);
  String mode = cmd.getArgument("mode").getValue();
  mode.toLowerCase();

  if (mode == "quick")
  {
    Serial.println("OK calibrate_started=1 mode=quick");
    if (!runQuickCalibration(true))
    {
      return;
    }

    if (!saveConfigToEEPROM(false))
    {
      Serial.println("ERR calibrate save failed");
      return;
    }
    Serial.println("OK calibrate_finished=1 mode=quick saved=1");
    printOffsetsWithPrefix("OK ");
    return;
  }

  if (mode == "precise")
  {
    Serial.println("OK calibrate_started=1 mode=precise");
    if (!runPreciseCalibration(true))
    {
      return;
    }

    if (!saveConfigToEEPROM(false))
    {
      Serial.println("ERR calibrate save failed");
      return;
    }
    Serial.println("OK calibrate_finished=1 mode=precise saved=1");
    printOffsetsWithPrefix("OK ");
    return;
  }

  if (mode != "quick" && mode != "precise")
  {
    Serial.println("ERR calibrate mode must be 'quick' or 'precise'");
    return;
  }
}

void onCliReboot(cmd* cPtr)
{
  (void)cPtr;
  Serial.println("OK rebooting");
  rebootController();
}

void onCliProtocol(cmd* cPtr)
{
  Command cmd(cPtr);
  String mode = cmd.getArgument("mode").getValue();
  mode.toLowerCase();
  mode.trim();

  if (mode == "machine" || mode == "nk4")
  {
    Serial.println("OK protocol=machine");
    Serial.flush();
    usbProtocolMode = USB_PROTOCOL_MACHINE;
    cliPromptShown = true;
    return;
  }

  if (mode == "human" || mode == "legacy")
  {
    usbProtocolMode = USB_PROTOCOL_HUMAN;
    Serial.println("OK protocol=human");
    return;
  }

  Serial.println("ERR protocol mode must be 'machine' or 'human'");
}

void onCliPatterns(cmd* cPtr)
{
  (void)cPtr;
  // Show all patterns with their current on/off state for button cycling.
  printPatternStates();
}

void onCliEnablePattern(cmd* cPtr)
{
  Command cmd(cPtr);
  uint32_t mask = 0;
  if (!parsePatternListMask(cmd.getArgument("pattern").getValue(), &mask))
  {
    Serial.println("ERR pattern list must contain IDs in range 1..27");
    return;
  }

  // Accept single IDs and comma-separated lists in one command.
  updateEnabledPatternsFromMask(mask, true);
  Serial.print("OK enabled_patterns=");
  printEnabledPatternsList();
  Serial.println();
}

void onCliDisablePattern(cmd* cPtr)
{
  Command cmd(cPtr);
  uint32_t mask = 0;
  if (!parsePatternListMask(cmd.getArgument("pattern").getValue(), &mask))
  {
    Serial.println("ERR pattern list must contain IDs in range 1..27");
    return;
  }

  // Reject requests that would leave button cycling with zero available patterns.
  if (!updateEnabledPatternsFromMask(mask, false))
  {
    Serial.println("ERR at least one pattern must remain enabled");
    return;
  }
  if (!isPatternEnabled((uint8_t)currentPattern))
  {
    switchToPattern(getNextEnabledPattern((uint8_t)currentPattern), true, "pattern_mask");
  }

  Serial.print("OK enabled_patterns=");
  printEnabledPatternsList();
  Serial.println();
}

void onCliInvertPattern(cmd* cPtr)
{
  Command cmd(cPtr);
  uint32_t mask = 0;
  if (!parsePatternListMask(cmd.getArgument("pattern").getValue(), &mask))
  {
    Serial.println("ERR pattern list must contain IDs in range 1..27");
    return;
  }

  updateInvertedPatternsFromMask(mask, true);
  Serial.print("OK inverted_patterns=");
  printInvertedPatternsList();
  Serial.println();
}

void onCliNormalPattern(cmd* cPtr)
{
  Command cmd(cPtr);
  uint32_t mask = 0;
  if (!parsePatternListMask(cmd.getArgument("pattern").getValue(), &mask))
  {
    Serial.println("ERR pattern list must contain IDs in range 1..27");
    return;
  }

  updateInvertedPatternsFromMask(mask, false);
  Serial.print("OK inverted_patterns=");
  printInvertedPatternsList();
  Serial.println();
}

void onCliError(cmd_error* e)
{
  CommandError cmdError(e);
  Serial.print("ERR ");
  Serial.println(cmdError.toString());
}

void setupCLI()
{
  Command help = cli.addCommand("help", onCliHelp);
  (void)help;
  Command show = cli.addCommand("show", onCliShow);
  (void)show;

  Command get = cli.addCommand("get", onCliGet);
  get.addPositionalArgument("key");

  Command set = cli.addCommand("set", onCliSet);
  set.addPositionalArgument("key");
  set.addPositionalArgument("value");

  Command save = cli.addCommand("save", onCliSave);
  (void)save;
  Command load = cli.addCommand("load", onCliLoad);
  (void)load;
  Command defaults = cli.addCommand("defaults", onCliDefaults);
  (void)defaults;
  Command patterns = cli.addCommand("patterns", onCliPatterns);
  (void)patterns;
  Command enablePattern = cli.addCommand("enable_pattern", onCliEnablePattern);
  enablePattern.addPositionalArgument("pattern");
  Command disablePattern = cli.addCommand("disable_pattern", onCliDisablePattern);
  disablePattern.addPositionalArgument("pattern");
  Command invertPattern = cli.addCommand("invert_pattern", onCliInvertPattern);
  invertPattern.addPositionalArgument("pattern");
  Command normalPattern = cli.addCommand("normal_pattern", onCliNormalPattern);
  normalPattern.addPositionalArgument("pattern");
  Command battery = cli.addCommand("battery", onCliBattery);
  (void)battery;
  Command sensor = cli.addCommand("sensor", onCliSensor);
  (void)sensor;
  Command timing = cli.addCommand("timing", onCliTiming);
  timing.addPositionalArgument("action", "");
  Command offsets = cli.addCommand("offsets", onCliOffsets);
  (void)offsets;
  Command calibrate = cli.addCommand("calibrate", onCliCalibrate);
  calibrate.addPositionalArgument("mode");
  Command reboot = cli.addCommand("reboot", onCliReboot);
  (void)reboot;
  Command restart = cli.addCommand("restart", onCliReboot);
  (void)restart;
  Command protocol = cli.addCommand("protocol", onCliProtocol);
  protocol.addPositionalArgument("mode");

  cli.setOnError(onCliError);
}

void resetCliInput()
{
  cliInputBuffer = "";
  cliInputOverflow = false;
  cliLastInputMs = 0;
}

void processCliInput(bool* commandExecuted)
{
  cliInputBuffer.trim();
  if (cliInputOverflow)
  {
    if (usbProtocolMode == USB_PROTOCOL_MACHINE || cliInputBuffer.startsWith("NK4"))
    {
      NkCommand partialCommand;
      parseNk4Line(cliInputBuffer, &partialCommand, NULL, NULL);
      SerialResponseWriter writer;
      nk4WriteError(writer, partialCommand.seq, "range_error", "line_too_long");
    }
    else
    {
      Serial.println("ERR line too long");
      if (commandExecuted != NULL)
      {
        *commandExecuted = true;
      }
    }
    resetCliInput();
    return;
  }

  if (cliInputBuffer.length() > 0)
  {
    if (cliInputBuffer.startsWith("NK4"))
    {
      handleNk4Line(cliInputBuffer);
    }
    else if (usbProtocolMode == USB_PROTOCOL_MACHINE)
    {
      SerialResponseWriter writer;
      nk4WriteError(writer, "0", "invalid_command", "expected_NK4");
    }
    else
    {
      cli.parse(cliInputBuffer);
      if (commandExecuted != NULL)
      {
        *commandExecuted = true;
      }
    }
  }
  resetCliInput();
}

void handleCLI()
{
  bool commandExecuted = false;

  if (!SerialSessionActive)
  {
    resetCliInput();
    cliPromptShown = false;
    cliSessionBannerPending = false;
    cliSessionBecameActiveMs = 0;
    return;
  }

  if (!cliPromptShown && cliSessionBannerPending && (millis() - cliSessionBecameActiveMs >= CLI_CONNECT_BANNER_DELAY_MS))
  {
    if (usbProtocolMode == USB_PROTOCOL_HUMAN)
    {
      Serial.println();
      Serial.println("[NightKite CLI] USB connected. Type 'help'.");
      printCliPrompt();
      Serial.flush();
    }
    cliPromptShown = true;
    cliSessionBannerPending = false;
  }

  while (Serial.available() > 0)
  {
    char ch = (char)Serial.read();

    if (ch == '\r' || ch == '\n')
    {
      processCliInput(&commandExecuted);
      continue;
    }

    if (isPrintable((unsigned char)ch))
    {
      if (usbInputHasCapacity(cliInputBuffer.length()))
      {
        cliInputBuffer += ch;
      }
      else
      {
        cliInputOverflow = true;
      }
      cliLastInputMs = millis();
    }
  }

  // Keep no-newline convenience for Legacy monitors; NK4 always waits for framing.
  if ((cliInputBuffer.length() > 0 || cliInputOverflow) &&
      cliLastInputMs > 0 &&
      millis() - cliLastInputMs >= CLI_AUTOPARSE_TIMEOUT_MS &&
      shouldAutoParseUsbInput(usbProtocolMode == USB_PROTOCOL_MACHINE, cliInputBuffer.c_str()))
  {
    processCliInput(&commandExecuted);
  }

  if (commandExecuted && usbProtocolMode == USB_PROTOCOL_HUMAN)
  {
    printCliPrompt();
  }
}

// ============================================================================
//  STATE HANDLERS
// ============================================================================

void ChargingEntry()
{
  fill_solid(Strip, NUM_LEDS * 2, CRGB::Black);
  FastLED.setBrightness(32);
  batteryViewActive = false;
}

void ChargingRunning()
{
  updateBatteryMeasurement(false);

  currentMillis = millis();

  if (currentMillis - previousMillis >= 500)
  {
    // Save the timestamp for the next blink toggle.
    previousMillis = currentMillis;
    // Toggle the status LED state.
    if (blink == 0)
    {
      blink = 1;
    }
    else
    {
      blink = 0;
    }
  }

  fill_solid(Strip, NUM_LEDS * 2, CRGB::Black);

  int statusStart = NUM_LEDS;
  if (statusStart < TOTAL_LEDS)
  {
    Strip[statusStart] = blink ? CRGB::Red : CRGB::Black;
  }

  int batteryBarMax = min(5, NUM_LEDS);
  renderBatteryBar(batteryBarMax);
}

void ChargingExit()
{
  applyEffectiveBrightness();
}

void BatteryEntry()
{
  fill_solid(Strip, NUM_LEDS * 2, CRGB::Black);
  batteryViewActive = true;
  batteryViewLastInteractionMs = millis();
}

void BatteryRunning()
{
  updateBatteryMeasurement(false);

  currentMillis = millis();

  if (currentMillis - previousMillis >= 500)
  {
    // Save the timestamp for the next blink toggle.
    previousMillis = currentMillis;
    // Toggle the status LED state.
    if (blink == 0)
    {
      blink = 1;
    }
    else
    {
      blink = 0;
    }
  }

  fill_solid(Strip, NUM_LEDS * 2, CRGB::Black);

  int statusStart = NUM_LEDS;
  if (statusStart < TOTAL_LEDS)
  {
    Strip[statusStart] = blink ? CRGB::Blue : CRGB::Black;
  }

  // Show brightness level (6 steps from 95 to 255) on the status strip.
  int brightnessLevel = ((BRIGHTNESS - MIN_BRIGHTNESS) / 32) + 1;
  brightnessLevel = constrain(brightnessLevel, 1, 6);
  int availableBrightnessPixels = max(0, NUM_LEDS - 1);
  int brightnessPixels = min(6, availableBrightnessPixels);
  for (int i = 0; i < brightnessPixels; ++i)
  {
    Strip[statusStart + 1 + i] = (i < brightnessLevel) ? CRGB::Yellow : CRGB::Black;
  }

  int batteryBarMax = min(5, NUM_LEDS);
  renderBatteryBar(batteryBarMax);

  int autoplayStatusPixel = statusStart + 1 + brightnessPixels;
  if (autoplayStatusPixel < TOTAL_LEDS)
  {
    const bool syncError = (currentPlayMode == PLAY_MODE_SYNC &&
        (currentSyncEnabled == 0 ||
         currentSyncRole == SYNC_ROLE_STANDALONE ||
         (syncEngine.state == SyncEngine::LOST && currentSyncLossBehavior == SYNC_LOSS_WARNING_ONLY)));
    CRGB statusColor = playModeIndicatorColor(currentPlayMode, currentSyncRole, syncError);
    if (syncError && !blink)
    {
      statusColor = CRGB::Black;
    }
    Strip[autoplayStatusPixel] = statusColor;
    if ((autoplayStatusPixel + 1) < TOTAL_LEDS)
    {
      Strip[autoplayStatusPixel + 1] = statusColor;
    }
  }
}

void RunEntry()
{
  fill_solid(Strip, NUM_LEDS * 2, CRGB::Black);
  currentPattern = 1;
  batteryViewActive = false;
}

void running()
{
  fill_rainbow(Strip, NUM_LEDS * 2, (uint8_t)(patternClock.phaseMs() / 20), 7);
}

void RunEntry2()
{
  fill_solid(Strip, NUM_LEDS * 2, CRGB::Black);
  currentPattern = 2;
  batteryViewActive = false;
}

void running2()
{
  const uint8_t hue = currentYawHue();
  fill_solid(Strip, NUM_LEDS * 2, CHSV(hue, 255, 255));
}

void RunEntry3()
{
  fill_solid(Strip, NUM_LEDS * 2, CRGB::Black);
  FastLED.setBrightness(30);
  currentPattern = 3;
  batteryViewActive = false;
}

void running3()
{
#define MASTER_BRIGHTNESS 255  // Set the master brigtness value [should be greater then min_brightness value].
  uint8_t min_brightness = 30; // Set a minimum brightness level.
  accel = map(smoothedMotion(), 2000, 20000, min_brightness, MASTER_BRIGHTNESS);
  accelcon = constrain(accel, min_brightness, MASTER_BRIGHTNESS);
  const uint8_t hue = currentYawHue();
  FastLED.setBrightness(accelcon); // Set master brightness based on acceleration.

  fill_solid(Strip, NUM_LEDS * 2, CHSV(hue, 255, 255));
}

void RunExit3()
{
  fill_solid(Strip, NUM_LEDS * 2, CRGB::Black);
  applyEffectiveBrightness();
}

void RunEntry4()
{
  fill_solid(Strip, NUM_LEDS * 2, CRGB::Black);
  currentPattern = 4;
  batteryViewActive = false;
}

void running4()
{
  const uint8_t hue = currentYawHue();
  const uint32_t phaseInCycle = patternClock.phaseMs() % 1500UL; // 40 BPM.
  uint8_t pos = (uint8_t)((phaseInCycle * (uint32_t)NUM_LEDS) / 1500UL);
  if (pos >= NUM_LEDS)
  {
    pos = NUM_LEDS - 1;
  }
  if (getPatternDirectionFactor(4) < 0)
  {
    pos = (uint8_t)((NUM_LEDS - 1) - pos);
  }
  Strip[pos] = CHSV(hue, 200, 255);
  Strip[pos + NUM_LEDS] = CHSV(hue, 200, 255);

  fadeToBlackBy(Strip, NUM_LEDS * 2, 12);
}

void RunEntry5()
{
  fill_solid(Strip, NUM_LEDS * 2, CRGB::Black);
  currentPattern = 5;
  batteryViewActive = false;
}

void running5()
{
  const int direction = getPatternDirectionFactor(5);
  const uint8_t hue = currentYawHue();
  accel = map(smoothedMotion(), 2000, 20000, 100, 0);
  fade = map(accel, 20, 160, 60, 12);
  accel = constrain(accel, 0, 100);

  EVERY_N_MILLIS_I(timingObj, 1)
  {

    if (ledeffect < 0 || ledeffect >= NUM_LEDS)
    {
      ledeffect = (direction > 0) ? 0 : (NUM_LEDS - 1);
    }

    fadeToBlackBy(Strip, NUM_LEDS * 2, fade);

    Strip[ledeffect] = CHSV(hue, 255, 255);
    Strip[ledeffect + NUM_LEDS] = CHSV(hue, 255, 255);

    ledeffect += direction;

    timingObj.setPeriod(accel);
  }
}

void RunEntry6()
{
  fill_solid(Strip, NUM_LEDS * 2, CRGB::Black);
  currentPattern = 6;
  batteryViewActive = false;
}

void running6()
{
  const int direction = getPatternDirectionFactor(6);
  const int yawDegrees = currentYawDegrees();
  const uint8_t hue = orientationHueFromDegrees(yawDegrees);
  const uint8_t reverseHue = orientationHueFromDegrees(-yawDegrees);
  accel = map(smoothedMotion(), 2000, 20000, 100, 0);
  fade = map(accel, 20, 160, 60, 12);
  accel = constrain(accel, 0, 100);

  EVERY_N_MILLIS_I(timingObj, 1)
  {

    if (ledeffect < 0 || ledeffect >= NUM_LEDS || ledeffect2 < 0 || ledeffect2 >= NUM_LEDS)
    {
      ledeffect = (direction > 0) ? 0 : (NUM_LEDS - 1);
      ledeffect2 = ledeffect;
    }

    fadeToBlackBy(Strip, NUM_LEDS * 2, fade);

    Strip[ledeffect] = CHSV(hue, 255, 255);
    Strip[ledeffect2] = CHSV(reverseHue, 255, 255);
    Strip[ledeffect + NUM_LEDS] = CHSV(hue, 255, 255);
    Strip[ledeffect2 + NUM_LEDS] = CHSV(reverseHue, 255, 255);

    ledeffect2 = ledeffect;
    ledeffect += direction;

    timingObj.setPeriod(accel);
  }
}

void RunEntry7()
{
  fill_solid(Strip, NUM_LEDS * 2, CRGB::Black);
  currentPattern = 7;
  batteryViewActive = false;
}

void running7()
{
  bloodHue = currentYawHue(); // Blood color [hue from 0-255]
  const int flow = flowDirection * getPatternDirectionFactor(7);

  for (int i = 0; i < NUM_LEDS; i++)
  {
    uint8_t bloodVal = sumPulse((5 / NUM_LEDS / 2) + (NUM_LEDS / 2) * i * flow);
    Strip[i] = CHSV(bloodHue, bloodSat, bloodVal);
    Strip[i + NUM_LEDS] = CHSV(bloodHue, bloodSat, bloodVal);
  }
}

void RunEntry8()
{
  fill_solid(Strip, NUM_LEDS * 2, CRGB::Black);
  currentPattern = 8;
  batteryViewActive = false;
}

void running8()
{
  const uint8_t hue = currentYawHue();
  accel = map(smoothedMotion(), 0, 10000, 20, 160);
  fade = map(accel, 20, 160, 48, 6);
  fade = constrain(fade, 6, 48);

  uint8_t pos = map(clockSin16(80), 0, 65535, 0, NUM_LEDS - 1);
  if (getPatternDirectionFactor(8) < 0)
  {
    pos = (uint8_t)((NUM_LEDS - 1) - pos);
  }

  Strip[pos] = CHSV(hue, 200, 255);
  Strip[pos + NUM_LEDS] = CHSV(hue, 200, 255);

  fadeToBlackBy(Strip, NUM_LEDS * 2, fade);
}

void RunEntry9()
{
  fill_solid(Strip, NUM_LEDS * 2, CRGB::Black);
  currentPattern = 9;
  batteryViewActive = false;
}

void running9()
{
  float speed = ypr[0] * getPatternDirectionFactor(9);
  static float headPos[NUM_COMETS];
  static bool initialized = false;

  const uint8_t tailLength = 3;           // Symmetric glow radius around each comet head.
  const uint8_t maxBrightness = 255;

  const CRGB cometColor = CHSV(currentYawHue(), 200, 255);

  auto ringToLogicalIndex = [](int ringIndex) {
    if (ringIndex < NUM_LEDS)
    {
      return ringIndex;
    }
    return NUM_LEDS + (TOTAL_LEDS - 1 - ringIndex);
  };

  if (!initialized)
  {
    for (int i = 0; i < NUM_COMETS; i++)
    {
      headPos[i] = (float)(i * TOTAL_LEDS / NUM_COMETS);
    }
    initialized = true;
  }

  fill_solid(Strip, TOTAL_LEDS, CRGB::Black);

  for (int k = 0; k < NUM_COMETS; k++)
  {
    headPos[k] += (speed * 0.5f);

    while (headPos[k] >= TOTAL_LEDS)
    {
      headPos[k] -= TOTAL_LEDS;
    }
    while (headPos[k] < 0)
    {
      headPos[k] += TOTAL_LEDS;
    }

    int headIndex = (int)(headPos[k] + 0.5f);
    if (headIndex >= TOTAL_LEDS)
    {
      headIndex -= TOTAL_LEDS;
    }

    for (int distance = 0; distance <= tailLength; distance++)
    {
      uint8_t brightness = (distance == 0)
        ? maxBrightness
        : (uint8_t)map(distance, 1, tailLength, 170, 40);

      CRGB pixelColor = cometColor;
      pixelColor.nscale8(brightness);

      int forwardIndex = (headIndex + distance) % TOTAL_LEDS;
      Strip[ringToLogicalIndex(forwardIndex)] |= pixelColor;

      if (distance > 0)
      {
        int backwardIndex = (headIndex - distance + TOTAL_LEDS) % TOTAL_LEDS;
        Strip[ringToLogicalIndex(backwardIndex)] |= pixelColor;
      }
    }
  }
}

void RunEntry10()
{
  fill_solid(Strip, NUM_LEDS * 2, CRGB::Black);
  currentPattern = 10;
  batteryViewActive = false;
}

void running10()
{
  uint32_t m = smoothedMotion();
  uint8_t hue = map((int)(ypr[0]*180.0f/M_PI), -180, 180, 0, 255);

  if (m < 3500) {
    // Calm mode: soft breathing.
    uint8_t breath = clockSin8(10, 40, 180);
    fill_solid(Strip, TOTAL_LEDS, CHSV(hue, 255, breath));
  } else {
    // Storm mode: spark count grows with movement.
    fadeToBlackBy(Strip, TOTAL_LEDS, 40);
    uint8_t sparks = constrain(map((int)m, 3500, 20000, 1, 8), 1, 12);
    for (uint8_t s=0; s<sparks; s++) {
      int p = random16(TOTAL_LEDS);
      Strip[p] += CHSV(hue, 200, 255);
    }
  }
}

void RunEntry11()
{
  fill_solid(Strip, NUM_LEDS * 2, CRGB::Black);
  currentPattern = 11;
  batteryViewActive = false;
}

void running11()
{
  // Simple motion trigger based on world-frame acceleration magnitude.
  uint32_t mag = (uint32_t)abs(aaWorld.x) + (uint32_t)abs(aaWorld.y) + (uint32_t)abs(aaWorld.z);

  static uint16_t phase = 10000;
  static uint32_t prevMag = 0;

  int32_t jerk = (int32_t)mag - (int32_t)prevMag;
  prevMag = mag;

  if (jerk > 4000) { // threshold adjustment
    phase = 0;
  }

  fadeToBlackBy(Strip, TOTAL_LEDS, 35);

  if (phase < 2000 + 24 * NUM_LEDS) {
    phase += 20 + map((int)smoothedMotion(), 2000, 20000, 0, 20);
    uint8_t hue = map((int)(ypr[0]*180.0f/M_PI), -180, 180, 0, 255);

    // Wave from the center towards the ends on both strips.
    int center = NUM_LEDS / 2;
    for (int i = 0; i < NUM_LEDS; i++) {
      int16_t d = abs(i - center);
      int16_t k = (int16_t)d * 24 - (int16_t)phase;   // 24 = wave spacing
      k = abs(k);
      if (k < 24) {
        uint8_t bri = map(k, 0, 24, 255, 0);
        Strip[i]            += CHSV(hue, 255, bri);
        Strip[i + NUM_LEDS] += CHSV(hue, 255, bri);
      }
    }
  }
}

void RunEntry12()
{
  fill_solid(Strip, NUM_LEDS * 2, CRGB::Black);
  currentPattern = 12;
  batteryViewActive = false;
}

void running12()
{
  static float head = 0;
  static float prevYaw = 0;
  static int   dir = +1;            // stored direction (+1 or -1)

  float yaw = ypr[0];
  float dy  = yaw - prevYaw;
  // Wrap around at +/-PI.
  if (dy >  M_PI) dy -= 2*M_PI;
  if (dy < -M_PI) dy += 2*M_PI;
  prevYaw = yaw;

  // Convert yaw rate to LED speed.
  float speed = dy * (TOTAL_LEDS * 0.5f) * getPatternDirectionFactor(12);  // Tuned scale factor.

  // Direction with dead band (prevents flickering around 0)
  const float DEAD_BAND = 0.02f;           // ~adjustable
  if (speed >  DEAD_BAND) dir = +1;
  if (speed < -DEAD_BAND) dir = -1;

  head += speed;
  while (head >= TOTAL_LEDS) head -= TOTAL_LEDS;
  while (head < 0)           head += TOTAL_LEDS;

  // smooth afterglow (symmetrical)
  blur1d(Strip, TOTAL_LEDS, 64);

  uint8_t hue = map((int)(yaw*180.0f/M_PI), -180, 180, 0, 255);
  int h = (int)head;
  Strip[h] += CHSV(hue, 220, 255);

  // dim tail always behind movement direction
  for (int i = 1; i <= 6; ++i) {
    int p = (h - dir * i + TOTAL_LEDS) % TOTAL_LEDS;  // behind head
    Strip[p].nscale8(230);                             // dim tail
  }
}

void RunEntry13()
{
  fill_solid(Strip, NUM_LEDS * 2, CRGB::Black);
  currentPattern = 13;
  batteryViewActive = false;
}

void running13()
{
  static float head = 0;
  static float prevYaw = 0;
  static int dir = +1;
  static CRGB ringBuffer[MAX_TOTAL_LEDS];

  float yaw = ypr[0];
  float dy = yaw - prevYaw;
  if (dy > M_PI) dy -= 2 * M_PI;
  if (dy < -M_PI) dy += 2 * M_PI;
  prevYaw = yaw;

  float speed = dy * (TOTAL_LEDS * 0.5f) * getPatternDirectionFactor(13);
  const float DEAD_BAND = 0.02f;
  if (speed > DEAD_BAND) dir = +1;
  if (speed < -DEAD_BAND) dir = -1;

  head += speed;
  while (head >= TOTAL_LEDS) head -= TOTAL_LEDS;
  while (head < 0) head += TOTAL_LEDS;

  blur1d(ringBuffer, TOTAL_LEDS, 64);

  uint8_t hue = map((int)(yaw * 180.0f / M_PI), -180, 180, 0, 255);
  auto ringToLogicalIndex = [](int ringIndex) {
    if (ringIndex < NUM_LEDS) {
      return ringIndex;
    }
    return NUM_LEDS + (TOTAL_LEDS - 1 - ringIndex);
  };

  int h = (int)head;
  ringBuffer[h] += CHSV(hue, 220, 255);

  for (int i = 1; i <= 6; ++i) {
    int p = (h - dir * i + TOTAL_LEDS) % TOTAL_LEDS;
    ringBuffer[p].nscale8(230);
  }

  fill_solid(Strip, TOTAL_LEDS, CRGB::Black);
  for (int ringIndex = 0; ringIndex < TOTAL_LEDS; ++ringIndex) {
    Strip[ringToLogicalIndex(ringIndex)] = ringBuffer[ringIndex];
  }
}

void RunEntry14()
{
  fill_solid(Strip, NUM_LEDS * 2, CRGB::Black);
  currentPattern = 14;
  batteryViewActive = false;
}

void running14()
{
  const int direction = getPatternDirectionFactor(14);
  const int yawDegrees = currentYawDegrees();
  const uint8_t hue = orientationHueFromDegrees(yawDegrees);
  const uint8_t reverseHue = orientationHueFromDegrees(-yawDegrees);
  accel = map(smoothedMotion(), 2000, 20000, 100, 0);
  fade = map(accel, 20, 160, 60, 12);
  accel = constrain(accel, 0, 100);

  EVERY_N_MILLIS_I(timingObj, 1)
  {

    if (ledeffect <= 0 || ledeffect > (NUM_LEDS - 1))
    {
      ledeffect = (direction > 0) ? (NUM_LEDS - 1) : 0;
      ledeffect2 = ledeffect;
    }

    fadeToBlackBy(Strip, NUM_LEDS * 2, fade);

    Strip[ledeffect] = CHSV(hue, 255, 255);
    Strip[ledeffect2] = CHSV(reverseHue, 255, 255);
    Strip[ledeffect + NUM_LEDS] = CHSV(hue, 255, 255);
    Strip[ledeffect2 + NUM_LEDS] = CHSV(reverseHue, 255, 255);

    ledeffect2 = ledeffect;
    ledeffect -= direction;

    timingObj.setPeriod(accel);
  }
}

void RunEntry15()
{
  fill_solid(Strip, NUM_LEDS * 2, CRGB::Black);
  currentPattern = 15;
  batteryViewActive = false;
}

void running15()
{
  static CRGBPalette16 currentPaletteA(CRGB::Black);
  static CRGBPalette16 currentPaletteB(CRGB::Black);
  static CRGBPalette16 targetPaletteA(RainbowColors_p);
  static CRGBPalette16 targetPaletteB(CRGB::Blue);
  static CRGBPalette16 beatPalette(CRGB::Black);
  static CRGB frameBuffer[MAX_TOTAL_LEDS];
  static float prevYaw = 0.0f;
  static float filteredYawRate = 0.0f;
  static float filteredMotion = 0.0f;
  static float scrollAccumulator = 0.0f;
  static int flowDirection = 1;
  static uint8_t scrollIndex = 0;
  static unsigned long lastScrollMs = 0;

  const uint32_t motion = smoothedMotion();
  const uint8_t yawHue = map((int)(ypr[0] * 180.0f / M_PI), -180, 180, 0, 255);

  float yaw = ypr[0];
  float dy = yaw - prevYaw;
  if (dy > M_PI) dy -= 2 * M_PI;
  if (dy < -M_PI) dy += 2 * M_PI;
  prevYaw = yaw;
  filteredYawRate = (filteredYawRate * 0.90f) + (dy * 0.10f);
  filteredMotion = (filteredMotion * 0.94f) + ((float)motion * 0.06f);

  const int filteredMotionInt = (int)filteredMotion;
  const uint8_t bpm = constrain(map(filteredMotionInt, 2000, 20000, 16, 28), 14, 32);
  const uint8_t paletteSpread = constrain(map(filteredMotionInt, 2000, 20000, 22, 30), 20, 32);
  const uint8_t accentValue = constrain(map(filteredMotionInt, 2000, 20000, 110, 205), 96, 212);
  const uint8_t accentSaturation = constrain(map(filteredMotionInt, 2000, 20000, 160, 225), 150, 232);
  const float scrollStep = (float)constrain(map(filteredMotionInt, 2000, 20000, 2, 6), 2, 7) / 10.0f;

  targetPaletteA = CRGBPalette16(
      CHSV(yawHue, 220, 16),
      CHSV(yawHue + 24, 255, 80),
      CHSV(yawHue + 56, 220, 170),
      CHSV(yawHue + 88, 180, 255));

  const uint8_t accentHue = yawHue + 128;
  targetPaletteB = CRGBPalette16(
      CHSV(accentHue, accentSaturation, 8),
      CHSV(accentHue + 32, 220, accentValue / 3),
      CHSV(accentHue + 96, accentSaturation, accentValue),
      CHSV(accentHue + 140, 160, 255));

  EVERY_N_MILLISECONDS(30)
  {
    nblendPaletteTowardPalette(currentPaletteA, targetPaletteA, 2);
    nblendPaletteTowardPalette(currentPaletteB, targetPaletteB, 2);
  }

  uint8_t beat = clockBeat8(bpm);
  uint8_t mixer = ease8InOutCubic(cubicwave8(beat));
  blend(currentPaletteA, currentPaletteB, beatPalette, 16, mixer);

  const int baseDirection = getPatternDirectionFactor(15);
  if (filteredYawRate > 0.095f)
  {
    flowDirection = baseDirection;
  }
  else if (filteredYawRate < -0.095f)
  {
    flowDirection = -baseDirection;
  }

  const unsigned long now = patternClock.phaseMs();
  if (now - lastScrollMs >= 50)
  {
    lastScrollMs = now;
    scrollAccumulator += scrollStep;
    while (scrollAccumulator >= 1.0f)
    {
      scrollAccumulator -= 1.0f;
      scrollIndex = (uint8_t)(scrollIndex + (flowDirection > 0 ? 1 : 255));
    }
  }

  fill_palette(frameBuffer, TOTAL_LEDS, scrollIndex, paletteSpread, beatPalette, 255, LINEARBLEND);
  blur1d(frameBuffer, TOTAL_LEDS, 72);

  const uint8_t pulseValue = clockSin8(bpm, 84, 156);
  const uint8_t pulseWidth = constrain(map(filteredMotionInt, 2000, 20000, 2, 5), 2, 6);
  int center = TOTAL_LEDS / 2;
  for (int offset = 0; offset < pulseWidth; ++offset)
  {
    uint8_t value = qsub8(pulseValue, offset * 10);
    frameBuffer[(center + offset) % TOTAL_LEDS] += CHSV(yawHue, 170, value);
    frameBuffer[(center - offset + TOTAL_LEDS) % TOTAL_LEDS] += CHSV(yawHue, 170, value);
  }

  uint8_t framePeak = 0;
  for (int i = 0; i < TOTAL_LEDS; ++i)
  {
    framePeak = max(framePeak, max(frameBuffer[i].r, max(frameBuffer[i].g, frameBuffer[i].b)));
  }

  if (framePeak > 0)
  {
    // Normalize the mixed palette output toward the configured global brightness
    // without changing hue relationships inside the frame.
    const uint8_t targetPeak = constrain(map(BRIGHTNESS, MIN_BRIGHTNESS, MAX_BRIGHTNESS, 176, 255), 176, 255);
    const uint16_t gain256 = min<uint16_t>(512, ((uint16_t)targetPeak * 256) / framePeak);

    for (int i = 0; i < TOTAL_LEDS; ++i)
    {
      frameBuffer[i].r = min<uint16_t>(255, ((uint16_t)frameBuffer[i].r * gain256) >> 8);
      frameBuffer[i].g = min<uint16_t>(255, ((uint16_t)frameBuffer[i].g * gain256) >> 8);
      frameBuffer[i].b = min<uint16_t>(255, ((uint16_t)frameBuffer[i].b * gain256) >> 8);
    }
  }

  for (int i = 0; i < TOTAL_LEDS; ++i)
  {
    nblend(Strip[i], frameBuffer[i], 48);
  }
}

void RunEntry16()
{
  fill_solid(Strip, NUM_LEDS * 2, CRGB::Black);
  currentPattern = 16;
  batteryViewActive = false;
}

void running16()
{
  const uint32_t motion = smoothedMotion();
  const uint8_t baseHue = map((int)(ypr[0] * 180.0f / M_PI), -180, 180, 0, 255);
  const uint8_t whitecap = constrain(map((int)motion, 2000, 20000, 24, 110), 16, 120);
  const uint16_t waveStepA = 10 + constrain(map((int)motion, 2000, 20000, 0, 18), 0, 20);
  const uint16_t waveStepB = 7 + constrain(map((int)motion, 2000, 20000, 0, 12), 0, 14);
  const uint16_t waveStepC = 4 + constrain(map((int)motion, 2000, 20000, 0, 8), 0, 10);
  const uint32_t phase = patternClock.phaseMs();
  const uint16_t waveA = (uint16_t)((phase * waveStepA) / 20UL);
  const uint16_t waveB = (uint16_t)((phase * waveStepB) / 20UL);
  const uint16_t waveC = (uint16_t)((phase * waveStepC) / 20UL);

  fill_solid(Strip, TOTAL_LEDS, CRGB::Black);

  for (int i = 0; i < TOTAL_LEDS; ++i)
  {
    uint8_t a = sin8((i * 11) + (waveA >> 4));
    uint8_t b = sin8((i * 17) - (waveB >> 5));
    uint8_t c = sin8((i * 7) + (waveC >> 6));
    uint8_t bri = qadd8(scale8(a, 90), scale8(b, 80));
    bri = qadd8(bri, scale8(c, 70));
    bri = scale8(bri, 200);

    uint8_t hue = baseHue + scale8(c, 28);
    Strip[i] = CHSV(hue, 210, bri);

    if (bri > 160)
    {
      Strip[i] += CHSV(baseHue + 10, 80, qsub8(bri, 160) + whitecap);
    }
  }
}

void RunEntry17()
{
  fill_solid(Strip, NUM_LEDS * 2, CRGB::Black);
  currentPattern = 17;
  batteryViewActive = false;
}

void running17()
{
  const uint32_t motion = smoothedMotion();
  const uint8_t hue = map((int)(ypr[0] * 180.0f / M_PI), -180, 180, 0, 255);
  const uint8_t fadeAmount = constrain(map((int)motion, 2000, 20000, 20, 50), 16, 60);
  const uint8_t spawnCount = constrain(map((int)motion, 2000, 20000, 1, 6), 1, 8);

  fadeToBlackBy(Strip, TOTAL_LEDS, fadeAmount);

  for (int i = 0; i < TOTAL_LEDS; ++i)
  {
    Strip[i] += CHSV(hue, 180, 8);
  }

  for (uint8_t s = 0; s < spawnCount; ++s)
  {
    if (random8() < 90)
    {
      int p = random16(TOTAL_LEDS);
      uint8_t twinkleHue = hue + random8(96);
      Strip[p] += CHSV(twinkleHue, 140 + random8(100), 180 + random8(75));
    }
  }
}

void RunEntry18()
{
  fill_solid(Strip, NUM_LEDS * 2, CRGB::Black);
  currentPattern = 18;
  batteryViewActive = false;
}

void running18()
{
  static uint8_t heat[2][MAX_LEDS_PER_STRIP];
  const uint32_t motion = smoothedMotion();
  const uint8_t cooling = constrain(map((int)motion, 2000, 20000, 68, 38), 28, 80);
  const uint8_t sparking = constrain(map((int)motion, 2000, 20000, 38, 92), 28, 110);
  const uint8_t flareWidth = min(3, NUM_LEDS);
  const uint8_t hueBias = map((int)(ypr[0] * 180.0f / M_PI), -180, 180, 0, 48);
  const uint8_t motionLevel = constrain(map((int)motion, 2000, 20000, 0, 255), 0, 255);
  const uint8_t restReach = constrain(map((int)motion, 2000, 20000, NUM_LEDS / 3, NUM_LEDS), max(2, NUM_LEDS / 4), NUM_LEDS);

  for (int stripIndex = 0; stripIndex < 2; ++stripIndex)
  {
    for (int i = 0; i < NUM_LEDS; ++i)
    {
      heat[stripIndex][i] = qsub8(heat[stripIndex][i], random8(0, ((cooling * 10) / max(1, NUM_LEDS)) + 2));
    }

    for (int k = NUM_LEDS - 1; k >= 2; --k)
    {
      heat[stripIndex][k] = (uint8_t)((heat[stripIndex][k - 1] + heat[stripIndex][k - 2] + heat[stripIndex][k - 2]) / 3);
    }

    if (random8() < sparking)
    {
      uint8_t y = random8(flareWidth);
      heat[stripIndex][y] = qadd8(heat[stripIndex][y], random8(140, 200));
    }

    for (int j = 0; j < NUM_LEDS; ++j)
    {
      uint8_t visibleHeat = heat[stripIndex][j];
      if (j >= restReach)
      {
        const uint8_t falloff = map(j, restReach, max(restReach + 1, NUM_LEDS - 1), 220, 40);
        visibleHeat = scale8(visibleHeat, max(falloff, motionLevel));
      }

      uint8_t colorindex = scale8(visibleHeat, 232);
      CRGB c = ColorFromPalette(HeatColors_p, colorindex);
      c += CHSV(hueBias, 180, scale8(visibleHeat, 24));
      const int logicalIndex = (stripIndex * NUM_LEDS) + j;
      nblend(Strip[logicalIndex], c, 192);
    }
  }
}

void RunEntry19()
{
  fill_solid(Strip, NUM_LEDS * 2, CRGB::Black);
  currentPattern = 19;
  batteryViewActive = false;
}

void running19()
{
  static uint8_t smoothedHue = 0;
  static CRGB frameBuffer[MAX_TOTAL_LEDS];
  const uint32_t motion = smoothedMotion();
  const uint8_t targetHue = map((int)(ypr[0] * 180.0f / M_PI), -180, 180, 0, 255);
  smoothedHue = lerp8by8(smoothedHue, targetHue, 24);

  const uint8_t scale = constrain(map((int)motion, 2000, 20000, 22, 12), 10, 28);
  const uint8_t timeStepTenths = constrain(map((int)motion, 2000, 20000, 2, 10), 1, 12);
  const uint16_t noiseTime = (uint16_t)((patternClock.phaseMs() * (uint32_t)timeStepTenths) / 200UL);

  CRGBPalette16 noisePalette(
      CHSV(smoothedHue, 210, 28),
      CHSV(smoothedHue + 24, 220, 96),
      CHSV(smoothedHue + 72, 180, 180),
      CHSV(smoothedHue + 128, 150, 255));

  for (int i = 0; i < TOTAL_LEDS; ++i)
  {
    uint8_t index = inoise8(i * scale, noiseTime * 20);
    uint8_t density = inoise8((i * (scale + 3)) + 1000, noiseTime * 16);
    uint8_t bri = qadd8(56, scale8(density, 168));
    frameBuffer[i] = ColorFromPalette(noisePalette, index, bri, LINEARBLEND);
  }

  blur1d(frameBuffer, TOTAL_LEDS, 84);

  for (int i = 0; i < TOTAL_LEDS; ++i)
  {
    nblend(Strip[i], frameBuffer[i], 72);
  }
}

void RunEntry20()
{
  fill_solid(Strip, NUM_LEDS * 2, CRGB::Black);
  currentPattern = 20;
  batteryViewActive = false;
}

void running20()
{
  const uint32_t motion = smoothedMotion();
  const uint8_t hueBase = map((int)(ypr[0] * 180.0f / M_PI), -180, 180, 0, 255);
  const uint8_t sat = constrain(map((int)motion, 2000, 20000, 180, 255), 170, 255);
  const uint8_t waveSpeed = constrain(map((int)motion, 2000, 20000, 2, 8), 1, 10);
  const uint16_t phase = (uint16_t)((patternClock.phaseMs() * (uint32_t)waveSpeed) / 20UL);

  for (int i = 0; i < TOTAL_LEDS; ++i)
  {
    uint8_t hue = hueBase + sin8((i * 9) + (phase >> 1)) / 3 + sin8((i * 5) - (phase >> 2)) / 5;
    uint8_t bri = qadd8(100, scale8(sin8((i * 13) + phase), 140));
    Strip[i] = CHSV(hue, sat, bri);
  }
}

void RunEntry21()
{
  fill_solid(Strip, NUM_LEDS * 2, CRGB::Black);
  currentPattern = 21;
  batteryViewActive = false;
}

void running21()
{
  static uint32_t prevMag = 0;
  uint32_t mag = (uint32_t)abs(aaWorld.x) + (uint32_t)abs(aaWorld.y) + (uint32_t)abs(aaWorld.z);
  int32_t jerk = (int32_t)mag - (int32_t)prevMag;
  prevMag = mag;

  const uint8_t baseHue = map((int)(ypr[0] * 180.0f / M_PI), -180, 180, 0, 255);
  fadeToBlackBy(Strip, TOTAL_LEDS, 26);

  for (int i = 0; i < TOTAL_LEDS; ++i)
  {
    Strip[i] += CHSV(baseHue, 180, 4);
  }

  uint8_t burstCount = 0;
  if (jerk > 2500)
  {
    burstCount = constrain(map((int)jerk, 2500, 12000, 2, 9), 1, 12);
  }
  else if (random8() < 32)
  {
    burstCount = 1;
  }

  for (uint8_t i = 0; i < burstCount; ++i)
  {
    int p = random16(TOTAL_LEDS);
    Strip[p] += CHSV(baseHue + random8(80), 180 + random8(75), 180 + random8(75));
  }
}

void RunEntry22()
{
  fill_solid(Strip, NUM_LEDS * 2, CRGB::Black);
  currentPattern = 22;
  batteryViewActive = false;
}

void running22()
{
  static int16_t phase = 20000;
  static uint32_t prevMag = 0;
  static unsigned long lastTriggerMs = 0;
  uint32_t mag = (uint32_t)abs(aaWorld.x) + (uint32_t)abs(aaWorld.y) + (uint32_t)abs(aaWorld.z);
  int32_t jerk = (int32_t)mag - (int32_t)prevMag;
  prevMag = mag;
  const unsigned long now = millis();

  if ((jerk > 4200 && (now - lastTriggerMs) > 220) || phase > 15000)
  {
    phase = 0;
    lastTriggerMs = now;
  }

  fadeToBlackBy(Strip, TOTAL_LEDS, 28);

  if (phase <= (NUM_LEDS + 4) * 24)
  {
    phase += 7 + constrain(map((int)smoothedMotion(), 2000, 20000, 0, 14), 0, 16);
    const uint8_t hue = map((int)(ypr[0] * 180.0f / M_PI), -180, 180, 0, 255);
    const int center = NUM_LEDS / 2;

    for (int i = 0; i < NUM_LEDS; ++i)
    {
      int distance = abs(i - center);
      int wave = abs((distance * 24) - phase);
      if (wave < 48)
      {
        uint8_t bri = map(wave, 0, 48, 220, 0);
        Strip[i] += CHSV(hue, 180, bri);
        Strip[i + NUM_LEDS] += CHSV(hue, 180, bri);
      }
    }
  }
}

void RunEntry23()
{
  fill_solid(Strip, TOTAL_LEDS, CRGB::Black);
  currentPattern = 23;
  batteryViewActive = false;
}

void running23()
{
  // Audio Pulse Angle Color: beat phase drives the global pulse, energy sets
  // its body, bass adds the flash, and local yaw/pitch select the color.
  const AudioPatternFrame audio = buildAudioPatternFrame();
  const uint8_t baseHue = audioPatternLocalHue();
  const int pitchOffset = audioPatternPitchOffset();
  const uint8_t baseValue = qadd8(22, scale8(audio.energy, 128));
  uint8_t flash = scale8(audio.beatPulse, qadd8(72, scale8(audio.bass, 156)));
  if (audio.beat)
  {
    flash = qadd8(flash, 48);
  }

  for (int stripIndex = 0; stripIndex < 2; ++stripIndex)
  {
    for (int i = 0; i < NUM_LEDS; ++i)
    {
      const uint8_t spatial = sin8((uint8_t)(i * 10 + audio.phase8));
      const uint8_t value = qadd8(baseValue, scale8(flash, qadd8(176, scale8(spatial, 78))));
      const uint8_t hue = baseHue + pitchOffset + scale8(spatial, 22) + (stripIndex * 8);
      const uint8_t saturation = qsub8(245, scale8(audio.energy, 44));
      nblend(Strip[(stripIndex * NUM_LEDS) + i], CHSV(hue, saturation, value), 96);
    }
  }
}

void RunEntry24()
{
  fill_solid(Strip, TOTAL_LEDS, CRGB::Black);
  currentPattern = 24;
  batteryViewActive = false;
}

void running24()
{
  // Audio Spectrum Ribbon: one broad bass wave and two broad mid ribbons
  // travel in opposite directions. Treble only brightens their crests.
  const AudioPatternFrame audio = buildAudioPatternFrame();
  const uint8_t baseHue = audioPatternLocalHue();
  const int pitchOffset = audioPatternPitchOffset();
  const uint8_t bandWeight = qadd8(72, scale8(audio.confidence, 120));
  const uint8_t bassLevel = lerp8by8(audio.energy, audio.bass, bandWeight);
  const uint8_t midLevel = lerp8by8(audio.energy, audio.mid, bandWeight);

  for (int stripIndex = 0; stripIndex < 2; ++stripIndex)
  {
    for (int i = 0; i < NUM_LEDS; ++i)
    {
      const uint8_t position = (uint8_t)(((uint16_t)i * 255U) / max(1, NUM_LEDS - 1));
      const uint8_t directionPhase = stripIndex == 0 ? audio.phase8 : (uint8_t)(255 - audio.phase8);
      const uint8_t bassWave = sin8(position - directionPhase);
      const uint8_t midWave = sin8((uint8_t)(position * 2U) + directionPhase);
      const uint8_t broadGlow = scale8(bassWave, scale8(bassLevel, 110));
      const uint8_t ribbon = scale8(midWave, scale8(midLevel, 120));
      const uint8_t highlight = scale8(
          scale8(qsub8(midWave, 208), audio.treble),
          qadd8(80, scale8(audio.confidence, 96)));
      const uint8_t value = qadd8(
          qadd8(14, scale8(audio.energy, 70)),
          qadd8(qadd8(broadGlow, ribbon), highlight));
      const uint8_t hue = baseHue + pitchOffset + scale8(midWave, 32) + scale8(bassWave, 12);
      const uint8_t saturation = qsub8(238, scale8(highlight, 64));
      nblend(Strip[(stripIndex * NUM_LEDS) + i], CHSV(hue, saturation, value), 40);
    }
  }
}

void RunEntry25()
{
  fill_solid(Strip, TOTAL_LEDS, CRGB::Black);
  currentPattern = 25;
  batteryViewActive = false;
}

void running25()
{
  // Audio Beat Ripples: the synchronized beat phase moves rings from each
  // strip center. Bass/energy set strength and confidence sharpens the wave.
  const AudioPatternFrame audio = buildAudioPatternFrame();
  const uint8_t baseHue = audioPatternLocalHue();
  const int pitchOffset = audioPatternPitchOffset();
  const int center = NUM_LEDS / 2;
  const int maxDistance = max(1, center);
  const int ringPosition = ((int)audio.phase8 * (maxDistance + 2)) / 255;
  const int ringWidth = audio.confidence >= 128 ? 1 : 2;
  uint8_t ringStrength = qadd8(scale8(audio.energy, 132), scale8(audio.bass, 116));
  ringStrength = qadd8(ringStrength, scale8(audio.beatPulse, 96));
  if (audio.beat)
  {
    ringStrength = qadd8(ringStrength, 40);
  }

  for (int i = 0; i < NUM_LEDS; ++i)
  {
    const int distance = abs(i - center);
    const int delta = abs(distance - ringPosition);
    uint8_t ring = 0;
    if (delta <= ringWidth)
    {
      ring = (uint8_t)map(delta, 0, ringWidth + 1, ringStrength, 0);
    }
    const uint8_t softWave = scale8(sin8((uint8_t)(distance * 34 - audio.phase8)), 52);
    const uint8_t value = qadd8(qadd8(18, scale8(audio.energy, 74)), qadd8(ring, softWave));
    const uint8_t hue = baseHue + pitchOffset + (distance * 7);
    const CRGB target = CHSV(hue, qsub8(240, scale8(audio.confidence, 52)), value);
    nblend(Strip[i], target, 104);
    nblend(Strip[i + NUM_LEDS], target, 104);
  }
}

void RunEntry26()
{
  fill_solid(Strip, TOTAL_LEDS, CRGB::Black);
  currentPattern = 26;
  batteryViewActive = false;
}

void running26()
{
  // Audio Band Comets: two broad bass/mid comets scan in opposite directions.
  // Energy lights their path and treble adds a restrained shared accent.
  const AudioPatternFrame audio = buildAudioPatternFrame();
  const uint8_t baseHue = audioPatternLocalHue() + audioPatternPitchOffset();
  uint8_t phase = audio.phase8;
  if (getPatternDirectionFactor(26) < 0)
  {
    phase = 255 - phase;
  }
  const int bassHead = ((uint16_t)triwave8(phase) * (NUM_LEDS - 1)) / 255U;
  const int midHead = ((uint16_t)triwave8((uint8_t)(phase + 128)) * (NUM_LEDS - 1)) / 255U;
  const int radius = constrain(NUM_LEDS / 10, 1, 3);
  const uint8_t bandWeight = qadd8(80, scale8(audio.confidence, 128));
  const uint8_t bassLevel = lerp8by8(audio.energy, audio.bass, bandWeight);
  const uint8_t midLevel = lerp8by8(audio.energy, audio.mid, bandWeight);
  const uint8_t trebleLevel = scale8(
      audio.treble,
      qadd8(24, scale8(audio.confidence, 48)));

  for (int stripIndex = 0; stripIndex < 2; ++stripIndex)
  {
    for (int i = 0; i < NUM_LEDS; ++i)
    {
      const int bassDistance = abs(i - bassHead);
      const int midDistance = abs(i - midHead);
      const uint8_t bassShape = bassDistance > radius
          ? 0
          : (uint8_t)(255 - ((uint16_t)bassDistance * 255U) / (radius + 1));
      const uint8_t midShape = midDistance > radius
          ? 0
          : (uint8_t)(255 - ((uint16_t)midDistance * 255U) / (radius + 1));
      const uint8_t bassValue = scale8(bassShape, qadd8(scale8(bassLevel, 150), scale8(audio.energy, 36)));
      const uint8_t midValue = scale8(midShape, qadd8(scale8(midLevel, 144), scale8(audio.energy, 30)));
      const uint8_t accentValue = scale8(max(bassShape, midShape), trebleLevel);

      CRGB target = CHSV(baseHue + (stripIndex * 8), 190, qadd8(10, scale8(audio.energy, 48)));
      target += CHSV(baseHue, 238, bassValue);
      target += CHSV(baseHue + 86, 216, midValue);
      target += CHSV(baseHue + 160, 138, accentValue);
      nblend(Strip[(stripIndex * NUM_LEDS) + i], target, 48);
    }
  }
}

void RunEntry27()
{
  fill_solid(Strip, TOTAL_LEDS, CRGB::Black);
  currentPattern = 27;
  batteryViewActive = false;
}

void running27()
{
  // Audio Beat Mosaic: three to five mirrored color zones stay spatially
  // stable while band levels and beat phase move a soft brightness focus.
  const AudioPatternFrame audio = buildAudioPatternFrame();
  const uint8_t baseHue = audioPatternLocalHue();
  const int pitchOffset = audioPatternPitchOffset();
  const int zoneCount = constrain(NUM_LEDS / 5, 3, 5);
  const uint8_t bandWeight = qadd8(72, scale8(audio.confidence, 128));
  uint8_t phase = audio.phase8;
  if (getPatternDirectionFactor(27) < 0)
  {
    phase = 255 - phase;
  }

  for (int stripIndex = 0; stripIndex < 2; ++stripIndex)
  {
    for (int i = 0; i < NUM_LEDS; ++i)
    {
      const int logicalIndex = stripIndex == 0 ? i : (NUM_LEDS - 1 - i);
      const int zone = (logicalIndex * zoneCount) / NUM_LEDS;
      const uint8_t band = (uint8_t)(zone % 3);
      const uint8_t bandValue = band == 0 ? audio.bass : (band == 1 ? audio.mid : audio.treble);
      const uint8_t bandLevel = lerp8by8(audio.energy, bandValue, bandWeight);
      const uint8_t zoneWave = sin8((uint8_t)(phase + ((uint16_t)zone * 256U) / zoneCount));
      const uint8_t movement = scale8(zoneWave, qadd8(20, scale8(bandLevel, 72)));
      const uint8_t beatAccent = scale8(
          scale8(audio.beatPulse, zoneWave),
          qadd8(12, scale8(audio.bass, 44)));
      const uint8_t value = qadd8(
          qadd8(14, scale8(audio.energy, 72)),
          qadd8(scale8(bandLevel, 92), qadd8(movement, beatAccent)));
      const uint8_t hue = baseHue + pitchOffset + ((uint16_t)zone * 256U) / zoneCount + scale8(zoneWave, 10);
      const uint8_t saturation = qsub8(232, scale8(bandLevel, 28));
      nblend(Strip[(stripIndex * NUM_LEDS) + i], CHSV(hue, saturation, value), 32);
    }
  }
}

const PatternDefinition patternDefinitions[] = {
    {1, "rainbow", RunEntry, running, NULL},
    {2, "full_color", RunEntry2, running2, NULL},
    {3, "motion_brightness", RunEntry3, running3, RunExit3},
    {4, "runner_fixed", RunEntry4, running4, NULL},
    {5, "runner_reactive", RunEntry5, running5, NULL},
    {6, "runner_dual", RunEntry6, running6, NULL},
    {7, "heartbeat", RunEntry7, running7, NULL},
    {8, "ping_pong", RunEntry8, running8, NULL},
    {9, "comet_swarm", RunEntry9, running9, NULL},
    {10, "breath_storm", RunEntry10, running10, NULL},
    {11, "jerk_wave", RunEntry11, running11, NULL},
    {12, "yaw_spinner", RunEntry12, running12, NULL},
    {13, "yaw_spinner_circle", RunEntry13, running13, NULL},
    {14, "runner_dual_inverted", RunEntry14, running14, NULL},
    {15, "palette_beat_motion", RunEntry15, running15, NULL},
    {16, "pacifica_kite", RunEntry16, running16, NULL},
    {17, "twinkle_motion", RunEntry17, running17, NULL},
    {18, "fire_jet", RunEntry18, running18, NULL},
    {19, "noise_ring", RunEntry19, running19, NULL},
    {20, "pride_yaw", RunEntry20, running20, NULL},
    {21, "confetti_jerk", RunEntry21, running21, NULL},
    {22, "center_ripple", RunEntry22, running22, NULL},
    {23, "audio_pulse_angle_color", RunEntry23, running23, NULL},
    {24, "audio_spectrum_ribbon", RunEntry24, running24, NULL},
    {25, "audio_beat_ripples", RunEntry25, running25, NULL},
    {26, "audio_band_comets", RunEntry26, running26, NULL},
    {27, "audio_beat_mosaic", RunEntry27, running27, NULL},
};

static_assert(
    (sizeof(patternDefinitions) / sizeof(patternDefinitions[0])) == PATTERN_COUNT,
    "Pattern registry must cover every supported pattern ID");

// Look up the callbacks and display name for a pattern ID.
const PatternDefinition* getPatternDefinition(uint8_t patternId)
{
  for (size_t i = 0; i < (sizeof(patternDefinitions) / sizeof(patternDefinitions[0])); i++)
  {
    if (patternDefinitions[i].id == patternId)
    {
      return &patternDefinitions[i];
    }
  }
  return NULL;
}

void runPatternEntry(uint8_t patternId)
{
  // Dispatch into the selected pattern's entry function, if it has one.
  const PatternDefinition* pattern = getPatternDefinition(patternId);
  if (pattern != NULL && pattern->entry != NULL)
  {
    pattern->entry();
  }
}

void runPatternFrame(uint8_t patternId)
{
  // Dispatch one animation frame of the selected pattern.
  const PatternDefinition* pattern = getPatternDefinition(patternId);
  if (pattern != NULL && pattern->run != NULL)
  {
    pattern->run();
  }
}

void runPatternExit(uint8_t patternId)
{
  // Dispatch the selected pattern's exit function, if one is defined.
  const PatternDefinition* pattern = getPatternDefinition(patternId);
  if (pattern != NULL && pattern->exit != NULL)
  {
    pattern->exit();
  }
}

void switchToPattern(uint8_t patternId, bool activatePatternState, const char* source, uint32_t latencyMs)
{
  if (!isValidPatternId(patternId))
  {
    return;
  }

  // Pattern cycling is only "live" while we are not inside battery or charging view.
  const bool patternCurrentlyActive = !batteryViewActive && !UsbConnected;
  const uint8_t previousPattern = (uint8_t)currentPattern;

  if (patternCurrentlyActive && previousPattern == patternId && !activatePatternState)
  {
    // Avoid needless reinitialization when only one enabled pattern exists.
    resetAutoplayTimer();
    batteryViewLastInteractionMs = millis();
    return;
  }

  if (patternCurrentlyActive)
  {
    runPatternExit(previousPattern);
  }

  currentPattern = patternId;
  patternClock.markPatternChange();
  recordPatternChange(previousPattern, patternId, source, latencyMs);
  resetAutoplayTimer();
  batteryViewLastInteractionMs = millis();

  if (activatePatternState)
  {
    // Used by CLI "set pattern" to force a clean re-entry through the state machine.
    fsm.setInitialState(&s[2]);
    fsm.reset();
    return;
  }

  if (patternCurrentlyActive)
  {
    runPatternEntry((uint8_t)currentPattern);
  }
}

void PatternStateEntry()
{
  // Enter whichever user pattern is currently selected.
  runPatternEntry((uint8_t)currentPattern);
}

void PatternStateRunning()
{
  // Render whichever user pattern is currently selected.
  runPatternFrame((uint8_t)currentPattern);
}

void PatternStateExit()
{
  // Give the active pattern a chance to restore temporary state such as brightness overrides.
  runPatternExit((uint8_t)currentPattern);
}

// ============================================================================
//  FSM TABLES & TRIGGERS
// ============================================================================
// The FSM handles only high-level modes.
// Individual LED patterns are dispatched dynamically inside the pattern state.

State s[] = {
    State("battery", BatteryEntry, BatteryRunning),
    State("charging", ChargingEntry, ChargingRunning, ChargingExit),
    State("pattern", PatternStateEntry, PatternStateRunning, PatternStateExit)};

enum triggers
{
  doubleClick = 1,
  longpress,
  usbpower
};

Transition transitions[] = {
    Transition(&s[2], &s[0], longpress),
    Transition(&s[0], &s[1], usbpower),
    Transition(&s[2], &s[1], usbpower)};

TimedTransition timedTransitions[] = {
    TimedTransition(&s[0], &s[2], 5000, NULL, "", batteryViewTimedOut),
    TimedTransition(&s[1], &s[2], 2000, NULL, "", chargingUsbDisconnected)};

int num_transitions = sizeof(transitions) / sizeof(Transition);
int num_timed = sizeof(timedTransitions) / sizeof(TimedTransition);

// ============================================================================
//  SETUP
// ============================================================================

void setup()
{
  pinMode(LED_BUILTIN, OUTPUT);
  digitalWrite(LED_BUILTIN, LOW);

  Serial.begin(115200); // USB CLI, NK4, and startup diagnostics.
  Serial.setTimeout(5);
  bootMark("setup");
  bootMark("serial");

  pinMode(PIN_BATTERY_ADC, INPUT);
  pinMode(PIN_USB_SENSE, INPUT);
  analogReadResolution(12);
  bootMark("pins");

#if I2CDEV_IMPLEMENTATION == I2CDEV_ARDUINO_WIRE
  Wire.begin();
  Wire.setTimeout(25, false);
  Wire.setClock(400000); // 400kHz I2C clock. Comment on this line if having compilation difficulties
#elif I2CDEV_IMPLEMENTATION == I2CDEV_BUILTIN_FASTWIRE
  Fastwire::setup(400, true);
#endif
  bootMark("i2c");

  // Load persisted config early so the remaining setup can use stored values.
  EEPROM.begin(EEPROM_SIZE);
  eepromReady = true;
  bootMark("config_load");
  Serial.println("EEPROM initialized.");
  readConfigFromEEPROM(true);
  bootMark(configValid ? "config_ok" : "config_defaults");

  /* Initialize device */
  bootMark("imu_begin");
  Serial.println(F("Initializing I2C devices..."));
  bool mpuConnected = false;
  if (!safeBootActive)
  {
    mpu.initialize();
    pinMode(INTERRUPT_PIN, INPUT);

    /* Verify connection */
    Serial.println(F("Testing MPU6050 connection..."));
    mpuConnected = (mpu.testConnection() == true);
    if (!mpuConnected)
    {
      Serial.println("MPU6050 connection failed - continuing without DMP.");
    }
    else
    {
      Serial.println("MPU6050 connection successful");
    }
  }
  else
  {
    devStatus = 1;
    Serial.println("INFO safe_boot=1 imu_skipped=1");
  }


  /* Initialize and configure the DMP */
  if (mpuConnected)
  {
    Serial.println(F("Initializing DMP..."));
    devStatus = mpu.dmpInitialize();
  }
  else
  {
    devStatus = 1;
  }

  if (mpuConnected)
  {
    applyConfiguredSensorRanges();
    Serial.print("Configured accel range (g): ");
    Serial.println(accelRegisterValueToRange(mpu.getFullScaleAccelRange()));
    Serial.print("Configured gyro range (dps): ");
    Serial.println(gyroRegisterValueToRange(mpu.getFullScaleGyroRange()));
    applyConfiguredOffsets();
    Serial.print("Boot calibration mode: ");
    Serial.println(bootCalibrationModeToString(currentBootCalibrationMode));
  }

  /* Making sure it worked (returns 0 if so) */
  if (devStatus == 0)
  {
    const bool allowBootCalibration = shouldRunBootCalibration(
        safeBootActive,
        currentBootCalibrationMode == BOOT_CALIBRATION_MODE_QUICK);
    if (allowBootCalibration)
    {
      runQuickCalibration(true);
    }
    else
    {
      Serial.println(configValid ? "Using stored offsets without boot calibration." : "Using defaults without boot calibration.");
      printOffsets();
    }

    Serial.println(F("Enabling DMP...")); // Turning ON DMP
    mpu.setDMPEnabled(true);

    /* Enable interrupt detection */
    Serial.print(F("Enabling interrupt detection (Arduino external interrupt "));
    Serial.print(digitalPinToInterrupt(INTERRUPT_PIN));
    Serial.println(F(")..."));
    attachInterrupt(digitalPinToInterrupt(INTERRUPT_PIN), DMPDataReady, RISING);
    MPUIntStatus = mpu.getIntStatus();

    /* Set the DMP Ready flag so the main loop() function knows it is okay to use it */
    Serial.println(F("DMP ready! Waiting for first interrupt..."));
    DMPReady = true;
    imuReady = true;
    packetSize = mpu.dmpGetFIFOPacketSize(); // Get expected DMP packet size for later comparison
  }
  else
  {
    Serial.print(F("DMP Initialization failed (code ")); // Print the error code
    Serial.print(devStatus);
    Serial.println(F(")"));
    // 1 = initial memory load failed
    // 2 = DMP configuration updates failed
  }
  bootMark(imuReady ? "imu_ok" : "imu_unavailable");

  // Register both physical LED segments with FastLED.
  bootMark("led_begin");
  FastLED.addLeds<LED_TYPE, PinStrip1, COLOR_ORDER>(PhysicalStrip, 0, MAX_LEDS_PER_STRIP).setCorrection(TypicalLEDStrip);
  FastLED.addLeds<LED_TYPE, PinStrip2, COLOR_ORDER>(PhysicalStrip, MAX_LEDS_PER_STRIP, MAX_LEDS_PER_STRIP).setCorrection(TypicalLEDStrip);

  // Apply persisted global brightness.
  updateBatteryMeasurement(true);
  applyEffectiveBrightness();

  // Initialize motion smoothing with the configured window size.
  applyConfiguredMotionSmoothing();
  bootMark("led_ok");

  applyPersistentConfig();
  patternClock.begin();
  syncEngine.begin();
  lastUpdateTime = millis();

#if NIGHTKITE_BLE && NIGHTKITE_RM2
  bootMark("rm2_begin");
  {
    String bleName = "NK-";
    bleName += currentShortId;
    rm2BleSetNk4Handler(handleNk4LineWithWriter);
    const bool bleStartRequested = rm2BleBegin(bleName.c_str());
    syncBeaconRadioBegin();
    if (!bleStartRequested)
    {
      bootMark("rm2_unavailable");
    }
    else if (rm2BleStatus().gatt)
    {
      bootMark("ble_gatt");
    }
    else if (rm2BleStatus().advertising)
    {
      bootMark("ble_advertising");
    }
    else
    {
      bootMark("ble_begin");
    }
  }
#endif
	
// State machine init.
  fsm.add(timedTransitions, num_timed);
  fsm.add(transitions, num_transitions);
  setupCLI();

  // Start in the generic pattern state; the active pattern comes from currentPattern.
  fsm.setInitialState(&s[2]);
  bootMark("setup_done");
}

// ============================================================================
//  LOOP
// ============================================================================

void loop()
{
  if (!bootLoopAnnounced)
  {
    bootMark("loop");
    bootLoopAnnounced = true;
  }

  const uint32_t loopStartUs = micros();
  /* Read a packet from FIFO */
  if (DMPReady && mpu.dmpGetCurrentFIFOPacket(FIFOBuffer))
  { // Get the Latest packet
    // Orientation for hue/rotation-reactive patterns.
    mpu.dmpGetQuaternion(&q, FIFOBuffer);
    mpu.dmpGetGravity(&gravity, &q);
    mpu.dmpGetYawPitchRoll(ypr, &q, &gravity);
    // Gravity-free world-frame acceleration for motion-reactive patterns.
    mpu.dmpGetAccel(&aa, FIFOBuffer);
    mpu.dmpGetLinearAccel(&aaReal, &aa, &gravity);
    mpu.dmpGetLinearAccelInWorld(&aaWorld, &aaReal, &q);

    /* Blink LED to indicate activity */
    blinkState = !blinkState;
    digitalWrite(LED_BUILTIN, blinkState);
  }

  // Periodically persist changed configuration values.
    unsigned long currentTime = millis();
    if (currentTime - lastUpdateTime >= UPDATE_INTERVAL) {
        // Set timestamp for the next check.
        lastUpdateTime = currentTime;

        if (usbProtocolMode == USB_PROTOCOL_HUMAN)
        {
          Serial.println("5 minute interval reached. Checking values for changes...");
        }

        const bool syncTimingActive = (syncEngine.state == SyncEngine::ARMED || syncEngine.state == SyncEngine::RUNNING);
        // Save only when something actually changed and no local sync timing is active.
        if (hasUnsavedConfigChanges() && !syncTimingActive) {
            // At least one value changed.
            if (usbProtocolMode == USB_PROTOCOL_HUMAN)
            {
              Serial.println("Values have changed. Saving new values to EEPROM...");
            }
            saveConfigToEEPROM(usbProtocolMode == USB_PROTOCOL_HUMAN);
        } else if (syncTimingActive && hasUnsavedConfigChanges()) {
            if (usbProtocolMode == USB_PROTOCOL_HUMAN)
            {
              Serial.println("Sync timing active. Deferring EEPROM update.");
            }
        } else {
            // No change.
            if (usbProtocolMode == USB_PROTOCOL_HUMAN)
            {
              Serial.println("Values are unchanged. No EEPROM update needed.");
            }
        }
    }

  // Periodic shared hue update used by several patterns.
  EVERY_N_MILLISECONDS(20) { gHue++; } // Slowly cycle the shared base hue.

  // USB power state for charging behavior and CLI session detection.
  UsbPowerRaw = digitalRead(PIN_USB_SENSE);
  const bool previousSerialSessionActive = SerialSessionActive;
  SerialSessionActive = ((bool)Serial) && Serial.dtr();
  if (SerialSessionActive && !previousSerialSessionActive)
  {
    cliSessionBecameActiveMs = millis();
    cliSessionBannerPending = true;
    cliPromptShown = false;
    resetCliInput();
  }
  // Disable the charging view while a serial session is active.
  UsbConnected = (UsbPowerRaw == 1 && !SerialSessionActive) ? 1 : 0;

  updateBatteryMeasurement(false);
  handleBatteryCutoff();
  if (lowPowerCutoffActive)
  {
    delay(50);
    return;
  }

  fsm.run(0);
  multiresponseButton.poll();

  if (multiresponseButton.longPress())
  {
    fsm.trigger(longpress);
  }

  if (multiresponseButton.doubleClick())
  {
    if (batteryViewActive)
    {
      cyclePlayMode();
      batteryViewLastInteractionMs = millis();
      if (usbProtocolMode == USB_PROTOCOL_MACHINE)
      {
        String fields = "play_mode=";
        fields += playModeToString(currentPlayMode);
        fields += " autoplay=";
        fields += currentAutoplayEnabled;
        emitNk4Event("play_mode_changed", fields);
      }
      else
      {
        Serial.print("INFO play_mode=");
        Serial.print(playModeToString(currentPlayMode));
        Serial.print(" autoplay=");
        Serial.println(autoplayEnabledToString());
      }
    }
    else if (!UsbConnected)
    {
      switchToPattern(getNextEnabledPattern((uint8_t)currentPattern), false, "button");
      announcePatternChange("button");
    }
  }

  // USB-only power enters the charging state unless a serial session is active.
  if (UsbConnected == 1)
  {
    fsm.trigger(usbpower);
  }

  handleCLI();
  rm2BleTick();
  patternClock.tick();
  syncEngine.tick();
  tickSyncBeaconRadio();
  applySyncStartIfDue();

  if (multiresponseButton.singleClick() && batteryViewActive)
  {
    BRIGHTNESS += 32; // Brightness range is 95..255 in steps of 32.
    if (BRIGHTNESS > MAX_BRIGHTNESS)
    {
      BRIGHTNESS = MIN_BRIGHTNESS; // Wrap around to the minimum brightness.
    }
    currentBrightness = BRIGHTNESS;
    applyEffectiveBrightness();
    batteryViewLastInteractionMs = millis();
  }

  const bool autoplayPaused = batteryViewActive || UsbConnected;
  if (shouldRunAutoplayTick())
  {
    if (autoplayPaused)
    {
      autoplayWasPaused = true;
    }
    else
    {
      if (autoplayWasPaused)
      {
        resetAutoplayTimer();
      }
      if (millis() - autoplayLastSwitchMs >= (unsigned long)currentAutoplayIntervalMs)
      {
        switchToPattern(getNextEnabledPattern((uint8_t)currentPattern), false, "autoplay");
        announcePatternChange("autoplay");
      }
    }
  }
  else
  {
    autoplayWasPaused = false;
  }

  // Copy the logical LEDs into the fixed physical strip layout and show them.
  clearInactiveLeds();
  applyBatteryBrightnessLimit();
  syncLogicalToPhysicalLeds();
  FastLED.show();

  lastWorkDurationUs = micros() - loopStartUs;
  totalWorkDurationUs += lastWorkDurationUs;
  if (lastWorkDurationUs > maxWorkDurationUs)
  {
    maxWorkDurationUs = lastWorkDurationUs;
  }

  // Keep the framerate bounded by the configured frame budget after all
  // per-frame work has completed.
  const uint32_t frameBudgetUs = 1000000UL / FRAMES_PER_SECOND;
  if (lastWorkDurationUs < frameBudgetUs)
  {
    delayMicroseconds(frameBudgetUs - lastWorkDurationUs);
  }

  lastLoopDurationUs = micros() - loopStartUs;
  totalLoopDurationUs += lastLoopDurationUs;
  if (lastLoopDurationUs > maxLoopDurationUs)
  {
    maxLoopDurationUs = lastLoopDurationUs;
  }
  loopTimingSamples++;
}
