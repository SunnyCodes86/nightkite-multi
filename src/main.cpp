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
#define PIN_RM2_WL_ON 32
#endif
#ifndef PIN_RM2_WL_CS
#define PIN_RM2_WL_CS 33
#endif
#ifndef PIN_RM2_WL_CLK
#define PIN_RM2_WL_CLK 34
#endif
#ifndef PIN_RM2_WL_DATA
#define PIN_RM2_WL_DATA 35
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

constexpr float BATTERY_BAR_5_THRESHOLD = 4.05f;
constexpr float BATTERY_BAR_4_THRESHOLD = 3.92f;
constexpr float BATTERY_BAR_3_THRESHOLD = 3.80f;
constexpr float BATTERY_BAR_2_THRESHOLD = 3.68f;
constexpr float BATTERY_BAR_1_YELLOW_THRESHOLD = 3.55f;
constexpr float BATTERY_BAR_1_RED_THRESHOLD = 3.40f;
constexpr float CHARGING_FULL_THRESHOLD = 4.20f;

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
const int CONFIG_VERSION_4_ALPHA = 400;

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
const int SYNC_ROLE_STANDALONE = 0;
const int SYNC_ROLE_MASTER = 1;
const int SYNC_ROLE_FOLLOWER = 2;
const int SYNC_LOSS_CONTINUE_LOCAL = 0;
const int SYNC_LOSS_FALLBACK_AUTOPLAY = 1;
const int SYNC_LOSS_WARNING_ONLY = 2;
const int WIRELESS_PROFILE_LONG_RANGE = 0;
const int WIRELESS_PROFILE_BALANCED = 1;
const int WIRELESS_PROFILE_FAST_SYNC = 2;

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
const uint8_t FIRST_PATTERN_ID = 1;
const uint8_t LAST_PATTERN_ID = 22;
const uint8_t PATTERN_COUNT = LAST_PATTERN_ID - FIRST_PATTERN_ID + 1;
const uint32_t ALL_ENABLED_PATTERN_MASK = (1ul << PATTERN_COUNT) - 1ul;
const uint32_t ALL_INVERTED_PATTERN_MASK = (1ul << PATTERN_COUNT) - 1ul;

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
int color;
int color2;
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

struct NkSyncBeaconV1
{
  uint8_t magic0;
  uint8_t magic1;
  uint8_t version;
  uint8_t groupId;
  uint8_t flags;
  uint16_t seq;
  uint8_t pattern;
  uint8_t brightness;
  uint32_t phaseMs;
  uint16_t beatMs;
  uint16_t crc;
} __attribute__((packed));

class PatternClock
{
public:
  void begin()
  {
    baseMs = millis();
    phaseMs = 0;
    running = true;
    armed = false;
  }

  uint32_t now() const
  {
    if (!running)
    {
      return phaseMs;
    }
    return phaseMs + (millis() - baseMs);
  }

  void setPhase(uint32_t phase)
  {
    phaseMs = phase;
    baseMs = millis();
    running = true;
    armed = false;
  }

  void armStart(uint32_t localStartMs, uint32_t phase)
  {
    armedStartMs = localStartMs;
    armedPhaseMs = phase;
    armed = true;
    running = false;
  }

  void tick()
  {
    if (armed && (int32_t)(millis() - armedStartMs) >= 0)
    {
      phaseMs = armedPhaseMs;
      baseMs = armedStartMs;
      running = true;
      armed = false;
    }
  }

  bool isRunning() const
  {
    return running;
  }

  bool isArmed() const
  {
    return armed;
  }

private:
  uint32_t baseMs = 0;
  uint32_t phaseMs = 0;
  uint32_t armedStartMs = 0;
  uint32_t armedPhaseMs = 0;
  bool running = false;
  bool armed = false;
};

class SyncEngine
{
public:
  enum State
  {
    IDLE,
    ARMED,
    RUNNING,
    LOST,
    ERROR
  };

  void begin()
  {
    state = IDLE;
    lastSeq = 0;
    localStartMs = 0;
    locked = false;
    driftMs = 0;
  }

  bool arm(uint8_t group, uint8_t pattern, uint8_t brightness, uint32_t startInMs, uint32_t phase)
  {
    if (state == ARMED)
    {
      return false;
    }
    armedGroup = group;
    armedPattern = pattern;
    armedBrightness = brightness;
    armedPhaseMs = phase;
    localStartMs = millis() + startInMs;
    state = ARMED;
    locked = false;
    lastSeq++;
    return true;
  }

  void cancel()
  {
    state = IDLE;
    locked = false;
    driftMs = 0;
  }

  void tick()
  {
    if (state == ARMED && (int32_t)(millis() - localStartMs) >= 0)
    {
      state = RUNNING;
      locked = true;
    }
  }

  const char* stateName() const
  {
    switch (state)
    {
      case IDLE: return "idle";
      case ARMED: return "armed";
      case RUNNING: return "running";
      case LOST: return "lost";
      case ERROR: return "error";
      default: return "error";
    }
  }

  State state = IDLE;
  uint16_t lastSeq = 0;
  uint32_t localStartMs = 0;
  uint32_t armedPhaseMs = 0;
  uint8_t armedGroup = 1;
  uint8_t armedPattern = 1;
  uint8_t armedBrightness = MIN_BRIGHTNESS;
  bool locked = false;
  int32_t driftMs = 0;
};

class IResponseWriter
{
public:
  virtual void print(const char* value) = 0;
  virtual void print(const String& value) = 0;
  virtual void print(int value) = 0;
  virtual void print(unsigned int value) = 0;
  virtual void print(unsigned long value) = 0;
  virtual void println() = 0;
};

class SerialResponseWriter : public IResponseWriter
{
public:
  void print(const char* value) override { Serial.print(value); }
  void print(const String& value) override { Serial.print(value); }
  void print(int value) override { Serial.print(value); }
  void print(unsigned int value) override { Serial.print(value); }
  void print(unsigned long value) override { Serial.print(value); }
  void println() override { Serial.println(); }
};

struct NkKeyValue
{
  String key;
  String value;
};

struct NkCommand
{
  String seq;
  String command;
  NkKeyValue pairs[18];
  uint8_t pairCount = 0;
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
void copyCString(char* dest, size_t destSize, const char* source);
void writeEEPROMCString(int address, const char* value, size_t maxLength);
void readEEPROMCString(int address, char* value, size_t maxLength);
bool isValidDeviceUid(const char* value);
void generateDeviceUid();
void setDefaultDeviceName();
bool sanitizeDeviceName(String value, char* output, size_t outputSize);
bool sanitizeUidString(String value, char* output, size_t outputSize);
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
int estimateBatteryPercent(float voltage);
String formatHex32(uint32_t value);
String buildPatternMaskFields();
bool hasUnsavedConfigChanges();
void markCurrentConfigSaved();
void emitNk4Event(const char* eventName, const String& fields);
int sanitizeAutoplayEnabled(int value);
int sanitizeAutoplayIntervalMs(int value);
void resetAutoplayTimer();
bool isAutoplayEnabled();
const char* autoplayEnabledToString();
int parseOnOffValue(String valueText);
void announcePatternChange(const char* source);
void printOffsets();
void printOffsetsWithPrefix(const char* prefix);
void printConfigSummaryWithPrefix(const char* prefix);
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
void switchToPattern(uint8_t patternId, bool activatePatternState);
bool batteryViewTimedOut();
bool chargingUsbDisconnected();
int readBatteryRawValue();
float convertBatteryRawToVoltage(int rawValue);
void printBatteryStatus();
void printSensorStatus();
void resetTimingStats();
void printTimingStatus();
bool saveConfigToEEPROM(bool verbose);
void readConfigFromEEPROM(bool verbose);
bool parseNk4Line(const String& line, NkCommand* outCommand, String* errorCode, String* errorMessage);
String nk4GetValue(const NkCommand& command, const char* key);
bool nk4HasKey(const NkCommand& command, const char* key);
bool handleNk4Line(const String& line);
void handleNk4Command(const NkCommand& command, IResponseWriter& writer);
void nk4WriteOk(IResponseWriter& writer, const String& seq, const String& fields);
void nk4WriteError(IResponseWriter& writer, const String& seq, const char* code, const char* message);
void showPlayModeIndicatorTest();
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
  int pulse1 = pulseWave8(millis() + time_shift, cycleLength, pulseLength);
  int pulse2 = pulseWave8(millis() + time_shift + pulseOffset, cycleLength, pulseLength);
  return qadd8(pulse1, pulse2); // Add pulses together without overflow
}

inline uint32_t smoothedMotion() {
  // The smoothing window is configurable and reused by many patterns.
  uint32_t s = (uint32_t)abs(aaWorld.x) + (uint32_t)abs(aaWorld.y);
  myAccel.add(s);
  return myAccel.get();
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
  applyConfiguredStripLength();
  currentAutoplayEnabled = sanitizeAutoplayEnabled(currentAutoplayEnabled);
  currentAutoplayIntervalMs = sanitizeAutoplayIntervalMs(currentAutoplayIntervalMs);
  BRIGHTNESS = currentBrightness;
  FastLED.setBrightness(BRIGHTNESS);
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

const char* autoplayEnabledToString()
{
  return isAutoplayEnabled() ? "on" : "off";
}

int parseOnOffValue(String valueText)
{
  valueText.trim();
  valueText.toLowerCase();
  if (valueText == "on")
  {
    return 1;
  }
  if (valueText == "off")
  {
    return 0;
  }
  return -1;
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
    if (!isalnum((int)ch) && ch != '-' && ch != '_' && ch != '.')
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
    if (!isxdigit((int)ch))
    {
      return false;
    }
  }
  copyCString(output, outputSize, value.c_str());
  return true;
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
  (void)value;
  return "last";
}

int parseBootMode(String value)
{
  value.trim();
  value.toLowerCase();
  if (value == "last" || value == "0") return BOOT_MODE_LAST;
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

int estimateBatteryPercent(float voltage)
{
  if (voltage >= CHARGING_FULL_THRESHOLD) return 100;
  if (voltage <= BATTERY_BAR_1_RED_THRESHOLD) return 0;
  return constrain((int)(((voltage - BATTERY_BAR_1_RED_THRESHOLD) * 100.0f) / (CHARGING_FULL_THRESHOLD - BATTERY_BAR_1_RED_THRESHOLD)), 0, 100);
}

String formatHex32(uint32_t value)
{
  char buffer[11];
  snprintf(buffer, sizeof(buffer), "0x%08lX", (unsigned long)value);
  return String(buffer);
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
  return fields;
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
  if (currentBootMode != BOOT_MODE_LAST)
  {
    currentBootMode = DEFAULT_BOOT_MODE;
  }
  currentSyncEnabled = sanitizeBinaryFlag(currentSyncEnabled);
  currentSyncGroupId = constrain(currentSyncGroupId, 0, 255);
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

    int patternId = token.toInt();
    if (!isValidPatternId(patternId))
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

void printBatteryStatus()
{
  RawVoltage = readBatteryRawValue();
  Voltage = convertBatteryRawToVoltage(RawVoltage);
  const int usbSenseRaw = digitalRead(PIN_USB_SENSE);

  Serial.print("OK battery_raw=");
  Serial.print(RawVoltage);
  Serial.print(" battery_voltage=");
  Serial.print(Voltage, 3);
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
  bool loadedExtendedConfig = false;
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

  EEPROM.get(EEPROM_ADDR_PATTERN, currentPattern);
  EEPROM.get(EEPROM_ADDR_BRIGHTNESS, currentBrightness);
  EEPROM.get(EEPROM_ADDR_MAGIC, magic);
  if (magic == EEPROM_MAGIC)
  {
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
    int storedConfigVersion = 0;
    EEPROM.get(EEPROM_ADDR_CONFIG_VERSION, storedConfigVersion);
    if (storedConfigVersion == CONFIG_VERSION_4_ALPHA)
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
  }
  // Future migrations can branch on storedConfigVersion here while leaving the
  // original 3.x EEPROM addresses intact for backward compatibility.
  if (!loadedExtendedConfig)
  {
    currentPlayMode = currentAutoplayEnabled ? PLAY_MODE_AUTOPLAY : PLAY_MODE_MANUAL;
  }
  normalizePersistentConfig();
  markCurrentConfigSaved();

  if (magic != EEPROM_MAGIC || !loadedExtendedConfig)
  {
    saveConfigToEEPROM(false);
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

void nk4WriteOk(IResponseWriter& writer, const String& seq, const String& fields)
{
  writer.print("NK4 seq=");
  writer.print(seq.length() > 0 ? seq : "0");
  writer.print(" ok");
  if (fields.length() > 0)
  {
    writer.print(" ");
    writer.print(fields);
  }
  writer.println();
}

void nk4WriteError(IResponseWriter& writer, const String& seq, const char* code, const char* message)
{
  writer.print("NK4 seq=");
  writer.print(seq.length() > 0 ? seq : "0");
  writer.print(" err code=");
  writer.print(code != NULL ? code : "internal_error");
  writer.print(" msg=");
  writer.print(message != NULL ? message : "error");
  writer.println();
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

String nk4GetValue(const NkCommand& command, const char* key)
{
  for (uint8_t i = 0; i < command.pairCount; i++)
  {
    if (command.pairs[i].key == key)
    {
      return command.pairs[i].value;
    }
  }
  return "";
}

bool nk4HasKey(const NkCommand& command, const char* key)
{
  for (uint8_t i = 0; i < command.pairCount; i++)
  {
    if (command.pairs[i].key == key)
    {
      return true;
    }
  }
  return false;
}

bool parseNk4Line(const String& line, NkCommand* outCommand, String* errorCode, String* errorMessage)
{
  if (outCommand == NULL)
  {
    return false;
  }

  *outCommand = NkCommand();
  String input = line;
  input.trim();
  if (!input.startsWith("NK4"))
  {
    if (errorCode != NULL) *errorCode = "invalid_command";
    if (errorMessage != NULL) *errorMessage = "expected_NK4";
    return false;
  }

  int start = 3;
  while (start < input.length())
  {
    while (start < input.length() && input[start] == ' ')
    {
      start++;
    }
    if (start >= input.length())
    {
      break;
    }

    int space = input.indexOf(' ', start);
    String token = (space >= 0) ? input.substring(start, space) : input.substring(start);
    int equals = token.indexOf('=');
    if (equals <= 0 || equals == token.length() - 1)
    {
      if (errorCode != NULL) *errorCode = "invalid_key";
      if (errorMessage != NULL) *errorMessage = "bad_token";
      return false;
    }

    String key = token.substring(0, equals);
    String value = token.substring(equals + 1);
    key.toLowerCase();
    if (key == "seq")
    {
      outCommand->seq = value;
    }
    else if (key == "cmd")
    {
      value.toLowerCase();
      outCommand->command = value;
    }
    else
    {
      if (outCommand->pairCount >= (sizeof(outCommand->pairs) / sizeof(outCommand->pairs[0])))
      {
        if (errorCode != NULL) *errorCode = "range_error";
        if (errorMessage != NULL) *errorMessage = "too_many_keys";
        return false;
      }
      outCommand->pairs[outCommand->pairCount].key = key;
      outCommand->pairs[outCommand->pairCount].value = value;
      outCommand->pairCount++;
    }

    if (space < 0)
    {
      break;
    }
    start = space + 1;
  }

  if (outCommand->seq.length() == 0)
  {
    if (errorCode != NULL) *errorCode = "invalid_key";
    if (errorMessage != NULL) *errorMessage = "missing_seq";
    return false;
  }
  if (outCommand->command.length() == 0)
  {
    if (errorCode != NULL) *errorCode = "invalid_command";
    if (errorMessage != NULL) *errorMessage = "missing_cmd";
    return false;
  }

  return true;
}

void showPlayModeIndicatorTest()
{
  const CRGB colors[] = {
      CRGB::Blue,
      CRGB::Green,
      CRGB::Cyan,
      CRGB::Magenta,
      CRGB::Red};
  const size_t colorCount = sizeof(colors) / sizeof(colors[0]);
  const uint8_t previousBrightness = BRIGHTNESS;
  FastLED.setBrightness(BRIGHTNESS);
  for (size_t i = 0; i < colorCount; i++)
  {
    fill_solid(Strip, TOTAL_LEDS, colors[i]);
    clearInactiveLeds();
    syncLogicalToPhysicalLeds();
    FastLED.show();
    delay(220);
  }
  fill_solid(Strip, TOTAL_LEDS, CRGB::Black);
  clearInactiveLeds();
  syncLogicalToPhysicalLeds();
  FastLED.show();
  FastLED.setBrightness(previousBrightness);
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
    fields += " sync_supported=1 patterns=";
    fields += PATTERN_COUNT;
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
    RawVoltage = readBatteryRawValue();
    Voltage = convertBatteryRawToVoltage(RawVoltage);
    String fields = "pattern=";
    fields += currentPattern;
    fields += " brightness=";
    fields += currentBrightness;
    fields += " battery_percent=";
    fields += estimateBatteryPercent(Voltage);
    fields += " battery_voltage=";
    fields += String(Voltage, 3);
    fields += " usb=";
    fields += (UsbPowerRaw == 1 ? 1 : 0);
    fields += " imu=";
    fields += (DMPReady ? 1 : 0);
    fields += " fps=";
    fields += FastLED.getFPS();
    fields += " play_mode=";
    fields += playModeToString(currentPlayMode);
    fields += " autoplay=";
    fields += currentAutoplayEnabled;
    fields += " sync_state=";
    fields += syncEngine.stateName();
    fields += " sync_role=";
    fields += syncRoleToString(currentSyncRole);
    fields += " sync_group=";
    fields += currentSyncGroupId;
    nk4WriteOk(writer, seq, fields);
    return;
  }

  if (command.command == "patterns")
  {
    nk4WriteOk(writer, seq, buildPatternMaskFields());
    return;
  }

  if (command.command == "get")
  {
    String section = nk4GetValue(command, "section");
    section.toLowerCase();
    if (section == "sync")
    {
      String fields = "sync_enabled=";
      fields += currentSyncEnabled;
      fields += " sync_group=";
      fields += currentSyncGroupId;
      fields += " sync_role=";
      fields += syncRoleToString(currentSyncRole);
      fields += " sync_loss_behavior=";
      fields += syncLossBehaviorToString(currentSyncLossBehavior);
      fields += " master_uid=";
      fields += strlen(currentSyncMasterUid) > 0 ? currentSyncMasterUid : "none";
      nk4WriteOk(writer, seq, fields);
      return;
    }
    if (section == "wireless")
    {
      String fields = "wireless_enabled=";
      fields += currentWirelessEnabled;
      fields += " wireless_profile=";
      fields += wirelessProfileToString(currentWirelessProfile);
      fields += " ble=";
      fields += (NIGHTKITE_BLE ? 1 : 0);
      fields += " rm2=";
      fields += (NIGHTKITE_RM2 ? 1 : 0);
      nk4WriteOk(writer, seq, fields);
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
    nk4WriteError(writer, seq, "invalid_value", "bad_section");
    return;
  }

  if (command.command == "set")
  {
    for (uint8_t i = 0; i < command.pairCount; i++)
    {
      const String key = command.pairs[i].key;
      const String value = command.pairs[i].value;
      if (key == "name")
      {
        char sanitized[DEVICE_NAME_LENGTH + 1];
        if (!sanitizeDeviceName(value, sanitized, sizeof(sanitized)))
        {
          nk4WriteError(writer, seq, "invalid_value", "bad_name");
          return;
        }
        copyCString(currentDeviceName, sizeof(currentDeviceName), sanitized);
      }
      else if (key == "uid" || key == "device_uid")
      {
        nk4WriteError(writer, seq, "locked", "uid_locked");
        return;
      }
      else if (key == "pattern")
      {
        int valueInt = value.toInt();
        if (!isValidPatternId(valueInt))
        {
          nk4WriteError(writer, seq, "range_error", "bad_pattern");
          return;
        }
        switchToPattern((uint8_t)valueInt, true);
      }
      else if (key == "brightness")
      {
        int valueInt = value.toInt();
        if (!isValidBrightnessLevel(valueInt))
        {
          nk4WriteError(writer, seq, "range_error", "bad_brightness");
          return;
        }
        currentBrightness = valueInt;
        BRIGHTNESS = currentBrightness;
        FastLED.setBrightness(BRIGHTNESS);
      }
      else if (key == "sync_enabled")
      {
        currentSyncEnabled = sanitizeBinaryFlag(value.toInt());
      }
      else if (key == "sync_group" || key == "sync_group_id")
      {
        int valueInt = value.toInt();
        if (valueInt < 0 || valueInt > 255)
        {
          nk4WriteError(writer, seq, "range_error", "bad_sync_group");
          return;
        }
        currentSyncGroupId = valueInt;
      }
      else if (key == "sync_role")
      {
        int role = parseSyncRole(value);
        if (role < 0)
        {
          nk4WriteError(writer, seq, "invalid_value", "bad_sync_role");
          return;
        }
        currentSyncRole = role;
      }
      else if (key == "sync_master_uid" || key == "master_uid")
      {
        if (value == "none" || value == "0")
        {
          currentSyncMasterUid[0] = '\0';
        }
        else
        {
          char sanitized[DEVICE_UID_LENGTH + 1];
          if (!sanitizeUidString(value, sanitized, sizeof(sanitized)))
          {
            nk4WriteError(writer, seq, "invalid_value", "bad_master_uid");
            return;
          }
          copyCString(currentSyncMasterUid, sizeof(currentSyncMasterUid), sanitized);
        }
      }
      else if (key == "sync_loss_behavior")
      {
        int behavior = parseSyncLossBehavior(value);
        if (behavior < 0)
        {
          nk4WriteError(writer, seq, "invalid_value", "bad_sync_loss");
          return;
        }
        currentSyncLossBehavior = behavior;
      }
      else if (key == "wireless_enabled")
      {
        currentWirelessEnabled = sanitizeBinaryFlag(value.toInt());
      }
      else if (key == "wireless_profile")
      {
        int profile = parseWirelessProfile(value);
        if (profile < 0)
        {
          nk4WriteError(writer, seq, "invalid_value", "bad_wireless_profile");
          return;
        }
        currentWirelessProfile = profile;
      }
      else if (key == "play_mode")
      {
        int mode = parsePlayMode(value);
        if (mode < 0)
        {
          nk4WriteError(writer, seq, "invalid_value", "bad_play_mode");
          return;
        }
        currentPlayMode = mode;
        currentAutoplayEnabled = (mode == PLAY_MODE_AUTOPLAY) ? 1 : 0;
        resetAutoplayTimer();
      }
      else if (key == "boot_mode")
      {
        int mode = parseBootMode(value);
        if (mode < 0)
        {
          nk4WriteError(writer, seq, "unsupported", "bad_boot_mode");
          return;
        }
        currentBootMode = mode;
      }
      else if (key == "autoplay" || key == "autoplay_enabled")
      {
        currentAutoplayEnabled = sanitizeBinaryFlag(value.toInt());
        currentPlayMode = currentAutoplayEnabled ? PLAY_MODE_AUTOPLAY : PLAY_MODE_MANUAL;
        resetAutoplayTimer();
      }
      else if (key == "autoplay_interval")
      {
        int intervalMs = value.toInt() * 1000;
        if (intervalMs < MIN_AUTOPLAY_INTERVAL_MS || intervalMs > MAX_AUTOPLAY_INTERVAL_MS)
        {
          nk4WriteError(writer, seq, "range_error", "bad_autoplay_interval");
          return;
        }
        currentAutoplayIntervalMs = sanitizeAutoplayIntervalMs(intervalMs);
        resetAutoplayTimer();
      }
      else if (key == "usb_mode" || key == "protocol")
      {
        String mode = value;
        mode.toLowerCase();
        if (mode == "human" || mode == "legacy")
        {
          usbProtocolMode = USB_PROTOCOL_HUMAN;
        }
        else if (mode == "machine" || mode == "nk4")
        {
          usbProtocolMode = USB_PROTOCOL_MACHINE;
        }
        else
        {
          nk4WriteError(writer, seq, "invalid_value", "bad_usb_mode");
          return;
        }
      }
      else
      {
        nk4WriteError(writer, seq, "invalid_key", "unknown_key");
        return;
      }
    }
    normalizePersistentConfig();
    String fields = "updated=1 play_mode=";
    fields += playModeToString(currentPlayMode);
    fields += " pattern=";
    fields += currentPattern;
    fields += " brightness=";
    fields += currentBrightness;
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
    Serial.flush();
    delay(50);
    rp2040.reboot();
    return;
  }

  if (command.command == "sync_status")
  {
    String fields = "sync_enabled=";
    fields += currentSyncEnabled;
    fields += " sync_group=";
    fields += currentSyncGroupId;
    fields += " sync_role=";
    fields += syncRoleToString(currentSyncRole);
    fields += " sync_state=";
    fields += syncEngine.stateName();
    fields += " master_uid=";
    fields += strlen(currentSyncMasterUid) > 0 ? currentSyncMasterUid : "none";
    fields += " last_seq=";
    fields += syncEngine.lastSeq;
    fields += " locked=";
    fields += syncEngine.locked ? 1 : 0;
    fields += " drift_ms=";
    fields += syncEngine.driftMs;
    nk4WriteOk(writer, seq, fields);
    return;
  }

  if (command.command == "sync_arm")
  {
    int group = nk4HasKey(command, "group") ? nk4GetValue(command, "group").toInt() : currentSyncGroupId;
    int pattern = nk4HasKey(command, "pattern") ? nk4GetValue(command, "pattern").toInt() : currentPattern;
    int brightness = nk4HasKey(command, "brightness") ? nk4GetValue(command, "brightness").toInt() : currentBrightness;
    int startIn = nk4HasKey(command, "start_in") ? nk4GetValue(command, "start_in").toInt() : 0;
    int phase = nk4HasKey(command, "phase") ? nk4GetValue(command, "phase").toInt() : 0;
    if (group < 0 || group > 255 || !isValidPatternId(pattern) || !isValidBrightnessLevel(brightness) || startIn < 0 || startIn > 60000 || phase < 0)
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
    currentSyncGroupId = group;
    String fields = "sync=armed group=";
    fields += group;
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
  Serial.println("  set pattern <1..22>");
  Serial.println("  set brightness <95|127|159|191|223|255>");
  Serial.println("  set strip_length <10..35>");
  Serial.println("  set smoothing <1..512>           (takes effect after reboot)");
  Serial.println("  set accel_range <2|4|8|16>       (takes effect after reboot)");
  Serial.println("  set gyro_range <250|500|1000|2000> (takes effect after reboot)");
  Serial.println("  set boot_calibration <off|quick>");
  Serial.println("  set autoplay <on|off>");
  Serial.println("  set autoplay_interval <1..300>");
  Serial.println("  patterns");
  Serial.println("  enable_pattern <1..22[,id...]>");
  Serial.println("  disable_pattern <1..22[,id...]>");
  Serial.println("  invert_pattern <1..22[,id...]>");
  Serial.println("  normal_pattern <1..22[,id...]>");
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
  int value = valueText.toInt();
  key.toLowerCase();

  if (key == "pattern")
  {
    if (!isValidPatternId(value))
    {
      Serial.println("ERR pattern range 1..22");
      return;
    }
    switchToPattern((uint8_t)value, true);
    announcePatternChange("cli");
    Serial.print("OK pattern=");
    Serial.println(currentPattern);
    return;
  }

  if (key == "brightness")
  {
    if (!isValidBrightnessLevel(value))
    {
      Serial.println("ERR brightness must be one of 95,127,159,191,223,255");
      return;
    }

    currentBrightness = value;
    BRIGHTNESS = currentBrightness;
    FastLED.setBrightness(BRIGHTNESS);
    batteryViewLastInteractionMs = millis();

    Serial.print("OK brightness=");
    Serial.println(currentBrightness);
    return;
  }
  if (key == "strip_length")
  {
    if (!isValidStripLength(value))
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
    if (!isValidMotionSmoothingSize(value))
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
    if (!isValidAccelRange(value))
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
    if (!isValidGyroRange(value))
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

    currentAutoplayEnabled = sanitizeAutoplayEnabled(autoplayValue);
    currentPlayMode = currentAutoplayEnabled ? PLAY_MODE_AUTOPLAY : PLAY_MODE_MANUAL;
    resetAutoplayTimer();
    Serial.print("OK autoplay=");
    Serial.println(autoplayEnabledToString());
    return;
  }
  if (key == "autoplay_interval")
  {
    int intervalMs = value * 1000;
    if (intervalMs < MIN_AUTOPLAY_INTERVAL_MS || intervalMs > MAX_AUTOPLAY_INTERVAL_MS)
    {
      Serial.println("ERR autoplay_interval range 1..300");
      return;
    }

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
  Serial.flush();
  delay(50);
  rp2040.reboot();
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
    Serial.println("ERR pattern list must contain IDs in range 1..22");
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
    Serial.println("ERR pattern list must contain IDs in range 1..22");
    return;
  }

  // Reject requests that would leave button cycling with zero available patterns.
  if (!updateEnabledPatternsFromMask(mask, false))
  {
    Serial.println("ERR at least one pattern must remain enabled");
    return;
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
    Serial.println("ERR pattern list must contain IDs in range 1..22");
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
    Serial.println("ERR pattern list must contain IDs in range 1..22");
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

void handleCLI()
{
  bool commandExecuted = false;

  if (!SerialSessionActive)
  {
    cliInputBuffer = "";
    cliPromptShown = false;
    cliLastInputMs = 0;
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

    if (ch == '\r')
    {
      continue;
    }
    if (ch == '\n')
    {
      cliInputBuffer.trim();
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
          commandExecuted = true;
        }
      }
      cliInputBuffer = "";
      continue;
    }

    if (isPrintable((int)ch) && cliInputBuffer.length() < 128)
    {
      cliInputBuffer += ch;
      cliLastInputMs = millis();
    }
  }

  // Some serial monitors send without newline; parse after a short idle time.
  if (cliInputBuffer.length() > 0 && cliLastInputMs > 0 && (millis() - cliLastInputMs >= CLI_AUTOPARSE_TIMEOUT_MS))
  {
    cliInputBuffer.trim();
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
        commandExecuted = true;
      }
    }
    cliInputBuffer = "";
    cliLastInputMs = 0;
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
  RawVoltage = analogRead(PIN_BATTERY_ADC);
  Voltage = RawVoltage * 3.0 * 3.3 / 4096.0;

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

  // Hysteresis for voltage measurement.
  static float vLast = 0;
  const float HYS = 0.03; // 30 mV
  if (fabsf(Voltage - vLast) < HYS)
  {
    Voltage = vLast;
  }
  else
  {
    vLast = Voltage;
  }

  fill_solid(Strip, NUM_LEDS * 2, CRGB::Black);

  int statusStart = NUM_LEDS;
  if (statusStart < TOTAL_LEDS)
  {
    Strip[statusStart] = blink ? CRGB::Red : CRGB::Black;
  }

  int batteryBarMax = min(5, NUM_LEDS);
  if (Voltage >= CHARGING_FULL_THRESHOLD)
  {
    fill_solid(Strip, min(5, batteryBarMax), CRGB::Blue);
  }
  else if (Voltage >= BATTERY_BAR_4_THRESHOLD)
  {
    fill_solid(Strip, min(4, batteryBarMax), CRGB::Green);
  }
  else if (Voltage >= BATTERY_BAR_3_THRESHOLD)
  {
    fill_solid(Strip, min(3, batteryBarMax), CRGB::Green);
  }
  else if (Voltage >= BATTERY_BAR_2_THRESHOLD)
  {
    fill_solid(Strip, min(2, batteryBarMax), CRGB::Yellow);
  }
  else if (Voltage >= BATTERY_BAR_1_YELLOW_THRESHOLD)
  {
    fill_solid(Strip, min(1, batteryBarMax), CRGB::Yellow);
  }
  else if (Voltage >= BATTERY_BAR_1_RED_THRESHOLD)
  {
    fill_solid(Strip, min(1, batteryBarMax), CRGB::Red);
  }
  // Below the red threshold, leave the battery bar off.
}

void ChargingExit()
{
  FastLED.setBrightness(BRIGHTNESS);
}

void BatteryEntry()
{
  fill_solid(Strip, NUM_LEDS * 2, CRGB::Black);
  batteryViewActive = true;
  batteryViewLastInteractionMs = millis();
}

void BatteryRunning()
{
  RawVoltage = analogRead(PIN_BATTERY_ADC);
  Voltage = RawVoltage * 3.0 * 3.3 / 4096.0;

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

  // Hysteresis for voltage measurement.
  static float vLast = 0;
  const float HYS = 0.03; // 30 mV
  if (fabsf(Voltage - vLast) < HYS)
  {
    Voltage = vLast;
  }
  else
  {
    vLast = Voltage;
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
  if (Voltage >= BATTERY_BAR_5_THRESHOLD)
  {
    fill_solid(Strip, min(5, batteryBarMax), CRGB::Blue);
  }
  else if (Voltage >= BATTERY_BAR_4_THRESHOLD)
  {
    fill_solid(Strip, min(4, batteryBarMax), CRGB::Green);
  }
  else if (Voltage >= BATTERY_BAR_3_THRESHOLD)
  {
    fill_solid(Strip, min(3, batteryBarMax), CRGB::Green);
  }
  else if (Voltage >= BATTERY_BAR_2_THRESHOLD)
  {
    fill_solid(Strip, min(2, batteryBarMax), CRGB::Yellow);
  }
  else if (Voltage >= BATTERY_BAR_1_YELLOW_THRESHOLD)
  {
    fill_solid(Strip, min(1, batteryBarMax), CRGB::Yellow);
  }
  else if (Voltage >= BATTERY_BAR_1_RED_THRESHOLD)
  {
    fill_solid(Strip, min(1, batteryBarMax), CRGB::Red);
  }
  // Below the red threshold, leave the battery bar off.

  int autoplayStatusPixel = statusStart + 1 + brightnessPixels;
  if (autoplayStatusPixel < TOTAL_LEDS)
  {
    CRGB statusColor = isAutoplayEnabled() ? CRGB::Green : CRGB::Red;
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
  fill_rainbow(Strip, NUM_LEDS * 2, gHue, 7);
}

void RunEntry2()
{
  fill_solid(Strip, NUM_LEDS * 2, CRGB::Black);
  currentPattern = 2;
  batteryViewActive = false;
}

void running2()
{
  color = map(color, -180, 180, 0, 255);
  fill_solid(Strip, NUM_LEDS * 2, CHSV(color, 255, 255));
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
  color = map(color, -180, 180, 0, 255);
  FastLED.setBrightness(accelcon); // Set master brightness based on acceleration.

  fill_solid(Strip, NUM_LEDS * 2, CHSV(color, 255, 255));
}

void RunExit3()
{
  fill_solid(Strip, NUM_LEDS * 2, CRGB::Black);
  FastLED.setBrightness(BRIGHTNESS);
}

void RunEntry4()
{
  fill_solid(Strip, NUM_LEDS * 2, CRGB::Black);
  currentPattern = 4;
  batteryViewActive = false;
}

void running4()
{
  color = map(color, -180, 180, 0, 255);
  uint8_t pos = map(beat16(40, 0), 0, 65535, 0, NUM_LEDS - 1);
  if (getPatternDirectionFactor(4) < 0)
  {
    pos = (uint8_t)((NUM_LEDS - 1) - pos);
  }
  Strip[pos] = CHSV(color, 200, 255);
  Strip[pos + NUM_LEDS] = CHSV(color, 200, 255);

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
  color = map(color, -180, 180, 0, 255);
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

    Strip[ledeffect] = CHSV(color, 255, 255);
    Strip[ledeffect + NUM_LEDS] = CHSV(color, 255, 255);

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
  color = map(color, -180, 180, 0, 255);
  color2 = map(color2, 180, -180, 0, 255);
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

    Strip[ledeffect] = CHSV(color, 255, 255);
    Strip[ledeffect2] = CHSV(color2, 255, 255);
    Strip[ledeffect + NUM_LEDS] = CHSV(color, 255, 255);
    Strip[ledeffect2 + NUM_LEDS] = CHSV(color2, 255, 255);

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
  color = map(color, -180, 180, 0, 255);
  bloodHue = color; // Blood color [hue from 0-255]
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
  color = map(color, -180, 180, 0, 255);
  accel = map(smoothedMotion(), 0, 10000, 20, 160);
  fade = map(accel, 20, 160, 48, 6);
  fade = constrain(fade, 6, 48);

  uint8_t pos = map(beatsin16(80, 0), 0, 65535, 0, NUM_LEDS - 1);
  if (getPatternDirectionFactor(8) < 0)
  {
    pos = (uint8_t)((NUM_LEDS - 1) - pos);
  }

  Strip[pos] = CHSV(color, 200, 255);
  Strip[pos + NUM_LEDS] = CHSV(color, 200, 255);

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

  color = map(color, -180, 180, 0, 255);
  const CRGB cometColor = CHSV(color, 200, 255);

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
    uint8_t breath = beatsin8(10, 40, 180);
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
  color = map(color, -180, 180, 0, 255);
  color2 = map(color2, 180, -180, 0, 255);
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

    Strip[ledeffect] = CHSV(color, 255, 255);
    Strip[ledeffect2] = CHSV(color2, 255, 255);
    Strip[ledeffect + NUM_LEDS] = CHSV(color, 255, 255);
    Strip[ledeffect2 + NUM_LEDS] = CHSV(color2, 255, 255);

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

  uint8_t beat = beat8(bpm);
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

  const unsigned long now = millis();
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

  const uint8_t pulseValue = beatsin8(bpm, 84, 156);
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
  static uint16_t waveA = 0;
  static uint16_t waveB = 0;
  static uint16_t waveC = 0;

  const uint32_t motion = smoothedMotion();
  const uint8_t baseHue = map((int)(ypr[0] * 180.0f / M_PI), -180, 180, 0, 255);
  const uint8_t whitecap = constrain(map((int)motion, 2000, 20000, 24, 110), 16, 120);

  waveA += 10 + constrain(map((int)motion, 2000, 20000, 0, 18), 0, 20);
  waveB += 7 + constrain(map((int)motion, 2000, 20000, 0, 12), 0, 14);
  waveC += 4 + constrain(map((int)motion, 2000, 20000, 0, 8), 0, 10);

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
  static uint16_t noiseTime = 0;
  static float noiseTimeAccumulator = 0.0f;
  static uint8_t smoothedHue = 0;
  static CRGB frameBuffer[MAX_TOTAL_LEDS];
  const uint32_t motion = smoothedMotion();
  const uint8_t targetHue = map((int)(ypr[0] * 180.0f / M_PI), -180, 180, 0, 255);
  smoothedHue = lerp8by8(smoothedHue, targetHue, 24);

  const uint8_t scale = constrain(map((int)motion, 2000, 20000, 22, 12), 10, 28);
  const float timeStep = (float)constrain(map((int)motion, 2000, 20000, 2, 10), 1, 12) / 10.0f;
  noiseTimeAccumulator += timeStep;
  while (noiseTimeAccumulator >= 1.0f)
  {
    noiseTimeAccumulator -= 1.0f;
    ++noiseTime;
  }

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
  static uint16_t phase = 0;
  const uint32_t motion = smoothedMotion();
  const uint8_t hueBase = map((int)(ypr[0] * 180.0f / M_PI), -180, 180, 0, 255);
  const uint8_t sat = constrain(map((int)motion, 2000, 20000, 180, 255), 170, 255);
  const uint8_t waveSpeed = constrain(map((int)motion, 2000, 20000, 2, 8), 1, 10);
  phase += waveSpeed;

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
};

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

void switchToPattern(uint8_t patternId, bool activatePatternState)
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
#if I2CDEV_IMPLEMENTATION == I2CDEV_ARDUINO_WIRE
  Wire.begin();
  Wire.setClock(400000); // 400kHz I2C clock. Comment on this line if having compilation difficulties
#elif I2CDEV_IMPLEMENTATION == I2CDEV_BUILTIN_FASTWIRE
  Fastwire::setup(400, true);
#endif

  Serial.begin(115200); // USB CLI and startup diagnostics.
  Serial.setTimeout(5);
  delay(1000); // Short startup delay for recovery / serial attach.

  // Load persisted config early so the remaining setup can use stored values.
  EEPROM.begin(EEPROM_SIZE);
  Serial.println("EEPROM initialized.");
  readConfigFromEEPROM(true);

  /* Initialize device */
  Serial.println(F("Initializing I2C devices..."));
  mpu.initialize();
  pinMode(INTERRUPT_PIN, INPUT);

  /* Verify connection */
  Serial.println(F("Testing MPU6050 connection..."));
  bool mpuConnected = (mpu.testConnection() == true);
  if (!mpuConnected)
  {
    Serial.println("MPU6050 connection failed - continuing without DMP.");
  }
  else
  {
    Serial.println("MPU6050 connection successful");
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
    if (currentBootCalibrationMode == BOOT_CALIBRATION_MODE_QUICK)
    {
      runQuickCalibration(true);
    }
    else
    {
      Serial.println("Using stored offsets without boot calibration.");
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
  pinMode(LED_BUILTIN, OUTPUT);

  // Battery / USB sense setup.
  pinMode(PIN_BATTERY_ADC, INPUT);
  pinMode(PIN_USB_SENSE, INPUT);
  analogReadResolution(12);

  delay(1000); // Give the power rail and USB state a moment to settle.

  // Register both physical LED segments with FastLED.

  FastLED.addLeds<LED_TYPE, PinStrip1, COLOR_ORDER>(PhysicalStrip, 0, MAX_LEDS_PER_STRIP).setCorrection(TypicalLEDStrip);
  FastLED.addLeds<LED_TYPE, PinStrip2, COLOR_ORDER>(PhysicalStrip, MAX_LEDS_PER_STRIP, MAX_LEDS_PER_STRIP).setCorrection(TypicalLEDStrip);

  // Apply persisted global brightness.
  FastLED.setBrightness(BRIGHTNESS);

  // Initialize motion smoothing with the configured window size.
  applyConfiguredMotionSmoothing();

  applyPersistentConfig();
  patternClock.begin();
  syncEngine.begin();
  lastUpdateTime = millis();
	
// State machine init.
  fsm.add(timedTransitions, num_timed);
  fsm.add(transitions, num_transitions);
  setupCLI();

  // Start in the generic pattern state; the active pattern comes from currentPattern.
  fsm.setInitialState(&s[2]);
}

// ============================================================================
//  LOOP
// ============================================================================

void loop()
{
  const uint32_t loopStartUs = micros();
  /* Read a packet from FIFO */
  if (DMPReady && mpu.dmpGetCurrentFIFOPacket(FIFOBuffer))
  { // Get the Latest packet
    // Orientation for hue/rotation-reactive patterns.
    mpu.dmpGetQuaternion(&q, FIFOBuffer);
    mpu.dmpGetGravity(&gravity, &q);
    mpu.dmpGetYawPitchRoll(ypr, &q, &gravity);
    color = ypr[0] * 180 / M_PI;
    color2 = ypr[0] * 180 / M_PI;

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

        // Save only when something actually changed.
        if (hasUnsavedConfigChanges()) {
            // At least one value changed.
            if (usbProtocolMode == USB_PROTOCOL_HUMAN)
            {
              Serial.println("Values have changed. Saving new values to EEPROM...");
            }
            saveConfigToEEPROM(usbProtocolMode == USB_PROTOCOL_HUMAN);
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
    cliInputBuffer = "";
    cliLastInputMs = 0;
  }
  // Disable the charging view while a serial session is active.
  UsbConnected = (UsbPowerRaw == 1 && !SerialSessionActive) ? 1 : 0;

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
      currentAutoplayEnabled = isAutoplayEnabled() ? 0 : 1;
      currentPlayMode = currentAutoplayEnabled ? PLAY_MODE_AUTOPLAY : PLAY_MODE_MANUAL;
      resetAutoplayTimer();
      batteryViewLastInteractionMs = millis();
      if (usbProtocolMode == USB_PROTOCOL_MACHINE)
      {
        String fields = "autoplay=";
        fields += autoplayEnabledToString();
        emitNk4Event("autoplay_changed", fields);
      }
      else
      {
        Serial.print("INFO autoplay=");
        Serial.println(autoplayEnabledToString());
      }
    }
    else if (!UsbConnected)
    {
      switchToPattern(getNextEnabledPattern((uint8_t)currentPattern), false);
      announcePatternChange("button");
    }
  }

  // USB-only power enters the charging state unless a serial session is active.
  if (UsbConnected == 1)
  {
    fsm.trigger(usbpower);
  }

  handleCLI();
  patternClock.tick();
  syncEngine.tick();

  if (multiresponseButton.singleClick() && batteryViewActive)
  {
    BRIGHTNESS += 32; // Brightness range is 95..255 in steps of 32.
    if (BRIGHTNESS > MAX_BRIGHTNESS)
    {
      BRIGHTNESS = MIN_BRIGHTNESS; // Wrap around to the minimum brightness.
    }
    FastLED.setBrightness(BRIGHTNESS);
    currentBrightness = BRIGHTNESS;
    batteryViewLastInteractionMs = millis();
  }

  const bool autoplayPaused = batteryViewActive || UsbConnected;
  if (isAutoplayEnabled())
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
        switchToPattern(getNextEnabledPattern((uint8_t)currentPattern), false);
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
