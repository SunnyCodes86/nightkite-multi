/*
  MPU6050 DMP6

  Digital Motion Processor or DMP performs complex motion processing tasks.
  - Fuses the data from the accel, gyro, and external magnetometer if applied,
  compensating individual sensor noise and errors.
  - Detect specific types of motion without the need to continuously monitor
  raw sensor data with a microcontroller.
  - Reduce workload on the microprocessor.
  - Output processed data such as quaternions, Euler angles, and gravity vectors.

  The code includes an auto-calibration and offsets generator tasks. Different
  output formats available.

  This code is compatible with the teapot project by using the teapot output format.

  Circuit: In addition to connection 3.3v, GND, SDA, and SCL, this sketch
  depends on the MPU6050's INT pin being connected to the Arduino's
  external interrupt #0 pin.

  The teapot processing example may be broken due FIFO structure change if using DMP
  6.12 firmware version.

  Find the full MPU6050 library documentation here:
  https://github.com/ElectronicCats/mpu6050/wiki

*/

// ============================================================================
//  INCLUDES
// ============================================================================

#include <Arduino.h> //Arduino.h
#include "MPU6050_6Axis_MotionApps612.h" // mpu5060 library
#include <FastLED.h> // FastLED
#include "SimpleFSM.h" // State Machine
#include "avdweb_Switch.h" // Button library
#include <Smoothed.h> // Smoothing library
#include <EEPROM.h> //EEPROM Library
#include <SimpleCLI.h> // CLI library
#include <math.h> // Math library
#include "hardware/watchdog.h" // RP2040 reset helper

// ============================================================================
//  OUTPUT FORMAT SELECTION
// ============================================================================

/* OUTPUT FORMAT DEFINITION-------------------------------------------------------------------------------------------
- Use "OUTPUT_READABLE_QUATERNION" for quaternion commponents in [w, x, y, z] format. Quaternion does not
suffer from gimbal lock problems but is harder to parse or process efficiently on a remote host or software
environment like Processing.

- Use "OUTPUT_READABLE_EULER" for Euler angles (in degrees) output, calculated from the quaternions coming
from the FIFO. EULER ANGLES SUFFER FROM GIMBAL LOCK PROBLEM.

- Use "OUTPUT_READABLE_YAWPITCHROLL" for yaw/pitch/roll angles (in degrees) calculated from the quaternions
coming from the FIFO. THIS REQUIRES GRAVITY VECTOR CALCULATION.
YAW/PITCH/ROLL ANGLES SUFFER FROM GIMBAL LOCK PROBLEM.

- Use "OUTPUT_READABLE_REALACCEL" for acceleration components with gravity removed. The accel reference frame
is not compensated for orientation. +X will always be +X according to the sensor.

- Use "OUTPUT_READABLE_WORLDACCEL" for acceleration components with gravity removed and adjusted for the world
reference frame. Yaw is relative if there is no magnetometer present.

-  Use "OUTPUT_TEAPOT" for output that matches the InvenSense teapot demo.
-------------------------------------------------------------------------------------------------------------------------------*/
#define OUTPUT_READABLE_YAWPITCHROLL
// #define OUTPUT_READABLE_QUATERNION
// #define OUTPUT_READABLE_EULER
// #define OUTPUT_READABLE_REALACCEL
#define OUTPUT_READABLE_WORLDACCEL
// #define OUTPUT_TEAPOT

// ============================================================================
//  HARDWARE & LED CONFIG
// ============================================================================

int const INTERRUPT_PIN = 3; // Define the interruption #0 pin

#define PinStrip1 12
#define PinStrip2 13
// #define CLK_PIN   4
#define LED_TYPE WS2811
#define COLOR_ORDER GRB

#define NUM_LEDS 25
#define TOTAL_LEDS 50 // both strips lenght
#define HALF_LEDS (TOTAL_LEDS / 2) // segment lenght
#define NUM_COMETS 4        // comet count
#define NUM_COMETS2 2        // comet count2

// CRGB Strip1[NUM_LEDS];
// CRGB Strip2[NUM_LEDS];
CRGB Strip[NUM_LEDS * 2];

int BRIGHTNESS = 95;
#define MAX_BRIGHTNESS 255
#define FRAMES_PER_SECOND 120

uint8_t gHue = 0; // rotating "base color" used by many of the patterns

// ============================================================================
//  CONFIGURATION & EEPROM
// ============================================================================

const uint8_t CONFIG_VERSION = 5;
const uint8_t PATTERN_MIN = 2;
const uint8_t PATTERN_MAX = 14;
const uint8_t PATTERN_COUNT = PATTERN_MAX - PATTERN_MIN + 1;
const uint8_t PATTERN_USER_MIN = 1;
const uint8_t PATTERN_USER_MAX = PATTERN_USER_MIN + PATTERN_COUNT - 1;
const uint8_t BRIGHTNESS_LEVEL_VALUES[] = {95, 127, 159, 191, 223, 255};
const uint8_t BRIGHTNESS_LEVEL_COUNT = sizeof(BRIGHTNESS_LEVEL_VALUES) / sizeof(BRIGHTNESS_LEVEL_VALUES[0]);
const uint16_t BATTERY_DISPLAY_MIN_MS = 1000;
const uint16_t BATTERY_DISPLAY_MAX_MS = 15000;

struct LedEffectConfig
{
  uint8_t bloodHue;
  uint8_t bloodSat;
  int8_t flowDirection;
  uint16_t cycleLength;
  uint16_t pulseLength;
  uint16_t pulseOffset;
  uint8_t baseBrightness;
};

struct MotionEffectConfig
{
  uint8_t running9TailLength;
  uint16_t running11JerkThreshold;
  uint8_t running11WaveSpacing;
  uint16_t running12DeadbandMilli;
};

struct PatternIndicatorConfig
{
  uint8_t enabled;
  uint8_t blink;
  uint16_t durationMs;
  uint8_t maxLeds;
  uint8_t hue;
  uint8_t displayMode;
  uint8_t dynamicDuration;
};

struct DeviceConfig
{
  uint8_t version;
  uint8_t currentPattern;
  uint8_t brightness;
  uint8_t smoothingSamples;
  uint8_t accelRange;
  uint8_t gyroRange;
  LedEffectConfig led;
  MotionEffectConfig motion;
  PatternIndicatorConfig indicator;
  uint16_t batteryDisplayMs;
  uint8_t brightnessBatteryOnly;
  uint8_t patternEnabled[PATTERN_COUNT];
};

const DeviceConfig DEFAULT_CONFIG = {
    CONFIG_VERSION,
    PATTERN_MIN,
    95,
    100,
    2,
    2,
    {96, 255, -1, 1300, 200, 250, 0},
    {3, 4000, 24, 20},
    {1, 1, 2000, NUM_LEDS, 160, 2, 0},
    5000,
    0,
    {1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1}};

DeviceConfig deviceConfig = DEFAULT_CONFIG;
DeviceConfig lastSavedConfig = DEFAULT_CONFIG;

constexpr size_t EEPROM_SIZE = sizeof(DeviceConfig);

int currentPattern = PATTERN_MIN;
int currentBrightness = 95;
bool configDirty = false;

const char *const PATTERN_NAMES[PATTERN_COUNT] = {
    "running",
    "running2",
    "running3",
    "running4",
    "running5",
    "running6",
    "running7",
    "running8",
    "running9",
    "running10",
    "running11",
    "running12",
    "running13"};

const uint16_t ACCEL_RANGE_G[4] = {2, 4, 8, 16};
const uint16_t GYRO_RANGE_DPS[4] = {250, 500, 1000, 2000};
const uint8_t INDICATOR_MODE_BLINK = 0;
const uint8_t INDICATOR_MODE_COUNT = 1;
const uint8_t INDICATOR_MODE_BOTH = 2;

// for timekeeping
unsigned long lastUpdateTime = 0;
bool indicatorShownForCurrentPattern = false;
const unsigned long UPDATE_INTERVAL = 5 * 60 * 1000; // 5 minutes in milliseconds (5 * 60 * 1000)
//const unsigned long UPDATE_INTERVAL = 30 * 1000; // 30 sec for debug

// ============================================================================
//  MPU6050 & MOTION STATE
// ============================================================================
/* MPU6050 default I2C address is 0x68*/
MPU6050 mpu;
// MPU6050 mpu(0x69); //Use for AD0 high
// MPU6050 mpu(0x68, &Wire1); //Use for AD0 low, but 2nd Wire (TWI/I2C) object.

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
float euler[3];      // [psi, theta, phi]    Euler angle container
float ypr[3];        // [yaw, pitch, roll]   Yaw/Pitch/Roll container and gravity vector

/*-Packet structure for InvenSense teapot demo-*/
uint8_t teapotPacket[14] = {'$', 0x02, 0, 0, 0, 0, 0, 0, 0, 0, 0x00, 0x00, '\r', '\n'};

volatile bool MPUInterrupt = false; // Indicates whether MPU6050 interrupt pin has gone high
/*------Interrupt detection routine------*/
void DMPDataReady(){
  MPUInterrupt = true;
}

// ============================================================================
//  BUTTON, BATTERY, TIMING
// ============================================================================

// Button config
const byte multiresponseButtonpin = 23;
Switch multiresponseButton = Switch(multiresponseButtonpin, INPUT);

// Battery & connection variables
int UsbConnected = 0;
bool serialTerminalActive = false;
int RawVoltage = 0;
float Voltage = 0;

unsigned long previousMillis = 0; // will store last time LED was updated
unsigned long currentMillis = 0;
bool blinkState;
bool blink;
bool batteryViewActive = false;

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

// variables for mpu to led mapping
int color;
int color2;
int accel;
uint8_t accelcon;
int accelabs;
int fade;

// ============================================================================
//  FSM INSTANCE
// ============================================================================
// FSM instance init
SimpleFSM fsm;

enum triggers
{
  doubleClick = 1,
  longpress,
  usbpower
};

// CLI instance
SimpleCLI cli;

// Forward declarations for CLI callbacks
void handlePatternList(cmd *c);
void handlePatternDisable(cmd *c);
void handlePatternEnable(cmd *c);
void handleSetAccel(cmd *c);
void handleSetGyro(cmd *c);
void handleSetSmoothing(cmd *c);
void handleLedParam(cmd *c);
void handleConfigSave(cmd *c);
void handleConfigShow(cmd *c);
void handleResetCommand(cmd *c);
void handlePatternIndicator(cmd *c);
void handlePatternIndicatorParam(cmd *c);
void handleConfigReset(cmd *c);
void handleBrightnessMode(cmd *c);
void handleBatteryDisplayDuration(cmd *c);

// Forward declarations for configuration helpers
void applyLedConfig();
void applySensorConfig();
void applySmoothingConfig();
float getRunning12Deadband();
void loadConfig();
void saveConfig(bool announce = true);
void resetConfigToDefaults();
void ensureValidPattern();
bool isPatternEnabled(uint8_t patternId);
void setPatternEnabled(uint8_t patternId, bool enabled);
void markConfigDirty();
void advanceToNextEnabledPattern();
int patternIdFromString(const String &value);
bool processPatternArgumentList(const String &input, bool enable);
bool processPatternToken(const String &token, bool enable, bool &anyToken);
void printPatternStatus();
void printConfig();
void setCurrentPatternValue(uint8_t patternId);
void printConfigDiff(const DeviceConfig &oldCfg, const DeviceConfig &newCfg);
void displayPatternIndicator(uint8_t patternId);
bool parseOnOff(const String &value, bool &result);
void setBatteryViewActive(bool active);
uint8_t brightnessLevelFromValue(uint8_t value);
void applyBatteryDisplayDuration();

// ============================================================================
//  HELPERS
// ============================================================================

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
  // time_shift = 0;  //Uncomment to heart beat/pulse all LEDs together
  int pulse1 = pulseWave8(millis() + time_shift, cycleLength, pulseLength);
  int pulse2 = pulseWave8(millis() + time_shift + pulseOffset, cycleLength, pulseLength);
  return qadd8(pulse1, pulse2); // Add pulses together without overflow
}

inline void logSerialMessage(const __FlashStringHelper *message)
{
  if (serialTerminalActive)
  {
    Serial.println(message);
  }
}

inline void logSerialValue(const __FlashStringHelper *prefix, long value)
{
  if (!serialTerminalActive)
    return;
  Serial.print(prefix);
  Serial.println(value);
}

inline uint32_t smoothedMotion() {
  uint32_t s = (uint32_t)abs(aaWorld.x) + (uint32_t)abs(aaWorld.y);
  myAccel.add(s);
  return myAccel.get();
}

inline bool shouldShowChargingAnimation()
{
  return (UsbConnected == 1) && !serialTerminalActive;
}

void setBatteryViewActive(bool active)
{
  batteryViewActive = active;
}

uint8_t brightnessLevelFromValue(uint8_t value)
{
  if (value <= BRIGHTNESS_LEVEL_VALUES[0])
  {
    return 1;
  }
  for (uint8_t i = 1; i < BRIGHTNESS_LEVEL_COUNT; ++i)
  {
    if (value <= BRIGHTNESS_LEVEL_VALUES[i])
    {
      return i + 1;
    }
  }
  return BRIGHTNESS_LEVEL_COUNT;
}


inline uint8_t userPatternIdFromInternal(uint8_t internalPatternId)
{
  if (internalPatternId < PATTERN_MIN || internalPatternId > PATTERN_MAX)
  {
    return 0;
  }
  return PATTERN_USER_MIN + (internalPatternId - PATTERN_MIN);
}

inline int internalPatternIdFromUser(int userPatternId)
{
  if (userPatternId < PATTERN_USER_MIN || userPatternId > PATTERN_USER_MAX)
  {
    return -1;
  }
  return PATTERN_MIN + (userPatternId - PATTERN_USER_MIN);
}

// ============================================================================
//  CONFIG HELPERS
// ============================================================================

bool configsEqual(const DeviceConfig &a, const DeviceConfig &b)
{
  if (a.version != b.version || a.currentPattern != b.currentPattern || a.brightness != b.brightness || a.smoothingSamples != b.smoothingSamples || a.accelRange != b.accelRange || a.gyroRange != b.gyroRange)
  {
    return false;
  }
  if (a.led.bloodHue != b.led.bloodHue || a.led.bloodSat != b.led.bloodSat || a.led.flowDirection != b.led.flowDirection ||
      a.led.cycleLength != b.led.cycleLength || a.led.pulseLength != b.led.pulseLength || a.led.pulseOffset != b.led.pulseOffset ||
      a.led.baseBrightness != b.led.baseBrightness)
  {
    return false;
  }
  if (a.motion.running9TailLength != b.motion.running9TailLength ||
      a.motion.running11JerkThreshold != b.motion.running11JerkThreshold ||
      a.motion.running11WaveSpacing != b.motion.running11WaveSpacing ||
      a.motion.running12DeadbandMilli != b.motion.running12DeadbandMilli)
  {
    return false;
  }
  if (a.indicator.enabled != b.indicator.enabled ||
      a.indicator.blink != b.indicator.blink ||
      a.indicator.durationMs != b.indicator.durationMs ||
      a.indicator.maxLeds != b.indicator.maxLeds ||
      a.indicator.hue != b.indicator.hue ||
      a.indicator.displayMode != b.indicator.displayMode ||
      a.indicator.dynamicDuration != b.indicator.dynamicDuration)
  {
    return false;
  }
  if (a.batteryDisplayMs != b.batteryDisplayMs)
  {
    return false;
  }
  if (a.brightnessBatteryOnly != b.brightnessBatteryOnly)
  {
    return false;
  }
  for (uint8_t i = 0; i < PATTERN_COUNT; ++i)
  {
    if (a.patternEnabled[i] != b.patternEnabled[i])
    {
      return false;
    }
  }
  return true;
}

void markConfigDirty()
{
  configDirty = true;
}

bool isPatternEnabled(uint8_t patternId)
{
  if (patternId < PATTERN_MIN || patternId > PATTERN_MAX)
    return false;
  return deviceConfig.patternEnabled[patternId - PATTERN_MIN] != 0;
}

uint8_t findFirstEnabledPattern()
{
  for (uint8_t i = 0; i < PATTERN_COUNT; ++i)
  {
    if (isPatternEnabled(PATTERN_MIN + i))
    {
      return PATTERN_MIN + i;
    }
  }
  return PATTERN_MIN;
}

uint8_t countEnabledPatterns()
{
  uint8_t count = 0;
  for (uint8_t i = 0; i < PATTERN_COUNT; ++i)
  {
    if (deviceConfig.patternEnabled[i])
    {
      ++count;
    }
  }
  return count;
}

void ensureValidPattern()
{
  uint8_t enabledCount = countEnabledPatterns();

  if (enabledCount == 0)
  {
    Serial.println(F("No pattern enabled. Re-enabling all patterns."));
    for (uint8_t i = 0; i < PATTERN_COUNT; ++i)
    {
      deviceConfig.patternEnabled[i] = 1;
    }
    markConfigDirty();
  }

  if (deviceConfig.currentPattern < PATTERN_MIN || deviceConfig.currentPattern > PATTERN_MAX)
  {
    deviceConfig.currentPattern = PATTERN_MIN;
  }

  if (!isPatternEnabled(deviceConfig.currentPattern))
  {
    uint8_t nextPattern = findFirstEnabledPattern();
    deviceConfig.currentPattern = nextPattern;
    Serial.print(F("Switching to enabled pattern "));
    Serial.println(userPatternIdFromInternal(nextPattern));
    markConfigDirty();
  }

  currentPattern = deviceConfig.currentPattern;
}

void setCurrentPatternValue(uint8_t patternId)
{
  if (patternId < PATTERN_MIN || patternId > PATTERN_MAX)
  {
    patternId = PATTERN_MIN;
  }
  uint8_t previousPattern = currentPattern;
  currentPattern = patternId;
  if (serialTerminalActive && previousPattern != currentPattern)
  {
    Serial.print(F("Pattern -> "));
    Serial.print(userPatternIdFromInternal(currentPattern));
    if (currentPattern >= PATTERN_MIN && currentPattern <= PATTERN_MAX)
    {
      uint8_t idx = currentPattern - PATTERN_MIN;
      if (idx < PATTERN_COUNT)
      {
        Serial.print(F(" ("));
        Serial.print(PATTERN_NAMES[idx]);
        Serial.print(F(")"));
      }
    }
    Serial.println();
  }
  if (deviceConfig.currentPattern != patternId)
  {
    deviceConfig.currentPattern = patternId;
    markConfigDirty();
  }
  if (previousPattern != currentPattern)
  {
    indicatorShownForCurrentPattern = false;
  }
}

void setPatternEnabled(uint8_t patternId, bool enabled)
{
  if (patternId < PATTERN_MIN || patternId > PATTERN_MAX)
    return;
  uint8_t index = patternId - PATTERN_MIN;
  uint8_t desired = enabled ? 1 : 0;
  if (deviceConfig.patternEnabled[index] == desired)
    return;
  if (!enabled && deviceConfig.patternEnabled[index] && countEnabledPatterns() <= 1)
  {
    Serial.println(F("At least one pattern must remain enabled."));
    return;
  }
  deviceConfig.patternEnabled[index] = desired;
  Serial.print(F("Pattern "));
  Serial.print(userPatternIdFromInternal(patternId));
  Serial.println(enabled ? F(" enabled.") : F(" disabled."));
  markConfigDirty();
  ensureValidPattern();
}

void applyLedConfig()
{
  bloodHue = deviceConfig.led.bloodHue;
  bloodSat = deviceConfig.led.bloodSat;
  flowDirection = deviceConfig.led.flowDirection;
  cycleLength = deviceConfig.led.cycleLength;
  pulseLength = deviceConfig.led.pulseLength;
  pulseOffset = deviceConfig.led.pulseOffset;
  baseBrightness = deviceConfig.led.baseBrightness;
}

void loadConfig()
{
  EEPROM.get(0, deviceConfig);
  if (deviceConfig.version != CONFIG_VERSION)
  {
    Serial.println(F("Config version mismatch. Loading defaults."));
    deviceConfig = DEFAULT_CONFIG;
    markConfigDirty();
  }

  if (deviceConfig.brightness < 95 || deviceConfig.brightness > MAX_BRIGHTNESS)
  {
    deviceConfig.brightness = 95;
    markConfigDirty();
  }

  if (deviceConfig.smoothingSamples < 1 || deviceConfig.smoothingSamples > 100)
  {
    deviceConfig.smoothingSamples = 100;
    markConfigDirty();
  }

  if (deviceConfig.accelRange > 3)
  {
    deviceConfig.accelRange = 2;
    markConfigDirty();
  }

  if (deviceConfig.gyroRange > 3)
  {
    deviceConfig.gyroRange = 2;
    markConfigDirty();
  }

  if (deviceConfig.motion.running9TailLength < 1 || deviceConfig.motion.running9TailLength > 15)
  {
    deviceConfig.motion.running9TailLength = DEFAULT_CONFIG.motion.running9TailLength;
    markConfigDirty();
  }

  if (deviceConfig.motion.running11JerkThreshold < 500 || deviceConfig.motion.running11JerkThreshold > 20000)
  {
    deviceConfig.motion.running11JerkThreshold = DEFAULT_CONFIG.motion.running11JerkThreshold;
    markConfigDirty();
  }

  if (deviceConfig.motion.running11WaveSpacing < 4 || deviceConfig.motion.running11WaveSpacing > 80)
  {
    deviceConfig.motion.running11WaveSpacing = DEFAULT_CONFIG.motion.running11WaveSpacing;
    markConfigDirty();
  }

  if (deviceConfig.motion.running12DeadbandMilli < 1 || deviceConfig.motion.running12DeadbandMilli > 200)
  {
    deviceConfig.motion.running12DeadbandMilli = DEFAULT_CONFIG.motion.running12DeadbandMilli;
    markConfigDirty();
  }

  if (deviceConfig.indicator.enabled > 1)
  {
    deviceConfig.indicator.enabled = 1;
    markConfigDirty();
  }
  if (deviceConfig.indicator.blink > 1)
  {
    deviceConfig.indicator.blink = 1;
    markConfigDirty();
  }
  if (deviceConfig.indicator.durationMs < 200 || deviceConfig.indicator.durationMs > 5000)
  {
    deviceConfig.indicator.durationMs = DEFAULT_CONFIG.indicator.durationMs;
    markConfigDirty();
  }
  if (deviceConfig.indicator.maxLeds < 1 || deviceConfig.indicator.maxLeds > NUM_LEDS)
  {
    deviceConfig.indicator.maxLeds = DEFAULT_CONFIG.indicator.maxLeds;
    markConfigDirty();
  }
  if (deviceConfig.indicator.displayMode > 2)
  {
    deviceConfig.indicator.displayMode = DEFAULT_CONFIG.indicator.displayMode;
    markConfigDirty();
  }
  if (deviceConfig.indicator.dynamicDuration > 1)
  {
    deviceConfig.indicator.dynamicDuration = DEFAULT_CONFIG.indicator.dynamicDuration;
    markConfigDirty();
  }
  if (deviceConfig.batteryDisplayMs < BATTERY_DISPLAY_MIN_MS || deviceConfig.batteryDisplayMs > BATTERY_DISPLAY_MAX_MS)
  {
    deviceConfig.batteryDisplayMs = DEFAULT_CONFIG.batteryDisplayMs;
    markConfigDirty();
  }
  if (deviceConfig.brightnessBatteryOnly > 1)
  {
    deviceConfig.brightnessBatteryOnly = DEFAULT_CONFIG.brightnessBatteryOnly;
    markConfigDirty();
  }
  // hue inherently 0-255; no change needed

  applyBatteryDisplayDuration();
  ensureValidPattern();

  currentPattern = deviceConfig.currentPattern;
  currentBrightness = deviceConfig.brightness;
  lastSavedConfig = deviceConfig;
  configDirty = false;
}

void saveConfig(bool announce)
{
  if (configsEqual(deviceConfig, lastSavedConfig))
  {
    Serial.println(F("No configuration changes detected."));
    configDirty = false;
    return;
  }

  if (announce)
  {
    Serial.println(F("Saving configuration..."));
  }
  printConfigDiff(lastSavedConfig, deviceConfig);

  deviceConfig.version = CONFIG_VERSION;
  EEPROM.put(0, deviceConfig);
  if (EEPROM.commit())
  {
    Serial.println(F("Configuration saved to EEPROM."));
    lastSavedConfig = deviceConfig;
    configDirty = false;
  }
  else
  {
    Serial.println(F("ERROR: Failed to save configuration."));
  }
}

void applySensorConfig()
{
  uint8_t accelIndex = constrain(deviceConfig.accelRange, (uint8_t)0, (uint8_t)3);
  uint8_t gyroIndex = constrain(deviceConfig.gyroRange, (uint8_t)0, (uint8_t)3);
  mpu.setFullScaleAccelRange(accelIndex);
  mpu.setFullScaleGyroRange(gyroIndex);
  Serial.print(F("Accel range set to ±"));
  Serial.print(ACCEL_RANGE_G[accelIndex]);
  Serial.println(F("g"));
  Serial.print(F("Gyro range set to ±"));
  Serial.print(GYRO_RANGE_DPS[gyroIndex]);
  Serial.println(F("°/s"));
}

void applySmoothingConfig()
{
  uint8_t samples = constrain(deviceConfig.smoothingSamples, (uint8_t)1, (uint8_t)100);
  if (deviceConfig.smoothingSamples != samples)
  {
    deviceConfig.smoothingSamples = samples;
    markConfigDirty();
  }
  myAccel.begin(SMOOTHED_AVERAGE, samples);
  myAccel.clear();
  Serial.print(F("Smoothing samples set to "));
  Serial.println(samples);
}

float getRunning12Deadband()
{
  return deviceConfig.motion.running12DeadbandMilli / 1000.0f;
}

int patternIdFromString(const String &value)
{
  if (value.length() == 0)
  {
    return -1;
  }
  bool numeric = true;
  for (uint16_t i = 0; i < value.length(); ++i)
  {
    if (!isDigit(value.charAt(i)))
    {
      numeric = false;
      break;
    }
  }
  if (numeric)
  {
    int userId = value.toInt();
    int internalId = internalPatternIdFromUser(userId);
    if (internalId >= PATTERN_MIN && internalId <= PATTERN_MAX)
    {
      return internalId;
    }
  }

  for (uint8_t i = 0; i < PATTERN_COUNT; ++i)
  {
    if (value.equalsIgnoreCase(String(PATTERN_NAMES[i])))
    {
      return PATTERN_MIN + i;
    }
  }
  return -1;
}

bool processPatternToken(const String &token, bool enable, bool &anyToken)
{
  String trimmed = token;
  trimmed.trim();
  if (trimmed.length() == 0)
  {
    return true;
  }
  int patternId = patternIdFromString(trimmed);
  if (patternId < 0)
  {
    Serial.print(F("Unknown pattern: "));
    Serial.println(trimmed);
    return false;
  }
  setPatternEnabled(patternId, enable);
  anyToken = true;
  return true;
}

bool processPatternArgumentList(const String &input, bool enable)
{
  String data = input;
  data.trim();
  if (data.length() == 0)
  {
    Serial.println(F("Please specify at least one pattern."));
    return false;
  }
  bool anyToken = false;
  String token;
  for (uint16_t i = 0; i < data.length(); ++i)
  {
    char ch = data.charAt(i);
    bool delimiter = (ch == ',' || ch == ';' || ch == ' ' || ch == '\t' || ch == '\n' || ch == '\r');
    if (delimiter)
    {
      if (!processPatternToken(token, enable, anyToken))
      {
        return false;
      }
      token = "";
    }
    else
    {
      token += ch;
    }
  }
  if (!processPatternToken(token, enable, anyToken))
  {
    return false;
  }
  if (!anyToken)
  {
    Serial.println(F("No valid patterns specified."));
    return false;
  }
  return true;
}

int parseRangeArgument(const String &value, const uint16_t *allowed, size_t count)
{
  String normalized = value;
  normalized.trim();
  normalized.toLowerCase();
  if (normalized.length() == 0)
  {
    return -1;
  }

  bool hasDigits = false;
  for (uint16_t i = 0; i < normalized.length(); ++i)
  {
    if (isDigit(normalized.charAt(i)))
    {
      hasDigits = true;
      break;
    }
  }

  if (!hasDigits)
  {
    return -1;
  }

  long number = normalized.toInt();

  for (size_t i = 0; i < count; ++i)
  {
    if (number == allowed[i])
    {
      return static_cast<int>(i);
    }
  }

  if (number >= 0 && number < static_cast<long>(count))
  {
    return static_cast<int>(number);
  }

  return -1;
}

bool parseOnOff(const String &value, bool &result)
{
  String normalized = value;
  normalized.trim();
  normalized.toLowerCase();
  if (normalized == "on" || normalized == "1" || normalized == "true")
  {
    result = true;
    return true;
  }
  if (normalized == "off" || normalized == "0" || normalized == "false")
  {
    result = false;
    return true;
  }
  return false;
}

void printPatternStatus()
{
  Serial.println(F("Pattern status:"));
  for (uint8_t i = 0; i < PATTERN_COUNT; ++i)
  {
    uint8_t patternId = PATTERN_MIN + i;
    Serial.print(F("  "));
    Serial.print(userPatternIdFromInternal(patternId));
    Serial.print(F(" ("));
    Serial.print(PATTERN_NAMES[i]);
    Serial.print(F("): "));
    Serial.print(isPatternEnabled(patternId) ? F("active") : F("inactive"));
    if (patternId == currentPattern)
    {
      Serial.print(F(" [current]"));
    }
    Serial.println();
  }
}

void printConfig()
{
  Serial.println(F("---- Current Configuration ----"));
  Serial.print(F("Pattern: "));
  Serial.println(currentPattern);
  Serial.print(F("Brightness: "));
  Serial.println(currentBrightness);
  Serial.print(F("Smoothing samples: "));
  Serial.println(deviceConfig.smoothingSamples);
  Serial.print(F("Accel range: ±"));
  Serial.print(ACCEL_RANGE_G[deviceConfig.accelRange]);
  Serial.print(F("g (index "));
  Serial.print(deviceConfig.accelRange);
  Serial.println(')');
  Serial.print(F("Gyro range: ±"));
  Serial.print(GYRO_RANGE_DPS[deviceConfig.gyroRange]);
  Serial.print(F("°/s (index "));
  Serial.print(deviceConfig.gyroRange);
  Serial.println(')');
  Serial.print(F("Led bloodHue: "));
  Serial.println(deviceConfig.led.bloodHue);
  Serial.print(F("Led bloodSat: "));
  Serial.println(deviceConfig.led.bloodSat);
  Serial.print(F("Led flowDirection: "));
  Serial.println(deviceConfig.led.flowDirection);
  Serial.print(F("Led cycleLength: "));
  Serial.println(deviceConfig.led.cycleLength);
  Serial.print(F("Led pulseLength: "));
  Serial.println(deviceConfig.led.pulseLength);
  Serial.print(F("Led pulseOffset: "));
  Serial.println(deviceConfig.led.pulseOffset);
  Serial.print(F("Led baseBrightness: "));
  Serial.println(deviceConfig.led.baseBrightness);
  Serial.print(F("Running9 tail length: "));
  Serial.println(deviceConfig.motion.running9TailLength);
  Serial.print(F("Running11 jerk threshold: "));
  Serial.println(deviceConfig.motion.running11JerkThreshold);
  Serial.print(F("Running11 wave spacing: "));
  Serial.println(deviceConfig.motion.running11WaveSpacing);
  Serial.print(F("Running12 deadband: "));
  Serial.print(getRunning12Deadband(), 3);
  Serial.println();
  Serial.print(F("Pattern indicator: "));
  Serial.println(deviceConfig.indicator.enabled ? F("enabled") : F("disabled"));
  Serial.print(F("Indicator blink: "));
  Serial.println(deviceConfig.indicator.blink ? F("yes") : F("no"));
  Serial.print(F("Indicator duration (ms): "));
  Serial.println(deviceConfig.indicator.durationMs);
  Serial.print(F("Indicator max LEDs: "));
  Serial.println(deviceConfig.indicator.maxLeds);
  Serial.print(F("Indicator hue: "));
  Serial.println(deviceConfig.indicator.hue);
  Serial.print(F("Indicator mode: "));
  Serial.println(deviceConfig.indicator.displayMode);
  Serial.print(F("Indicator dynamic duration: "));
  Serial.println(deviceConfig.indicator.dynamicDuration ? F("yes") : F("no"));
  Serial.print(F("Battery display duration (ms): "));
  Serial.println(deviceConfig.batteryDisplayMs);
  Serial.print(F("Brightness only in battery view: "));
  Serial.println(deviceConfig.brightnessBatteryOnly ? F("yes") : F("no"));
  printPatternStatus();
  Serial.println(F("--------------------------------"));
}

void printScalarChange(const char *label, long oldVal, long newVal, bool &anyChange)
{
  if (oldVal != newVal)
  {
    Serial.print(F("  "));
    Serial.print(label);
    Serial.print(F(": "));
    Serial.print(oldVal);
    Serial.print(F(" -> "));
    Serial.println(newVal);
    anyChange = true;
  }
}

void printFloatChange(const char *label, float oldVal, float newVal, bool &anyChange, uint8_t precision = 3)
{
  if (fabsf(oldVal - newVal) > 0.0001f)
  {
    Serial.print(F("  "));
    Serial.print(label);
    Serial.print(F(": "));
    Serial.print(oldVal, precision);
    Serial.print(F(" -> "));
    Serial.println(newVal, precision);
    anyChange = true;
  }
}

void printConfigDiff(const DeviceConfig &oldCfg, const DeviceConfig &newCfg)
{
  bool anyChange = false;
  Serial.println(F("Changed values since last save:"));
  printScalarChange("Pattern ID", userPatternIdFromInternal(oldCfg.currentPattern), userPatternIdFromInternal(newCfg.currentPattern), anyChange);
  printScalarChange("Brightness", oldCfg.brightness, newCfg.brightness, anyChange);
  printScalarChange("Smoothing samples", oldCfg.smoothingSamples, newCfg.smoothingSamples, anyChange);
  printScalarChange("Accel range index", oldCfg.accelRange, newCfg.accelRange, anyChange);
  printScalarChange("Gyro range index", oldCfg.gyroRange, newCfg.gyroRange, anyChange);

  printScalarChange("bloodHue", oldCfg.led.bloodHue, newCfg.led.bloodHue, anyChange);
  printScalarChange("bloodSat", oldCfg.led.bloodSat, newCfg.led.bloodSat, anyChange);
  printScalarChange("flowDirection", oldCfg.led.flowDirection, newCfg.led.flowDirection, anyChange);
  printScalarChange("cycleLength", oldCfg.led.cycleLength, newCfg.led.cycleLength, anyChange);
  printScalarChange("pulseLength", oldCfg.led.pulseLength, newCfg.led.pulseLength, anyChange);
  printScalarChange("pulseOffset", oldCfg.led.pulseOffset, newCfg.led.pulseOffset, anyChange);
  printScalarChange("baseBrightness", oldCfg.led.baseBrightness, newCfg.led.baseBrightness, anyChange);

  printScalarChange("running9Tail", oldCfg.motion.running9TailLength, newCfg.motion.running9TailLength, anyChange);
  printScalarChange("running11JerkThreshold", oldCfg.motion.running11JerkThreshold, newCfg.motion.running11JerkThreshold, anyChange);
  printScalarChange("running11WaveSpacing", oldCfg.motion.running11WaveSpacing, newCfg.motion.running11WaveSpacing, anyChange);
  printFloatChange("running12Deadband", oldCfg.motion.running12DeadbandMilli / 1000.0f, newCfg.motion.running12DeadbandMilli / 1000.0f, anyChange);
  printScalarChange("Battery display (ms)", oldCfg.batteryDisplayMs, newCfg.batteryDisplayMs, anyChange);
  if (oldCfg.indicator.enabled != newCfg.indicator.enabled)
  {
    Serial.print(F("  Pattern indicator: "));
    Serial.print(oldCfg.indicator.enabled ? F("on") : F("off"));
    Serial.print(F(" -> "));
    Serial.println(newCfg.indicator.enabled ? F("on") : F("off"));
    anyChange = true;
  }
  if (oldCfg.indicator.blink != newCfg.indicator.blink)
  {
    Serial.print(F("  Indicator blink: "));
    Serial.print(oldCfg.indicator.blink ? F("on") : F("off"));
    Serial.print(F(" -> "));
    Serial.println(newCfg.indicator.blink ? F("on") : F("off"));
    anyChange = true;
  }
  printScalarChange("Indicator duration", oldCfg.indicator.durationMs, newCfg.indicator.durationMs, anyChange);
  printScalarChange("Indicator max LEDs", oldCfg.indicator.maxLeds, newCfg.indicator.maxLeds, anyChange);
  printScalarChange("Indicator hue", oldCfg.indicator.hue, newCfg.indicator.hue, anyChange);
  printScalarChange("Indicator mode", oldCfg.indicator.displayMode, newCfg.indicator.displayMode, anyChange);
  if (oldCfg.indicator.dynamicDuration != newCfg.indicator.dynamicDuration)
  {
    Serial.print(F("  Indicator dynamic: "));
    Serial.print(oldCfg.indicator.dynamicDuration ? F("on") : F("off"));
    Serial.print(F(" -> "));
    Serial.println(newCfg.indicator.dynamicDuration ? F("on") : F("off"));
    anyChange = true;
  }
  if (oldCfg.brightnessBatteryOnly != newCfg.brightnessBatteryOnly)
  {
    Serial.print(F("  Brightness battery-only: "));
    Serial.print(oldCfg.brightnessBatteryOnly ? F("on") : F("off"));
    Serial.print(F(" -> "));
    Serial.println(newCfg.brightnessBatteryOnly ? F("on") : F("off"));
    anyChange = true;
  }

  for (uint8_t i = 0; i < PATTERN_COUNT; ++i)
  {
    if (oldCfg.patternEnabled[i] != newCfg.patternEnabled[i])
    {
      Serial.print(F("  Pattern "));
      uint8_t patternId = PATTERN_MIN + i;
      Serial.print(userPatternIdFromInternal(patternId));
      Serial.print(F(" ("));
      Serial.print(PATTERN_NAMES[i]);
      Serial.print(F("): "));
      Serial.print(oldCfg.patternEnabled[i] ? F("enabled") : F("disabled"));
      Serial.print(F(" -> "));
      Serial.println(newCfg.patternEnabled[i] ? F("enabled") : F("disabled"));
      anyChange = true;
    }
  }

  if (!anyChange)
  {
    Serial.println(F("  No configuration differences detected."));
  }
}

void advanceToNextEnabledPattern()
{
  for (uint8_t i = 0; i < PATTERN_COUNT; ++i)
  {
    fsm.trigger(doubleClick);
    if (isPatternEnabled(currentPattern))
    {
      return;
    }
  }
}

void displayPatternIndicator(uint8_t patternId)
{
  if (deviceConfig.indicator.enabled == 0)
  {
    return;
  }
  if (patternId < PATTERN_MIN || patternId > PATTERN_MAX)
  {
    return;
  }
  if (!isPatternEnabled(patternId))
  {
    return;
  }
  if (patternId == deviceConfig.currentPattern && indicatorShownForCurrentPattern)
  {
    return;
  }

  const PatternIndicatorConfig &cfg = deviceConfig.indicator;
  uint8_t prevBrightness = FastLED.getBrightness();
  FastLED.setBrightness(MAX_BRIGHTNESS);

  uint8_t relativePattern = patternId - PATTERN_MIN + 1;
  uint8_t displayUnits = constrain(relativePattern, (uint8_t)1, cfg.maxLeds);
  uint32_t displayDurationMs = cfg.durationMs;
  if (cfg.dynamicDuration)
  {
    uint32_t perPattern = max<uint32_t>(cfg.durationMs / 4, 100U);
    uint32_t extra = perPattern * displayUnits;
    displayDurationMs = constrain(displayDurationMs + extra, (uint32_t)cfg.durationMs, (uint32_t)6000);
  }
  if (displayDurationMs < 200)
  {
    displayDurationMs = 200;
  }
  uint16_t blinkCycles = 0;
  uint32_t blinkPhaseDurationMs = 0;
  if (cfg.blink)
  {
    blinkCycles = max<uint16_t>(displayUnits, 1);
    blinkPhaseDurationMs = displayDurationMs / max<uint16_t>(blinkCycles * 2, 1);
    if (blinkPhaseDurationMs == 0)
    {
      blinkPhaseDurationMs = 1;
    }
  }

  auto drawFrame = [&](bool showBlink, bool blinkOn, bool showCount, bool blinkCountLeds) {
    fill_solid(Strip, NUM_LEDS * 2, CRGB::Black);
    CHSV color(cfg.hue, 230, 255);
    if (showCount)
    {
      uint8_t ledsToLight = displayUnits;
      bool countOn = true;
      if (blinkCountLeds && cfg.blink)
      {
        countOn = blinkOn;
      }
      if (countOn)
      {
        for (uint8_t i = 0; i < ledsToLight; ++i)
        {
          Strip[i] = color;
        }
      }
    }
    if (showBlink && !blinkCountLeds && (blinkOn || !cfg.blink))
    {
      Strip[NUM_LEDS - 1] = color;
      Strip[NUM_LEDS] = color;
    }
    FastLED.show();
  };

  unsigned long start = millis();
  uint32_t totalPhases = blinkCycles * 2;
  uint32_t currentPhase = 0;
  unsigned long phaseStart = start;
  while (millis() - start < displayDurationMs)
  {
    bool showBlinkPhase = false;
    bool showCountPhase = false;
    bool blinkCountLeds = cfg.displayMode == INDICATOR_MODE_BOTH;

    if (cfg.displayMode == INDICATOR_MODE_BLINK)
    {
      showBlinkPhase = true;
    }
    else if (cfg.displayMode == INDICATOR_MODE_COUNT)
    {
      showCountPhase = true;
    }
    else
    {
      showBlinkPhase = true;
      showCountPhase = true;
    }

    if (cfg.blink && blinkCycles > 0 && currentPhase < totalPhases)
    {
      unsigned long now = millis();
      if (now - phaseStart >= blinkPhaseDurationMs)
      {
        phaseStart = now;
        ++currentPhase;
      }
    }

    bool sequenceFinished = cfg.blink && blinkCycles > 0 && currentPhase >= totalPhases;
    bool blinkOn = true;
    if (showBlinkPhase && cfg.blink && blinkCycles > 0)
    {
      uint32_t activePhase = (currentPhase < totalPhases) ? currentPhase : (totalPhases - 1);
      blinkOn = (activePhase % 2) == 0;
    }

    if (sequenceFinished)
    {
      break;
    }

    drawFrame(showBlinkPhase, blinkOn, showCountPhase, blinkCountLeds);
    FastLED.delay(20);
  }

  fill_solid(Strip, NUM_LEDS * 2, CRGB::Black);
  FastLED.show();
  FastLED.setBrightness(prevBrightness);
  if (patternId == deviceConfig.currentPattern)
  {
    indicatorShownForCurrentPattern = true;
  }
}

// ============================================================================
//  CLI CALLBACKS
// ============================================================================

void handlePatternList(cmd *c)
{
  (void)c;
  printPatternStatus();
}

void handlePatternEnable(cmd *c)
{
  Command command(c);
  Argument target = command.getArgument("pattern");
  String input = target.getValue();
  if (input.equalsIgnoreCase("all"))
  {
    for (uint8_t i = PATTERN_MIN; i <= PATTERN_MAX; ++i)
    {
      setPatternEnabled(i, true);
    }
    return;
  }
  processPatternArgumentList(input, true);
}

void handlePatternDisable(cmd *c)
{
  Command command(c);
  Argument target = command.getArgument("pattern");
  String input = target.getValue();
  if (input.equalsIgnoreCase("all"))
  {
    for (uint8_t i = PATTERN_MIN; i <= PATTERN_MAX; ++i)
    {
      if (i == PATTERN_MIN)
      {
        continue;
      }
      setPatternEnabled(i, false);
    }
    return;
  }
  processPatternArgumentList(input, false);
}

void handleSetAccel(cmd *c)
{
  Command command(c);
  Argument target = command.getArgument("range");
  int idx = parseRangeArgument(target.getValue(), ACCEL_RANGE_G, 4);
  if (idx < 0)
  {
    Serial.println(F("Invalid accelerometer range. Use 0-3 or 2/4/8/16g."));
    return;
  }
  deviceConfig.accelRange = idx;
  applySensorConfig();
  markConfigDirty();
}

void handleSetGyro(cmd *c)
{
  Command command(c);
  Argument target = command.getArgument("range");
  int idx = parseRangeArgument(target.getValue(), GYRO_RANGE_DPS, 4);
  if (idx < 0)
  {
    Serial.println(F("Invalid gyro range. Use 0-3 or 250/500/1000/2000."));
    return;
  }
  deviceConfig.gyroRange = idx;
  applySensorConfig();
  markConfigDirty();
}

void handleSetSmoothing(cmd *c)
{
  Command command(c);
  Argument target = command.getArgument("samples");
  long value = target.getValue().toInt();
  value = constrain(value, 1L, 100L);
  deviceConfig.smoothingSamples = static_cast<uint8_t>(value);
  applySmoothingConfig();
  markConfigDirty();
  Serial.print(F("Smoothing window updated to "));
  Serial.println(deviceConfig.smoothingSamples);
}

void handleLedParam(cmd *c)
{
  Command command(c);
  String param = command.getArgument("name").getValue();
  String valueStr = command.getArgument("value").getValue();
  long value = valueStr.toInt();
  long responseInt = value;
  float responseFloat = 0.0f;
  bool responseIsFloat = false;
  bool handled = true;
  bool ledConfigChanged = false;

  if (param.equalsIgnoreCase("bloodHue"))
  {
    value = constrain(value, 0, 255);
    deviceConfig.led.bloodHue = value;
    responseInt = value;
    ledConfigChanged = true;
  }
  else if (param.equalsIgnoreCase("bloodSat"))
  {
    value = constrain(value, 0, 255);
    deviceConfig.led.bloodSat = value;
    responseInt = value;
    ledConfigChanged = true;
  }
  else if (param.equalsIgnoreCase("flowDirection"))
  {
    value = constrain(value, -1, 1);
    deviceConfig.led.flowDirection = value;
    responseInt = value;
    ledConfigChanged = true;
  }
  else if (param.equalsIgnoreCase("cycleLength"))
  {
    value = constrain(value, 100, 5000);
    deviceConfig.led.cycleLength = value;
    responseInt = value;
    ledConfigChanged = true;
  }
  else if (param.equalsIgnoreCase("pulseLength"))
  {
    value = constrain(value, 50, 2000);
    deviceConfig.led.pulseLength = value;
    responseInt = value;
    ledConfigChanged = true;
  }
  else if (param.equalsIgnoreCase("pulseOffset"))
  {
    value = constrain(value, 0, 2000);
    deviceConfig.led.pulseOffset = value;
    responseInt = value;
    ledConfigChanged = true;
  }
  else if (param.equalsIgnoreCase("baseBrightness"))
  {
    value = constrain(value, 0, 255);
    deviceConfig.led.baseBrightness = value;
    responseInt = value;
    ledConfigChanged = true;
  }
  else if (param.equalsIgnoreCase("running9Tail"))
  {
    value = constrain(value, 1, 15);
    deviceConfig.motion.running9TailLength = value;
    responseInt = value;
  }
  else if (param.equalsIgnoreCase("running11JerkThreshold"))
  {
    value = constrain(value, 500, 20000);
    deviceConfig.motion.running11JerkThreshold = value;
    responseInt = value;
  }
  else if (param.equalsIgnoreCase("running11WaveSpacing"))
  {
    value = constrain(value, 4, 80);
    deviceConfig.motion.running11WaveSpacing = value;
    responseInt = value;
  }
  else if (param.equalsIgnoreCase("running12Deadband"))
  {
    float deadband = valueStr.toFloat();
    if (deadband <= 0.001f)
    {
      deadband = 0.001f;
    }
    if (deadband > 0.2f)
    {
      deadband = 0.2f;
    }
    uint16_t stored = (uint16_t)lroundf(deadband * 1000.0f);
    deviceConfig.motion.running12DeadbandMilli = stored;
    responseFloat = stored / 1000.0f;
    responseIsFloat = true;
  }
  else
  {
    handled = false;
  }

  if (!handled)
  {
    Serial.println(F("Unknown LED parameter."));
    return;
  }

  if (ledConfigChanged)
  {
    applyLedConfig();
  }
  markConfigDirty();
  Serial.print(F("Updated "));
  Serial.print(param);
  Serial.print(F(" to "));
  if (responseIsFloat)
  {
    Serial.println(responseFloat, 3);
  }
  else
  {
    Serial.println(responseInt);
  }
}

void handleConfigSave(cmd *c)
{
  (void)c;
  Serial.println(F("Manual save requested."));
  saveConfig(true);
}

void handleConfigShow(cmd *c)
{
  (void)c;
  printConfig();
}

void handleResetCommand(cmd *c)
{
  (void)c;
  Serial.println(F("Resetting RP2040..."));
  Serial.flush();
  delay(50);
  watchdog_reboot(0, 0, 0);
  while (true)
  {
  }
}

void handlePatternIndicator(cmd *c)
{
  Command command(c);
  bool enabled = true;
  if (!parseOnOff(command.getArgument("state").getValue(), enabled))
  {
    Serial.println(F("Usage: pattern-indicator <on|off>"));
    return;
  }
  deviceConfig.indicator.enabled = enabled ? 1 : 0;
  markConfigDirty();
  Serial.print(F("Pattern indicator "));
  Serial.println(enabled ? F("enabled") : F("disabled"));
}

void handlePatternIndicatorParam(cmd *c)
{
  Command command(c);
  String param = command.getArgument("name").getValue();
  String valueStr = command.getArgument("value").getValue();
  long value = valueStr.toInt();
  bool handled = true;

  if (param.equalsIgnoreCase("blink"))
  {
    bool blinkValue = true;
    if (!parseOnOff(valueStr, blinkValue))
    {
      Serial.println(F("Use 'on' or 'off' for blink."));
      return;
    }
    deviceConfig.indicator.blink = blinkValue ? 1 : 0;
  }
  else if (param.equalsIgnoreCase("duration"))
  {
    value = constrain(value, 200, 5000);
    deviceConfig.indicator.durationMs = value;
  }
  else if (param.equalsIgnoreCase("maxleds"))
  {
    value = constrain(value, 1, NUM_LEDS);
    deviceConfig.indicator.maxLeds = value;
  }
  else if (param.equalsIgnoreCase("hue"))
  {
    value = constrain(value, 0, 255);
    deviceConfig.indicator.hue = value;
  }
  else if (param.equalsIgnoreCase("mode"))
  {
    int mode = -1;
    if (isDigit(valueStr.charAt(0)))
    {
      mode = constrain(value, 0, 2);
    }
    else if (valueStr.equalsIgnoreCase("blink"))
    {
      mode = INDICATOR_MODE_BLINK;
    }
    else if (valueStr.equalsIgnoreCase("count"))
    {
      mode = INDICATOR_MODE_COUNT;
    }
    else if (valueStr.equalsIgnoreCase("both"))
    {
      mode = INDICATOR_MODE_BOTH;
    }
    if (mode < 0)
    {
      Serial.println(F("Mode must be 0,1,2 or blink/count/both."));
      return;
    }
    deviceConfig.indicator.displayMode = mode;
  }
  else if (param.equalsIgnoreCase("dynamic"))
  {
    bool dyn = false;
    if (!parseOnOff(valueStr, dyn))
    {
      Serial.println(F("Use on/off for dynamic."));
      return;
    }
    deviceConfig.indicator.dynamicDuration = dyn ? 1 : 0;
  }
  else
  {
    handled = false;
  }

  if (!handled)
  {
    Serial.println(F("Unknown indicator parameter."));
    return;
  }

  markConfigDirty();
  Serial.print(F("Indicator "));
  Serial.print(param);
  Serial.print(F(" set to "));
  Serial.println(valueStr);
}

void handleBrightnessMode(cmd *c)
{
  Command command(c);
  String mode = command.getArgument("mode").getValue();
  bool batteryOnly = false;
  if (mode.equalsIgnoreCase("battery") || mode.equalsIgnoreCase("on"))
  {
    batteryOnly = true;
  }
  else if (mode.equalsIgnoreCase("anywhere") || mode.equalsIgnoreCase("off"))
  {
    batteryOnly = false;
  }
  else
  {
    Serial.println(F("Usage: brightness-mode <battery|anywhere>"));
    return;
  }
  deviceConfig.brightnessBatteryOnly = batteryOnly ? 1 : 0;
  markConfigDirty();
  Serial.print(F("Brightness adjustment "));
  Serial.println(batteryOnly ? F("limited to battery view.") : F("allowed in all modes."));
}

void handleBatteryDisplayDuration(cmd *c)
{
  Command command(c);
  Argument arg = command.getArgument("ms");
  long value = arg.getValue().toInt();
  if (value <= 0)
  {
    Serial.print(F("Usage: battery-display <milliseconds "));
    Serial.print(BATTERY_DISPLAY_MIN_MS);
    Serial.print(F("-"));
    Serial.print(BATTERY_DISPLAY_MAX_MS);
    Serial.println(F(">"));
    return;
  }
  value = constrain(value, (long)BATTERY_DISPLAY_MIN_MS, (long)BATTERY_DISPLAY_MAX_MS);
  deviceConfig.batteryDisplayMs = static_cast<uint16_t>(value);
  applyBatteryDisplayDuration();
  markConfigDirty();
  Serial.print(F("Battery display duration set to "));
  Serial.print(deviceConfig.batteryDisplayMs);
  Serial.println(F(" ms"));
}

// ============================================================================
//  FSM STATES — ENTRY/RUN/EXIT
// ============================================================================

void ChargingEntry()
{
  setBatteryViewActive(false);
  fill_solid(Strip, NUM_LEDS * 2, CRGB::Black);
  FastLED.setBrightness(32);
}

void ChargingRunning()
{
  RawVoltage = analogRead(29);
  Voltage = RawVoltage * 3.0 * 3.3 / 4096.0;

  currentMillis = millis();

  if (currentMillis - previousMillis >= 500)
  {
    // save the last time you blinked the LED
    previousMillis = currentMillis;
    // if the LED is off turn it on and vice-versa:
    if (blink == 0){
      blink = 1;
    }
    else{
      blink = 0;
    }
  }

//hysteresis for voltage measurement
  static float vLast = 0;
const float HYS = 0.03;  // 30 mV
if (fabsf(Voltage - vLast) < HYS) Voltage = vLast; else vLast = Voltage;

fill_solid(Strip, NUM_LEDS*2, CRGB::Black);

  Strip[25] = blink ? CRGB::Red : CRGB::Black;

  if      (Voltage >= 4.20) fill_solid(Strip, 5, CRGB::Blue);
  else if (Voltage >  4.00) fill_solid(Strip, 4, CRGB::Green);
  else if (Voltage >  3.80) fill_solid(Strip, 3, CRGB::Green);
  else if (Voltage >  3.60) fill_solid(Strip, 2, CRGB::Yellow);
  else if (Voltage >  3.40) fill_solid(Strip, 1, CRGB::Yellow);
  else if (Voltage >  3.20) fill_solid(Strip, 1, CRGB::Red);
  else                      {/* leave empty = very empty */}

}

void ChargingExit()
{
  FastLED.setBrightness(BRIGHTNESS);
}

void BatteryEntry()
{
  setBatteryViewActive(true);
  fill_solid(Strip, NUM_LEDS * 2, CRGB::Black);
}

void BatteryRunning()
{
  RawVoltage = analogRead(29);
  Voltage = RawVoltage * 3.0 * 3.3 / 4096.0;

  currentMillis = millis();

  if (currentMillis - previousMillis >= 500)
  {
    // save the last time you blinked the LED
    previousMillis = currentMillis;
    // if the LED is off turn it on and vice-versa:
    if (blink == 0){
      blink = 1;
    }
    else{
      blink = 0;
    }
  }

  //hysteresis for voltage measurement
  static float vLast = 0;
const float HYS = 0.03;  // 30 mV
if (fabsf(Voltage - vLast) < HYS) Voltage = vLast; else vLast = Voltage;

fill_solid(Strip, NUM_LEDS*2, CRGB::Black);

Strip[25] = blink ? CRGB::Blue : CRGB::Black;

if      (Voltage >= 4.20) fill_solid(Strip, 5, CRGB::Blue);
else if (Voltage >  4.00) fill_solid(Strip, 4, CRGB::Green);
else if (Voltage >  3.80) fill_solid(Strip, 3, CRGB::Green);
else if (Voltage >  3.60) fill_solid(Strip, 2, CRGB::Yellow);
else if (Voltage >  3.40) fill_solid(Strip, 1, CRGB::Yellow);
else if (Voltage >  3.20) fill_solid(Strip, 1, CRGB::Red);
else                      {/* leave empty = very empty */}

  uint8_t brightnessLevel = brightnessLevelFromValue(static_cast<uint8_t>(currentBrightness));
  brightnessLevel = constrain(brightnessLevel, (uint8_t)1, BRIGHTNESS_LEVEL_COUNT);
  const uint8_t indicatorStart = NUM_LEDS + 2;
  for (uint8_t i = 0; i < BRIGHTNESS_LEVEL_COUNT && (indicatorStart + i) < NUM_LEDS * 2; ++i)
  {
    Strip[indicatorStart + i] = (i < brightnessLevel) ? CRGB::Yellow : CRGB::Black;
  }
}

void RunEntry()
{
  setBatteryViewActive(false);
  fill_solid(Strip, NUM_LEDS * 2, CRGB::Black);
  setCurrentPatternValue(2);
  displayPatternIndicator(currentPattern);
}

void running()
{
  fill_rainbow(Strip, NUM_LEDS * 2, gHue, 7);
}

void RunEntry2()
{
  setBatteryViewActive(false);
  fill_solid(Strip, NUM_LEDS * 2, CRGB::Black);
  setCurrentPatternValue(3);
  displayPatternIndicator(currentPattern);
}

void running2()
{
  color = map(color, -180, 180, 0, 255);
  fill_solid(Strip, NUM_LEDS * 2, CHSV(color, 255, 255));
}

void RunEntry3()
{
  setBatteryViewActive(false);
  fill_solid(Strip, NUM_LEDS * 2, CRGB::Black);
  FastLED.setBrightness(30);
  setCurrentPatternValue(4);
  displayPatternIndicator(currentPattern);
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
  setBatteryViewActive(false);
  fill_solid(Strip, NUM_LEDS * 2, CRGB::Black);
  setCurrentPatternValue(5);
  displayPatternIndicator(currentPattern);
}

void running4()
{
  color = map(color, -180, 180, 0, 255);
  uint8_t pos = map(beat16(40, 0), 0, 65535, 0, NUM_LEDS - 1);
  Strip[pos] = CHSV(color, 200, 255);
  Strip[pos + NUM_LEDS] = CHSV(color, 200, 255);

  fadeToBlackBy(Strip, NUM_LEDS * 2, 12);
}

void RunEntry5()
{
  setBatteryViewActive(false);
  fill_solid(Strip, NUM_LEDS * 2, CRGB::Black);
  setCurrentPatternValue(6);
  displayPatternIndicator(currentPattern);
}

void running5()
{
  color = map(color, -180, 180, 0, 255);
  accel = map(smoothedMotion(), 2000, 20000, 100, 0);
  fade = map(accel, 20, 160, 60, 12);
  accel = constrain(accel, 0, 100);

  EVERY_N_MILLIS_I(timingObj, 1)
  {

    if (ledeffect == NUM_LEDS)
    {
      ledeffect = 0;
    }

    fadeToBlackBy(Strip, NUM_LEDS * 2, fade);

    Strip[ledeffect] = CHSV(color, 255, 255);
    Strip[ledeffect + NUM_LEDS] = CHSV(color, 255, 255);

    ledeffect = ledeffect + 1;

    timingObj.setPeriod(accel);
  }
}

void RunEntry6()
{
  setBatteryViewActive(false);
  fill_solid(Strip, NUM_LEDS * 2, CRGB::Black);
  setCurrentPatternValue(7);
  displayPatternIndicator(currentPattern);
}

void running6()
{
  color = map(color, -180, 180, 0, 255);
  color2 = map(color2, 180, -180, 0, 255);
  accel = map(smoothedMotion(), 2000, 20000, 100, 0);
  fade = map(accel, 20, 160, 60, 12);
  accel = constrain(accel, 0, 100);

  EVERY_N_MILLIS_I(timingObj, 1)
  {

    if (ledeffect == NUM_LEDS)
    {
      ledeffect = 0;
      ledeffect2 = 0;
    }

    fadeToBlackBy(Strip, NUM_LEDS * 2, fade);

    Strip[ledeffect] = CHSV(color, 255, 255);
    Strip[ledeffect2] = CHSV(color2, 255, 255);
    Strip[ledeffect + NUM_LEDS] = CHSV(color, 255, 255);
    Strip[ledeffect2 + NUM_LEDS] = CHSV(color2, 255, 255);

    ledeffect2 = ledeffect;
    ledeffect = ledeffect + 1;

    timingObj.setPeriod(accel);
  }
}

void RunEntry7()
{
  setBatteryViewActive(false);
  fill_solid(Strip, NUM_LEDS * 2, CRGB::Black);

  setCurrentPatternValue(8);
  displayPatternIndicator(currentPattern);
}

void running7()
{
  color = map(color, -180, 180, 0, 255);
  bloodHue = color; // Blood color [hue from 0-255]

  for (int i = 0; i < NUM_LEDS; i++)
  {
    uint8_t bloodVal = sumPulse((5 / NUM_LEDS / 2) + (NUM_LEDS / 2) * i * flowDirection);
    Strip[i] = CHSV(bloodHue, bloodSat, bloodVal);
    Strip[i + NUM_LEDS] = CHSV(bloodHue, bloodSat, bloodVal);
  }
}

void RunEntry8()
{
  setBatteryViewActive(false);
  fill_solid(Strip, NUM_LEDS * 2, CRGB::Black);
  setCurrentPatternValue(9);
  displayPatternIndicator(currentPattern);
}

void running8()
{
  color = map(color, -180, 180, 0, 255);
  accel = map(smoothedMotion(), 0, 10000, 20, 160);
  fade = map(accel, 20, 160, 48, 6);
  fade = constrain(fade, 6, 48);

  uint8_t pos = map(beatsin16(80, 0), 0, 65535, 0, NUM_LEDS - 1);

  Strip[pos] = CHSV(color, 200, 255);
  Strip[pos + NUM_LEDS] = CHSV(color, 200, 255);

  fadeToBlackBy(Strip, NUM_LEDS * 2, fade);
}

void RunEntry9()
{
  setBatteryViewActive(false);
  fill_solid(Strip, NUM_LEDS * 2, CRGB::Black);
  setCurrentPatternValue(10);
  displayPatternIndicator(currentPattern);
}

void running9()
{

float speed = ypr[0];
 
    static float headPos[NUM_COMETS];
    static bool initialized = false;

    const uint8_t tailLength = constrain(deviceConfig.motion.running9TailLength, (uint8_t)1, (uint8_t)15);
    const uint8_t maxBrightness = 255;      // Max brightness of comet (global brightness control still applies)

    color = map(color, -180, 180, 0, 255);
    const CRGB cometColor = CHSV(color, 200, 255); //comet color

    // init once
    if (!initialized) {
        for (int i = 0; i < NUM_COMETS; i++) {
            headPos[i] = (float)(i * TOTAL_LEDS / NUM_COMETS);
        }
        initialized = true;
    }

    // clear strip
    FastLED.clear();

    // draw comets
    for (int k = 0; k < NUM_COMETS; k++) {
        headPos[k] += (speed * 0.5);

        while (headPos[k] >= TOTAL_LEDS) {
            headPos[k] -= TOTAL_LEDS;
        }
        while (headPos[k] < 0) {
            headPos[k] += TOTAL_LEDS;
        }
    }

    for (int p_idx = 0; p_idx < TOTAL_LEDS; p_idx++) {
        int logical_idx;

        if (p_idx < HALF_LEDS) {
            logical_idx = p_idx;
        } else {
            logical_idx = (TOTAL_LEDS - 1) - p_idx + HALF_LEDS;
        }

        CRGB finalColorForPixel = CRGB::Black;
        for (int k = 0; k < NUM_COMETS; k++) {
            float delta = fmod(logical_idx - headPos[k] + TOTAL_LEDS, TOTAL_LEDS);
            
            if (delta > HALF_LEDS) {
                delta = TOTAL_LEDS - delta;
            }

            uint8_t brightness = 0;
            if (delta >= 0 && delta <= tailLength) {
                brightness = map((int)(delta * 100), 0, (tailLength * 100), maxBrightness, 0);
            }
            
            if ((int)roundf(headPos[k]) == logical_idx) {
                brightness = maxBrightness;
            }
            finalColorForPixel |= cometColor.scale8(brightness);
        }
        Strip[p_idx] = finalColorForPixel;
    }
}

void RunEntry10()
{
  setBatteryViewActive(false);
  fill_solid(Strip, NUM_LEDS * 2, CRGB::Black);
  setCurrentPatternValue(11);
  displayPatternIndicator(currentPattern);
}

void running10()
{
  uint32_t m = smoothedMotion();
  uint8_t hue = map((int)(ypr[0]*180.0f/M_PI), -180, 180, 0, 255);

  if (m < 3500) {
    // calm  „Breathing“
    uint8_t breath = beatsin8(10, 40, 180);
    fill_solid(Strip, TOTAL_LEDS, CHSV(hue, 255, breath));
  } else {
    // Storm: Sparcs propotional to Movement.
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
  setBatteryViewActive(false);
  fill_solid(Strip, NUM_LEDS * 2, CRGB::Black);
  setCurrentPatternValue(12);
  displayPatternIndicator(currentPattern);
}

void running11()
{
  // simple Magnitude-Trigger (|x|+|y|+|z|)
  uint32_t mag = (uint32_t)abs(aaWorld.x) + (uint32_t)abs(aaWorld.y) + (uint32_t)abs(aaWorld.z);

  static uint16_t phase = 10000;
  static uint32_t prevMag = 0;

  int32_t jerk = (int32_t)mag - (int32_t)prevMag;
  prevMag = mag;

  uint16_t jerkThreshold = constrain(deviceConfig.motion.running11JerkThreshold, (uint16_t)500, (uint16_t)20000);
  if (jerk > (int32_t)jerkThreshold) { // threshold adjustment
    phase = 0;
  }

  fadeToBlackBy(Strip, TOTAL_LEDS, 35);

  uint8_t waveSpacing = constrain(deviceConfig.motion.running11WaveSpacing, (uint8_t)4, (uint8_t)80);
  uint32_t maxPhase = 2000UL + (uint32_t)waveSpacing * NUM_LEDS;
  if (phase < maxPhase) {
    phase += 20 + map((int)smoothedMotion(), 2000, 20000, 0, 20);
    uint8_t hue = map((int)(ypr[0]*180.0f/M_PI), -180, 180, 0, 255);

    // wave from middle to the ends - identical on both strips
    int center = NUM_LEDS / 2;
    for (int i = 0; i < NUM_LEDS; i++) {
      int16_t d = abs(i - center);
      int16_t k = (int16_t)d * waveSpacing - (int16_t)phase;   // adjustable wave distance
      k = abs(k);
      if (k < waveSpacing) {
        uint8_t bri = map(k, 0, waveSpacing, 255, 0);
        Strip[i]            += CHSV(hue, 255, bri);
        Strip[i + NUM_LEDS] += CHSV(hue, 255, bri);
      }
    }
  }
}

void RunEntry12()
{
  setBatteryViewActive(false);
  fill_solid(Strip, NUM_LEDS * 2, CRGB::Black);
  setCurrentPatternValue(13);
  displayPatternIndicator(currentPattern);
}

void running12()
{
  static float head = 0;
  static float prevYaw = 0;
  static int   dir = +1;            // stored direction (+1 or -1)

  float yaw = ypr[0];
  float dy  = yaw - prevYaw;
  // Wrap um ±PI
  if (dy >  M_PI) dy -= 2*M_PI;
  if (dy < -M_PI) dy += 2*M_PI;
  prevYaw = yaw;

  // Rate -> LED-Speed
  float speed = dy * (TOTAL_LEDS * 0.5f);  // Skala nach Geschmack

  // Direction with dead band (prevents flickering around 0)
  const float deadBand = getRunning12Deadband();
  if (speed >  deadBand) dir = +1;
  if (speed < -deadBand) dir = -1;

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
  setBatteryViewActive(false);
  fill_solid(Strip, NUM_LEDS * 2, CRGB::Black);
  setCurrentPatternValue(14);
  displayPatternIndicator(currentPattern);
}

void running13()
{
  color = map(color, -180, 180, 0, 255);
  color2 = map(color2, 180, -180, 0, 255);
  accel = map(smoothedMotion(), 2000, 20000, 100, 0);
  fade = map(accel, 20, 160, 60, 12);
  accel = constrain(accel, 0, 100);

  EVERY_N_MILLIS_I(timingObj, 1)
  {

    if (ledeffect == 0)
    {
      ledeffect = NUM_LEDS;
      ledeffect2 = NUM_LEDS;
    }

    fadeToBlackBy(Strip, NUM_LEDS * 2, fade);

    Strip[ledeffect] = CHSV(color, 255, 255);
    Strip[ledeffect2] = CHSV(color2, 255, 255);
    Strip[ledeffect + NUM_LEDS] = CHSV(color, 255, 255);
    Strip[ledeffect2 + NUM_LEDS] = CHSV(color2, 255, 255);

    ledeffect2 = ledeffect;
    ledeffect = ledeffect -1;

    timingObj.setPeriod(accel);
  }
}

// ============================================================================
//  FSM TABLES & TRIGGERS
// ============================================================================

State s[] = {
    State("battery", BatteryEntry, BatteryRunning),
    State("charging", ChargingEntry, ChargingRunning, ChargingExit),
    State("running", RunEntry, running),
    State("running2", RunEntry2, running2),
    State("running3", RunEntry3, running3, RunExit3),
    State("running4", RunEntry4, running4),
    State("running5", RunEntry5, running5),
    State("running6", RunEntry6, running6),
    State("running7", RunEntry7, running7),
    State("running8", RunEntry8, running8),
    State("running9", RunEntry9, running9),
    State("running10", RunEntry10, running10),
    State("running11", RunEntry11, running11),
    State("running12", RunEntry12, running12),
    State("running13", RunEntry13, running13)};

template<int N>
static inline bool PatternIs() { return currentPattern == N; }

template<int N>
static inline bool unplugged() {
  return !shouldShowChargingAnimation() && (currentPattern == N);
}

const GuardCondition BATTERY_PATTERN_GUARDS[PATTERN_COUNT] = {
    PatternIs<2>,
    PatternIs<3>,
    PatternIs<4>,
    PatternIs<5>,
    PatternIs<6>,
    PatternIs<7>,
    PatternIs<8>,
    PatternIs<9>,
    PatternIs<10>,
    PatternIs<11>,
    PatternIs<12>,
    PatternIs<13>,
    PatternIs<14>};

Transition transitions[] = {
    Transition(&s[2], &s[0], longpress),
    Transition(&s[3], &s[0], longpress),
    Transition(&s[4], &s[0], longpress),
    Transition(&s[5], &s[0], longpress),
    Transition(&s[6], &s[0], longpress),
    Transition(&s[7], &s[0], longpress),
    Transition(&s[8], &s[0], longpress),
    Transition(&s[9], &s[0], longpress),
    Transition(&s[10], &s[0], longpress),
    Transition(&s[11], &s[0], longpress),
    Transition(&s[12], &s[0], longpress),
    Transition(&s[13], &s[0], longpress),
    Transition(&s[14], &s[0], longpress),
    Transition(&s[2], &s[3], doubleClick),
    Transition(&s[3], &s[4], doubleClick),
    Transition(&s[4], &s[5], doubleClick),
    Transition(&s[5], &s[6], doubleClick),
    Transition(&s[6], &s[7], doubleClick),
    Transition(&s[7], &s[8], doubleClick),
    Transition(&s[8], &s[9], doubleClick),
    Transition(&s[9], &s[10], doubleClick),
    Transition(&s[10], &s[11], doubleClick),
    Transition(&s[11], &s[12], doubleClick),
    Transition(&s[12], &s[13], doubleClick),
    Transition(&s[13], &s[14], doubleClick),
    Transition(&s[14], &s[2], doubleClick),
    Transition(&s[0], &s[1], usbpower),
    Transition(&s[2], &s[1], usbpower),
    Transition(&s[3], &s[1], usbpower),
    Transition(&s[4], &s[1], usbpower),
    Transition(&s[5], &s[1], usbpower),
    Transition(&s[6], &s[1], usbpower),
    Transition(&s[7], &s[1], usbpower),
    Transition(&s[8], &s[1], usbpower),
    Transition(&s[9], &s[1], usbpower),
    Transition(&s[10], &s[1], usbpower),
    Transition(&s[11], &s[1], usbpower),
    Transition(&s[12], &s[1], usbpower),
    Transition(&s[13], &s[1], usbpower),
    Transition(&s[14], &s[1], usbpower)};

TimedTransition timedTransitions[] = {
    TimedTransition(&s[0], &s[2], 5000, NULL, "", PatternIs<2>),
    TimedTransition(&s[0], &s[3], 5000, NULL, "", PatternIs<3>),
    TimedTransition(&s[0], &s[4], 5000, NULL, "", PatternIs<4>),
    TimedTransition(&s[0], &s[5], 5000, NULL, "", PatternIs<5>),
    TimedTransition(&s[0], &s[6], 5000, NULL, "", PatternIs<6>),
    TimedTransition(&s[0], &s[7], 5000, NULL, "", PatternIs<7>),
    TimedTransition(&s[0], &s[8], 5000, NULL, "", PatternIs<8>),
    TimedTransition(&s[0], &s[9], 5000, NULL, "", PatternIs<9>),
    TimedTransition(&s[0], &s[10], 5000, NULL, "", PatternIs<10>),
    TimedTransition(&s[0], &s[11], 5000, NULL, "", PatternIs<11>),
    TimedTransition(&s[0], &s[12], 5000, NULL, "", PatternIs<12>),
    TimedTransition(&s[0], &s[13], 5000, NULL, "", PatternIs<13>),
    TimedTransition(&s[0], &s[14], 5000, NULL, "", PatternIs<14>),
    TimedTransition(&s[1], &s[2], 2000, NULL, "", unplugged<2>),
    TimedTransition(&s[1], &s[3], 2000, NULL, "", unplugged<3>),
    TimedTransition(&s[1], &s[4], 2000, NULL, "", unplugged<4>),
    TimedTransition(&s[1], &s[5], 2000, NULL, "", unplugged<5>),
    TimedTransition(&s[1], &s[6], 2000, NULL, "", unplugged<6>),
    TimedTransition(&s[1], &s[7], 2000, NULL, "", unplugged<7>),
    TimedTransition(&s[1], &s[8], 2000, NULL, "", unplugged<8>),
    TimedTransition(&s[1], &s[9], 2000, NULL, "", unplugged<9>),
    TimedTransition(&s[1], &s[10], 2000, NULL, "", unplugged<10>),
    TimedTransition(&s[1], &s[11], 2000, NULL, "", unplugged<11>),
    TimedTransition(&s[1], &s[12], 2000, NULL, "", unplugged<12>),
    TimedTransition(&s[1], &s[13], 2000, NULL, "", unplugged<13>),
    TimedTransition(&s[1], &s[14], 2000, NULL, "", unplugged<14>)};

int num_transitions = sizeof(transitions) / sizeof(Transition);
int num_timed = sizeof(timedTransitions) / sizeof(TimedTransition);

void applyBatteryDisplayDuration()
{
  uint16_t interval = constrain(deviceConfig.batteryDisplayMs, BATTERY_DISPLAY_MIN_MS, BATTERY_DISPLAY_MAX_MS);
  for (uint8_t i = 0; i < PATTERN_COUNT; ++i)
  {
    timedTransitions[i].setup(&s[0], &s[2 + i], interval, NULL, "", BATTERY_PATTERN_GUARDS[i]);
  }
}

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

  Serial.begin(115200); // 115200 is required for Teapot Demo output
  // while (!Serial);
  delay(1000); // 1 second delay for recovery

  EEPROM.begin(EEPROM_SIZE);
  Serial.println(F("EEPROM initialized."));
  loadConfig();

  applyLedConfig();

  /*Initialize device*/
  Serial.println(F("Initializing I2C devices..."));
  mpu.initialize();
  pinMode(INTERRUPT_PIN, INPUT);

  /*Verify connection*/
  Serial.println(F("Testing MPU6050 connection..."));
  if (mpu.testConnection() == false)
  {
    Serial.println("MPU6050 connection failed");
    while (true)
      ;
  }
  else
  {
    Serial.println("MPU6050 connection successful");
  }

  /* set ranges for accel and gyro */
  applySensorConfig();
  Serial.print("Accelrange index:");
  Serial.println(mpu.getFullScaleAccelRange());
  Serial.print("Gyrorange index:");
  Serial.println(mpu.getFullScaleGyroRange());

  /*Wait for Serial input*/
  // Serial.println(F("\nSend any character to begin: "));
  // while (Serial.available() && Serial.read()); // Empty buffer
  // while (!Serial.available());                 // Wait for data
  // while (Serial.available() && Serial.read()); // Empty buffer again

  /* Initializate and configure the DMP*/
  Serial.println(F("Initializing DMP..."));
  devStatus = mpu.dmpInitialize();

  /* Supply your gyro offsets here, scaled for min sensitivity */
  mpu.setXGyroOffset(111);
  mpu.setYGyroOffset(-6);
  mpu.setZGyroOffset(34);
  mpu.setXAccelOffset(-3137);
  mpu.setYAccelOffset(-7);
  mpu.setZAccelOffset(3687);

  /* Making sure it worked (returns 0 if so) */
  if (devStatus == 0)
  {
    mpu.CalibrateAccel(6); // Calibration Time: generate offsets and calibrate our MPU6050
    mpu.CalibrateGyro(6);
    Serial.println("These are the Active offsets: ");
    mpu.PrintActiveOffsets();
    Serial.println(F("Enabling DMP...")); // Turning ON DMP
    mpu.setDMPEnabled(true);

    /*Enable Arduino interrupt detection*/
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

  // Battery stuff
  pinMode(29, INPUT);
  pinMode(24, INPUT);
  analogReadResolution(12);

  delay(1000); // 1 second delay for recovery

  // tell FastLED about the LED strip configuration
 // FastLED.addLeds<LED_TYPE, PinStrip1, COLOR_ORDER>(Strip1, NUM_LEDS).setCorrection(TypicalLEDStrip);
 // FastLED.addLeds<LED_TYPE, PinStrip2, COLOR_ORDER>(Strip2, NUM_LEDS).setCorrection(TypicalLEDStrip);

  FastLED.addLeds<LED_TYPE, PinStrip1, COLOR_ORDER>(Strip, 0, NUM_LEDS).setCorrection(TypicalLEDStrip);
  FastLED.addLeds<LED_TYPE, PinStrip2, COLOR_ORDER>(Strip, NUM_LEDS, NUM_LEDS).setCorrection(TypicalLEDStrip);

  // set master brightness control
  FastLED.setBrightness(BRIGHTNESS);

  // FastLED.setMaxRefreshRate(120);

  //motion smoothing init
  applySmoothingConfig();

  Serial.println(F("Current configuration values:"));
  printConfig();

  // Set Timestamp for first check
  lastUpdateTime = millis();

//state machine init
  fsm.add(timedTransitions, num_timed);
  fsm.add(transitions, num_transitions);

  // Brightness on Powerup
  BRIGHTNESS = currentBrightness;
  FastLED.setBrightness(BRIGHTNESS);

  // initialState on Powerup
  fsm.setInitialState(&s[currentPattern]);

  // CLI command registration
  cli.addCommand("pattern-list", handlePatternList);
  Command enableCmd = cli.addCommand("pattern-enable", handlePatternEnable);
  enableCmd.addPositionalArgument("pattern");
  Command disableCmd = cli.addCommand("pattern-disable", handlePatternDisable);
  disableCmd.addPositionalArgument("pattern");
  Command accelCmd = cli.addCommand("set-accel", handleSetAccel);
  accelCmd.addPositionalArgument("range");
  Command gyroCmd = cli.addCommand("set-gyro", handleSetGyro);
  gyroCmd.addPositionalArgument("range");
  Command smoothingCmd = cli.addCommand("set-smoothing", handleSetSmoothing);
  smoothingCmd.addPositionalArgument("samples");
  Command ledCmd = cli.addCommand("led-param", handleLedParam);
  ledCmd.addPositionalArgument("name");
  ledCmd.addPositionalArgument("value");
  cli.addCommand("config-save", handleConfigSave);
  cli.addCommand("config-show", handleConfigShow);
  cli.addCommand("reset", handleResetCommand);
  cli.addCommand("config-reset", handleConfigReset);
  Command indicatorCmd = cli.addCommand("pattern-indicator", handlePatternIndicator);
  indicatorCmd.addPositionalArgument("state");
  Command indicatorParamCmd = cli.addCommand("pattern-indicator-param", handlePatternIndicatorParam);
  indicatorParamCmd.addPositionalArgument("name");
  indicatorParamCmd.addPositionalArgument("value");
  Command brightnessModeCmd = cli.addCommand("brightness-mode", handleBrightnessMode);
  brightnessModeCmd.addPositionalArgument("mode");
  Command batteryDurationCmd = cli.addCommand("battery-display", handleBatteryDisplayDuration);
  batteryDurationCmd.addPositionalArgument("ms");
}

// ============================================================================
//  LOOP
// ============================================================================

void loop()
{
  serialTerminalActive = Serial ? true : false;
  if (Serial.available())
  {
    String input = Serial.readStringUntil('\n');
    input.trim();
    if (input.length() > 0)
    {
      cli.parse(input);
    }
  }

  while (cli.available())
  {
    Command command = cli.getCmd();
    command.run();
  }

  while (cli.errored())
  {
    CommandError error = cli.getError();
    Serial.print(F("CLI error: "));
    Serial.println(error.getMessage());
  }

  if (!DMPReady)
    return; // Stop the program if DMP programming fails.

  /* Read a packet from FIFO */
  if (mpu.dmpGetCurrentFIFOPacket(FIFOBuffer))
  { // Get the Latest packet
#ifdef OUTPUT_READABLE_YAWPITCHROLL
    /* Display Euler angles in degrees */
    mpu.dmpGetQuaternion(&q, FIFOBuffer);
    mpu.dmpGetGravity(&gravity, &q);
    mpu.dmpGetYawPitchRoll(ypr, &q, &gravity);
    // Serial.print("ypr\t");
    // Serial.print(ypr[0] * 180/M_PI);
    color = ypr[0] * 180 / M_PI;
    color2 = ypr[0] * 180 / M_PI;
// Serial.print(";");
// Serial.print("\t");
// Serial.print(ypr[1] * 180/M_PI);
// Serial.print(";");
// Serial.print("\t");
// Serial.println(ypr[2] * 180/M_PI);
#endif

#ifdef OUTPUT_READABLE_QUATERNION
    /* Display Quaternion values in easy matrix form: [w, x, y, z] */
    mpu.dmpGetQuaternion(&q, FIFOBuffer);
    // Serial.print("quat\t");
    // Serial.print(q.w);
    // Serial.print(";");
    // Serial.print("\t");
    // Serial.print(q.x);
    // Serial.print(";");
    // Serial.print("\t");
    // Serial.print(q.y);
    // Serial.print(";");
    // Serial.print("\t");
    // Serial.println(q.z);
#endif

#ifdef OUTPUT_READABLE_EULER
    /* Display Euler angles in degrees */
    mpu.dmpGetQuaternion(&q, FIFOBuffer);
    mpu.dmpGetEuler(euler, &q);
// Serial.print("euler\t");
// Serial.print(euler[0] * 180/M_PI);
// Serial.print(";");
// Serial.print("\t");
// Serial.print(euler[1] * 180/M_PI);
// Serial.print(";");
// Serial.print("\t");
// Serial.println(euler[2] * 180/M_PI);
#endif

#ifdef OUTPUT_READABLE_REALACCEL
    /* Display real acceleration, adjusted to remove gravity */
    mpu.dmpGetQuaternion(&q, FIFOBuffer);
    mpu.dmpGetAccel(&aa, FIFOBuffer);
    mpu.dmpGetGravity(&gravity, &q);
    mpu.dmpGetLinearAccel(&aaReal, &aa, &gravity);
// Serial.print("areal\t");
// Serial.print(aaReal.x);
// Serial.print(";");
// Serial.print("\t");
// Serial.print(aaReal.y);
// Serial.print(";");
// Serial.print("\t");
// Serial.println(aaReal.z);
#endif

#ifdef OUTPUT_READABLE_WORLDACCEL
    /* Display initial world-frame acceleration, adjusted to remove gravity
    and rotated based on known orientation from Quaternion */
    mpu.dmpGetQuaternion(&q, FIFOBuffer);
    mpu.dmpGetAccel(&aa, FIFOBuffer);
    mpu.dmpGetGravity(&gravity, &q);
    mpu.dmpGetLinearAccel(&aaReal, &aa, &gravity);
    mpu.dmpGetLinearAccelInWorld(&aaWorld, &aaReal, &q);
    // Serial.print("aworld\t");
    // Serial.print(aaWorld.x);
    // Serial.print("\t");
    // Serial.print(";");
    // Serial.print(aaWorld.y);
    // Serial.print("\t");
    // Serial.print(";");
    // Serial.println(aaWorld.z);
#endif

#ifdef OUTPUT_TEAPOT
    /* Display quaternion values in InvenSense Teapot demo format */
    teapotPacket[2] = FIFOBuffer[0];
    teapotPacket[3] = FIFOBuffer[1];
    teapotPacket[4] = FIFOBuffer[4];
    teapotPacket[5] = FIFOBuffer[5];
    teapotPacket[6] = FIFOBuffer[8];
    teapotPacket[7] = FIFOBuffer[9];
    teapotPacket[8] = FIFOBuffer[12];
    teapotPacket[9] = FIFOBuffer[13];
    Serial.write(teapotPacket, 14);
    teapotPacket[11]++; // PacketCount, loops at 0xFF on purpose
#endif

    /* Blink LED to indicate activity */
    blinkState = !blinkState;
    digitalWrite(LED_BUILTIN, blinkState);
  }

  // Periodically store configuration if needed
  unsigned long currentTime = millis();
  if (currentTime - lastUpdateTime >= UPDATE_INTERVAL)
  {
    lastUpdateTime = currentTime;
    if (configDirty && !configsEqual(deviceConfig, lastSavedConfig))
    {
      Serial.println(F("Autosaving configuration..."));
      saveConfig(false);
    }
  }


  // send the 'leds' array out to the actual LED strip
  // FastLED.show();
  // insert a delay to keep the framerate modest
  FastLED.delay(1000 / FRAMES_PER_SECOND);

  //FastLED.countFPS();
  // Serial.println(LEDS.getFPS());
  // Serial.println(FastLED.getFPS());
  // Serial.println(fsm.getPreviousState()->getID());
  // Serial.println(fsm.getDotDefinition());
  // Serial.println(State3());

  // do some periodic updates
  EVERY_N_MILLISECONDS(20) { gHue++; } // slowly cycle the "base color" through the rainbow

  fsm.run(0);
  multiresponseButton.poll();

  if (multiresponseButton.longPress())
  {
    fsm.trigger(longpress);
    logSerialMessage(F("Button: long press detected"));
  }

  if (multiresponseButton.doubleClick())
  {
    logSerialMessage(F("Button: double click -> next pattern"));
    advanceToNextEnabledPattern();
  }

  UsbConnected = digitalRead(24);
  //UsbConnected = 0;
  if (shouldShowChargingAnimation())
  {
    fsm.trigger(usbpower);
  }

  if (multiresponseButton.singleClick())
  {
    if (deviceConfig.brightnessBatteryOnly && !batteryViewActive)
    {
      logSerialMessage(F("Brightness change only in battery view."));
      return;
    }
    BRIGHTNESS += 32; // brightness = 95-255, so steps of 32
    if (BRIGHTNESS > MAX_BRIGHTNESS)
    {
      BRIGHTNESS = 95; // we roll over to 95 bright
    }
    FastLED.setBrightness(BRIGHTNESS);
    currentBrightness = BRIGHTNESS;
    deviceConfig.brightness = currentBrightness;
    markConfigDirty();
    logSerialValue(F("Button: brightness -> "), BRIGHTNESS);
  }
}
void resetConfigToDefaults()
{
  deviceConfig = DEFAULT_CONFIG;
  currentPattern = deviceConfig.currentPattern;
  currentBrightness = deviceConfig.brightness;
  applyLedConfig();
  applySensorConfig();
  applySmoothingConfig();
  applyBatteryDisplayDuration();
  ensureValidPattern();
  fsm.setInitialState(&s[currentPattern]);
  Serial.println(F("Configuration reset to defaults."));
}
void handleConfigReset(cmd *c)
{
  (void)c;
  Serial.println(F("Resetting configuration to defaults."));
  resetConfigToDefaults();
  markConfigDirty();
  saveConfig();
}
