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
#include <SimpleCLI.h> // Serial command-line interface
#include "avdweb_Switch.h" // Button library
#include <Smoothed.h> // Smoothing library
#include <EEPROM.h> //EEPROM Library
#include <math.h> // Math library

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

int const INTERRUPT_PIN = PIN_MPU_INTERRUPT; // Define the interruption #0 pin

#define PinStrip1 PIN_LED_STRIP_1
#define PinStrip2 PIN_LED_STRIP_2
// #define CLK_PIN   4
#define LED_TYPE WS2811
#define COLOR_ORDER GRB

#define MIN_LEDS_PER_STRIP 10
#define MAX_LEDS_PER_STRIP 35
#define DEFAULT_LEDS_PER_STRIP 25
#define MAX_TOTAL_LEDS (MAX_LEDS_PER_STRIP * 2)

int ledsPerStrip = DEFAULT_LEDS_PER_STRIP;
int totalLeds = (DEFAULT_LEDS_PER_STRIP * 2); // both strips length
int halfLeds = DEFAULT_LEDS_PER_STRIP; // segment length

#define NUM_LEDS ledsPerStrip
#define TOTAL_LEDS totalLeds
#define HALF_LEDS halfLeds
#define NUM_COMETS 4        // comet count
#define NUM_COMETS2 2        // comet count2

// CRGB Strip1[NUM_LEDS];
// CRGB Strip2[NUM_LEDS];
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

// Memory addresses in the emulated EEPROM
// An int on the Pico is 4 bytes.
// We store value1 at address 0 and value2 directly after it.
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
const int EEPROM_MAGIC = 0x4E4B3434; // "NK44"

// Size of emulated EEPROM.
// has to big enough to store our variables
// 14 * sizeof(int) = 56 Bytes.
#define EEPROM_SIZE 64 // 64 Bytes leaves a little headroom

// variables we want to store in EEPROM
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
uint16_t currentEnabledPatternMask = 0;

// copies of current values
// needed to detect changes (to limit wear of flash memory)
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
uint16_t lastSavedEnabledPatternMask = 0;

const int DEFAULT_MOTION_SMOOTHING_SIZE = 100;
const int MIN_MOTION_SMOOTHING_SIZE = 1;
const int MAX_MOTION_SMOOTHING_SIZE = 512;

const int DEFAULT_ACCEL_RANGE = 2;
const int DEFAULT_GYRO_RANGE = 2000;
const int DEFAULT_BOOT_CALIBRATION_MODE = 1;

const int BOOT_CALIBRATION_MODE_OFF = 0;
const int BOOT_CALIBRATION_MODE_QUICK = 1;

const int DEFAULT_X_ACCEL_OFFSET = -3137;
const int DEFAULT_Y_ACCEL_OFFSET = -7;
const int DEFAULT_Z_ACCEL_OFFSET = 3687;
const int DEFAULT_X_GYRO_OFFSET = 111;
const int DEFAULT_Y_GYRO_OFFSET = -6;
const int DEFAULT_Z_GYRO_OFFSET = 34;
const uint8_t FIRST_PATTERN_ID = 1;
const uint8_t LAST_PATTERN_ID = 13;
const uint8_t PATTERN_COUNT = LAST_PATTERN_ID - FIRST_PATTERN_ID + 1;
const uint16_t ALL_ENABLED_PATTERN_MASK = (1u << PATTERN_COUNT) - 1u;

int activeMotionSmoothingSize = DEFAULT_MOTION_SMOOTHING_SIZE;

// supported brightness levels for button + CLI
const int BRIGHTNESS_LEVELS[] = {95, 127, 159, 191, 223, 255};
const size_t BRIGHTNESS_LEVEL_COUNT = sizeof(BRIGHTNESS_LEVELS) / sizeof(BRIGHTNESS_LEVELS[0]);

// for timekeeping
unsigned long lastUpdateTime = 0;
const unsigned long UPDATE_INTERVAL = 5 * 60 * 1000; // 5 minutes in milliseconds (5 * 60 * 1000)
//const unsigned long UPDATE_INTERVAL = 30 * 1000; // 30 sec for debug
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
const byte multiresponseButtonpin = PIN_BUTTON_MULTI;
Switch multiresponseButton = Switch(multiresponseButtonpin, INPUT);

// Battery variables
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
extern State s[];

typedef void (*PatternCallback)();

struct PatternDefinition
{
  uint8_t id;
  const char* name;
  PatternCallback entry;
  PatternCallback run;
  PatternCallback exit;
};

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
void printOffsets();
void printOffsetsWithPrefix(const char* prefix);
void printConfigSummaryWithPrefix(const char* prefix);
void printEnabledPatternsList();
void printPatternStates();
bool beginCalibrationSession(bool verbose, bool* restartDMP);
void endCalibrationSession(bool restartDMP);
bool runQuickCalibration(bool verbose);
bool runPreciseCalibration(bool verbose);
void clearInactiveLeds();
void syncLogicalToPhysicalLeds();
void normalizePersistentConfig();
bool isValidPatternId(int value);
uint16_t sanitizeEnabledPatternMask(uint16_t mask);
bool isPatternEnabled(uint8_t patternId);
bool setPatternEnabled(uint8_t patternId, bool enabled);
bool parsePatternListMask(String valueText, uint16_t* maskOut);
bool updateEnabledPatternsFromMask(uint16_t mask, bool enabled);
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
void printTimingStatus();
bool saveConfigToEEPROM(bool verbose);
void readConfigFromEEPROM(bool verbose);
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
void onCliPatterns(cmd* cPtr);
void onCliEnablePattern(cmd* cPtr);
void onCliDisablePattern(cmd* cPtr);
void onCliError(cmd_error* e);

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

inline uint32_t smoothedMotion() {
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
  halfLeds = totalLeds / 2;
}

void applyPersistentConfig()
{
  applyConfiguredStripLength();
  BRIGHTNESS = currentBrightness;
  FastLED.setBrightness(BRIGHTNESS);
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

void printConfigSummaryWithPrefix(const char* prefix)
{
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
  Serial.print(" enabled_patterns=");
  printEnabledPatternsList();
  Serial.println();
}

void printPatternStates()
{
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
}

bool isValidPatternId(int value)
{
  return value >= FIRST_PATTERN_ID && value <= LAST_PATTERN_ID;
}

uint16_t sanitizeEnabledPatternMask(uint16_t mask)
{
  mask &= ALL_ENABLED_PATTERN_MASK;
  if (mask == 0)
  {
    return ALL_ENABLED_PATTERN_MASK;
  }
  return mask;
}

bool isPatternEnabled(uint8_t patternId)
{
  if (!isValidPatternId(patternId))
  {
    return false;
  }
  const uint8_t bitIndex = (uint8_t)(patternId - FIRST_PATTERN_ID);
  return (currentEnabledPatternMask & (1u << bitIndex)) != 0;
}

bool setPatternEnabled(uint8_t patternId, bool enabled)
{
  if (!isValidPatternId(patternId))
  {
    return false;
  }

  const uint16_t bit = (uint16_t)(1u << (patternId - FIRST_PATTERN_ID));
  uint16_t nextMask = currentEnabledPatternMask;
  if (enabled)
  {
    nextMask |= bit;
  }
  else
  {
    nextMask &= (uint16_t)~bit;
    if (nextMask == 0)
    {
      return false;
    }
  }

  currentEnabledPatternMask = sanitizeEnabledPatternMask(nextMask);
  return true;
}

bool parsePatternListMask(String valueText, uint16_t* maskOut)
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

  uint16_t mask = 0;
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

    mask |= (uint16_t)(1u << (patternId - FIRST_PATTERN_ID));
    if (comma < 0)
    {
      break;
    }
    start = comma + 1;
  }

  *maskOut = mask;
  return mask != 0;
}

bool updateEnabledPatternsFromMask(uint16_t mask, bool enabled)
{
  mask &= ALL_ENABLED_PATTERN_MASK;
  if (mask == 0)
  {
    return false;
  }

  uint16_t nextMask = currentEnabledPatternMask;
  if (enabled)
  {
    nextMask |= mask;
  }
  else
  {
    nextMask &= (uint16_t)~mask;
    if (nextMask == 0)
    {
      return false;
    }
  }

  currentEnabledPatternMask = sanitizeEnabledPatternMask(nextMask);
  return true;
}

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
  return batteryViewActive && !UsbConnected && (millis() - batteryViewLastInteractionMs >= BATTERY_VIEW_TIMEOUT_MS);
}

bool chargingUsbDisconnected()
{
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
    Serial.print("Enabled patterns: ");
    printEnabledPatternsList();
    Serial.println();
    printOffsets();
  }

  if (EEPROM.commit())
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
  }
  normalizePersistentConfig();
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

  if (magic != EEPROM_MAGIC)
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
    Serial.print("Enabled patterns: ");
    printEnabledPatternsList();
    Serial.println();
    printOffsets();
  }
}

void printCliHelp()
{
  Serial.println("Commands:");
  Serial.println("  help");
  Serial.println("  show");
  Serial.println("  get <pattern|brightness|strip_length|smoothing|accel_range|gyro_range|boot_calibration|enabled_patterns>");
  Serial.println("  set pattern <1..13>");
  Serial.println("  set brightness <95|127|159|191|223|255>");
  Serial.println("  set strip_length <10..35>");
  Serial.println("  set smoothing <1..512>           (takes effect after reboot)");
  Serial.println("  set accel_range <2|4|8|16>       (takes effect after reboot)");
  Serial.println("  set gyro_range <250|500|1000|2000> (takes effect after reboot)");
  Serial.println("  set boot_calibration <off|quick>");
  Serial.println("  patterns");
  Serial.println("  enable_pattern <1..13[,id...]>");
  Serial.println("  disable_pattern <1..13[,id...]>");
  Serial.println("  battery");
  Serial.println("  sensor");
  Serial.println("  timing");
  Serial.println("  offsets");
  Serial.println("  calibrate quick");
  Serial.println("  calibrate precise");
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
  if (key == "enabled_patterns")
  {
    Serial.print("OK enabled_patterns=");
    printEnabledPatternsList();
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
      Serial.println("ERR pattern range 1..13");
      return;
    }
    switchToPattern((uint8_t)value, true);
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
  applyPersistentConfig();
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
  (void)cPtr;
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

void onCliPatterns(cmd* cPtr)
{
  (void)cPtr;
  printPatternStates();
}

void onCliEnablePattern(cmd* cPtr)
{
  Command cmd(cPtr);
  uint16_t mask = 0;
  if (!parsePatternListMask(cmd.getArgument("pattern").getValue(), &mask))
  {
    Serial.println("ERR pattern list must contain IDs in range 1..13");
    return;
  }

  updateEnabledPatternsFromMask(mask, true);
  Serial.print("OK enabled_patterns=");
  printEnabledPatternsList();
  Serial.println();
}

void onCliDisablePattern(cmd* cPtr)
{
  Command cmd(cPtr);
  uint16_t mask = 0;
  if (!parsePatternListMask(cmd.getArgument("pattern").getValue(), &mask))
  {
    Serial.println("ERR pattern list must contain IDs in range 1..13");
    return;
  }

  if (!updateEnabledPatternsFromMask(mask, false))
  {
    Serial.println("ERR at least one pattern must remain enabled");
    return;
  }

  Serial.print("OK enabled_patterns=");
  printEnabledPatternsList();
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
  Command battery = cli.addCommand("battery", onCliBattery);
  (void)battery;
  Command sensor = cli.addCommand("sensor", onCliSensor);
  (void)sensor;
  Command timing = cli.addCommand("timing", onCliTiming);
  (void)timing;
  Command offsets = cli.addCommand("offsets", onCliOffsets);
  (void)offsets;
  Command calibrate = cli.addCommand("calibrate", onCliCalibrate);
  calibrate.addPositionalArgument("mode");
  Command reboot = cli.addCommand("reboot", onCliReboot);
  (void)reboot;
  Command restart = cli.addCommand("restart", onCliReboot);
  (void)restart;

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
    Serial.println();
    Serial.println("[NightKite CLI] USB connected. Type 'help'.");
    printCliPrompt();
    Serial.flush();
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
        cli.parse(cliInputBuffer);
        commandExecuted = true;
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
      cli.parse(cliInputBuffer);
      commandExecuted = true;
    }
    cliInputBuffer = "";
    cliLastInputMs = 0;
  }

  if (commandExecuted)
  {
    printCliPrompt();
  }
}

// ============================================================================
//  FSM STATES — ENTRY/RUN/EXIT
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

int statusStart = NUM_LEDS;
if (statusStart < TOTAL_LEDS)
{
  Strip[statusStart] = blink ? CRGB::Red : CRGB::Black;
}

int batteryBarMax = min(5, NUM_LEDS);
if      (Voltage >= 4.20) fill_solid(Strip, min(5, batteryBarMax), CRGB::Blue);
else if (Voltage >  4.00) fill_solid(Strip, min(4, batteryBarMax), CRGB::Green);
else if (Voltage >  3.80) fill_solid(Strip, min(3, batteryBarMax), CRGB::Green);
else if (Voltage >  3.60) fill_solid(Strip, min(2, batteryBarMax), CRGB::Yellow);
else if (Voltage >  3.40) fill_solid(Strip, min(1, batteryBarMax), CRGB::Yellow);
else if (Voltage >  3.20) fill_solid(Strip, min(1, batteryBarMax), CRGB::Red);
else                      {/* leave empty = very empty */}
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
for (int i = 0; i < brightnessPixels; ++i) {
  Strip[statusStart + 1 + i] = (i < brightnessLevel) ? CRGB::Yellow : CRGB::Black;
}

int batteryBarMax = min(5, NUM_LEDS);
if      (Voltage >= 4.20) fill_solid(Strip, min(5, batteryBarMax), CRGB::Blue);
else if (Voltage >  4.00) fill_solid(Strip, min(4, batteryBarMax), CRGB::Green);
else if (Voltage >  3.80) fill_solid(Strip, min(3, batteryBarMax), CRGB::Green);
else if (Voltage >  3.60) fill_solid(Strip, min(2, batteryBarMax), CRGB::Yellow);
else if (Voltage >  3.40) fill_solid(Strip, min(1, batteryBarMax), CRGB::Yellow);
else if (Voltage >  3.20) fill_solid(Strip, min(1, batteryBarMax), CRGB::Red);
else                      {/* leave empty = very empty */}
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
  color = map(color, -180, 180, 0, 255);
  accel = map(smoothedMotion(), 2000, 20000, 100, 0);
  fade = map(accel, 20, 160, 60, 12);
  accel = constrain(accel, 0, 100);

  EVERY_N_MILLIS_I(timingObj, 1)
  {

    if (ledeffect < 0 || ledeffect >= NUM_LEDS)
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
  fill_solid(Strip, NUM_LEDS * 2, CRGB::Black);
  currentPattern = 6;
  batteryViewActive = false;
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

    if (ledeffect < 0 || ledeffect >= NUM_LEDS || ledeffect2 < 0 || ledeffect2 >= NUM_LEDS)
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
  fill_solid(Strip, NUM_LEDS * 2, CRGB::Black);
  currentPattern = 7;
  batteryViewActive = false;
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

float speed = ypr[0];
 
    static float headPos[NUM_COMETS];
    static bool initialized = false;
    
    const uint8_t tailLength = 3;          // tail lenght
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

    // clear logical strip
    fill_solid(Strip, TOTAL_LEDS, CRGB::Black);

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
  fill_solid(Strip, NUM_LEDS * 2, CRGB::Black);
  currentPattern = 10;
  batteryViewActive = false;
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
  fill_solid(Strip, NUM_LEDS * 2, CRGB::Black);
  currentPattern = 11;
  batteryViewActive = false;
}

void running11()
{
  // simple Magnitude-Trigger (|x|+|y|+|z|)
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

    // wave from middle to the ends - identical on both strips
    int center = NUM_LEDS / 2;
    for (int i = 0; i < NUM_LEDS; i++) {
      int16_t d = abs(i - center);
      int16_t k = (int16_t)d * 24 - (int16_t)phase;   // 24 = Wave distance
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
  // Wrap um ±PI
  if (dy >  M_PI) dy -= 2*M_PI;
  if (dy < -M_PI) dy += 2*M_PI;
  prevYaw = yaw;

  // Rate -> LED-Speed
  float speed = dy * (TOTAL_LEDS * 0.5f);  // Skala nach Geschmack

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
  color = map(color, -180, 180, 0, 255);
  color2 = map(color2, 180, -180, 0, 255);
  accel = map(smoothedMotion(), 2000, 20000, 100, 0);
  fade = map(accel, 20, 160, 60, 12);
  accel = constrain(accel, 0, 100);

  EVERY_N_MILLIS_I(timingObj, 1)
  {

    if (ledeffect <= 0 || ledeffect > (NUM_LEDS - 1))
    {
      ledeffect = NUM_LEDS - 1;
      ledeffect2 = NUM_LEDS - 1;
    }

    fadeToBlackBy(Strip, NUM_LEDS * 2, fade);

    Strip[ledeffect] = CHSV(color, 255, 255);
    Strip[ledeffect2] = CHSV(color2, 255, 255);
    Strip[ledeffect + NUM_LEDS] = CHSV(color, 255, 255);
    Strip[ledeffect2 + NUM_LEDS] = CHSV(color2, 255, 255);

    ledeffect2 = ledeffect;
    ledeffect = ledeffect - 1;

    timingObj.setPeriod(accel);
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
    {13, "runner_dual_inverted", RunEntry13, running13, NULL},
};

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
  const PatternDefinition* pattern = getPatternDefinition(patternId);
  if (pattern != NULL && pattern->entry != NULL)
  {
    pattern->entry();
  }
}

void runPatternFrame(uint8_t patternId)
{
  const PatternDefinition* pattern = getPatternDefinition(patternId);
  if (pattern != NULL && pattern->run != NULL)
  {
    pattern->run();
  }
}

void runPatternExit(uint8_t patternId)
{
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

  const bool patternCurrentlyActive = !batteryViewActive && !UsbConnected;
  const uint8_t previousPattern = (uint8_t)currentPattern;

  if (patternCurrentlyActive && previousPattern == patternId && !activatePatternState)
  {
    batteryViewLastInteractionMs = millis();
    return;
  }

  if (patternCurrentlyActive)
  {
    runPatternExit(previousPattern);
  }

  currentPattern = patternId;
  batteryViewLastInteractionMs = millis();

  if (activatePatternState)
  {
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
  runPatternEntry((uint8_t)currentPattern);
}

void PatternStateRunning()
{
  runPatternFrame((uint8_t)currentPattern);
}

void PatternStateExit()
{
  runPatternExit((uint8_t)currentPattern);
}

// ============================================================================
//  FSM TABLES & TRIGGERS
// ============================================================================

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

  Serial.begin(115200); // 115200 is required for Teapot Demo output
  Serial.setTimeout(5);
  // while (!Serial);
  delay(1000); // 1 second delay for recovery

  // Load persisted config early so subsequent setup can depend on it.
  EEPROM.begin(EEPROM_SIZE);
  Serial.println("EEPROM initialized.");
  readConfigFromEEPROM(true);

  /*Initialize device*/
  Serial.println(F("Initializing I2C devices..."));
  mpu.initialize();
  pinMode(INTERRUPT_PIN, INPUT);

  /*Verify connection*/
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

  /*Wait for Serial input*/
  // Serial.println(F("\nSend any character to begin: "));
  // while (Serial.available() && Serial.read()); // Empty buffer
  // while (!Serial.available());                 // Wait for data
  // while (Serial.available() && Serial.read()); // Empty buffer again

  /* Initializate and configure the DMP*/
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
  pinMode(PIN_BATTERY_ADC, INPUT);
  pinMode(PIN_USB_SENSE, INPUT);
  analogReadResolution(12);

  delay(1000); // 1 second delay for recovery

  // tell FastLED about the LED strip configuration
 // FastLED.addLeds<LED_TYPE, PinStrip1, COLOR_ORDER>(Strip1, NUM_LEDS).setCorrection(TypicalLEDStrip);
 // FastLED.addLeds<LED_TYPE, PinStrip2, COLOR_ORDER>(Strip2, NUM_LEDS).setCorrection(TypicalLEDStrip);

  FastLED.addLeds<LED_TYPE, PinStrip1, COLOR_ORDER>(PhysicalStrip, 0, MAX_LEDS_PER_STRIP).setCorrection(TypicalLEDStrip);
  FastLED.addLeds<LED_TYPE, PinStrip2, COLOR_ORDER>(PhysicalStrip, MAX_LEDS_PER_STRIP, MAX_LEDS_PER_STRIP).setCorrection(TypicalLEDStrip);

  // set master brightness control
  FastLED.setBrightness(BRIGHTNESS);

  // FastLED.setMaxRefreshRate(120);

  //motion smoothing init
  applyConfiguredMotionSmoothing();

  applyPersistentConfig();
  lastUpdateTime = millis();
	
//state machine init
  fsm.add(timedTransitions, num_timed);
  fsm.add(transitions, num_transitions);
  setupCLI();

  // initialState on Powerup
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

  // Check if 5 minutes has passed
    unsigned long currentTime = millis();
    if (currentTime - lastUpdateTime >= UPDATE_INTERVAL) {
        // Set timestamp for next check
        lastUpdateTime = currentTime;

        Serial.println("5 minute interval reached. Checking values for changes...");

        // Check if the current values differ from the last saved ones
        if (currentPattern != lastSavedPattern ||
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
            currentEnabledPatternMask != lastSavedEnabledPatternMask) {
            // At least one value has changed
            Serial.println("Values have changed. Saving new values to EEPROM...");
            saveConfigToEEPROM(true);
        } else {
            // No change
            Serial.println("Values are unchanged. No EEPROM update needed.");
        }
    }


  // send the 'leds' array out to the actual LED strip
  clearInactiveLeds();
  syncLogicalToPhysicalLeds();
  FastLED.show();
  lastWorkDurationUs = micros() - loopStartUs;
  totalWorkDurationUs += lastWorkDurationUs;
  if (lastWorkDurationUs > maxWorkDurationUs)
  {
    maxWorkDurationUs = lastWorkDurationUs;
  }
  // insert a delay to keep the framerate modest
  delay(1000 / FRAMES_PER_SECOND);

  //FastLED.countFPS();
  // Serial.println(LEDS.getFPS());
  // Serial.println(FastLED.getFPS());
  // Serial.println(fsm.getPreviousState()->getID());
  // Serial.println(fsm.getDotDefinition());
  // Serial.println(State3());

  // do some periodic updates
  EVERY_N_MILLISECONDS(20) { gHue++; } // slowly cycle the "base color" through the rainbow

  // USB power state for charging logic and serial session detection for CLI.
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
  // Disable charging view while a serial session is active.
  UsbConnected = (UsbPowerRaw == 1 && !SerialSessionActive) ? 1 : 0;

  fsm.run(0);
  multiresponseButton.poll();

  if (multiresponseButton.longPress())
  {
    fsm.trigger(longpress);
    // Serial.println("longpress");
  }

  if (multiresponseButton.doubleClick())
  {
    if (!batteryViewActive && !UsbConnected)
    {
      switchToPattern(getNextEnabledPattern((uint8_t)currentPattern), false);
    }
    // Serial.println("singleclick");
  }

  //UsbConnected = 0;
  if (UsbConnected == 1)
  {
    fsm.trigger(usbpower);
  }

  handleCLI();

  if (multiresponseButton.singleClick() && batteryViewActive)
  {
    BRIGHTNESS += 32; // brightness = 95-255, so steps of 32
    if (BRIGHTNESS > MAX_BRIGHTNESS)
    {
      BRIGHTNESS = MIN_BRIGHTNESS; // we roll over to minimum bright
    }
    // Serial.println(BRIGHTNESS);
    FastLED.setBrightness(BRIGHTNESS);
    currentBrightness = BRIGHTNESS;
    batteryViewLastInteractionMs = millis();
  }

  lastLoopDurationUs = micros() - loopStartUs;
  totalLoopDurationUs += lastLoopDurationUs;
  if (lastLoopDurationUs > maxLoopDurationUs)
  {
    maxLoopDurationUs = lastLoopDurationUs;
  }
  loopTimingSamples++;
}
