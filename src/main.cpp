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
//  EEPROM LAYOUT
// ============================================================================

// Memory addresses in the emulated EEPROM
// An int on the Pico is 4 bytes.
// We store value1 at address 0 and value2 directly after it.
#define EEPROM_ADDR_VALUE1 0
#define EEPROM_ADDR_VALUE2 sizeof(int) // Address for value2, directly after value1

// Size of emulated EEPROM.
// has to big enough to store our variables
// 2 * sizeof(int) = 8 Bytes.
#define EEPROM_SIZE 16 // 16 Bytes should be plenty

// variables we want to store in EEPROM
int currentPattern = 2;
int currentBrightness = 95;

// copies of current values
// needed to detect changes (to limit wear of flash memory)
int lastSavedPattern = 2;
int lastSavedBrightness = 95;

// for timekeeping
unsigned long lastUpdateTime = 0;
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

// Battery variables
int UsbConnected = 0;
int RawVoltage = 0;
float Voltage = 0;

unsigned long previousMillis = 0; // will store last time LED was updated
unsigned long currentMillis = 0;
bool blinkState;
bool blink;

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

inline uint32_t smoothedMotion() {
  uint32_t s = (uint32_t)abs(aaWorld.x) + (uint32_t)abs(aaWorld.y);
  myAccel.add(s);
  return myAccel.get();
}

// ============================================================================
//  FSM STATES — ENTRY/RUN/EXIT
// ============================================================================

void ChargingEntry()
{
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
}

void RunEntry()
{
  fill_solid(Strip, NUM_LEDS * 2, CRGB::Black);
  currentPattern = 2;
}

void running()
{
  fill_rainbow(Strip, NUM_LEDS * 2, gHue, 7);
}

void RunEntry2()
{
  fill_solid(Strip, NUM_LEDS * 2, CRGB::Black);
  currentPattern = 3;
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
  currentPattern = 4;
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
  currentPattern = 5;
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
  currentPattern = 6;
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
  fill_solid(Strip, NUM_LEDS * 2, CRGB::Black);
  currentPattern = 7;
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
  fill_solid(Strip, NUM_LEDS * 2, CRGB::Black);

  currentPattern = 8;
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
  currentPattern = 9;
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
  currentPattern = 10;
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
  fill_solid(Strip, NUM_LEDS * 2, CRGB::Black);
  currentPattern = 11;
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
  currentPattern = 12;
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
  currentPattern = 13;
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
  currentPattern = 14;
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

enum triggers
{
  doubleClick = 1,
  longpress,
  usbpower
};

template<int N>
static inline bool PatternIs() { return currentPattern == N; }

template<int N>
static inline bool unplugged() {
  return !UsbConnected && (currentPattern == N);
}

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
  mpu.setFullScaleAccelRange(2);
  mpu.setFullScaleGyroRange(2);
  Serial.print("Accelrange:");
  Serial.println(mpu.getFullScaleAccelRange());
  Serial.print("Gyrorange:");
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
  myAccel.begin(SMOOTHED_AVERAGE, 100);

  // Although it is unnecessary here, the stored values can be cleared if needed.
  myAccel.clear();

// Init the emulated EEPROM
    EEPROM.begin(EEPROM_SIZE);
    Serial.println("EEPROM initialized.");


    // Read values from EEPROM
    EEPROM.get(EEPROM_ADDR_VALUE1, currentPattern);
    EEPROM.get(EEPROM_ADDR_VALUE2, currentBrightness);
  // Guards
  if (currentPattern < 2 || currentPattern > 14) currentPattern = 2;
  if (currentBrightness < 95 || currentBrightness > 255) currentBrightness = 95;
    Serial.println("Current values:");
    Serial.print("Pattern: ");
    Serial.println(currentPattern);
    Serial.print("Brightness: ");
    Serial.println(currentBrightness);


    // Store current values for changecheck
    lastSavedPattern = currentPattern;
    lastSavedBrightness = currentBrightness;

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
}

// ============================================================================
//  LOOP
// ============================================================================

void loop()
{
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

  // Check if 5 minutes has passed
    unsigned long currentTime = millis();
    if (currentTime - lastUpdateTime >= UPDATE_INTERVAL) {
        // Set timestamp for next check
        lastUpdateTime = currentTime;

        Serial.println("5 minute interval reached. Checking values for changes...");

        // Check if the current values differ from the last saved ones
        if (currentPattern != lastSavedPattern || currentBrightness != lastSavedBrightness) {
            // At least one value has changed
            Serial.println("Values have changed. Saving new values to EEPROM...");

           // Update values in EEPROM
            EEPROM.put(EEPROM_ADDR_VALUE1, currentPattern);
            EEPROM.put(EEPROM_ADDR_VALUE2, currentBrightness);
            
            Serial.print("Pattern: ");
            Serial.println(currentPattern);
            Serial.print("Brightness: ");
            Serial.println(currentBrightness);

            // Important: For flash-based emulation, EEPROM.commit() must be called
            // to actually write the changes to flash!
            if (EEPROM.commit()) {
                Serial.println("New values successfully saved to EEPROM.");

                // Update the last saved values for the next check
                lastSavedPattern = currentPattern;
                lastSavedBrightness = currentBrightness;

            } else {
                Serial.println("ERROR: Could not save values to EEPROM.");
            }

        } else {
            // No change
            Serial.println("Values are unchanged. No EEPROM update needed.");
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
    // Serial.println("longpress");
  }

  if (multiresponseButton.doubleClick())
  {
    fsm.trigger(doubleClick);
    // Serial.println("singleclick");
  }

  UsbConnected = digitalRead(24);
  //UsbConnected = 0;
  if (UsbConnected == 1)
  {
    fsm.trigger(usbpower);
  }

  if (multiresponseButton.singleClick())
  {
    BRIGHTNESS += 32; // brightness = 95-255, so steps of 32
    if (BRIGHTNESS > MAX_BRIGHTNESS)
    {
      BRIGHTNESS = 95; // we roll over to 95 bright
    }
    // Serial.println(BRIGHTNESS);
    FastLED.setBrightness(BRIGHTNESS);
    currentBrightness = BRIGHTNESS;
  }
}
