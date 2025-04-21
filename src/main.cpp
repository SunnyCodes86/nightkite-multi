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

// #include "I2Cdev.h"
// #include "MPU6050_6Axis_MotionApps20.h"
#include "MPU6050_6Axis_MotionApps612.h" // Uncomment this library to work with DMP 6.12 and comment on the above library.
// FastLED
#include <FastLED.h>
// State Machine
#include "SimpleFSM.h"
// Button library
#include "avdweb_Switch.h"
// Smoothing library
#include <Smoothed.h>

/* MPU6050 default I2C address is 0x68*/
MPU6050 mpu;
// MPU6050 mpu(0x69); //Use for AD0 high
// MPU6050 mpu(0x68, &Wire1); //Use for AD0 low, but 2nd Wire (TWI/I2C) object.

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

// Controller definition für offsets und led reihenfolge
#define Controller2

int const INTERRUPT_PIN = 3; // Define the interruption #0 pin
bool blinkState;

// Generally, you should use "unsigned long" for variables that hold time
// The value will quickly become too large for an int to store
unsigned long previousMillis = 0; // will store last time LED was updated
unsigned long currentMillis = millis();
// constants won't change:
const long interval = 1000; // interval at which to blink (milliseconds)
bool blink;

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

/*------Interrupt detection routine------*/
volatile bool MPUInterrupt = false; // Indicates whether MPU6050 interrupt pin has gone high
void DMPDataReady()
{
  MPUInterrupt = true;
}

// LED config

#define PinStrip1 12
#define PinStrip2 13
// #define CLK_PIN   4
#define LED_TYPE WS2811

#ifdef Controller1
#define COLOR_ORDER BGR
#endif
#ifdef Controller2
#define COLOR_ORDER GRB
#endif

#define NUM_LEDS 25
CRGB Strip1[NUM_LEDS];
CRGB Strip2[NUM_LEDS];

int BRIGHTNESS = 95;
#define MAX_BRIGHTNESS 256
#define FRAMES_PER_SECOND 120

uint8_t gHue = 0; // rotating "base color" used by many of the patterns

// led stuff
u_int32_t combined = 0;
u_int32_t CombinedSmooth;

Smoothed<u_int32_t> myAccel;

uint8_t bloodHue = 96;       // Blood color [hue from 0-255]
uint8_t bloodSat = 255;      // Blood staturation [0-255]
int flowDirection = -1;      // Use either 1 or -1 to set flow direction
uint16_t cycleLength = 1300; // Lover values = continuous flow, higher values = distinct pulses.
uint16_t pulseLength = 200;  // How long the pulse takes to fade out.  Higher value is longer.
uint16_t pulseOffset = 250;  // Delay before second pulse.  Higher value is more delay.
uint8_t baseBrightness = 0;  // Brightness of LEDs when not pulsing. Set to 0 for off.

int ledeffect;
int ledeffect2;

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

// Button config
const byte multiresponseButtonpin = 23;
Switch multiresponseButton = Switch(multiresponseButtonpin, INPUT);

// Battery variables
float full_battery = 4.2;  // reference voltages for a full/empty battery, in volts
float empty_battery = 3.2; // the values could vary by battery size/manufacturer so you might need to adjust them
int UsbConnected = 0;
int RawVoltage = 0;
float Voltage = 0;

// FSM init
SimpleFSM fsm;

// variables for mpu to led mapping
int color;
int color2;
int accel;
uint8_t accelcon;
int accelabs;
int fade;

void ChargingEntry()
{
  fill_solid(Strip1, NUM_LEDS, CRGB::Black);
  fill_solid(Strip2, NUM_LEDS, CRGB::Black);
  FastLED.setBrightness(32);
}
void ChargingRunning()
{
  RawVoltage = analogRead(29);
  Voltage = RawVoltage * 3.0 * 3.3 / 4096.0;

  UsbConnected = digitalRead(24);
  currentMillis = millis();

  if (currentMillis - previousMillis >= 500)
  {
    // save the last time you blinked the LED
    previousMillis = currentMillis;
    // if the LED is off turn it on and vice-versa:
    if (blink == 0)
    {
      blink = 1;
    }
    else
    {
      blink = 0;
    }
  }

  if (UsbConnected == 1 && blink == 1)
  {
    fill_solid(Strip2, 1, CRGB::Red);
  }
  else if (UsbConnected == 1 && blink == 0)
  {
    fill_solid(Strip2, 1, CRGB::Black);
  }

  if (Voltage >= 4.2)
  {
    fill_solid(Strip1, 5, CRGB::Blue);
  }
  if (Voltage < 4.2 && Voltage > 4.0)
  {
    fill_solid(Strip1, 4, CRGB::Green);
  }
  if (Voltage < 4.0 && Voltage > 3.8)
  {
    fill_solid(Strip1, 3, CRGB::Green);
  }
  if (Voltage < 3.8 && Voltage > 3.6)
  {
    fill_solid(Strip1, 2, CRGB::Yellow);
  }
  if (Voltage < 3.6 && Voltage > 3.4)
  {
    fill_solid(Strip1, 1, CRGB::Yellow);
  }
  if (Voltage < 3.4 && Voltage > 3.2)
  {
    fill_solid(Strip1, 1, CRGB::Red);
  }
  if (Voltage < 3.2 && Voltage > 3.0)
  {
    fill_solid(Strip1, 1, CRGB::Black);
  }
  if (Voltage < 3.0 && Voltage > 2.8)
  {
    fill_solid(Strip1, 1, CRGB::Black);
  }
}

void ChargingExit()
{
  FastLED.setBrightness(BRIGHTNESS);
}

void BatteryEntry()
{
  fill_solid(Strip1, NUM_LEDS, CRGB::Black);
  fill_solid(Strip2, NUM_LEDS, CRGB::Black);
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
    if (blink == 0)
    {
      blink = 1;
    }
    else
    {
      blink = 0;
    }
  }

  if (blink == 1)
  {
    fill_solid(Strip2, 1, CRGB::Blue);
  }
  else if (blink == 0)
  {
    fill_solid(Strip2, 1, CRGB::Black);
  }

  if (Voltage >= 4.2)
  {
    fill_solid(Strip1, 5, CRGB::Blue);
  }
  if (Voltage < 4.2 && Voltage > 4.0)
  {
    fill_solid(Strip1, 4, CRGB::Green);
  }
  if (Voltage < 4.0 && Voltage > 3.8)
  {
    fill_solid(Strip1, 3, CRGB::Green);
  }
  if (Voltage < 3.8 && Voltage > 3.6)
  {
    fill_solid(Strip1, 2, CRGB::Yellow);
  }
  if (Voltage < 3.6 && Voltage > 3.4)
  {
    fill_solid(Strip1, 1, CRGB::Yellow);
  }
  if (Voltage < 3.4 && Voltage > 3.2)
  {
    fill_solid(Strip1, 1, CRGB::Red);
  }
  if (Voltage < 3.2 && Voltage > 3.0)
  {
    fill_solid(Strip1, 1, CRGB::Black);
  }
  if (Voltage < 3.0 && Voltage > 2.8)
  {
    fill_solid(Strip1, 1, CRGB::Black);
  }
}

void RunEntry()
{
  fill_solid(Strip1, NUM_LEDS, CRGB::Black);
  fill_solid(Strip2, NUM_LEDS, CRGB::Black);
}

void running()
{
  fill_rainbow(Strip1, NUM_LEDS, gHue, 7);
  fill_rainbow(Strip2, NUM_LEDS, gHue, 7);
}

void running2()
{
  color = map(color, -180, 180, 0, 255);
  fill_solid(Strip1, NUM_LEDS, CHSV(color, 255, 255));
  fill_solid(Strip2, NUM_LEDS, CHSV(color, 255, 255));
}

void RunEntry3()
{
  fill_solid(Strip1, NUM_LEDS, CRGB::Black);
  fill_solid(Strip2, NUM_LEDS, CRGB::Black);
  FastLED.setBrightness(30);
}

void running3()
{
#define MASTER_BRIGHTNESS 255  // Set the master brigtness value [should be greater then min_brightness value].
  uint8_t min_brightness = 30; // Set a minimum brightness level.
  combined = abs(aaWorld.x) + abs(aaWorld.y);
  myAccel.add(combined);
  CombinedSmooth = myAccel.get();
  accel = map(CombinedSmooth, 2000, 20000, min_brightness, MASTER_BRIGHTNESS);
  accelcon = constrain(accel, min_brightness, MASTER_BRIGHTNESS);
  color = map(color, -180, 180, 0, 255);
  FastLED.setBrightness(accelcon); // Set master brightness based on potentiometer position.

  fill_solid(Strip1, NUM_LEDS, CHSV(color, 255, 255));
  fill_solid(Strip2, NUM_LEDS, CHSV(color, 255, 255));
}

void RunExit3()
{
  fill_solid(Strip1, NUM_LEDS, CRGB::Black);
  fill_solid(Strip2, NUM_LEDS, CRGB::Black);
  FastLED.setBrightness(BRIGHTNESS);
}

void running4()
{
  color = map(color, -180, 180, 0, 255);
  uint8_t pos = map(beat16(40, 0), 0, 65535, 0, NUM_LEDS - 1);
  Strip1[pos] = CHSV(color, 200, 255);
  Strip2[pos] = CHSV(color, 200, 255);

  fadeToBlackBy(Strip1, NUM_LEDS, 12);
  fadeToBlackBy(Strip2, NUM_LEDS, 12);
}

void running5()
{
  color = map(color, -180, 180, 0, 255);
  combined = abs(aaWorld.x) + abs(aaWorld.y);
  myAccel.add(combined);
  CombinedSmooth = myAccel.get();
  accel = map(CombinedSmooth, 2000, 20000, 100, 0);
  fade = map(accel, 20, 160, 60, 12);
  accel = constrain(accel, 0, 100);

  EVERY_N_MILLIS_I(timingObj, 1)
  {

    if (ledeffect == NUM_LEDS)
    {
      ledeffect = 0;
    }

    fadeToBlackBy(Strip1, NUM_LEDS, fade);
    fadeToBlackBy(Strip2, NUM_LEDS, fade);

    Strip1[ledeffect] = CHSV(color, 255, 255);
    Strip2[ledeffect] = CHSV(color, 255, 255);

    ledeffect = ledeffect + 1;

    timingObj.setPeriod(accel);
  }
}

void running6()
{
  color = map(color, -180, 180, 0, 255);
  color2 = map(color2, 180, -180, 0, 255);
  combined = abs(aaWorld.x) + abs(aaWorld.y);
  myAccel.add(combined);
  CombinedSmooth = myAccel.get();
  accel = map(CombinedSmooth, 2000, 20000, 100, 0);
  fade = map(accel, 20, 160, 60, 12);
  accel = constrain(accel, 0, 100);

  EVERY_N_MILLIS_I(timingObj, 1)
  {

    if (ledeffect == NUM_LEDS)
    {
      ledeffect = 0;
      ledeffect2 = 0;
    }

    fadeToBlackBy(Strip1, NUM_LEDS, fade);
    fadeToBlackBy(Strip2, NUM_LEDS, fade);

    Strip1[ledeffect] = CHSV(color, 255, 255);
    Strip1[ledeffect2] = CHSV(color2, 255, 255);
    Strip2[ledeffect] = CHSV(color, 255, 255);
    Strip2[ledeffect2] = CHSV(color2, 255, 255);

    ledeffect2 = ledeffect;
    ledeffect = ledeffect + 1;

    timingObj.setPeriod(accel);
  }
}

void running7()
{
  color = map(color, -180, 180, 0, 255);
  bloodHue = color; // Blood color [hue from 0-255]

  for (int i = 0; i < NUM_LEDS; i++)
  {
    uint8_t bloodVal = sumPulse((5 / NUM_LEDS / 2) + (NUM_LEDS / 2) * i * flowDirection);
    Strip1[i] = CHSV(bloodHue, bloodSat, bloodVal);
    Strip2[i] = CHSV(bloodHue, bloodSat, bloodVal);
  }
}

void running8()
{
  color = map(color, -180, 180, 0, 255);
  combined = abs(aaWorld.x) + abs(aaWorld.y);
  myAccel.add(combined);
  CombinedSmooth = myAccel.get();
  accel = map(CombinedSmooth, 0, 10000, 20, 160);
  fade = map(accel, 20, 160, 48, 6);
  fade = constrain(fade, 6, 48);

  uint8_t pos = map(beatsin16(80, 0), 0, 65535, 0, NUM_LEDS - 1);

  Strip1[pos] = CHSV(color, 200, 255);
  Strip2[pos] = CHSV(color, 200, 255);

  fadeToBlackBy(Strip1, NUM_LEDS, fade);
  fadeToBlackBy(Strip2, NUM_LEDS, fade);
}

State s[] = {
    State("battery", BatteryEntry, BatteryRunning),
    State("charging", ChargingEntry, ChargingRunning, ChargingExit),
    State("running", RunEntry, running),
    State("running2", RunEntry, running2),
    State("running3", RunEntry3, running3, RunExit3),
    State("running4", RunEntry, running4),
    State("running5", RunEntry, running5),
    State("running6", RunEntry, running6),
    State("running7", RunEntry, running7),
    State("running8", RunEntry, running8)};

enum triggers
{
  doubleClick = 1,
  longpress,
  usbpower
};

bool unplugged()
{
  return !UsbConnected;
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
    Transition(&s[2], &s[3], doubleClick),
    Transition(&s[3], &s[4], doubleClick),
    Transition(&s[4], &s[5], doubleClick),
    Transition(&s[5], &s[6], doubleClick),
    Transition(&s[6], &s[7], doubleClick),
    Transition(&s[7], &s[8], doubleClick),
    Transition(&s[8], &s[9], doubleClick),
    Transition(&s[9], &s[2], doubleClick),
    Transition(&s[0], &s[1], usbpower),
    Transition(&s[2], &s[1], usbpower),
    Transition(&s[3], &s[1], usbpower),
    Transition(&s[4], &s[1], usbpower),
    Transition(&s[5], &s[1], usbpower),
    Transition(&s[6], &s[1], usbpower),
    Transition(&s[7], &s[1], usbpower),
    Transition(&s[8], &s[1], usbpower),
    Transition(&s[9], &s[1], usbpower)};

TimedTransition timedTransitions[] = {
    TimedTransition(&s[0], &s[2], 5000),
    TimedTransition(&s[1], &s[2], 2000, NULL, "", unplugged)};

int num_transitions = sizeof(transitions) / sizeof(Transition);
int num_timed = sizeof(timedTransitions) / sizeof(TimedTransition);

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
#ifdef Controller1
  /*controller1 */
  mpu.setXGyroOffset(80);
  mpu.setYGyroOffset(-13);
  mpu.setZGyroOffset(14);
  mpu.setXAccelOffset(-503);
  mpu.setYAccelOffset(-537);
  mpu.setZAccelOffset(2601);
#endif
#ifdef Controller2
  /*controller2*/
  mpu.setXGyroOffset(111);
  mpu.setYGyroOffset(-6);
  mpu.setZGyroOffset(34);
  mpu.setXAccelOffset(-3137);
  mpu.setYAccelOffset(-7);
  mpu.setZAccelOffset(3687);
#endif

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
  FastLED.addLeds<LED_TYPE, PinStrip1, COLOR_ORDER>(Strip1, NUM_LEDS).setCorrection(TypicalLEDStrip);
  FastLED.addLeds<LED_TYPE, PinStrip2, COLOR_ORDER>(Strip2, NUM_LEDS).setCorrection(TypicalLEDStrip);

  // set master brightness control
  FastLED.setBrightness(BRIGHTNESS);

  // FastLED.setMaxRefreshRate(120);

  myAccel.begin(SMOOTHED_AVERAGE, 100);

  // Although it is unnecessary here, the stored values can be cleared if needed.
  myAccel.clear();

  fsm.add(timedTransitions, num_timed);
  fsm.add(transitions, num_transitions);

  // initialState on Powerup
  fsm.setInitialState(&s[2]);
}

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

  // send the 'leds' array out to the actual LED strip
  // FastLED.show();
  // insert a delay to keep the framerate modest
  FastLED.delay(1000 / FRAMES_PER_SECOND);

  // FastLED.countFPS();
  // Serial.println(LEDS.getFPS());
  // Serial.println(FastLED.getFPS());
  //  Serial.println(fsm.getPreviousState()->getID());
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
  // UsbConnected = 0;
  if (UsbConnected == 1)
  {
    fsm.trigger(usbpower);
  }

  if (multiresponseButton.singleClick())
  {
    BRIGHTNESS += 32; // brightness = 95-255, so steps of 32
    if (BRIGHTNESS >= MAX_BRIGHTNESS)
    {
      BRIGHTNESS = 95; // we roll over to 95 bright
    }
    // Serial.println(BRIGHTNESS);
    FastLED.setBrightness(BRIGHTNESS);
  }
}
