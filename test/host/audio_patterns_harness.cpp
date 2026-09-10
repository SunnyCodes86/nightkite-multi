// Runs the actual firmware implementations of patterns 23-27 with a tiny
// FastLED-compatible host shim. ASan/UBSan guard the supported strip lengths.
#include <assert.h>
#include <algorithm>
#include <initializer_list>
#include <math.h>
#include <stdio.h>
#include <stdint.h>
#include <string.h>

#include "app/AudioPatternMath.h"

#define MAX_LEDS_PER_STRIP 35

template <typename T> static T min(T a, T b) { return a < b ? a : b; }
template <typename T> static T max(T a, T b) { return a > b ? a : b; }
template <typename T> static T constrain(T value, T low, T high)
{
  return min(max(value, low), high);
}

static uint8_t qadd8(uint8_t a, uint8_t b)
{
  const uint16_t sum = (uint16_t)a + b;
  return sum > 255 ? 255 : (uint8_t)sum;
}

static uint8_t qsub8(uint8_t a, uint8_t b) { return a > b ? a - b : 0; }
static uint8_t scale8(uint8_t value, uint8_t scale)
{
  return (uint8_t)(((uint16_t)value * (scale + 1U)) >> 8);
}

static uint8_t lerp8by8(uint8_t a, uint8_t b, uint8_t fraction)
{
  return b >= a
      ? a + scale8(b - a, fraction)
      : a - scale8(a - b, fraction);
}

static uint8_t sin8(uint8_t phase)
{
  const double radians = ((double)phase / 256.0) * 6.2831853071795864769;
  return (uint8_t)(127.5 + sin(radians) * 127.5 + 0.5);
}

static uint32_t fakeMillis = 0;
static unsigned long millis() { return fakeMillis; }

struct CHSV
{
  uint8_t h, s, v;
  CHSV(uint8_t hue, uint8_t saturation, uint8_t value) : h(hue), s(saturation), v(value) {}
};

struct CRGB
{
  uint8_t r = 0, g = 0, b = 0;
  bool initialized = true;
  CRGB() = default;
  CRGB(uint8_t red, uint8_t green, uint8_t blue) : r(red), g(green), b(blue) {}
  CRGB(CHSV hsv)
  {
    const uint8_t region = hsv.h / 43;
    const uint8_t remainder = (hsv.h - region * 43) * 6;
    const uint8_t p = scale8(hsv.v, 255 - hsv.s);
    const uint8_t q = scale8(hsv.v, 255 - scale8(hsv.s, remainder));
    const uint8_t t = scale8(hsv.v, 255 - scale8(hsv.s, 255 - remainder));
    switch (region)
    {
      case 0: r = hsv.v; g = t; b = p; break;
      case 1: r = q; g = hsv.v; b = p; break;
      case 2: r = p; g = hsv.v; b = t; break;
      case 3: r = p; g = q; b = hsv.v; break;
      case 4: r = t; g = p; b = hsv.v; break;
      default: r = hsv.v; g = p; b = q; break;
    }
  }
  CRGB& operator+=(CRGB other)
  {
    r = qadd8(r, other.r); g = qadd8(g, other.g); b = qadd8(b, other.b);
    initialized = true;
    return *this;
  }
  static const CRGB Black;
};
const CRGB CRGB::Black;

static void fill_solid(CRGB* leds, int count, CRGB color)
{
  for (int i = 0; i < count; ++i) leds[i] = color;
}

static void nblend(CRGB& current, CRGB target, uint8_t amount)
{
  current.r = current.r + (((int)target.r - current.r) * (amount + 1) >> 8);
  current.g = current.g + (((int)target.g - current.g) * (amount + 1) >> 8);
  current.b = current.b + (((int)target.b - current.b) * (amount + 1) >> 8);
  current.initialized = true;
}

int NUM_LEDS = 25;
int TOTAL_LEDS = 50;
bool batteryViewActive = false;
AudioPatternFrame audioPatternFrame;
static CRGB storage[72];
#define Strip (storage + 1)

static uint8_t audioPatternLocalHue() { return 37; }
static int audioPatternPitchOffset() { return 9; }
static int getPatternDirectionFactor(uint8_t) { return 1; }

#include "audio_patterns_under_test.inc"

static void entry(uint8_t pattern)
{
  switch (pattern)
  {
    case 23: RunEntry23(); break;
    case 24: RunEntry24(); break;
    case 25: RunEntry25(); break;
    case 26: RunEntry26(); break;
    case 27: RunEntry27(); break;
  }
}

static void render(uint8_t pattern, const AudioPatternFrame& frame)
{
  audioPatternFrame = frame;
  if (!frame.valid)
  {
    fill_solid(Strip, TOTAL_LEDS, CRGB::Black);
    return;
  }
  switch (pattern)
  {
    case 23: running23(); break;
    case 24: running24(); break;
    case 25: running25(); break;
    case 26: running26(); break;
    case 27: running27(); break;
  }
}

static unsigned brightness(const CRGB& pixel) { return pixel.r + pixel.g + pixel.b; }
static uint8_t channelPeak(const CRGB& pixel) { return max(pixel.r, max(pixel.g, pixel.b)); }

struct Visibility
{
  uint8_t peak;
  uint8_t median;
  int average;
  int visiblePercent;
};

static Visibility visibility()
{
  uint8_t values[70];
  unsigned sum = 0;
  int visible = 0;
  for (int i = 0; i < TOTAL_LEDS; ++i)
  {
    values[i] = channelPeak(Strip[i]);
    sum += values[i];
    if (values[i] >= 20) ++visible;
  }
  std::sort(values, values + TOTAL_LEDS);
  return {values[TOTAL_LEDS - 1], values[TOTAL_LEDS / 2],
          (int)(sum / TOTAL_LEDS), visible * 100 / TOTAL_LEDS};
}

static AudioPatternFrame frame(bool locked = true)
{
  AudioPatternFrame result;
  result.valid = true;
  result.fresh = true;
  result.beatLocked = locked;
  result.energy = 210;
  result.bass = 180;
  result.mid = 140;
  result.treble = 100;
  result.confidence = 220;
  return result;
}

static void resetStorage()
{
  for (CRGB& pixel : storage)
  {
    pixel = CRGB(0xA5, 0x5A, 0xC3);
    pixel.initialized = false;
  }
}

static void assertGuardsAndPixels()
{
  assert(!storage[0].initialized && !storage[71].initialized);
  assert(storage[0].r == 0xA5 && storage[71].b == 0xC3);
  for (int i = 0; i < TOTAL_LEDS; ++i) assert(Strip[i].initialized);
}

static void testLengthsLossAndReentry()
{
  const int lengths[] = {10, 15, 25, 35};
  for (int length : lengths)
  {
    NUM_LEDS = length;
    TOTAL_LEDS = length * 2;
    for (uint8_t pattern = 23; pattern <= 27; ++pattern)
    {
      resetStorage();
      entry(pattern);
      render(pattern, frame());
      assertGuardsAndPixels();

      render(pattern, AudioPatternFrame{});
      for (int i = 0; i < TOTAL_LEDS; ++i)
        assert(Strip[i].r == 0 && Strip[i].g == 0 && Strip[i].b == 0);

      AudioPatternFrame reentry = frame(false);
      reentry.energy = 40;
      render(pattern, reentry);
      CRGB reacquired[70];
      memcpy(reacquired, Strip, sizeof(CRGB) * TOTAL_LEDS);
      entry(pattern);
      render(pattern, reentry);
      assert(memcmp(reacquired, Strip, sizeof(CRGB) * TOTAL_LEDS) == 0);
    }
  }
}

static void testNoLockAndDeterminism()
{
  NUM_LEDS = 25;
  TOTAL_LEDS = 50;
  for (uint8_t pattern = 23; pattern <= 27; ++pattern)
  {
    AudioPatternFrame unlocked = frame(false);
    unlocked.phase8 = 19;
    unlocked.beatPulse = 240;
    resetStorage(); entry(pattern); render(pattern, unlocked);
    CRGB first[70]; memcpy(first, Strip, sizeof(CRGB) * TOTAL_LEDS);
    unsigned light = 0;
    for (int i = 0; i < TOTAL_LEDS; ++i) light += brightness(Strip[i]);
    assert(light > 0);

    resetStorage(); entry(pattern); render(pattern, unlocked);
    assert(memcmp(first, Strip, sizeof(CRGB) * TOTAL_LEDS) == 0);

    if (pattern >= 25)
    {
      unlocked.phase8 = 211;
      unlocked.beatPulse = 7;
      resetStorage(); entry(pattern); render(pattern, unlocked);
      assert(memcmp(first, Strip, sizeof(CRGB) * TOTAL_LEDS) == 0);
    }
  }
}

static void renderTimed(uint8_t pattern, AudioPatternFrame& value, uint32_t dtMs)
{
  fakeMillis += dtMs;
  render(pattern, value);
  value.fresh = false;
}

static void settle(uint8_t pattern, AudioPatternFrame& value, int frames = 12)
{
  for (int i = 0; i < frames; ++i) renderTimed(pattern, value, 40);
}

static void testStablePulse()
{
  NUM_LEDS = 35; TOTAL_LEDS = 70;
  AudioPatternFrame value = frame(false);
  value.energy = 120; value.bass = 40;
  value.beat = false; value.beatPulse = 0;
  entry(23); settle(23, value);
  int stableMin = 255, stableMax = 0;
  for (int i = 0; i < 12; ++i)
  {
    renderTimed(23, value, 40);
    stableMin = min(stableMin, (int)visibility().median);
    stableMax = max(stableMax, (int)visibility().median);
  }
  assert(stableMax - stableMin <= 1);

  const uint8_t energies[] = {40, 60, 80, 100, 120, 100, 80, 60, 40};
  int levels[9];
  entry(23);
  for (int step = 0; step < 9; ++step)
  {
    value.fresh = step == 0;
    value.energy = energies[step];
    settle(23, value);
    levels[step] = visibility().median;
  }
  for (int step = 1; step < 5; ++step) assert(levels[step] > levels[step - 1]);
  for (int step = 5; step < 9; ++step) assert(levels[step] < levels[step - 1]);

  AudioPatternFrame noBeat = frame();
  noBeat.energy = 120; noBeat.bass = 40; noBeat.confidence = 220;
  noBeat.beat = false; noBeat.beatPulse = 0;
  entry(23); settle(23, noBeat);
  const int plain = visibility().peak;
  const uint8_t pulses[] = {255, 192, 128, 64, 0};
  int decay[5];
  for (int step = 0; step < 5; ++step)
  {
    noBeat.beat = step == 0;
    noBeat.beatPulse = pulses[step];
    renderTimed(23, noBeat, 32);
    decay[step] = visibility().peak;
  }
  assert(decay[0] >= 190);
  assert(decay[1] >= 190);
  assert(decay[2] < decay[1] && decay[3] < decay[2]);
  assert(decay[0] - plain >= 130);

  AudioPatternFrame unlocked = noBeat;
  unlocked.fresh = true; unlocked.beatLocked = false;
  unlocked.beat = true; unlocked.beatPulse = 255;
  entry(23); render(23, unlocked);
  assert(visibility().peak == plain);

  render(23, AudioPatternFrame{});
  for (int i = 0; i < TOTAL_LEDS; ++i) assert(channelPeak(Strip[i]) == 0);
  unlocked.fresh = true;
  render(23, unlocked);
  CRGB reacquired[70];
  memcpy(reacquired, Strip, sizeof(CRGB) * TOTAL_LEDS);
  entry(23); render(23, unlocked);
  assert(memcmp(reacquired, Strip, sizeof(CRGB) * TOTAL_LEDS) == 0);

  printf("P23 stable_range=%d body_40/60/80/100/120=%d/%d/%d/%d/%d "
         "nonbeat_peak=%d beat_decay=%d/%d/%d/%d/%d\n",
         stableMax - stableMin, levels[0], levels[1], levels[2], levels[3], levels[4],
         plain, decay[0], decay[1], decay[2], decay[3], decay[4]);
}

static int fireHeatTotal()
{
  int total = 0;
  for (int strip = 0; strip < 2; ++strip)
    for (int i = 0; i < NUM_LEDS; ++i)
      total += pattern24Heat[strip][i];
  return total;
}

static int fireHeatHeight(int strip)
{
  int height = 0;
  for (int i = 0; i < NUM_LEDS; ++i)
    if (pattern24Heat[strip][i] >= 32) height = i + 1;
  return height;
}

static int fireRoughness()
{
  int roughness = 0;
  for (int strip = 0; strip < 2; ++strip)
    for (int i = 1; i < NUM_LEDS; ++i)
      roughness += abs(pattern24Heat[strip][i] - pattern24Heat[strip][i - 1]);
  return roughness;
}

static uint32_t frameHash()
{
  uint32_t hash = 2166136261UL;
  for (int i = 0; i < TOTAL_LEDS; ++i)
  {
    hash = (hash ^ Strip[i].r) * 16777619UL;
    hash = (hash ^ Strip[i].g) * 16777619UL;
    hash = (hash ^ Strip[i].b) * 16777619UL;
  }
  return hash;
}

struct FireMetrics
{
  Visibility visual;
  uint32_t finalHash;
  int uniqueFrames;
  int meanFrameDifference;
  int averageHeat;
  int averageHeight;
  int p90Height;
  int p95Height;
  int averageRoughness;
  int maxHeat;
  int maxHeight;
  int sparks;
  int maxSparkPosition;
  int topBodyPercent;
  int headroomPercent;
  int zeroPercent;
  int lowPercent;
  int midPercent;
  int highPercent;
  int hotPercent;
  int peakPixel;
};

static FireMetrics runFire(
    AudioPatternFrame value,
    int length,
    int frames,
    int beatFrame = -1,
    int warmupFrames = 40)
{
  assert(frames <= 60);
  NUM_LEDS = length; TOTAL_LEDS = length * 2;
  fakeMillis = 1000;
  resetStorage();
  entry(24);
  uint32_t hashes[60];
  CRGB previous[70];
  int unique = 0, difference = 0, heat = 0, height = 0, roughness = 0;
  int maxHeat = 0, maxHeight = 0, maxSparkPosition = -1, peakPixel = 0;
  int topBodyFrames = 0, headroomFrames = 0;
  uint32_t zero = 0, low = 0, mid = 0, high = 0, hot = 0;
  int heights[60];
  bool havePrevious = false;
  for (int frameIndex = 0; frameIndex < warmupFrames; ++frameIndex)
  {
    value.fresh = frameIndex == 0;
    value.beat = false;
    renderTimed(24, value, 40);
  }
  for (int frameIndex = 0; frameIndex < frames; ++frameIndex)
  {
    value.fresh = warmupFrames == 0 && frameIndex == 0;
    value.beat = frameIndex == beatFrame;
    renderTimed(24, value, 40);
    assertGuardsAndPixels();
    const uint32_t hash = frameHash();
    bool seen = false;
    for (int i = 0; i < frameIndex; ++i) seen |= hashes[i] == hash;
    hashes[frameIndex] = hash;
    if (!seen) ++unique;
    if (havePrevious)
    {
      for (int i = 0; i < TOTAL_LEDS; ++i)
        difference += abs((int)channelPeak(Strip[i]) - (int)channelPeak(previous[i]));
    }
    memcpy(previous, Strip, sizeof(CRGB) * TOTAL_LEDS);
    havePrevious = true;
    const int frameHeat = fireHeatTotal();
    const int strip0Height = fireHeatHeight(0);
    const int strip1Height = fireHeatHeight(1);
    const int frameHeight = max(strip0Height, strip1Height);
    heights[frameIndex] = frameHeight;
    heat += frameHeat;
    height += strip0Height + strip1Height;
    roughness += fireRoughness();
    maxHeight = max(maxHeight, frameHeight);
    bool topBody = false;
    bool hasHeadroom = true;
    const int headroomStart = (NUM_LEDS * 4 + 4) / 5;
    for (int strip = 0; strip < 2; ++strip)
    {
      for (int i = 0; i < NUM_LEDS; ++i)
      {
        maxHeat = max(maxHeat, (int)pattern24Heat[strip][i]);
        const uint8_t level = channelPeak(Strip[strip * NUM_LEDS + i]);
        peakPixel = max(peakPixel, (int)level);
        if (level == 0) ++zero;
        else if (level < 20) ++low;
        else if (level < 100) ++mid;
        else if (level < 200) ++high;
        else ++hot;
        if (i >= NUM_LEDS - 2 && pattern24Heat[strip][i] >= 32) topBody = true;
        if (i >= headroomStart && pattern24Heat[strip][i] >= 32) hasHeadroom = false;
      }
      for (const Pattern24Spark& spark : pattern24Sparks[strip])
        if (spark.life > 0) maxSparkPosition = max(maxSparkPosition, (int)spark.position);
    }
    topBodyFrames += topBody;
    headroomFrames += hasHeadroom;
  }
  std::sort(heights, heights + frames);
  const uint32_t samples = frames * TOTAL_LEDS;
  FireMetrics result{};
  result.visual = visibility();
  result.finalHash = frameHash();
  result.uniqueFrames = unique;
  result.meanFrameDifference = frames > 1 ? difference / ((frames - 1) * TOTAL_LEDS) : 0;
  result.averageHeat = heat / samples;
  result.averageHeight = height / (frames * 2);
  result.p90Height = heights[(frames * 90 - 1) / 100];
  result.p95Height = heights[(frames * 95 - 1) / 100];
  result.averageRoughness = roughness / frames;
  result.maxHeat = maxHeat;
  result.maxHeight = maxHeight;
  result.sparks = pattern24SparkSpawnCount;
  result.maxSparkPosition = maxSparkPosition;
  result.topBodyPercent = topBodyFrames * 100 / frames;
  result.headroomPercent = headroomFrames * 100 / frames;
  result.zeroPercent = zero * 100 / samples;
  result.lowPercent = low * 100 / samples;
  result.midPercent = mid * 100 / samples;
  result.highPercent = high * 100 / samples;
  result.hotPercent = hot * 100 / samples;
  result.peakPixel = peakPixel;
  return result;
}

static void recordFireHeat(
    AudioPatternFrame value,
    bool locked,
    int beatFrame,
    int* totals,
    int* heights,
    int frames)
{
  NUM_LEDS = 35; TOTAL_LEDS = 70;
  fakeMillis = 5000;
  resetStorage(); entry(24);
  value.beatLocked = locked;
  for (int i = 0; i < 40; ++i)
  {
    value.fresh = i == 0;
    value.beat = false;
    renderTimed(24, value, 40);
  }
  for (int i = 0; i < frames; ++i)
  {
    value.fresh = false;
    value.beat = i == beatFrame;
    renderTimed(24, value, 40);
    totals[i] = fireHeatTotal();
    heights[i] = max(fireHeatHeight(0), fireHeatHeight(1));
  }
}

static void testAudioFirestorm()
{
  AudioPatternFrame typical = frame(false);
  typical.energy = 128; typical.bass = 40; typical.mid = 45; typical.treble = 67;
  FireMetrics diagnostic{};
  for (int length : {10, 15, 25, 35})
  {
    const FireMetrics constant = runFire(typical, length, 50);
    printf("P24 length=%d height_avg/p90/p95/max=%d/%d/%d/%d headroom=%d%% top=%d%%\n",
           length, constant.averageHeight, constant.p90Height, constant.p95Height,
           constant.maxHeight, constant.headroomPercent, constant.topBodyPercent);
    assert(constant.uniqueFrames >= 40);
    assert(constant.meanFrameDifference > 0 && constant.meanFrameDifference < 60);
    assert(constant.averageHeight >= max(3, length / 2));
    assert(constant.averageHeight <= (length * 4) / 5 + 1);
    assert(constant.p95Height < length);
    assert(constant.topBodyPercent < 80);
    assert(constant.headroomPercent > 0);
    assert(constant.maxHeat >= 120);
    if (length == 35) diagnostic = constant;
  }
  const FireMetrics repeat = runFire(typical, 35, 50);
  assert(repeat.finalHash == diagnostic.finalHash);
  assert(repeat.uniqueFrames == diagnostic.uniqueFrames);
  assert(repeat.meanFrameDifference == diagnostic.meanFrameDifference);
  assert(repeat.averageHeat == diagnostic.averageHeat);
  assert(repeat.p95Height == diagnostic.p95Height);
  assert(diagnostic.averageHeight >= 20 && diagnostic.averageHeight <= 28);
  assert(diagnostic.p90Height < 35);
  assert(diagnostic.topBodyPercent < 50);
  assert(diagnostic.headroomPercent >= 20);
  assert(diagnostic.zeroPercent + diagnostic.lowPercent > 5);
  assert(diagnostic.midPercent > 5);
  assert(diagnostic.highPercent + diagnostic.hotPercent < 90);
  assert(diagnostic.hotPercent > 0);
  assert(diagnostic.peakPixel >= 250);

  AudioPatternFrame lowEnergy = typical;
  lowEnergy.energy = 45;
  AudioPatternFrame highEnergy = typical;
  highEnergy.energy = 210;
  const FireMetrics quiet = runFire(lowEnergy, 35, 50);
  const FireMetrics loud = runFire(highEnergy, 35, 50);
  assert(loud.averageHeat > quiet.averageHeat);
  assert(loud.averageHeight > quiet.averageHeight);
  assert(loud.visual.average > quiet.visual.average);
  assert(loud.topBodyPercent < 100);
  assert(loud.p95Height < 35);

  AudioPatternFrame lowBass = typical;
  lowBass.bass = 10; lowBass.treble = 0;
  AudioPatternFrame highBass = lowBass;
  highBass.bass = 200;
  const FireMetrics lightBody = runFire(lowBass, 35, 50);
  const FireMetrics heavyBody = runFire(highBass, 35, 50);
  assert(heavyBody.averageHeat > lightBody.averageHeat + 8);
  assert(heavyBody.maxHeat >= lightBody.maxHeat);
  assert(heavyBody.maxHeight > diagnostic.p95Height);

  AudioPatternFrame lowMid = typical;
  lowMid.bass = 0; lowMid.mid = 8; lowMid.treble = 0;
  AudioPatternFrame highMid = lowMid;
  highMid.mid = 220;
  const FireMetrics calm = runFire(lowMid, 35, 50);
  const FireMetrics storm = runFire(highMid, 35, 50);
  printf("P24 mid_roughness=%d->%d\n", calm.averageRoughness, storm.averageRoughness);
  assert(abs(storm.averageRoughness - calm.averageRoughness) > calm.averageRoughness / 10);

  AudioPatternFrame lowTreble = typical;
  lowTreble.treble = 15;
  AudioPatternFrame highTreble = typical;
  highTreble.treble = 220;
  const FireMetrics fewSparks = runFire(lowTreble, 35, 50);
  const FireMetrics manySparks = runFire(highTreble, 35, 50);
  assert(manySparks.sparks > fewSparks.sparks);
  assert(manySparks.visual.peak >= fewSparks.visual.peak);
  assert(manySparks.averageHeat == fewSparks.averageHeat);
  assert(manySparks.maxSparkPosition >= 28);

  AudioPatternFrame beatValue = typical;
  beatValue.energy = 80; beatValue.bass = 0; beatValue.mid = 0; beatValue.treble = 0;
  beatValue.confidence = 230;
  int baseline[30], beat[30], unlocked[30];
  int baselineHeight[30], beatHeight[30], unlockedHeight[30];
  recordFireHeat(beatValue, true, -1, baseline, baselineHeight, 30);
  recordFireHeat(beatValue, true, 5, beat, beatHeight, 30);
  recordFireHeat(beatValue, false, 5, unlocked, unlockedHeight, 30);
  int flareLife = 0;
  for (int i = 5; i < 30; ++i)
  {
    if (beat[i] > baseline[i] + 20) ++flareLife;
    assert(unlocked[i] == baseline[i]);
    assert(unlockedHeight[i] == baselineHeight[i]);
  }
  assert(beat[5] > baseline[5] + 300);
  assert(flareLife >= 6);
  assert(*std::max_element(beatHeight + 5, beatHeight + 30) >
         *std::max_element(baselineHeight + 5, baselineHeight + 30));
  assert(*std::max_element(beatHeight + 5, beatHeight + 30) >= diagnostic.p95Height);

  NUM_LEDS = 35; TOTAL_LEDS = 70;
  fakeMillis = 9000;
  resetStorage(); entry(24);
  typical.fresh = true; renderTimed(24, typical, 40);
  for (int i = 0; i < 12; ++i) { typical.fresh = false; renderTimed(24, typical, 40); }
  render(24, AudioPatternFrame{});
  for (int i = 0; i < TOTAL_LEDS; ++i) assert(channelPeak(Strip[i]) == 0);
  typical.fresh = true;
  renderTimed(24, typical, 40);
  CRGB reacquired[70]; memcpy(reacquired, Strip, sizeof(CRGB) * TOTAL_LEDS);
  const uint32_t reentryTime = fakeMillis;
  entry(24); fakeMillis = reentryTime - 40; typical.fresh = true; renderTimed(24, typical, 40);
  assert(memcmp(reacquired, Strip, sizeof(CRGB) * TOTAL_LEDS) == 0);

  printf("P24 firestorm peak=%u median=%u avg=%d unique=%d/50 mean_frame_diff=%d "
         "avg_heat=%d height_avg/p90/p95/max=%d/%d/%d/%d top_body=%d%% headroom=%d%% "
         "pixels_zero/low/mid/high/hot=%d/%d/%d/%d/%d%% pixel_peak=%d max_heat=%d sparks=%d "
         "spark_top=%d beat_height=%d->%d beat_flare_frames=%d roughness=%d\n",
         diagnostic.visual.peak, diagnostic.visual.median, diagnostic.visual.average,
         diagnostic.uniqueFrames, diagnostic.meanFrameDifference,
         diagnostic.averageHeat, diagnostic.averageHeight, diagnostic.p90Height,
         diagnostic.p95Height, diagnostic.maxHeight, diagnostic.topBodyPercent,
         diagnostic.headroomPercent, diagnostic.zeroPercent, diagnostic.lowPercent,
         diagnostic.midPercent, diagnostic.highPercent, diagnostic.hotPercent,
         diagnostic.peakPixel, diagnostic.maxHeat, diagnostic.sparks,
         diagnostic.maxSparkPosition,
         *std::max_element(baselineHeight, baselineHeight + 30),
         *std::max_element(beatHeight, beatHeight + 30), flareLife,
         diagnostic.averageRoughness);
  printf("P24 response quiet_heat/height/avg=%d/%d/%d loud=%d/%d/%d "
         "loud_p95/headroom=%d/%d%% bass_heat=%d->%d bass_height=%d->%d "
         "mid_roughness=%d->%d sparks=%d->%d\n",
         quiet.averageHeat, quiet.averageHeight, quiet.visual.average,
         loud.averageHeat, loud.averageHeight, loud.visual.average,
         loud.p95Height, loud.headroomPercent,
         lightBody.averageHeat, heavyBody.averageHeat,
         lightBody.p90Height, heavyBody.maxHeight,
         calm.averageRoughness, storm.averageRoughness,
         fewSparks.sparks, manySparks.sparks);
}

static int brightestIndex()
{
  int best = 0;
  for (int i = 1; i < NUM_LEDS; ++i)
    if (brightness(Strip[i]) > brightness(Strip[best])) best = i;
  return best;
}

static void testRippleTravel()
{
  for (int length : {10, 15, 25, 35})
  {
    NUM_LEDS = length; TOTAL_LEDS = length * 2;
    int previousDistance = -1;
    for (uint8_t phase : {0, 48, 96, 144, 192})
    {
      AudioPatternFrame value = frame();
      value.phase8 = phase;
      value.treble = 220;
      entry(25); render(25, value);
      const int distance = abs(brightestIndex() - NUM_LEDS / 2);
      assert(distance >= previousDistance);
      previousDistance = distance;
    }
  }
}

static void testScannerPositions()
{
  for (int length : {10, 35})
  {
    NUM_LEDS = length; TOTAL_LEDS = length * 2;
    const uint8_t phases[] = {0, 128, 255, 0, 128, 255};
    const int expected[] = {0, (length - 1) / 2, length - 1,
                            length - 1, (length - 1) / 2, 0};
    entry(26);
    for (int p = 0; p < 6; ++p)
    {
      AudioPatternFrame value = frame();
      value.fresh = p == 0;
      value.phase8 = phases[p];
      value.treble = 255;
      fill_solid(Strip, TOTAL_LEDS, CRGB::Black);
      render(26, value);
      const int actual = brightestIndex();
      if (abs(actual - expected[p]) > 1)
        fprintf(stderr, "scanner length=%d step=%d phase=%u expected=%d actual=%d\n",
                length, p, phases[p], expected[p], actual);
      assert(abs(actual - expected[p]) <= 1);
    }
  }
}

static void tileAverages(int& groupA, int& groupB)
{
  groupA = groupB = 0;
  int countA = 0, countB = 0;
  const int zones = constrain(NUM_LEDS / 5, 2, 6);
  for (int i = 0; i < NUM_LEDS; ++i)
  {
    const int zone = (i * zones) / NUM_LEDS;
    if (zone & 1) { groupB += brightness(Strip[i]); ++countB; }
    else { groupA += brightness(Strip[i]); ++countA; }
  }
  groupA /= countA;
  groupB /= countB;
}

static void testRhythmTiles()
{
  for (int length : {10, 15, 25, 35})
  {
    NUM_LEDS = length; TOTAL_LEDS = length * 2;
    AudioPatternFrame value = frame();
    value.bass = value.mid = 220;
    value.treble = 0;
    const uint8_t phases[] = {0, 64, 128, 192, 255, 0, 64, 128, 192, 255};
    int groupA[10], groupB[10];
    resetStorage();
    entry(27);
    for (int step = 0; step < 10; ++step)
    {
      value.fresh = step == 0;
      value.phase8 = phases[step];
      for (int settle = 0; settle < 6; ++settle) renderTimed(27, value, 16);
      tileAverages(groupA[step], groupB[step]);
      assertGuardsAndPixels();
      const Visibility visible = visibility();
      assert(visible.median >= 35);
      assert(visible.visiblePercent >= 80);
    }
    for (int step = 1; step < 5; ++step)
    {
      if (groupA[step] > groupA[step - 1] || groupB[step] < groupB[step - 1] ||
          groupA[step + 5] < groupA[step + 4] || groupB[step + 5] > groupB[step + 4])
        fprintf(stderr, "tiles length=%d step=%d A=%d/%d B=%d/%d returnA=%d/%d returnB=%d/%d\n",
                length, step, groupA[step - 1], groupA[step], groupB[step - 1], groupB[step],
                groupA[step + 4], groupA[step + 5], groupB[step + 4], groupB[step + 5]);
      assert(groupA[step] <= groupA[step - 1]);
      assert(groupB[step] >= groupB[step - 1]);
      assert(groupA[step + 5] >= groupA[step + 4]);
      assert(groupB[step + 5] <= groupB[step + 4]);
    }
    assert(groupA[0] > groupB[0]);
    assert(groupB[4] > groupA[4]);
    assert(groupB[5] > groupA[5]);
    assert(groupA[9] > groupB[9]);
  }
}

static void testFamilyVisibility()
{
  NUM_LEDS = 25;
  TOTAL_LEDS = 50;
  for (uint8_t pattern = 25; pattern <= 26; ++pattern)
  {
    AudioPatternFrame typical = frame();
    typical.energy = 128;
    typical.bass = 40;
    typical.mid = 45;
    typical.treble = 67;
    typical.confidence = 160;
    typical.phase8 = 64;
    typical.beatPulse = 64;
    entry(pattern); render(pattern, typical);
    const Visibility normal = visibility();
    assert(normal.peak >= 150);
    assert(normal.median >= 35);
    assert(normal.visiblePercent >= 80);

    AudioPatternFrame strong = typical;
    strong.energy = 180;
    strong.bass = 180;
    strong.mid = 160;
    strong.treble = 160;
    strong.confidence = 220;
    strong.beatPulse = 180;
    entry(pattern); render(pattern, strong);
    const Visibility loud = visibility();
    assert(loud.peak >= 230);
    assert(loud.median >= 50);
    assert(loud.visiblePercent >= 80);
    if (pattern == 25)
    {
      assert(normal.peak == 255 && normal.median == 74 && normal.average == 112);
      assert(loud.peak == 255 && loud.median == 121 && loud.average == 149);
    }
    else
    {
      assert(normal.peak == 255 && normal.median == 50 && normal.average == 76);
      assert(loud.peak == 255 && loud.median == 77 && loud.average == 112);
    }
    printf("REGRESSION pattern=%u typical_peak=%u typical_median=%u typical_avg=%d typical_visible=%d%% "
           "strong_peak=%u strong_median=%u strong_avg=%d strong_visible=%d%%\n",
           pattern, normal.peak, normal.median, normal.average, normal.visiblePercent,
           loud.peak, loud.median, loud.average, loud.visiblePercent);
  }
}

int main()
{
  testLengthsLossAndReentry();
  testNoLockAndDeterminism();
  testStablePulse();
  testAudioFirestorm();
  testRippleTravel();
  testScannerPositions();
  testRhythmTiles();
  testFamilyVisibility();
  return 0;
}
