# User Manual for the NightKite Multi LED Kite Lighting (v3.0)

Welcome to your **NightKite Multi LED Kite Lighting System**!  
This system brings your stunt kite to life at night by generating dynamic LED effects that respond to the kite’s movement and acceleration.  
The LED color is primarily determined by the kite’s current yaw angle, while speed, brightness, or fade depend on motion intensity and the selected pattern.  

This guide will help you get the most out of your LED lighting system.

---

## Table of Contents

1. [Overview and Features](#1-overview-and-features)  
   1. [Animation Patterns in Detail](#11-animation-patterns-in-detail)  
2. [Components](#2-components)  
3. [Operation and Controls](#3-operation-and-controls)  
   1. [Power On and Initialization](#31-power-on-and-initialization)  
   2. [Functions of the Right Button](#32-functions-of-the-right-button)  
   3. [Power Off](#33-power-off)  
   4. [USB CLI (Configuration)](#34-usb-cli-configuration)  
4. [Power Supply and Charging](#4-power-supply-and-charging)  
5. [Important Notes](#5-important-notes)

---

## 1. Overview and Features

The NightKite Multi lighting system offers the following core features:

- **Motion-Reactive Lighting:**  
  LEDs adapt their color and animation dynamics to the kite’s flight movements  
  *(Angle → Color, Motion → Speed/Fade/Brightness, depending on the pattern)*.
- **Brightness Levels:**  
  Six selectable brightness levels (95 → 255 in steps of 32), which also affect battery life.
- **Animation Patterns:**  
  22 predefined animation modes to customize the kite’s appearance (each affects runtime differently).
- **Autoplay:**  
  Optional automatic cycling through enabled patterns with a configurable global interval.
- **Battery Indicator:**  
  Displays the current battery level directly via the LED strip.
- **Memory Function:**  
  Pattern, brightness, strip length, motion smoothing, sensor ranges, boot calibration, and MPU offsets are stored persistently.
- **USB CLI:**  
  USB serial interface for configuration, diagnostics, timing, and calibration.

---

### 1.1. Animation Patterns in Detail

The NightKite Multi (v3.0) includes 22 predefined animation patterns that can be changed with a double-click.  
After powering on and calibration, the controller starts with the last used pattern and brightness level.  
On first startup: **Pattern ID 1**, **Brightness 95**.

1. **Rainbow Pattern:** Smooth, continuous rainbow cycle across the strip. Not motion-reactive.  
2. **Full String, Angle-Color:** Entire LED strip glows in a color determined by the current yaw angle.  
3. **Full String, Angle-Color & Motion Brightness:** Like (2), but brightness follows motion intensity.  
4. **LED Runner (fixed) with Fadeout, Angle-Color:** A single light point moves at fixed speed; color = angle, smooth fadeout.  
5. **LED Runner (reactive) with Fadeout-Speed, Angle-Color:** Like (4), but speed and fade respond to motion.  
6. **LED Runner (dual-color, reactive), Angle-Color:** Like (5), but with a two-color trail; speed and fade depend on movement.  
7. **Heartbeat (Angle-Color):** Pulsating heartbeat animation; color = angle.  
8. **Ping-Pong (bouncing) with reactive fade:** Point moves back and forth; fadeout duration increases with motion.  
9. **Comet Swarm (4 Comets):** Four short comets race across the currently configured total length (both halves mirrored). Color = angle.  
10. **Breath / Storm (adaptive):**  
    - Calm: smooth “breathing” (brightness sinus).  
    - Turbulent: “storm” with sparks that increase with movement.  
11. **Jerk Wave:** Sudden motion triggers a wave from the center to the ends; wave speed increases with motion; color = angle.  
12. **Yaw Spinner (with direction memory):** Light head with blurred trail follows yaw rate with small dead zone to prevent flicker; color = angle.  
13. **Yaw Spinner Circle:** Variant of (12) where the light head appears to travel in a circular loop across both strips. Direction still follows yaw rate; color = angle.  
14. **LED Runner (dual-color, reactive, inverted):** Like (6), but with inverted running direction.
15. **Palette Beat Motion:** A pulsing palette pattern inspired by FastLED PaletteBeat. Yaw sets the base color family, movement intensity changes pulse rate and palette spread, and yaw movement nudges the apparent flow direction.
16. **Pacifica Kite:** Layered ocean-like waves with soft whitecaps. Motion increases wave energy; yaw shifts the cool color family.
17. **Twinkle Motion:** Soft star-like twinkles on a dim base glow. Motion increases twinkle density and brightness.
18. **Fire Jet:** Fire-like energy building from the center and spreading through the strip. Motion controls spark intensity.
19. **Noise Ring:** Flowing noise-based ring texture with palette movement driven by motion and yaw.
20. **Pride Yaw:** Smooth, saturated flowing rainbow bands with yaw-based hue offset and motion-dependent pace.
21. **Confetti Jerk:** Calm glow with bursts of bright confetti when the kite gets sharp motion impulses.
22. **Center Ripple:** Clear ripple waves starting from the middle and running outward after motion impulses.

---

## 2. Components

The NightKite Multi lighting system consists of the following main components:

- **Microcontroller:** Pimoroni Pico LiPo with integrated battery management (USB-C)  
- **Battery:** 500 mAh LiPo battery connected directly to the microcontroller  
- **Sensor:** MPU6050 (gyroscope/accelerometer) to detect orientation (Yaw/Pitch/Roll) and motion  
- **LED Strips:** Two symmetric strings with equal length (configurable via CLI, 10 to 35 pixels per string), WS281x / “fairy string”, GRB order

---

## 3. Operation and Controls

### 3.1. Power On and Initialization

The controller has two buttons: **left** and **right**.

1. **Power On:** Press the left button.  
2. **Initialization (Calibration):** After power-on, the stored configuration is loaded. By default, a short MPU6050 quick calibration then starts automatically.  
   Hold the controller still for a few seconds to ensure accurate sensor offsets.  
3. **Ready:** After calibration, the LEDs turn on.  
   The system starts with the last used pattern and brightness  
   *(on first use: Pattern ID 1, Brightness 95)*.

---

### 3.2. Functions of the Right Button

The right button is a **multi-function control**:

- **Change Pattern / Animation (double-click):**  
  → Cycles through the 22 available animation patterns.
- **Toggle Autoplay (double-click while the battery display is active):**  
  → Turns autoplay on or off globally.
- **Change Brightness (short press):**  
  → Only while the battery display is active.  
    Switches through the six brightness levels: 95 → 127 → 159 → 191 → 223 → 255 → 95.
- **Show Battery Level (hold):**  
  → Shows the battery level on the main strip.  
    On the second strip, a blue marker LED flashes, yellow LEDs show the current brightness step (6 levels), and two additional status LEDs behind them show autoplay state (`green/green` = on, `red/red` = off).  
    Return to the previously active pattern happens 5 seconds after the last interaction in the battery display.

**Battery Level Scale (voltage-based):**

| Indicator | Voltage Range | Color |
|------------|----------------|--------|
| 5 LEDs     | ≥ 4.05 V       | Blue |
| 4 LEDs     | 4.05 – 3.92 V  | Green |
| 3 LEDs     | 3.92 – 3.80 V  | Green |
| 2 LEDs     | 3.80 – 3.68 V  | Yellow |
| 1 LED      | 3.68 – 3.55 V  | Yellow |
| 1 LED      | 3.55 – 3.40 V  | Red |
| None       | < 3.40 V       | (very empty) |

---

### 3.3. Power Off

- **Power Off:** Press the left button again to turn the controller off.

---

### 3.4. USB CLI (Configuration)

As soon as an active USB serial connection exists, the CLI is available.

- Prompt: `nk>`
- Help: `help`
- Show current values: `show`
- Read single value: `get pattern`, `get brightness`, `get strip_length`, `get smoothing`, `get accel_range`, `get gyro_range`, `get boot_calibration`, `get enabled_patterns`, `get inverted_patterns`, `get autoplay`, `get autoplay_interval`
- Set single value:
  - `set pattern <1..22>`
  - `set brightness <95|127|159|191|223|255>`
  - `set strip_length <10..35>`
  - `set smoothing <1..512>`
  - `set accel_range <2|4|8|16>`
  - `set gyro_range <250|500|1000|2000>`
  - `set boot_calibration <off|quick>`
  - `set autoplay <on|off>`
  - `set autoplay_interval <1..300>`
- Pattern selection:
  - `patterns`
  - `enable_pattern <1..22[,id...]>`
  - `disable_pattern <1..22[,id...]>`
  - `invert_pattern <1..22[,id...]>`
  - `normal_pattern <1..22[,id...]>`
- Diagnostics:
  - `battery`
  - `sensor`
  - `timing`
  - `timing reset`
  - `offsets`
- Calibration:
  - `calibrate quick`
  - `calibrate precise`
- Save/load:
  - `save` (write immediately to EEPROM)
  - `load` (load from EEPROM)
  - `defaults` (load defaults, not saved yet)
  - `reboot` / `restart`

Notes:
- Data commands reply consistently with `OK ...` or `ERR ...`. Example: `OK pattern=1`.
- `show` returns all relevant configuration values as a compact `key=value` line.
- `show` also includes the current autoplay state and autoplay interval.
- `patterns` lists all patterns with `on` or `off` state.
- `strip_length` always applies to both strips together (symmetric).
- `set pattern` switches the active pattern immediately.
- `set pattern` can also select patterns that are currently disabled for button cycling.
- A manual pattern change keeps autoplay enabled, but resets the autoplay timer.
- `set brightness` takes effect immediately.
- `set strip_length` applies immediately to both strips.
- `enable_pattern` and `disable_pattern` control which patterns are included when cycling with the button.
- Both commands also accept multiple pattern IDs as a comma-separated list, for example `disable_pattern 3,5,7`.
- `invert_pattern` and `normal_pattern` control the animation direction per pattern and store that setting persistently.
- These commands also accept multiple pattern IDs as a comma-separated list, for example `invert_pattern 4,12,13`.
- Not every pattern has a visually meaningful direction. On supported patterns, this option reverses the animation movement.
- At least one pattern must always remain enabled.
- Autoplay cycles only through enabled patterns.
- Autoplay can be stored persistently and starts automatically after boot if it was saved as enabled.
- `smoothing`, `accel_range`, `gyro_range`, and `boot_calibration` only take effect after reboot. The CLI marks this in the reply with `(applies after reboot)`.
- `timing` reports `FastLED` FPS, `loop`/`work` times, frame budget, and sample count in microseconds.
- `timing reset` clears the timing statistics (`avg`, `max`, `samples`) so individual patterns or changes can be compared directly.
- `offsets` reports the currently active MPU offsets.
- `calibrate quick` is the fast everyday calibration path and saves the resulting offsets.
- `calibrate precise` uses the much slower `IMU_Zero`-style precision path and saves the resulting offsets.
- `calibrate quick` and `calibrate precise` start with `OK calibrate_started=1 ...` and finish with `OK calibrate_finished=1 ...` plus a final offset line.
- `defaults` loads the default values into working memory only. Run `save` afterwards if you want to keep them permanently.
- Saved values are restored automatically after restart.

## 4. Power Supply and Charging

The system is powered by an integrated **500 mAh LiPo battery**.  
The Pimoroni Pico LiPo features intelligent charging and power management.

- **Charging:** Connect via USB-C to a PC or USB power adapter.  
- **Charging Indicator:** While charging, the LED strip shows the charge level as a bar;  
  a red LED blinks during the active charging process.  
- **USB CLI active:** While a serial session is active, the charging display is suppressed so CLI usage stays predictable.  
- **Fully Charged:** The charging display switches to five blue LEDs only near full cell voltage (firmware threshold: ≥ 4.20 V, approx. 4.2 V); then the red LED turns off.  
- **Automatic Return:** After disconnecting USB, the system automatically returns to the last active pattern.  
- **Runtime:** Typically between 1 – 2.5 hours depending on brightness and pattern.

**Automatic Memory Function:**

- Pattern and brightness are checked roughly every 5 minutes and saved if changed.  
- Strip length is also checked and saved if changed.  
- Motion smoothing, sensor ranges, boot calibration, and MPU offsets are also tracked and saved when changed.
- On the next startup, the last saved values are automatically restored.

---

## 5. Important Notes

- **Keep Still During Initialization:** After powering on, keep the controller still until the LEDs activate.  
- **Precise Calibration:** For `calibrate precise`, place the controller on a perfectly still, level surface and ideally let it thermally stabilize for 5-10 minutes beforehand.  
- **Weather Conditions:** Designed for use on stunt kites; protect electronics from moisture (no rain or fog).  
- **Safety:** Night flights require extra attention. Keep a safe distance from people, trees, power lines, and roads.  
- **Battery Care:** Charge only with suitable USB power supplies.  
  Do not leave the battery unattended while charging. Do not short-circuit, puncture, or deep-discharge.

---

**Quick Start Summary:**  
`Power On → Hold Still (Calibration) → Double-Click = Pattern → Long Press = Battery Display → Short Press (in battery display) = Brightness → Double-Click (in battery display) = Autoplay`
