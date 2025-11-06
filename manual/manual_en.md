# User Manual for the NightKite Multi LED Kite Lighting (v2.0)

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
  13 predefined animation modes to customize the kite’s appearance (each affects runtime differently).
- **Battery Indicator:**  
  Displays the current battery level directly via the LED strip.
- **Memory Function:**  
  Pattern and brightness are automatically stored and restored at the next startup.

---

### 1.1. Animation Patterns in Detail

The NightKite Multi (v2.0) includes 12 predefined animation patterns that can be changed with a double-click.  
After powering on and calibration, the controller starts with the last used pattern and brightness level.  
On first startup: **Pattern 1 “Rainbow”**, **Brightness 95**.

1. **Rainbow Pattern:** Smooth, continuous rainbow cycle across the strip. Not motion-reactive.  
2. **Full String, Angle-Color:** Entire LED strip glows in a color determined by the current yaw angle.  
3. **Full String, Angle-Color & Motion Brightness:** Like (2), but brightness follows motion intensity.  
4. **LED Runner (fixed) with Fadeout, Angle-Color:** A single light point moves at fixed speed; color = angle, smooth fadeout.  
5. **LED Runner (reactive) with Fadeout-Speed, Angle-Color:** Like (4), but speed and fade respond to motion.  
6. **LED Runner (dual-color, reactive), Angle-Color:** Like (5), but with a two-color trail; speed and fade depend on movement.  
7. **Heartbeat (Angle-Color):** Pulsating heartbeat animation; color = angle.  
8. **Ping-Pong (bouncing) with reactive fade:** Point moves back and forth; fadeout duration increases with motion.  
9. **Comet Swarm (4 Comets):** Four short comets race across all 50 pixels (both halves mirrored). Color = angle.  
10. **Breath / Storm (adaptive):**  
    - Calm: smooth “breathing” (brightness sinus).  
    - Turbulent: “storm” with sparks that increase with movement.  
11. **Jerk Wave:** Sudden motion triggers a wave from the center to the ends; wave speed increases with motion; color = angle.  
12. **Yaw Spinner (with direction memory):** Light head with blurred trail follows yaw rate with small dead zone to prevent flicker; color = angle.  
13. **LED Runner (dual-color, reactive, inverted):** Like (6), but with inverted running direction.

---

## 2. Components

The NightKite Multi lighting system consists of the following main components:

- **Microcontroller:** Pimoroni Pico LiPo with integrated battery management (USB-C)  
- **Battery:** 500 mAh LiPo battery connected directly to the microcontroller  
- **Sensor:** MPU6050 (gyroscope/accelerometer) to detect orientation (Yaw/Pitch/Roll) and motion  
- **LED Strips:** Two strings of 25 pixels each (50 total), WS281x / “fairy string”, GRB order

---

## 3. Operation and Controls

### 3.1. Power On and Initialization

The controller has two buttons: **left** and **right**.

1. **Power On:** Press the left button.  
2. **Initialization (Calibration):** After powering on, a short calibration starts automatically.  
   Hold the controller still for a few seconds to ensure accurate sensor offsets.  
3. **Ready:** After calibration, the LEDs turn on.  
   The system starts with the last used pattern and brightness  
   *(on first use: Pattern 1, Brightness 95)*.

---

### 3.2. Functions of the Right Button

The right button is a **multi-function control**:

- **Change Pattern / Animation (double-click):**  
  → Cycles through the 12 available animation patterns.
- **Change Brightness (short press):**  
  → Switches through the six brightness levels: 95 → 127 → 159 → 191 → 223 → 255 → 95.
- **Show Battery Level (hold):**  
  → Displays battery level for about 5 seconds.  
    During this time, a blue LED marker flashes on the strip.

**Battery Level Scale (voltage-based):**

| Indicator | Voltage Range | Color |
|------------|----------------|--------|
| 5 LEDs     | ≥ 4.2 V        | Blue |
| 4 LEDs     | 4.2 – 4.0 V    | Green |
| 3 LEDs     | 4.0 – 3.8 V    | Green |
| 2 LEDs     | 3.8 – 3.6 V    | Yellow |
| 1 LED      | 3.6 – 3.4 V    | Yellow |
| 1 LED      | 3.4 – 3.2 V    | Red |
| None       | 3.2 – 3.0 V    | (shutdown protection) |

---

### 3.3. Power Off

- **Power Off:** Press the left button again to turn the controller off.

---

## 4. Power Supply and Charging

The system is powered by an integrated **500 mAh LiPo battery**.  
The Pimoroni Pico LiPo features intelligent charging and power management.

- **Charging:** Connect via USB-C to a PC or USB power adapter.  
- **Charging Indicator:** While charging, the LED strip shows the charge level as a bar;  
  a red LED blinks during the active charging process.  
- **Fully Charged:** When five blue LEDs are lit (≥ 4.2 V), the battery is full and the red LED turns off.  
- **Automatic Return:** After disconnecting USB, the system automatically returns to the last active pattern.  
- **Runtime:** Typically between 1 – 2.5 hours depending on brightness and pattern.

**Automatic Memory Function:**

- Pattern and brightness are checked roughly every 5 minutes and saved if changed.  
- On the next startup, the last saved values are automatically restored.

---

## 5. Important Notes

- **Keep Still During Initialization:** After powering on, keep the controller still until the LEDs activate.  
- **Weather Conditions:** Designed for use on stunt kites; protect electronics from moisture (no rain or fog).  
- **Safety:** Night flights require extra attention. Keep a safe distance from people, trees, power lines, and roads.  
- **Battery Care:** Charge only with suitable USB power supplies.  
  Do not leave the battery unattended while charging. Do not short-circuit, puncture, or deep-discharge.

---

**Quick Start Summary:**  
`Power On → Hold Still (Calibration) → Short Press = Brightness → Double-Click = Pattern → Long Press = Battery Display`
