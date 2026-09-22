# AutoCar — Autonomous Line-Following & Parking Robot

Arduino firmware for a self-driving model car that follows a line with PID control, recognizes traffic signs, performs parking maneuvers, and gives visual feedback through NeoPixel LEDs and an RGB LCD.

> **Note:** Some identifiers in the source are UTF-8 byte sequences (e.g. `_E5_B7_A1_E7_BA_BF_PID` = 巡线PID / "line-following PID", `_E5_B7_A6_E8_BD_AC` = 左转 / "turn left", `_E5_8F_B3_E8_BD_AC` = 右转 / "turn right", `_E8_BD_AC_E5_BC_AF_E5_9F_BA_E9_80_9F` = 轮毂基础速度 / "wheel base speed"). They are kept as-is to match the toolchain's auto-generated names.

---

## Features

- **PID line following** using left/right distance sensors (`car.getLDistance()`, `car.getRDistance()`)
- **Traffic sign recognition** via the `AutoCar` library (stop, forward, left, right, crosswalk)
- **Automatic parking** sequence using three VL53L1X ToF sensors
- **Speed-limit display** on a 16×2 RGB LCD (e.g. `50km/h`, `60km/h`)
- **Ambient/status lighting** with two 60-LED NeoPixel (RGBW) strips
- **Watering indicator** — blue LED when an obstacle is detected within 250 mm

---

## Hardware

| Component | Details |
|---|---|
| MCU | Arduino (UNO / Mega class) |
| Distance sensors | 3× VL53L1X ToF (I²C, short-range mode) |
| LED strips | 2× Adafruit NeoPixel RGBW, 60 LEDs each |
| Display | Grove RGB LCD (16×2, I²C) |
| Motor driver | 4-channel H-bridge (2 pins per channel) |
| Vision / AI | `AutoCar` library (MLP, edge detect, Hough, sign detect) |

---

## Pin Mapping

### Motors

| Channel | Direction Pin | PWM Pin | Usage |
|---|---|---|---|
| A | 7 | 6 | Left side |
| B | 8 | 9 | Left side |
| C | 4 | 5 | Right side |
| D | 2 | 10 | Right side |

- `HIGH` on the direction pin + PWM value = forward for that channel.
- Left channels use `base_speed - output`, right channels use `base_speed + output`.

### Peripherals

| Device | Pin / Address |
|---|---|
| VL53L1X #1 (front) | XSHUT = `A2`, I²C = `0x2A` |
| VL53L1X #2 (right) | XSHUT = `A1`, I²C = `0x2B` |
| VL53L1X #3 | XSHUT = `A0`, I²C = `0x2C` |
| NeoPixel strip | Pin `3`, 60 LEDs, RGBW + 800 kHz |
| NeoPixel strip | Pin `11`, 60 LEDs, RGBW + 800 kHz |
| RGB LCD | I²C (default address) |

---

## Dependencies

Install via the Arduino Library Manager (or vendor them manually):

- `Wire` (built-in)
- [`VL53L1X`](https://github.com/pololu/vl53l1x-arduino) by Pololu
- [`Adafruit_NeoPixel`](https://github.com/adafruit/Adafruit_NeoPixel)
- `rgb_lcd` (Grove RGB LCD)
- `AutoCar` (project-specific — provides `car.init()`, `car.probe()`, `car.getLDistance()`, `car.getSignType()`, etc.)

### `AutoCar` API used

| Call | Purpose |
|---|---|
| `car.init()` | Initialize the car platform |
| `car.initMLPModule()` | Initialize neural-net sign classifier |
| `car.initEdgeDetectModule()` | Initialize edge detection |
| `car.initHoughModule()` | Initialize Hough line detection |
| `car.configDriverless(1)` | Enable driverless mode |
| `car.probe()` | Poll all sensors/camera; must be called each cycle |
| `car.getLDistance()` / `car.getRDistance()` | Left / right line distance |
| `car.signDetected()` / `car.getSignType()` / `car.getSignRatio()` | Sign detection state |

---

## Key Parameters

```cpp
kp = 0.79;          // proportional gain
ki = 0;             // integral gain
kd = 0.045;         // derivative gain
base_speed = 150;   // default PWM base speed
BRIGHTNESS = 100;   // NeoPixel brightness
