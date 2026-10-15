// The robot's two RGBW NeoPixel strips.
#pragma once

#include <Adafruit_NeoPixel.h>
#include <Arduino.h>

class LightStrips {
 public:
  LightStrips();

  // Initializes both strips and switches every LED off.
  void begin();

  // Fills the indicator strip with a packed 0xRRGGBB color (0 turns it off).
  void setIndicatorColor(uint32_t color);

 private:
  static void fill(Adafruit_NeoPixel& strip, uint32_t color);

  Adafruit_NeoPixel auxStrip_;
  Adafruit_NeoPixel indicatorStrip_;
  uint32_t indicatorColor_ = 0;
};
