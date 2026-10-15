// 16x2 Grove RGB LCD showing the current speed limit and live range readings.
#pragma once

#include <Arduino.h>
#include <rgb_lcd.h>

class StatusDisplay {
 public:
  void begin();

  // Top row, e.g. "     60km/h".
  void showSpeedLimit(uint8_t kmh);

  // Bottom row, fixed width so shorter values overwrite longer ones cleanly.
  void showDistance(uint16_t mm);

  void clear();

 private:
  rgb_lcd lcd_;
};
