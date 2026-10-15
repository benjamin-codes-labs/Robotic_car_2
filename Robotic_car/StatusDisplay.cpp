#include "StatusDisplay.h"

#include <stdio.h>

#include "Config.h"

namespace {

constexpr uint8_t kSpeedLimitColumn = 5;
constexpr uint8_t kSpeedLimitRow = 0;
constexpr uint8_t kDistanceRow = 1;

}  // namespace

void StatusDisplay::begin() {
  lcd_.begin(config::kLcdColumns, config::kLcdRows);
}

void StatusDisplay::showSpeedLimit(uint8_t kmh) {
  lcd_.setCursor(kSpeedLimitColumn, kSpeedLimitRow);
  lcd_.print(kmh);
  lcd_.print(F("km/h"));
}

void StatusDisplay::showDistance(uint16_t mm) {
  char text[config::kLcdColumns + 1];
  snprintf(text, sizeof(text), "%5u mm", static_cast<unsigned>(mm));

  lcd_.setCursor(0, kDistanceRow);
  lcd_.print(text);
}

void StatusDisplay::clear() {
  lcd_.clear();
}
