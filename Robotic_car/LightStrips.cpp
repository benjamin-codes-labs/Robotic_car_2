#include "LightStrips.h"

#include "Config.h"

namespace {

constexpr neoPixelType kStripType = NEO_RGBW + NEO_KHZ800;

}  // namespace

LightStrips::LightStrips()
    : auxStrip_(config::kLedsPerStrip, config::kAuxLedStripPin, kStripType),
      indicatorStrip_(config::kLedsPerStrip, config::kIndicatorLedStripPin, kStripType) {}

void LightStrips::begin() {
  Adafruit_NeoPixel* const strips[] = {&auxStrip_, &indicatorStrip_};
  for (Adafruit_NeoPixel* strip : strips) {
    strip->setBrightness(config::kLedBrightness);
    strip->begin();
    fill(*strip, 0);
  }
  indicatorColor_ = 0;
}

void LightStrips::setIndicatorColor(uint32_t color) {
  // show() briefly disables interrupts, so skip redundant refreshes.
  if (color == indicatorColor_) {
    return;
  }
  fill(indicatorStrip_, color);
  indicatorColor_ = color;
}

void LightStrips::fill(Adafruit_NeoPixel& strip, uint32_t color) {
  strip.fill(color);
  strip.show();
}
