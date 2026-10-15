#include "RangeSensors.h"

#include "Config.h"

namespace {

struct SensorSlot {
  config::RangeSensorPins pins;
  const char* name;
};

// Indexed by RangeSensor; keep the order in sync with the enum.
const SensorSlot kSlots[] = {
    {config::kLeftRangeSensor, "left"},
    {config::kFrontRangeSensor, "front"},
    {config::kAuxRangeSensor, "aux"},
};

constexpr uint32_t kResetHoldMs = 20;
constexpr uint32_t kBootTimeMs = 10;

}  // namespace

bool RangeSensors::begin() {
  static_assert(sizeof(kSlots) / sizeof(kSlots[0]) == kSensorCount,
                "Every RangeSensor needs a pin configuration");

  // All sensors boot with the same default address, so hold them in reset and
  // release them one at a time, re-addressing each before waking the next.
  for (const SensorSlot& slot : kSlots) {
    pinMode(slot.pins.xshutPin, OUTPUT);
    digitalWrite(slot.pins.xshutPin, LOW);
  }
  delay(kResetHoldMs);

  for (uint8_t i = 0; i < kSensorCount; ++i) {
    const SensorSlot& slot = kSlots[i];
    VL53L1X& sensor = sensors_[i];

    // Let the breakout's pull-up release XSHUT instead of driving it to 5 V.
    pinMode(slot.pins.xshutPin, INPUT);
    delay(kBootTimeMs);

    sensor.setTimeout(config::kRangeTimeoutMs);
    if (!sensor.init()) {
      Serial.print(F("Failed to initialize the "));
      Serial.print(slot.name);
      Serial.println(F(" VL53L1X range sensor"));
      return false;
    }

    sensor.setAddress(slot.pins.i2cAddress);
    sensor.setDistanceMode(VL53L1X::Short);
    sensor.setMeasurementTimingBudget(config::kRangeTimingBudgetUs);
    sensor.startContinuous(config::kRangeMeasurementPeriodMs);
  }

  return true;
}

uint16_t RangeSensors::readMm(RangeSensor sensor) {
  return sensors_[static_cast<uint8_t>(sensor)].read();
}
