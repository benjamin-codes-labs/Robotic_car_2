// The robot's VL53L1X time-of-flight sensors, which share a single I2C bus.
#pragma once

#include <Arduino.h>
#include <VL53L1X.h>

enum class RangeSensor : uint8_t {
  kLeft,
  kFront,
  kAux,  // Initialized for future use; not read by the current routines.
};

class RangeSensors {
 public:
  // Brings every sensor up at its own I2C address and starts continuous ranging.
  // Returns false (after logging the culprit to Serial) if any sensor fails.
  bool begin();

  // Blocks until the sensor's next measurement is available.
  uint16_t readMm(RangeSensor sensor);

 private:
  static constexpr uint8_t kSensorCount = 3;

  VL53L1X sensors_[kSensorCount];
};
