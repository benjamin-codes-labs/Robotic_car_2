// Keeps the car centered in its lane using the lane offsets measured by AutoCar.
#pragma once

#include <Arduino.h>

#include "PidController.h"

class AutoCar;
class Drivetrain;

class LaneFollower {
 public:
  LaneFollower(AutoCar& car, Drivetrain& drivetrain);

  // True when the camera currently sees at least one lane line.
  bool laneDetected();

  // Runs one steering step from the most recent camera frame. Call car.probe() first.
  void update();

  void setBaseSpeed(int16_t speed) { baseSpeed_ = speed; }

  uint32_t lastUpdateMs() const { return lastUpdateMs_; }

 private:
  AutoCar& car_;
  Drivetrain& drivetrain_;
  PidController steering_;
  int16_t baseSpeed_;
  uint32_t lastUpdateMs_ = 0;
};
