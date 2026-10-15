// Top-level behavior: follow the course, react to traffic signs, then park.
#pragma once

#include <Arduino.h>

#include "Drivetrain.h"
#include "RangeSensors.h"

class AutoCar;
class LaneFollower;
class LightStrips;
class StatusDisplay;

// Sign IDs reported by AutoCar::getSignType().
enum class TrafficSign : int {
  kStop = 1,
  kStraight = 7,
  kTurnLeft = 8,
  kTurnRight = 9,
  kCrosswalk = 15,
};

class Mission {
 public:
  Mission(AutoCar& car, Drivetrain& drivetrain, LaneFollower& laneFollower,
          RangeSensors& rangeSensors, StatusDisplay& display, LightStrips& lights);

  // Advances the mission by one step. Call from loop().
  void update();

 private:
  enum class State : uint8_t {
    kFollowingCourse,
    kParking,
    kParked,
  };

  void followCourse();
  void park();

  void handleTrafficSign();
  void onStopSign(float ratio);
  void onStraightSign();
  void onCrosswalkSign();
  void onLeftTurnSign(float ratio);
  void onRightTurnSign(float ratio);

  void updateWateringIndicator();

  // Blocking motion primitives.
  void followLaneFor(uint32_t durationMs);
  void turnWhileSignVisible(TrafficSign sign, int16_t leftSpeed, int16_t rightSpeed,
                            uint32_t holdMs);
  void moveUntilCloserThan(RangeSensor sensor, uint16_t thresholdMm, Direction direction,
                           uint8_t speed);
  void moveUntilFartherThan(RangeSensor sensor, uint16_t thresholdMm, Direction direction,
                            uint8_t speed);
  void stopFor(uint32_t durationMs);

  TrafficSign currentSign();
  uint16_t readAndShowRange(RangeSensor sensor);

  AutoCar& car_;
  Drivetrain& drivetrain_;
  LaneFollower& laneFollower_;
  RangeSensors& rangeSensors_;
  StatusDisplay& display_;
  LightStrips& lights_;

  State state_ = State::kFollowingCourse;
  bool leftTurnSignSeen_ = false;
  uint32_t lastIndicatorUpdateMs_ = 0;
};
