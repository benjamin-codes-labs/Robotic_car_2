#include "Mission.h"

#include <AutoCar.h>

#include "Config.h"
#include "LaneFollower.h"
#include "LightStrips.h"
#include "StatusDisplay.h"

Mission::Mission(AutoCar& car, Drivetrain& drivetrain, LaneFollower& laneFollower,
                 RangeSensors& rangeSensors, StatusDisplay& display, LightStrips& lights)
    : car_(car),
      drivetrain_(drivetrain),
      laneFollower_(laneFollower),
      rangeSensors_(rangeSensors),
      display_(display),
      lights_(lights) {}

void Mission::update() {
  switch (state_) {
    case State::kFollowingCourse:
      followCourse();
      break;
    case State::kParking:
      park();
      state_ = State::kParked;
      break;
    case State::kParked:
      break;  // Mission complete: stay put.
  }
}

// -----------------------------------------------------------------------------
// States
// -----------------------------------------------------------------------------

void Mission::followCourse() {
  car_.probe();

  if (laneFollower_.laneDetected()) {
    laneFollower_.update();
  }
  handleTrafficSign();
  updateWateringIndicator();
}

void Mission::park() {
  // Slide sideways into the bay until the left side is close to its boundary.
  moveUntilCloserThan(RangeSensor::kLeft, config::kParkSideApproachMm, Direction::kStrafeLeft,
                      config::kStrafeLeftSpeed);
  stopFor(config::kParkSettleMs);

  // Center the car lengthways using the wall ahead.
  moveUntilFartherThan(RangeSensor::kFront, config::kParkMinFrontClearanceMm, Direction::kBackward,
                       config::kCreepSpeed);
  moveUntilCloserThan(RangeSensor::kFront, config::kParkMaxFrontClearanceMm, Direction::kForward,
                      config::kCreepSpeed);
  stopFor(config::kParkPauseMs);

  // Short pivot to correct the heading before the final adjustment.
  drivetrain_.move(Direction::kRotateLeft, config::kPivotSpeed);
  delay(config::kParkHeadingCorrectionMs);
  stopFor(config::kParkPauseMs);

  // Back off from the side boundary to the final clearance.
  moveUntilFartherThan(RangeSensor::kLeft, config::kParkFinalSideClearanceMm,
                       Direction::kStrafeRight, config::kStrafeRightSpeed);
  drivetrain_.stop();
}

// -----------------------------------------------------------------------------
// Traffic signs
// -----------------------------------------------------------------------------

void Mission::handleTrafficSign() {
  if (!car_.signDetected()) {
    return;
  }

  const float ratio = car_.getSignRatio();
  if (ratio <= config::kMinSignRatio) {
    return;
  }

  switch (currentSign()) {
    case TrafficSign::kStop:
      onStopSign(ratio);
      break;
    case TrafficSign::kStraight:
      onStraightSign();
      break;
    case TrafficSign::kCrosswalk:
      onCrosswalkSign();
      break;
    case TrafficSign::kTurnLeft:
      onLeftTurnSign(ratio);
      break;
    case TrafficSign::kTurnRight:
      onRightTurnSign(ratio);
      break;
  }
}

void Mission::onStopSign(float ratio) {
  if (ratio < config::kStopSignRatio) {
    return;
  }

  followLaneFor(config::kStopSignApproachMs);
  stopFor(config::kStopSignPauseMs);

  // Square up to the wall ahead so parking always starts from a known position.
  moveUntilFartherThan(RangeSensor::kFront, config::kStopMinFrontClearanceMm, Direction::kBackward,
                       config::kCreepSpeed);
  moveUntilCloserThan(RangeSensor::kFront, config::kStopMaxFrontClearanceMm, Direction::kForward,
                      config::kCreepSpeed);
  drivetrain_.stop();

  state_ = State::kParking;
}

void Mission::onStraightSign() {
  display_.showSpeedLimit(config::kStraightSpeedLimitKmh);
  followLaneFor(config::kStraightSignFollowMs);
  display_.clear();
}

void Mission::onCrosswalkSign() {
  drivetrain_.move(Direction::kStrafeRight, config::kStrafeRightSpeed);
  delay(config::kCrosswalkSidestepMs);
  stopFor(config::kCrosswalkPauseMs);

  display_.showSpeedLimit(config::kCrosswalkSpeedLimitKmh);
  laneFollower_.setBaseSpeed(config::kCrosswalkSpeed);
  followLaneFor(config::kCrosswalkFollowMs);
  laneFollower_.setBaseSpeed(config::kCruiseSpeed);
  display_.clear();
}

void Mission::onLeftTurnSign(float ratio) {
  // Only the very first frame showing a left-turn sign may trigger a turn from
  // further away; every later sign must be approached much more closely.
  const bool firstSighting = !leftTurnSignSeen_;
  leftTurnSignSeen_ = true;

  const float triggerRatio = firstSighting ? config::kFirstLeftTurnRatio : config::kLeftTurnRatio;
  if (ratio < triggerRatio) {
    return;
  }

  turnWhileSignVisible(TrafficSign::kTurnLeft, config::kArcInnerWheelSpeed,
                       config::kArcOuterWheelSpeed,
                       firstSighting ? config::kFirstLeftTurnHoldMs : config::kLeftTurnHoldMs);
}

void Mission::onRightTurnSign(float ratio) {
  if (ratio < config::kRightTurnRatio) {
    return;
  }

  drivetrain_.move(Direction::kForward, config::kCreepSpeed);
  delay(config::kRightTurnLeadInMs);
  turnWhileSignVisible(TrafficSign::kTurnRight, config::kArcOuterWheelSpeed,
                       config::kArcInnerWheelSpeed, config::kRightTurnHoldMs);
}

// -----------------------------------------------------------------------------
// Watering indicator
// -----------------------------------------------------------------------------

void Mission::updateWateringIndicator() {
  // Reading a range sensor blocks, so only refresh the indicator once the lane
  // follower has been idle for a full interval to avoid starving the steering loop.
  const uint32_t now = millis();
  if (now - laneFollower_.lastUpdateMs() < config::kIndicatorRefreshIntervalMs ||
      now - lastIndicatorUpdateMs_ < config::kIndicatorRefreshIntervalMs) {
    return;
  }

  const uint16_t leftMm = rangeSensors_.readMm(RangeSensor::kLeft);
  const bool objectNearby = leftMm > 0 && leftMm <= config::kWateringTriggerMm;
  lights_.setIndicatorColor(objectNearby ? config::kWateringColor : 0);

  lastIndicatorUpdateMs_ = millis();
}

// -----------------------------------------------------------------------------
// Motion primitives
// -----------------------------------------------------------------------------

void Mission::followLaneFor(uint32_t durationMs) {
  const uint32_t start = millis();
  while (millis() - start < durationMs) {
    car_.probe();
    laneFollower_.update();
  }
}

void Mission::turnWhileSignVisible(TrafficSign sign, int16_t leftSpeed, int16_t rightSpeed,
                                   uint32_t holdMs) {
  drivetrain_.tankDrive(leftSpeed, rightSpeed);
  while (currentSign() == sign) {
    car_.probe();
  }
  delay(holdMs);
}

void Mission::moveUntilCloserThan(RangeSensor sensor, uint16_t thresholdMm, Direction direction,
                                  uint8_t speed) {
  while (readAndShowRange(sensor) >= thresholdMm) {
    drivetrain_.move(direction, speed);
    delay(config::kRangePollIntervalMs);
  }
}

void Mission::moveUntilFartherThan(RangeSensor sensor, uint16_t thresholdMm, Direction direction,
                                   uint8_t speed) {
  while (readAndShowRange(sensor) <= thresholdMm) {
    drivetrain_.move(direction, speed);
    delay(config::kRangePollIntervalMs);
  }
}

void Mission::stopFor(uint32_t durationMs) {
  drivetrain_.stop();
  delay(durationMs);
}

// -----------------------------------------------------------------------------
// Helpers
// -----------------------------------------------------------------------------

TrafficSign Mission::currentSign() {
  return static_cast<TrafficSign>(car_.getSignType());
}

uint16_t Mission::readAndShowRange(RangeSensor sensor) {
  const uint16_t mm = rangeSensors_.readMm(sensor);
  display_.showDistance(mm);
  return mm;
}
