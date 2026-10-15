#include "LaneFollower.h"

#include <AutoCar.h>

#include "Config.h"
#include "Drivetrain.h"

LaneFollower::LaneFollower(AutoCar& car, Drivetrain& drivetrain)
    : car_(car),
      drivetrain_(drivetrain),
      steering_(config::kSteeringKp, config::kSteeringKi, config::kSteeringKd,
                config::kMaxSteeringCorrection),
      baseSpeed_(config::kCruiseSpeed) {}

bool LaneFollower::laneDetected() {
  return car_.getLDistance() > 0 || car_.getRDistance() > 0;
}

void LaneFollower::update() {
  const uint32_t now = millis();
  const float dtSeconds = (now - lastUpdateMs_) / 1000.0f;
  lastUpdateMs_ = now;

  // A positive error slows the left wheels and speeds up the right ones, steering left.
  const float error = car_.getLDistance() - car_.getRDistance() - config::kLaneCenterOffset;
  const float correction = steering_.update(error, dtSeconds);

  drivetrain_.tankDrive(static_cast<int16_t>(baseSpeed_ - correction),
                        static_cast<int16_t>(baseSpeed_ + correction));
}
