// Four-wheel mecanum drivetrain built on direction + PWM H-bridge channels.
#pragma once

#include <Arduino.h>

#include "Config.h"

enum class Direction : uint8_t {
  kForward,
  kBackward,
  kStrafeLeft,
  kStrafeRight,
  kRotateLeft,
  kRotateRight,
};

// A single DC motor driven by one H-bridge channel.
class Motor {
 public:
  explicit Motor(const config::MotorPins& pins) : pins_(pins) {}

  void begin();

  // Positive speeds turn the wheel forward, negative speeds backward.
  // Values are clamped to +/-config::kMaxMotorSpeed.
  void setSpeed(int16_t speed);

 private:
  config::MotorPins pins_;
};

class Drivetrain {
 public:
  Drivetrain();

  void begin();

  // Differential drive: one signed speed per side of the car.
  void tankDrive(int16_t leftSpeed, int16_t rightSpeed);

  // Moves in a straight line, sideways or on the spot at the given speed.
  void move(Direction direction, uint8_t speed);

  void stop();

 private:
  void setWheelSpeeds(int16_t frontLeft, int16_t rearLeft, int16_t frontRight, int16_t rearRight);

  Motor frontLeft_;
  Motor rearLeft_;
  Motor frontRight_;
  Motor rearRight_;
};
