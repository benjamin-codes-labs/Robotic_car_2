#include "Drivetrain.h"

void Motor::begin() {
  pinMode(pins_.direction, OUTPUT);
  pinMode(pins_.pwm, OUTPUT);
}

void Motor::setSpeed(int16_t speed) {
  speed = constrain(speed, -config::kMaxMotorSpeed, config::kMaxMotorSpeed);
  const bool forward = speed >= 0;

  digitalWrite(pins_.direction, forward != pins_.inverted ? HIGH : LOW);
  analogWrite(pins_.pwm, forward ? speed : -speed);
}

Drivetrain::Drivetrain()
    : frontLeft_(config::kFrontLeftMotor),
      rearLeft_(config::kRearLeftMotor),
      frontRight_(config::kFrontRightMotor),
      rearRight_(config::kRearRightMotor) {}

void Drivetrain::begin() {
  frontLeft_.begin();
  rearLeft_.begin();
  frontRight_.begin();
  rearRight_.begin();
  stop();
}

void Drivetrain::tankDrive(int16_t leftSpeed, int16_t rightSpeed) {
  setWheelSpeeds(leftSpeed, leftSpeed, rightSpeed, rightSpeed);
}

void Drivetrain::move(Direction direction, uint8_t speed) {
  const int16_t s = speed;

  switch (direction) {
    case Direction::kForward:
      setWheelSpeeds(s, s, s, s);
      break;
    case Direction::kBackward:
      setWheelSpeeds(-s, -s, -s, -s);
      break;
    case Direction::kStrafeLeft:
      setWheelSpeeds(-s, s, s, -s);
      break;
    case Direction::kStrafeRight:
      setWheelSpeeds(s, -s, -s, s);
      break;
    case Direction::kRotateLeft:
      setWheelSpeeds(-s, -s, s, s);
      break;
    case Direction::kRotateRight:
      setWheelSpeeds(s, s, -s, -s);
      break;
  }
}

void Drivetrain::stop() {
  setWheelSpeeds(0, 0, 0, 0);
}

void Drivetrain::setWheelSpeeds(int16_t frontLeft, int16_t rearLeft, int16_t frontRight,
                                int16_t rearRight) {
  frontLeft_.setSpeed(frontLeft);
  rearLeft_.setSpeed(rearLeft);
  frontRight_.setSpeed(frontRight);
  rearRight_.setSpeed(rearRight);
}
