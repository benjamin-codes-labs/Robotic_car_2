// Pin assignments and tuning parameters for the whole robot.
//
// Units, unless stated otherwise:
//   distances  - millimetres
//   durations  - milliseconds
//   speeds     - signed 8-bit PWM duty cycle (-255..255, positive = forward)
#pragma once

#include <Arduino.h>

namespace config {

// -----------------------------------------------------------------------------
// System
// -----------------------------------------------------------------------------

constexpr uint32_t kSerialBaudRate = 115200;
constexpr uint32_t kI2cClockHz = 400000;

// -----------------------------------------------------------------------------
// Drivetrain: four mecanum wheels, one H-bridge channel (direction + PWM) each
// -----------------------------------------------------------------------------

struct MotorPins {
  uint8_t direction;
  uint8_t pwm;
  bool inverted;  // True when a HIGH direction pin spins the wheel backwards.
};

// The right-hand motors are mounted mirrored, so their direction logic is inverted.
constexpr MotorPins kFrontLeftMotor = {7, 6, false};
constexpr MotorPins kRearLeftMotor = {8, 9, false};
constexpr MotorPins kFrontRightMotor = {4, 5, true};
constexpr MotorPins kRearRightMotor = {2, 10, true};

constexpr int16_t kMaxMotorSpeed = 255;

// -----------------------------------------------------------------------------
// VL53L1X time-of-flight range sensors (shared I2C bus)
// -----------------------------------------------------------------------------

struct RangeSensorPins {
  uint8_t xshutPin;    // Active-low shutdown; used to bring sensors up one by one.
  uint8_t i2cAddress;  // Assigned at start-up; every sensor boots as 0x29.
};

constexpr RangeSensorPins kLeftRangeSensor = {A2, 0x2A};
constexpr RangeSensorPins kFrontRangeSensor = {A1, 0x2B};
constexpr RangeSensorPins kAuxRangeSensor = {A0, 0x2C};

constexpr uint16_t kRangeTimeoutMs = 500;
constexpr uint32_t kRangeTimingBudgetUs = 50000;
constexpr uint32_t kRangeMeasurementPeriodMs = 50;

// Pause between readings while performing a distance-guided maneuver.
constexpr uint32_t kRangePollIntervalMs = 100;

// -----------------------------------------------------------------------------
// Feedback: NeoPixel strips and 16x2 RGB LCD
// -----------------------------------------------------------------------------

constexpr uint8_t kAuxLedStripPin = 3;
constexpr uint8_t kIndicatorLedStripPin = 11;
constexpr uint16_t kLedsPerStrip = 60;
constexpr uint8_t kLedBrightness = 100;

constexpr uint8_t kLcdColumns = 16;
constexpr uint8_t kLcdRows = 2;

// -----------------------------------------------------------------------------
// Lane following (PID on the lateral offset reported by AutoCar)
// -----------------------------------------------------------------------------

constexpr float kSteeringKp = 0.79f;
constexpr float kSteeringKi = 0.0f;
constexpr float kSteeringKd = 0.045f;
constexpr float kMaxSteeringCorrection = 100.0f;

// Biases the target position so the car runs slightly off the geometric center.
constexpr float kLaneCenterOffset = 5.0f;

constexpr int16_t kCruiseSpeed = 150;
constexpr int16_t kCrosswalkSpeed = 60;

// -----------------------------------------------------------------------------
// Maneuver speeds
// -----------------------------------------------------------------------------

constexpr uint8_t kCreepSpeed = 50;
constexpr uint8_t kStrafeLeftSpeed = 100;
constexpr uint8_t kStrafeRightSpeed = 80;
constexpr uint8_t kPivotSpeed = 150;
constexpr int16_t kArcOuterWheelSpeed = 140;
constexpr int16_t kArcInnerWheelSpeed = 30;

// -----------------------------------------------------------------------------
// Traffic signs
//
// The sign ratio reported by AutoCar grows as the sign gets closer, so each
// threshold effectively sets the distance at which the car reacts.
// -----------------------------------------------------------------------------

constexpr float kMinSignRatio = 1.0f;  // Detections at or below this are ignored.

// Stop sign: pull up, square up to the wall ahead, then hand over to parking.
constexpr float kStopSignRatio = 10.0f;
constexpr uint32_t kStopSignApproachMs = 500;
constexpr uint32_t kStopSignPauseMs = 500;
constexpr uint16_t kStopMinFrontClearanceMm = 330;
constexpr uint16_t kStopMaxFrontClearanceMm = 700;

// Go-straight sign.
constexpr uint8_t kStraightSpeedLimitKmh = 60;
constexpr uint32_t kStraightSignFollowMs = 2000;

// Crosswalk sign: side-step, then pass slowly.
constexpr uint32_t kCrosswalkSidestepMs = 200;
constexpr uint32_t kCrosswalkPauseMs = 100;
constexpr uint8_t kCrosswalkSpeedLimitKmh = 50;
constexpr uint32_t kCrosswalkFollowMs = 3000;

// Turn signs: the car arcs while the sign is visible, then holds the arc.
constexpr float kFirstLeftTurnRatio = 12.0f;
constexpr float kLeftTurnRatio = 35.0f;
constexpr uint32_t kFirstLeftTurnHoldMs = 1150;
constexpr uint32_t kLeftTurnHoldMs = 1200;

constexpr float kRightTurnRatio = 20.0f;
constexpr uint32_t kRightTurnLeadInMs = 100;
constexpr uint32_t kRightTurnHoldMs = 1150;

// -----------------------------------------------------------------------------
// Parking (left-hand bay, positioned with the left and front range sensors)
// -----------------------------------------------------------------------------

constexpr uint16_t kParkSideApproachMm = 250;
constexpr uint16_t kParkMinFrontClearanceMm = 240;
constexpr uint16_t kParkMaxFrontClearanceMm = 350;
constexpr uint16_t kParkFinalSideClearanceMm = 135;

constexpr uint32_t kParkSettleMs = 50;
constexpr uint32_t kParkPauseMs = 100;
constexpr uint32_t kParkHeadingCorrectionMs = 90;

// -----------------------------------------------------------------------------
// Watering indicator: lights up when an object is close on the left
// -----------------------------------------------------------------------------

constexpr uint16_t kWateringTriggerMm = 250;
constexpr uint32_t kWateringColor = 0x3366FF;  // RGB(51, 102, 255)
constexpr uint32_t kIndicatorRefreshIntervalMs = 100;

}  // namespace config
