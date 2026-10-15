// Minimal PID controller with symmetric output clamping.
#pragma once

#include <Arduino.h>

class PidController {
 public:
  PidController(float kp, float ki, float kd, float outputLimit)
      : kp_(kp), ki_(ki), kd_(kd), outputLimit_(outputLimit) {}

  float update(float error, float dtSeconds) {
    integral_ += error * dtSeconds;
    const float derivative = dtSeconds > 0.0f ? (error - previousError_) / dtSeconds : 0.0f;
    previousError_ = error;

    const float output = kp_ * error + ki_ * integral_ + kd_ * derivative;
    return constrain(output, -outputLimit_, outputLimit_);
  }

 private:
  float kp_;
  float ki_;
  float kd_;
  float outputLimit_;

  float integral_ = 0.0f;
  float previousError_ = 0.0f;
};
