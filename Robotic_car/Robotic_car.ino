// AutoCar firmware: an autonomous model car that follows its lane, obeys
// traffic signs and parks itself. See README.md for wiring and behavior.

#include <AutoCar.h>
#include <Wire.h>

#include "Config.h"
#include "Drivetrain.h"
#include "LaneFollower.h"
#include "LightStrips.h"
#include "Mission.h"
#include "RangeSensors.h"
#include "StatusDisplay.h"

AutoCar car;
Drivetrain drivetrain;
RangeSensors rangeSensors;
LightStrips lights;
StatusDisplay display;
LaneFollower laneFollower(car, drivetrain);
Mission mission(car, drivetrain, laneFollower, rangeSensors, display, lights);

void setup() {
  Serial.begin(config::kSerialBaudRate);
  Wire.begin();
  Wire.setClock(config::kI2cClockHz);

  drivetrain.begin();
  lights.begin();
  display.begin();

  if (!rangeSensors.begin()) {
    Serial.println(F("Halting: range sensor initialization failed."));
    while (true) {
    }
  }

  car.init();
  car.initMLPModule();
  car.initEdgeDetectModule();
  car.initHoughModule();
  car.configDriverless(1);  // 1 = enable driverless mode.

  Serial.println(F("AutoCar ready."));
}

void loop() {
  mission.update();
}
