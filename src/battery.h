#pragma once
#include "math.h"
#include "sensors.h"
#include <Arduino.h>

const float BATTERY_MAX_VOLTAGE = 4.2;
const float BATTERY_MIN_VOLTAGE = 3.3;
const float R1 = 100000;
const float R2 = 22000;
const float SCALING = 3.93f;

class Battery {
public:
  Battery(AnalogSensor sensor) : mSensor(sensor) {}
  void init() { mSensor.init(); }
  void update() { mSensor.update(); }

  float getRaw() const { return mSensor.value(); }
  float getVoltage() const { return getRaw() * (R1 + R2) * SCALING / R2; }
  float getPercentage() const {
    return 123.0f - 123.0f / powf(1.0f + powf(getVoltage() / 3.7f, 80.0f), 0.165f);
    // return 100.0f * clamp(0.0f, 1.0f, unlerp(BATTERY_MIN_VOLTAGE, BATTERY_MAX_VOLTAGE, getVoltage()));
  }

private:
  AnalogSensor mSensor;
};