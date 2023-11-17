#pragma once
#include "math.h"
#include <Arduino.h>

const float BATTERY_MAX_VOLTAGE = 4.2;
const float BATTERY_MIN_VOLTAGE = 3.3;
const float R1 = 100000;
const float R2 = 22000;
const float SCALING = 3.93f;

class Battery {
public:
  Battery(int pin) : _pin(pin) {}
  void init() { pinMode(_pin, INPUT); }
  void update() { _raw = analogRead(_pin); }

  uint32_t getRaw() const { return _raw; }
  float getVoltage() const { return static_cast<float>(getRaw()) * (R1 + R2) * SCALING / (R2 * 4095.0f); }
  float getPercentage() const {
    return 123.0f - 123.0f / powf(1.0f + powf(getVoltage() / 3.7f, 80.0f), 0.165f);
    // return 100.0f * clamp(0.0f, 1.0f, unlerp(BATTERY_MIN_VOLTAGE, BATTERY_MAX_VOLTAGE, getVoltage()));
  }

private:
  int _pin;
  uint32_t _raw;
};