#pragma once
#include "math.h"
#include <Arduino.h>

float readAndSmooth(uint8_t pin, float &value, float alpha) {
  auto newValue = analogRead(pin) / 4096.0f;
  value = lerp(value, newValue, alpha);
  return value;
}

struct AnalogSensor {
  uint8_t pin;

  float value = 0.0f;
  float alpha;
  unsigned long lastUpdate = 0;

  AnalogSensor(uint8_t pin, float smoothing) : pin(pin), alpha(1.0f / smoothing) {}

  void init() { pinMode(pin, INPUT); }

  float update() {
    auto now = millis();
    auto dt = (now - lastUpdate) * 0.000001f;
    lastUpdate = now;
    readAndSmooth(pin, value, alpha * dt);
    return value;
  }
};
