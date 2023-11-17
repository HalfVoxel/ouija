#pragma once
#include "math.h"
#include <Arduino.h>

float readAndSmooth(uint8_t pin, float &value, float alpha) {
  auto newValue = analogRead(pin) / 4096.0f;
  value = lerp(value, newValue, alpha);
  return value;
}

struct AnalogSensor {
  uint8_t mPin;

  float mValue = 0.0f;
  float mAlpha;
  unsigned long mLastUpdate = 0;

  AnalogSensor(uint8_t pin, float smoothing) : mPin(pin), mAlpha(1.0f / smoothing) {}

  void init() { pinMode(mPin, INPUT); }

  float value() { return mValue; }

  float update() {
    auto now = millis();
    auto dt = (now - lastUpdate) * 0.000001f;
    lastUpdate = now;
    readAndSmooth(pin, value, alpha * dt);
    return value;
  }
};
