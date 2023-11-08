#pragma once
#include<Arduino.h>
#include "math.h"

float readAndSmooth(uint8_t pin, float& value, float alpha) {
  auto newValue = analogRead(pin) / 4096.0f;
  value = lerp(value, newValue, alpha);
  return value;
}
