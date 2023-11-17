#pragma once
const uint32_t MICROS_PER_SECOND = 1000000;
const uint32_t MILLIS_PER_SECOND = 1000;

bool edge(unsigned long lastT, unsigned long t, unsigned long interval) {
  interval *= 1000;
  return (lastT / interval) != (t / interval);
}
