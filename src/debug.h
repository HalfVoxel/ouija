#pragma once
#include <Arduino.h>
#include <stdint.h>

const uint32_t LED_PIN = 2; // 22;

void flash(uint32_t count = 5, uint32_t delayMs = 50) {
  for (int i = 0; i < count; i++) {
    digitalWrite(LED_PIN, HIGH);
    delay(delayMs);
    digitalWrite(LED_PIN, LOW);
    delay(delayMs);
  }
}

void flashError() { flash(10, 100); }

void criticalError(const String message) {
  flashError();
  Serial.print("Critical error: ");
  Serial.println(message);
  while (true) {
    delay(1000);
  }
}