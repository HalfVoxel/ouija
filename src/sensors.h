#pragma once
#include "driver/touch_pad.h"
#include "math.h"
#include <Arduino.h>

float readAndSmooth(uint8_t pin, float &value, float alpha) {
  auto newValue = analogRead(pin) / 4095.0f;
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

  float value() const { return mValue; }

  float update() {
    auto now = millis();
    auto dt = (now - mLastUpdate) * 0.001f;
    if (now == mLastUpdate)
      return mValue;
    auto alpha = mLastUpdate != 0 ? mAlpha * dt : 1.0f;
    mLastUpdate = now;
    readAndSmooth(mPin, mValue, alpha);
    return mValue;
  }
};

bool touchInitialized = false;

#define TOUCH_THRESH_NO_USE 0
#define TOUCH_FILTER_MODE_EN true
#define TOUCHPAD_FILTER_TOUCH_PERIOD 10

void initTouch() {
  if (touchInitialized)
    return;

  ESP_ERROR_CHECK(touch_pad_init());

  // This is required when using touch pad for wakeup
  // See https://github.com/espressif/esp-idf/blob/master/examples/system/deep_sleep/main/touch_wakeup.c
  touch_pad_set_fsm_mode(TOUCH_FSM_MODE_TIMER);

  ESP_ERROR_CHECK(touch_pad_set_voltage(TOUCH_HVOLT_2V7, TOUCH_LVOLT_0V5, TOUCH_HVOLT_ATTEN_1V));
  ESP_ERROR_CHECK(touch_pad_filter_start(TOUCHPAD_FILTER_TOUCH_PERIOD));
  touchInitialized = true;
}

struct TouchSensor {
  touch_pad_t mPin;
  uint16_t mThreshold;
  uint16_t mThresholdSleep;
  uint16_t mValue = 0;

  TouchSensor(touch_pad_t pin, uint16_t threshold, uint16_t thresholdSleep)
      : mPin(pin), mThreshold(threshold), mThresholdSleep(thresholdSleep) {}

  void init() {
    initTouch();
    ESP_ERROR_CHECK(touch_pad_config(mPin, mThresholdSleep));
    // Keep power to peripherals like touch pads on during deep sleep
    ESP_ERROR_CHECK(esp_sleep_pd_config(ESP_PD_DOMAIN_RTC_PERIPH, ESP_PD_OPTION_ON));
    ESP_ERROR_CHECK(esp_sleep_enable_touchpad_wakeup());
  }

  void update() { touch_pad_read_filtered(mPin, &mValue); }

  bool isTouched() { return mValue < mThreshold; }

  uint32_t value() { return mValue; }

  void setWakeupFromTouch() {
    // ESP_ERROR_CHECK(touch_pad_set_trigger_source(TOUCH_TRIGGER_SOURCE_BOTH));
    // ESP_ERROR_CHECK(touch_pad_set_group_mask(1 << mPin, 0, 1 << mPin));
    // auto trigger_mode = TOUCH_TRIGGER_BELOW;
    // ESP_ERROR_CHECK(touch_pad_get_trigger_mode(&trigger_mode));
    // ESP_ERROR_CHECK(esp_sleep_enable_touchpad_wakeup());
  }
};
