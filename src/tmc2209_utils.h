#pragma once
#include <FastAccelStepper.h>
#include <TMC2209x.h>

uint32_t microsteps = 16;

// TODO: Might require a small delay before reading status? Otherwise spurious errors can happen
void printDriverStatus(TMC2209 &driver) {
  auto st = driver.getStatus();
  auto ms = static_cast<int32_t>(driver.getMicrostepsPerStep());

  Serial.print("Current scaling=");
  Serial.println(st.current_scaling);
  Serial.print("Microsteps=");
  Serial.println(ms);
  auto s1 = driver.getPwmScaleAuto();
  auto s2 = driver.getPwmOffsetAuto();
  auto s3 = driver.getPwmScaleSum();
  auto s4 = driver.getPwmGradientAuto();
  Serial.print("S=");
  Serial.print(s1);
  Serial.print(", ");
  Serial.print(s2);
  Serial.print(", ");
  Serial.print(s3);
  Serial.print(", ");
  Serial.print(s4);
  Serial.println();

  if (st.over_temperature_warning) {
    Serial.println("over_temperature_warning");
  }
  if (st.over_temperature_shutdown) {
    Serial.println("over_temperature_shutdown");
  }
  if (st.short_to_ground_a) {
    Serial.println("short_to_ground_a");
  }
  if (st.short_to_ground_b) {
    Serial.println("short_to_ground_b");
  }
  if (st.low_side_short_a) {
    Serial.println("low_side_short_a");
  }
  if (st.low_side_short_b) {
    Serial.println("low_side_short_b");
  }
  if (st.open_load_a) {
    Serial.println("open_load_a");
  }
  if (st.open_load_b) {
    Serial.println("open_load_b");
  }
  if (st.over_temperature_120c) {
    Serial.println("over_temperature_120c");
  }
  if (st.over_temperature_143c) {
    Serial.println("over_temperature_143c");
  }
  if (st.over_temperature_150c) {
    Serial.println("over_temperature_150c");
  }
  if (st.over_temperature_157c) {
    Serial.println("over_temperature_157c");
  }
  if (st.stealth_chop_mode) {
    Serial.println("stealth_chop_mode");
  }
  if (st.standstill) {
    Serial.println("standstill");
  }
}

const uint32_t STALL_BASE_STRIDE_HZ = 10;
int32_t stallBase[] = {
    10, 38, 80, 133, 187, 239, 297, 341, 381, 384,
};

int32_t getStallBase(uint32_t speedHz) {
  uint32_t maxIndex = sizeof(stallBase) / sizeof(*stallBase);
  float t = sqrt(speedHz / static_cast<float>(STALL_BASE_STRIDE_HZ)) - 1;
  uint32_t index1 = static_cast<uint32_t>(t);
  uint32_t index2 = index1 + 1;
  float interp = t - index1;
  index1 = max(min(index1, maxIndex - 1), 0U);
  index2 = max(min(index2, maxIndex - 1), 0U);
  return lerp(stallBase[index1], stallBase[index2], interp);
}

void calibrate(FastAccelStepper *stepper, TMC2209 &driver) {
  for (uint32_t i = 1; i <= 10; i++) {
    uint32_t speed = i * i * STALL_BASE_STRIDE_HZ;
    stepper->setSpeedInHz(speed * microsteps);
    stepper->runForward();
    delay(10);
    int32_t stall = 0;
    const int32_t SAMPLES = 100;
    for (int j = 0; j < SAMPLES; j++) {
      delay(10);
      stall += driver.getStallGuardResult();
    }
    stall /= SAMPLES;
    Serial.print("Stall base for ");
    Serial.print(speed);
    Serial.print(" ");
    Serial.println(stall);
    stallBase[i - 1] = stall;
  }
  for (uint32_t i = 1; i <= 10; i++) {
    uint32_t speed = i * i * STALL_BASE_STRIDE_HZ;
    Serial.print("Stall check: ");
    Serial.print(stallBase[i - 1]);
    Serial.print(" = ");
    Serial.println(getStallBase(speed));
  }
  delay(2000);
}
