#include "TMC2209x.h"

// TODO: Might require a small delay before reading status? Otherwise spurious errors can happen
void printDriverStatus(TMC2209& driver) {
  auto st = driver.getStatus();
  auto ms = driver.getMicrostepsPerStep();

  Serial.print("Current scaling=");
  Serial.println(st.current_scaling);
  Serial.print("Microsteps=");
  Serial.println(ms);
  auto s1 =driver.getPwmScaleAuto();
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