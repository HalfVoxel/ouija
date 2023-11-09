#pragma once
#include <Arduino.h>

using namespace std;

bool split(const String &cmd, const String &prefix, String &suffix) {
  if (cmd.startsWith(prefix)) {
    suffix = cmd.substring(prefix.length());
    return true;
  }
  return false;
}