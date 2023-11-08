#pragma once
#include <string>

bool split(const String &cmd, const String &prefix, String &suffix) {
  if (cmd.startsWith(prefix)) {
    suffix = cmd.substring(prefix.length());
    return true;
  }
  return false;
}