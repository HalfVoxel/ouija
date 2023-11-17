#pragma once
#include <algorithm>

using namespace std;

float lerp(float a, float b, float t) { return a + (b - a) * min(1.0f, max(0.0f, t)); }

float unlerp(float a, float b, float t) { return (t - a) / (b - a); }

float clamp(float a, float b, float t) { return min(b, max(a, t)); }