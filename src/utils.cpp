#include "utils.h"

float vec2length2(const vec2 &v) {
    return v.x * v.x + v.y * v.y;
}

float vec2length(const vec2 &v) {
    return std::sqrt(vec2length2(v));
}

float Lerp(float a, float b, float t) {
    return a + ((b - a) * t);
}