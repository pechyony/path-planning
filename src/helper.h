#ifndef _HELPER_H_
#define _HELPER_H_

#include <math.h>

constexpr float pi() { return M_PI; }

// For converting back and forth between radians and degrees.
float deg2rad(float x);
float rad2deg(float x);

float distance(float x1, float y1, float x2, float y2);

#endif