#include "helper.h"

// For converting back and forth between radians and degrees.
float deg2rad(float x) { return x * pi() / 180; }
float rad2deg(float x) { return x * 180 / pi(); }

// compute distance between two points
float distance(float x1, float y1, float x2, float y2)
{
	return sqrt((x2-x1)*(x2-x1)+(y2-y1)*(y2-y1));
}