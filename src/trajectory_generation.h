#ifndef _TRAJECTORY_GENERATION_H_
#define _TRAJECTORY_GENERATION_H_

#include <vector>
#include "road.h"

using namespace std;

// Trajectory - list of x and y coordinates
struct Trajectory {
    vector<float> x;
    vector<float> y;
};

/**
 * spline_trajectory Extend existing trajectory by using spline
 * @param road Road where the car drives
 * @param car Ego car
 * @param previous_path Previously generated trajectory that was not used by simulator
 * @param target_lane Lane where to extend the trajectory
 * @param target_velocity Desired velocity at the end of the new trajectory
 */
Trajectory spline_trajectory(Road& road, Car& car, Trajectory& previous_path, int target_lane, float target_velocity);

#endif

