#include <iostream>
#include <vector>
#include <math.h>
#include "trajectory_generation.h"
#include "helper.h"
#include "road.h"
#include "spline.h"
#include "car.h"

using namespace std;

/**
 * spline_trajectory Extend existing trajectory by using spline
 * @param road Road where the car drives
 * @param car Ego car
 * @param previous_path Previously generated trajectory that was not used by simulator
 * @param target_lane Lane where to extend the trajectory
 * @param target_velocity Desired velocity at the end of the new trajectory
 * @param end_prev_trajectory Frenet coordinates of the last point of previously unused trajectory 
 */
Trajectory spline_trajectory(Road& road, Car& car, Trajectory& previous_path, int target_lane, float target_velocity,
                             FrenetState& end_prev_trajectory) 
{
    // The code in this function was taken from Q&A video

    // Create a list of widely spaced (x,y) waypoints, evenly spaced at 30m
	// later we will interpolate these waypoints with a spline and fill it in with more points that control 

	vector<double> ptsx;   // we have to use double since spline library doesn't work with float
	vector<double> ptsy;

	// reference x,y,yaw states
	// we will reference either the starting point as where the car is or at the previous path end point
    State state = car.getState();
	float ref_x = state.x;
	float ref_y = state.y;
	float car_yaw = atan2(state.vy, state.vx);
	float ref_yaw; 
    
	FrenetState last_point;  // Frenet coordinates of the last known point of trajectory

	// if previous size is almost empty, use the car as starting reference
    int prev_size = previous_path.x.size();
	if (prev_size < 2) {
		if (car.getSpeed() > 0) {
            float prev_car_x = state.x - state.vx/50.0; 
		    float prev_car_y = state.y - state.vy/50.0; 
			ptsx.push_back(prev_car_x);
			ptsy.push_back(prev_car_y);
		}
		
		ptsx.push_back(state.x);
		ptsy.push_back(state.y);

        ref_yaw = car_yaw;
		last_point = state.f;
	}
	// use the previous path's end point as starting reference
    else {
        ref_x = previous_path.x[prev_size - 1];
		ref_y = previous_path.y[prev_size - 1];

		float ref_x_prev = previous_path.x[prev_size - 2];
		float ref_y_prev = previous_path.y[prev_size - 2];
		ref_yaw = atan2(ref_y - ref_y_prev, ref_x - ref_x_prev);

		ptsx.push_back(ref_x_prev);
		ptsx.push_back(ref_x);

		ptsy.push_back(ref_y_prev);
		ptsy.push_back(ref_y);

		last_point = end_prev_trajectory;
	}

    // In Frenet add evenly 50m spaced points ahead of the starting reference
	float lane_width = road.getLaneWidth();
	vector<float> next_wp0 = road.getXY(last_point.s+30, lane_width/2+lane_width*target_lane);   
	vector<float> next_wp1 = road.getXY(last_point.s+60, lane_width/2+lane_width*target_lane);
	vector<float> next_wp2 = road.getXY(last_point.s+90, lane_width/2+lane_width*target_lane);

    ptsx.push_back(next_wp0[0]);
	ptsx.push_back(next_wp1[0]);
	ptsx.push_back(next_wp2[0]);

    ptsy.push_back(next_wp0[1]);
	ptsy.push_back(next_wp1[1]);
	ptsy.push_back(next_wp2[1]);

    // convert reference points to car's coordinate system
    for (int i = 0; i < ptsx.size(); i++) {
		float shift_x = ptsx[i] - ref_x;
	    float shift_y = ptsy[i] - ref_y;

		ptsx[i] = shift_x * cos(0 - ref_yaw) - shift_y * sin(0 - ref_yaw);
		ptsy[i] = shift_x * sin(0 - ref_yaw) + shift_y * cos(0 - ref_yaw);
	}

	// create a spline in car's coordinate system
	tk::spline s;

	// set (x,y) points to the spline
    s.set_points(ptsx, ptsy);

	// Define the actual (x,y) points we will use for the planner
    Trajectory trajectory;

	// Start with all of the previous path points from the last time
    for (int i = 0; i < previous_path.x.size(); i++) {
	    trajectory.x.push_back(previous_path.x[i]);
		trajectory.y.push_back(previous_path.y[i]);
	}

    // CaLculate how to break up the spline so that we travel at our desired reference velocity
	float target_x = 30;
	float target_y = s(target_x);
    float target_dist = sqrt(pow(target_x,2) + pow(target_y,2));

	float x_add_on = 0;

	// Fill up the rest of our path planner after filling it with previous points, here we will always output 50 points
    for (int i=1; i <= 20 - prev_size; i++) {
	    float N = (target_dist / (0.02*target_velocity/2.24));  // 2.24 - convert from mph to m/s
		float x_point = x_add_on + target_x / N;
		float y_point = s(x_point);

		x_add_on = x_point;

		float x_ref = x_point;
	    float y_ref = y_point;

		// convert back to global coordinate system (previously spline was built in car's coordinate system)
		x_point = x_ref * cos(ref_yaw) - y_ref * sin(ref_yaw);
		y_point = x_ref * sin(ref_yaw) + y_ref * cos(ref_yaw);

		x_point += ref_x;
		y_point += ref_y; 

		trajectory.x.push_back(x_point);
		trajectory.y.push_back(y_point); 
	}

	return trajectory;
}