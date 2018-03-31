#include <iostream>
#include <tuple>
#include <math.h>
#include "behavioral_planner.h"
#include "car.h"
#include "trajectory_generation.h"

using namespace std;

/**
  * Constructor
  * @param n_lanes Number of lanes in the road
  */ 
FSM::FSM(int n_lanes_) : n_lanes (n_lanes_), curr_state(KL)  // set current state to "keep current lane"
{}

/**
  * successor_states Get all possible successor states of the given state
  * @param lane Current lane
  */ 
vector<PossibleStates> FSM::successor_states(int lane)
{
    vector<PossibleStates> states;

    // the car can continue to drive in the same lane
    states.push_back(KL);

    if (curr_state == KL) {
        // check if the car can switch lane
        if (lane != 0)
            states.push_back(LCL);
        if (lane != n_lanes-1)    
            states.push_back(LCR);
    } else
        // after switching lane the car can only continue to drive straight ahead
        states.push_back(KL); 

    return states;
}

/**
  * set_state Set the state of FSM
  * @param next_state Next state of FSM
  */ 
void FSM::set_state(PossibleStates next_state)
{
    curr_state = next_state;
}

/**
 * Constructor
 * @param road Road where the car drives
 * @param n_lanes Number of lanes in the road
 * @param init_lane Initial lane of the car 
 * @param init_vel Initial velocity of the car
 * @param max_vel Maximal velocity of the car
 * @param safety_buffer Minimal safety distance to the closest car in the same lane  
 * @param max_s Maximal value of coordinate s in the road
 */
BehavioralPlanner::BehavioralPlanner(Road& road_, int n_lanes, int init_lane, float init_vel_, float max_vel_, 
                                     float safety_buffer_, float max_s_) : 
    road(road_), fsm(n_lanes), target_lane(init_lane), ref_vel(init_vel_), max_vel(max_vel_), 
    safety_buffer(safety_buffer_), max_s(max_s_)
{}

/**
 * update_trajectory Add new points to the planned trajectory of the car
 * @param car Car that drives this trajectory
 * @param predictions Predictions of the locations of all other cars
 * @param prev_trajectory Trajectory that was generated in the previous call and was not used yet by simulator
 * @param end_prev_trajectory Frenet coordinates of the last point of previously unused trajectory 
 */ 
Trajectory BehavioralPlanner::update_trajectory(Car& car, vector<tuple<FrenetState,float,int>> predictions, Trajectory& prev_trajectory, 
                                                FrenetState& end_prev_trajectory)
{
    vector<PossibleStates> possible_successor_states;
    float max_value = 0, value, best_ref_vel;
    PossibleStates best_state = KL;
    int best_lane = target_lane;
    tuple<Trajectory, float, float, int> trajectory_value_lane;
    Trajectory best_trajectory;

    // create list of possible successor states
    possible_successor_states = fsm.successor_states(target_lane);

    // iterate over possible successor states
    vector<PossibleStates>::iterator next_state;
    for (next_state = possible_successor_states.begin(); next_state != possible_successor_states.end(); next_state++) {
        
        // create trajectory if we were chosing this state and compute the value of this trajectory
        // value = average speed of the lane if the car switches to a new lane
        //         speed of the car at the end of trajectory if the car stays in the current lane 
        trajectory_value_lane = candidate_trajectory(*next_state, predictions, prev_trajectory, end_prev_trajectory, car);
        value = get<1>(trajectory_value_lane);
        
        if (value > max_value) {
            // save trajectory and its parameters (value, lane, speed at the end of trajectory) if the new trajectory is 
            // the best one found so far 
            max_value = value;
            best_trajectory = get<0>(trajectory_value_lane);
            best_ref_vel = get<2>(trajectory_value_lane);
            best_lane = get<3>(trajectory_value_lane);
        }
    }

    // save the speed at the end of the best trajectory and the lane of the best trajectory 
    ref_vel = best_ref_vel;
    target_lane = best_lane;

    // return the best trajectory
    return best_trajectory;     
}

/**
 * candidate_trajectory Generate new candidate trajectory when the car switches to the given new state
 *                      Return candidate trajectory, average speed of the new lane, speed of the car at the end of the trajectory,
 *                             lane of the car at the end of trajectory
 * @param next_state New state fo the car
 * @param predictions Predictions of the locations of all other cars
 * @param prev_trajectory Trajectory that was generated in the previous call and was not used yet by simulator
 * @param end_prev_trajectory Frenet coordinates of the last point of previously unused trajectory 
 * @param car Car that drives this trajectory
 */ 
tuple<Trajectory,float,float,int> BehavioralPlanner::candidate_trajectory(PossibleStates next_state, 
                                                                          vector<tuple<FrenetState,float,int>> predictions, 
                                                                          Trajectory& prev_trajectory, FrenetState& end_prev_trajectory, 
                                                                          Car& car) 
{
    int end_lane;
    tuple<Trajectory, float, float> trajectory_value; 
    tuple<Trajectory, float, float, int> trajectory_value_lane;
    
    // generate trajectory according to the new candidate state
    switch(next_state) {
        case KL:  // keep lane
            trajectory_value = straight_trajectory(predictions, prev_trajectory, end_prev_trajectory, car);
            end_lane = target_lane;
            break;
        case LCL: // change to the left lane
            trajectory_value = turn_trajectory(predictions, prev_trajectory, target_lane - 1, end_prev_trajectory, car);
            end_lane = target_lane - 1;
            break;
        case LCR: // change to the right lane
            trajectory_value = turn_trajectory(predictions, prev_trajectory, target_lane + 1, end_prev_trajectory, car);
            end_lane = target_lane + 1;
    }

    Trajectory trajectory = get<0>(trajectory_value); 
    trajectory_value_lane = make_tuple(trajectory, // new trajectory 
                                       float(get<1>(trajectory_value)), // average speed of the lane
                                       float(get<2>(trajectory_value)), //speed of the car at the end of trajectory
                                       end_lane);                       // lane of the car at the end of trajectory
    return trajectory_value_lane;
} 

/**
 * straight_trajectory Generate new candidate trajectory when the car keeps driving in the same lane
 *                     Return candidate trajectory, average speed of the new lane, speed of the car at the end of the trajectory
 * @param predictions Predictions of the locations of all other cars
 * @param prev_trajectory Trajectory that was generated in the previous call and was not used yet by simulator
 * @param end_prev_trajectory Frenet coordinates of the last point of previously unused trajectory 
 * @param car Car that drives this trajectory
 */  
tuple<Trajectory,float,float> BehavioralPlanner::straight_trajectory(vector<tuple<FrenetState,float,int>> predictions, 
                                                                     Trajectory& prev_trajectory, FrenetState& end_prev_trajectory, 
                                                                     Car& car)
{
    // check if there are cars ahead of us
    float new_ref_vel = ref_vel;
    if (other_car(predictions, end_prev_trajectory, 0, safety_buffer, target_lane) == true)
		new_ref_vel -= 0.224; // decrease of velocity by 0.224 mph is an acceleration of -5 m/s^2
	else if (ref_vel < max_vel - 0.224)
        // no car ahead of us and we are driving slower than max velocity, hence we can increase the velocity
		new_ref_vel += 0.224;
    else if (ref_vel > max_vel)
        // we drove a bit faster than maximal speed (might happen because of the approximations when generating a trajectory)
        new_ref_vel -= 0.224;  // decrease the velocity

    // extend current trajectory
    Trajectory trajectory;
    trajectory = spline_trajectory(road, car, prev_trajectory, target_lane, new_ref_vel);

    return make_tuple(trajectory, new_ref_vel, new_ref_vel);
}

/**
 * turn_trajectory Generate new candidate trajectory when the car swithes lanes
 *                 Return candidate trajectory, average speed of the new lane, speed of the car at the end of the trajectory  
 * @param predictions Predictions of the locations of all other cars
 * @param prev_trajectory Trajectory that was generated in the previous call and was not used yet by simulator
 * @param target_lane New lane of the car
 * @param end_prev_trajectory Frenet coordinates of the last point of previously unused trajectory 
 * @param car Car that drives this trajectory
 */
tuple<Trajectory,float,float> BehavioralPlanner::turn_trajectory(vector<tuple<FrenetState,float,int>> predictions,  
                                                                 Trajectory& prev_trajectory, int future_target_lane, 
                                                                 FrenetState& end_prev_trajectory, Car& car)
{
    Trajectory trajectory;

    // get average speed at target lane in safety_buffer+10000 meters ahead
    float end_horizon = end_prev_trajectory.s+safety_buffer+1000;
    if (end_horizon > max_s)
        end_horizon -= max_s;

    // compute average speed of the cars ahead
    float lane_average_speed = getAverageSpeed(predictions, future_target_lane, end_prev_trajectory.s+safety_buffer, end_horizon) * 2.24;

    if (lane_average_speed < ref_vel - 0.224) 
        // the other lane is slower than the current speed (even the ego car will start to slow down), no need to generate trajectory
        return make_tuple(trajectory, lane_average_speed, ref_vel);

    // check if there is no car nearby in the target lane
    if (other_car(predictions, end_prev_trajectory, safety_buffer, safety_buffer, future_target_lane) == true)
        return make_tuple(trajectory, -1, ref_vel); // switching lane is not safe because of the car nearby
    
    // speed up when switching to a new lane
    float new_ref_vel = ref_vel + 0.224;
    if (new_ref_vel > max_vel)
        new_ref_vel = max_vel;

    // extend existing trajectory by adding segments in the new lane    
    trajectory = spline_trajectory(road, car, prev_trajectory, future_target_lane, new_ref_vel);

    return make_tuple(trajectory, lane_average_speed, new_ref_vel);  
}

/**
 * other_car Check if there is other car in a given segment of the road
 * @param predictions Predictions of the locations of all other cars
 * @param end_prev_trajectory Frenet coordinates of the last point of previously unused trajectory 
 * @param buffer_rear safery distance behind the last point of previously unused trajectory
 * @param buffer_front safery distance in fron of the last point of previously unused trajectory
 * @param lane Lane of the road segment
 */    
bool BehavioralPlanner::other_car(vector<tuple<FrenetState,float,int>> predictions, FrenetState& end_prev_trajectory,
                                  float buffer_rear, float buffer_front, int lane)           
{
    vector<tuple<FrenetState,float,int>>::iterator prediction;
    float other_car_s;
    int other_car_lane;
    bool car_close = false;

    // iterate over all cars
    for (prediction = predictions.begin(); prediction != predictions.end() && car_close == false; prediction++) {
        other_car_s = (get<0>(*prediction)).s;
        other_car_lane = get<2>(*prediction);

        // check if the car is in designated lane and is close to predicted end point of trajectory
        if (other_car_lane == lane && other_car_s > end_prev_trajectory.s - buffer_rear && 
            other_car_s < end_prev_trajectory.s + buffer_front) 
            car_close = true;
    }

    return car_close;
}

/**
 * getAverageSpeed Get average speed of cars (in m/s) in a given road segment 
 * @param lane Lane number
 * @param min_position minimal s coordinate of the segment
 * @param max_position maximal s coordinate of the segment
 */ 
float BehavioralPlanner::getAverageSpeed(vector<tuple<FrenetState,float,int>> predictions, int lane, 
                                         float min_position, float max_position)
{
    float sum = 0;
    int n = 0;
    map<int,Car>::iterator it;
    vector<tuple<FrenetState,float,int>>::iterator prediction;

    // find cars in a given road segment
    for (prediction = predictions.begin(); prediction != predictions.end(); prediction++) 
        if (get<2>(*prediction) == lane) {
			float car_s = (get<0>(*prediction)).s;
			if ((max_position > min_position && car_s >= min_position && car_s < max_position) ||
			    (max_position < min_position && (car_s >= min_position || car_s < max_position))) {
                    sum += get<1>(*prediction);
                    n++;
		    }
        }

    if (n != 0)
	    // there are cars in a given road segment, return their average speed
        return sum/n;
	else
	    // no cars in a given segment, return maximum allowable speed
	    return max_vel;
}