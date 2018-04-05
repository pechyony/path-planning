#ifndef _BEHAVIORAL_PLANNER_H_
#define _BEHAVIORAL_PLANNER_H_

#include <vector>
#include "car.h"
#include "trajectory_generation.h"

using namespace std;

// list of possible states of finite state machine
enum PossibleStates {
    KL,    // keep driving in the current lane
    LCR,   // change to the right lane 
    LCL    // change to the left lane
};

// class that represents finite state machine (FSM)
class FSM {
  public:
    
    /**
     * Constructor
     * @param n_lanes Number of lanes in the road
     * @param max_turn_counter Minimal number of "KEEP LANE" states between two changes of lanes
     * @param max_init Number of initial iterations before any lane change is permitted
     */ 
    FSM(int n_lanes, int max_turn_counter, int max_init);

    /**
     * successor_states Get all possible successor states of the given state
     * @param lane Current lane
     */ 
    vector<PossibleStates> successor_states(int lane);

    /**
     * set_state Set the state of FSM
     * @param next_state Next state of FSM
     */ 
    void set_state(PossibleStates next_state);

  private:
    PossibleStates curr_state;  // current state
    int n_lanes;                // number of lanes in the road
    int turn_counter;           // number of state updates until change of lane is permitted
    int max_turn_counter;       // minimal number of "KEEP LANE" states between two changes of lanes
    int init_counter;           // number of iterations until any lane change is permitted               
};

// class that represents behavioral planner
class BehavioralPlanner {
  public:

    /**
     * Constructor
     * @param road Road where the car drives
     * @param n_lanes Number of lanes in the road
     * @param init_lane Initial lane of the car 
     * @param init_vel Initial velocity of the car
     * @param max_vel Maximal velocity of the car
     * @param safety_buffer Minimal safety distance to the closest car in the same lane  
     * @param max_s Maximal value of coordinate s in the road
     * @param max_turn_counter Minimal number of "KEEP LANE" states between two changes of lanes
     * @param max_init Number of initial iterations before any lane change is permitted
     * @param path_length Length of the generated trajectory
     */ 
    BehavioralPlanner(Road& road, int n_lanes, int init_lane, float init_vel, float max_vel, float safety_buffer, 
                      float max_s, int max_turn_counter, int max_init, int path_length);

    /**
     * update_trajectory Add new points to the planned trajectory of the car
     * @param car Car that drives this trajectory
     * @param predictions Predictions of the locations of all other cars
     * @param prev_trajectory Trajectory that was generated in the previous call and was not used yet by simulator
     * @param end_prev_trajectory Frenet coordinates of the last point of previously unused trajectory 
     */ 
    Trajectory update_trajectory(Car& car, vector<tuple<FrenetState,float,int>> predictions, Trajectory& prev_trajectory, 
                                 FrenetState& end_prev_trajectory);
    
  private:
    FSM fsm;                    // finite state machine
    int target_lane;            // lane where the trajecotry should be extended
    float ref_vel;              // current velocity
    float max_vel;              // maximal allowable velocity
    float safety_buffer;        // minimal safety distance to the closest car in the same lane
    float max_s;                // maximal value of s coodinate
    int path_length;            // length of the generated trajectory
    Road& road;                 // road where the car drives       

    /**
     * candidate_trajectory Generate new candidate trajectory when the car switches to the given new state
     *                      Return candidate trajectory, average speed of the new lane, speed of the car at the end of trajectory,
     *                             lane of the car at the end of the trajectory
     * @param next_state New state fo the car
     * @param predictions Predictions of the locations of all other cars
     * @param prev_trajectory Trajectory that was generated in the previous call and was not used yet by simulator
     * @param end_prev_trajectory Frenet coordinates of the last point of previously unused trajectory 
     * @param car Car that drives this trajectory
     */ 
    tuple<Trajectory,float,float,int> candidate_trajectory(PossibleStates next_state, vector<tuple<FrenetState,float,int>> predictions, 
                                                           Trajectory& prev_trajectory, FrenetState& end_prev_trajectory, Car& car); 

    /**
     * straight_trajectory Generate new candidate trajectory when the car keeps driving in the same lane
     *                     Return candidate trajectory, average speed of the new lane, speed of the car at the end of the trajectory
     * @param predictions Predictions of the locations of all other cars
     * @param prev_trajectory Trajectory that was generated in the previous call and was not used yet by simulator
     * @param end_prev_trajectory Frenet coordinates of the last point of previously unused trajectory 
     * @param car Car that drives this trajectory
     */                                                   
    tuple<Trajectory,float,float> straight_trajectory(vector<tuple<FrenetState,float,int>> predictions, Trajectory& prev_trajectory,
                                                FrenetState& end_prev_trajectory, Car& car);

    /**
     * turn_trajectory Generate new candidate trajectory when the car swithes lanes
     *                 Return candidate trajectory, average speed of the new lane, speed of the car at the end of the trajectory  
     * @param predictions Predictions of the locations of all other cars
     * @param prev_trajectory Trajectory that was generated in the previous call and was not used yet by simulator
     * @param target_lane New lane of the car
     * @param end_prev_trajectory Frenet coordinates of the last point of previously unused trajectory 
     * @param car Car that drives this trajectory
     */                                                
    tuple<Trajectory,float,float> turn_trajectory(vector<tuple<FrenetState,float,int>> predictions, Trajectory& prev_trajectory, 
                                            int target_lane, FrenetState& end_prev_trajectory, Car& car);

    /**
     * other_car Check if there is other car in a given segment of the road
     * @param predictions Predictions of the locations of all other cars
     * @param end_prev_trajectory Frenet coordinates of the last point of previously unused trajectory 
     * @param buffer_rear safery distance behind the last point of previously unused trajectory
     * @param buffer_front safery distance in fron of the last point of previously unused trajectory
     * @param lane Lane of the road segment
     */                                          
    bool other_car(vector<tuple<FrenetState,float,int>> predictions, FrenetState& end_prev_trajectory, 
                   float buffer_rear, float buffer_front, int lane);

    /**
     * getAverageSpeed Get average speed of cars in a given road segment 
     * @param lane Lane number
     * @param min_position minimal s coordinate of the segment
     * @param max_position maximal s coordinate of the segment
     */ 
    float getAverageSpeed(vector<tuple<FrenetState,float,int>> predictions, int lane, float min_position, float max_position);
};

#endif