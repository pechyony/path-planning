#include <iostream>
#include <tuple>
#include <math.h>
#include "car.h"
#include "road.h"

using namespace std;

/**
 * Constructor
 * @param s Coordinate s of the car
 * @param d Coordinate d of the car
 */ 
FrenetState::FrenetState(float s_, float d_) : s(s_), d(d_)
{
}

/**
 * Constructor
 * @param s Coordinate s of the car
 * @param d Coordinate d of the car
 * @param x Coordinate x of the car
 * @param y Coordinate y of the car
 * @param vx Velocity of the car along x axis
 * @param vy Velocity of the car along y axis
 * @param t Timestamp of update
 */ 
State::State(float s_, float d_, float x_, float y_, float vx_, float vy_, milliseconds t_) : 
    x(x_), y(y_), vx(vx_), vy(vy_), ax(0), ay(0), t(t_)
{
    f.s = s_;
    f.d = d_;
}

/**
 * Constructor
 * @param road Road object where thecar drives
 */ 
Car::Car(Road& road_) : road(road_)
{
    state.f.s = -1;  // location of the car is not initialized yet
}

/**
 * Constructor
 * @param s Coordinate s of the car
 * @param d Coordinate d of the car
 * @param x Coordinate x of the car
 * @param y Coordinate y of the car
 * @param vx Velocity of the car along x axis
 * @param vy Velocity of the car along y axis
 * @param road Road object where thecar drives
 * @param t Timestamp of update
 */ 
Car::Car(float s, float d, float x, float y, float vx, float vy, Road& road_, milliseconds t) : road(road_) 
{
    state = State(s,d,x,y,vx,vy,t); // initialize state
    lane = road.getLane(state.f.d); // get current lane
}

/**
 * getLane Return lane of the car
 */
int Car::getLane()
{
    return lane;
}

/**
 * getSpeed Return speed of the car
 */
float Car::getSpeed()
{
    return sqrt(pow(state.vx,2)+pow(state.vy,2));
}

/**
 * getState Return state of the car
 */
State Car::getState()
{
    return state;
}

/**
 * update Update state of the car
 * @param s Coordinate s of the car
 * @param d Coordinate d of the car
 * @param x Coordinate x of the car
 * @param y Coordinate y of the car
 * @param vx Velocity of the car along x axis
 * @param vy Velocity of the car along y axis
 * @param t Timestamp of update
 */ 
void Car::update(float s, float d, float x, float y, float vx, float vy, milliseconds t) 
{
    if (state.f.s != -1) {
        // state of the car was initialized previously
        // update the state and the lane, compute acceleration
        prev_state = state;
        state = State(s,d,x,y,vx,vy,t);
        float dt = (state.t - prev_state.t).count()/1000.0;  // convert from milliseconds to seconds
        state.ax = (state.vx - prev_state.vx) / dt;
        state.ay = (state.vy - prev_state.vy) / dt;
        lane = road.getLane(state.f.d);
    }
    else {
        // state of the car was not initialized previously
        state = State(s,d,x,y,vx,vy,t);  // initialize state
        lane = road.getLane(state.f.d);  // get current lane
    }
}

/**
 * predict Predict location and speed of the car using constant acceleration model
 * @param dt Prediction horizon
 */ 
tuple<FrenetState,float> Car::predict(float dt) {
    FrenetState predicted_state;
    
    // predict location
    float pred_x = state.x + state.vx * dt + state.ax * pow(dt,2) / 2.0;
    float pred_y = state.y + state.vy * dt + state.ay * pow(dt,2) / 2.0;

    // predict speed and yaw angle
    float pred_vx = state.vx + state.ax * dt;
    float pred_vy = state.vy + state.ay * dt;
    float pred_theta = atan2(pred_vy,pred_vx);

    // compute predicted Frenet coordinates
    predicted_state = road.getFrenet(pred_x, pred_y, pred_theta);

    float predicted_speed = sqrt(pow(pred_vx,2) + pow(pred_vy,2));
    
    return make_tuple(predicted_state, predicted_speed);
}