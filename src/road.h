#ifndef _ROAD_H_
#define _ROAD_H_

#include <vector>
#include <map>
#include <tuple>
#include <chrono>
#include "car.h"

using namespace std;
using namespace std::chrono;

// Class that represents the road 
class Road {
  public:

    /**
     * Constructor
     * @param n_lanes Number of lanes
     * @param lane_width Windth of the lane (in meters)
     * @param map_file Name of the file with coordinates of waypoints 
     * @param max_speed Maximal allowable speed (in mph)
     */ 
    Road(int n_lanes, float lane_width, string map_file, float max_speed);

    /**
     * getLane Convert Frenet d coordinate to lane number
     * @param d Frenet d coordinate 
     */
    int getLane(float d);

    /**
     * getLaneWidth Return width of the lane
     */
    float getLaneWidth();

    /**
     * getCenter Find d coordinate of a given lane
     * @param lane Lane number
     */
    float getCenter(int lane);

    /**
     * getFrenet Convert Cartesian coordinates to Frenet
     * @param x Coordinate x
     * @param y Coordinate y
     * @param theta Yaw of the car
     */
    FrenetState getFrenet(float x, float y, float theta);

    /**
     *  getXY Convert Frenet coordinates to cartesian 
     *  @param s Coordinate s
     *  @param d Coordinate d
     */
    vector<float> getXY(float s, float d);

    /**
     * updateCar Update state vector of the car
     * @param id - ID of the car
     * @param x Coordinate x
     * @param y Coordinate y
     * @param vx Projection of velocity to x axis 
     * @param vy Projection of velocity to y axis
     * @param s Coordinate s
     * @param d Coordinate d
     * @param t Timestamp (in milliseconds since 1/1/1970)
     */
    Car& updateCar(int id, float x, float y, float vx, float vy, float s, float d, milliseconds t);

    /**
     * predict Return list of predicted Frenet states and lanes of all observed cars
     * @param dt - predictiom time insterval (in milliseconds)
     */ 
    vector<tuple<FrenetState,float,int>> predict(float dt);

  private:
    int n_lanes;            // number of lanes
    float lane_width;       // lane width
    float max_speed;        // maximal speed
    map<int,Car> cars;      // mapping from car ID to car object

    vector<float> map_waypoints_x;  // x coordinates of waypoints
    vector<float> map_waypoints_y;  // y coordinates of waypoints
    vector<float> map_waypoints_s;  // s coordinates of waypoints
    vector<float> map_waypoints_dx; // x component of the normal to the road at the location of waypoint
    vector<float> map_waypoints_dy; // y component of the normal to the road at the location of waypoint

    /**
     * closestWaypoints Find a closest waypoint to a given (x,y) point
     * @param x Coordinate x
     * @param y Coordinate y
     */
    int closestWaypoint(float x, float y);

    /**
     * nextWaypoint Find the next waypoint ahead of car
     * @param x Coordinate x of the car
     * @param y Coordinate y of the car
     * @param theta Yaw of the car
     */ 
    int nextWaypoint(float x, float y, float theta);
};

#endif