#include <map>
#include <fstream>
#include <math.h>
#include "json.hpp"
#include "road.h"
#include "helper.h"

using namespace std;

/**
 * Constructor
 * @param n_lanes Number of lanes
 * @param lane_width Windth of the lane (in meters)
 * @param map_file Name of the file with coordinates of waypoints 
 * @param max_speed Maximal allowable speed (in mph)
 */ 
Road::Road(int n_lanes_, float lane_width_, string map_file, float max_speed_) : 
    n_lanes(n_lanes_), lane_width(lane_width_), max_speed(max_speed_/2.24)
{
	// read waypoints from the map file
    ifstream in_map(map_file.c_str(), ifstream::in);

    string line;
    while (getline(in_map, line)) {
  	    istringstream iss(line);
  	    float x, y, s, d_x, d_y;
  	
        iss >> x;
  	    iss >> y;
  	    iss >> s;
  	    iss >> d_x;
  	    iss >> d_y;
  	    map_waypoints_x.push_back(x);
  	    map_waypoints_y.push_back(y);
  	    map_waypoints_s.push_back(s);
  	    map_waypoints_dx.push_back(d_x);
  	    map_waypoints_dy.push_back(d_y);
    }
}

/**
 * getLane Convert Frenet d coordinate to lane number
 * @param d Frenet d coordinate 
 */
int Road::getLane(float d)
{
	// return -1 if the car is outside of the lanes
    if (d < 0 || d >= n_lanes * lane_width)
        return -1;

    return int(d / lane_width); 
}

/**
 * getLaneWidth Return width of the lane
 */
float Road::getLaneWidth()
{
    return lane_width;
}

/**
 * getCenter Find d coordinate of a given lane
 * @param lane Lane number
 */
float Road::getCenter(int lane)
{
    return lane_width / 2 + lane * lane_width;
}

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
Car& Road::updateCar(int id, float x, float y, float vx, float vy, float s, float d, milliseconds t)
{
	// check if the car with the given id was observed previously
    map<int,Car>::iterator it = cars.find(id);
    if (it != cars.end()) {
		// update location of the previously observed car
        it->second.update(s,d,x,y,vx,vy,t);
	}
    else {
		// create a new car and update the mapping from id to this car 
        cars.insert(pair<int,Car>(id, Car(s,d,x,y,vx,vy,*this,t)));
		it = cars.find(id);
	}

	// return updated car
	return it->second;
}

/**
 * predict Return list of predicted Frenet states and lanes of all observed cars
 * @param dt - predictiom time insterval (in milliseconds)
 */ 
vector<tuple<FrenetState,float,int>> Road::predict(float dt)
{
    vector<tuple<FrenetState,float,int>> predictions;
    map<int,Car>::iterator it;
    tuple<FrenetState,float> pred_state_speed;
    FrenetState pred_state;
	float pred_speed;
    int lane;

    // iterate over all cars, predict their Frenet locations and lanes 
    for (it = cars.begin(); it != cars.end(); it++) {
        pred_state_speed = it->second.predict(dt);
        pred_state = get<0>(pred_state_speed);
		pred_speed = get<1>(pred_state_speed);
        lane = getLane(pred_state.d);
        predictions.push_back(make_tuple(pred_state, pred_speed, lane));
    }

    return predictions;
}

/**
 * getFrenet Convert Cartesian coordinates to Frenet
 * @param x Coordinate x
 * @param y Coordinate y
 * @param theta Yaw of the car
 */
FrenetState Road::getFrenet(float x, float y, float theta)
{
	// the code in this function is Q&A video

	// find closest waypoint ahead of P=(x,y) 
    int next_wp = nextWaypoint(x,y, theta);

	int prev_wp;
   	prev_wp = next_wp-1;
  	if (next_wp == 0) 
		prev_wp  = map_waypoints_x.size()-1;

    // create a vector N from a previous waypoint to the closes waypoint ahead of (x,y)
  	float n_x = map_waypoints_x[next_wp]-map_waypoints_x[prev_wp];
	float n_y = map_waypoints_y[next_wp]-map_waypoints_y[prev_wp];

	// create a vector PN from a previous waypoint to P
	float x_x = x - map_waypoints_x[prev_wp];
	float x_y = y - map_waypoints_y[prev_wp];

	// find the projection PNproj of PN onto N
	float proj_norm = (x_x*n_x+x_y*n_y)/(n_x*n_x+n_y*n_y);
	float proj_x = proj_norm*n_x;
  	float proj_y = proj_norm*n_y;

    // since waypoints have d=0, the distance between d of (x,y) is the distance between PN and PNproj
	float frenet_d = distance(x_x,x_y,proj_x,proj_y);

	// see if d value is positive or negative by comparing it to a center point of the map
	// convert center of the map point to coordinate system centered at the previous waypoint 
	// the converted center point in CenterN 
	float center_x = 1000-map_waypoints_x[prev_wp];
	float center_y = 2000-map_waypoints_y[prev_wp];

    // check if d value should be negative
    // in coordinate system centered at the previous waypoint, CenterN should be to the left of the road 
    // if PN is closer to CenterN than PNproj then d is negative 
	float centerToPos = distance(center_x,center_y,x_x,x_y);
	float centerToRef = distance(center_x,center_y,proj_x,proj_y);
	if (centerToPos <= centerToRef) 
		frenet_d *= -1;

	// calculate s value
	float frenet_s = 0;
	for (int i = 0; i < prev_wp; i++) 
		frenet_s += distance(map_waypoints_x[i], map_waypoints_y[i], map_waypoints_x[i+1], map_waypoints_y[i+1]);

	frenet_s += distance(0,0,proj_x,proj_y);

	return FrenetState(frenet_s, frenet_d);
}

/**
 * closestWaypoints Find a closest waypoint to a given (x,y) point
 * @param x Coordinate x
 * @param y Coordinate y
 */
int Road::closestWaypoint(float x, float y)
{
	  float closest_len = 100000; //large number
	  int closest_wp = 0;

	  for (int i = 0; i < map_waypoints_x.size(); i++) {
		    float map_x = map_waypoints_x[i];
		    float map_y = map_waypoints_y[i];
		    float dist = distance(x,y,map_x,map_y);
		    if (dist < closest_len) {
			      closest_len = dist;
			      closest_wp = i;
	    	}
	  }

	  return closest_wp;
}

/**
 * nextWaypoint Find the next waypoint ahead of car
 * @param x Coordinate x of the car
 * @param y Coordinate y of the car
 * @param theta Yaw of the car
 */ 
int Road::nextWaypoint(float x, float y, float theta)
{
	// find the closes waypoint
	int closest_wp = closestWaypoint(x,y);

	float map_x = map_waypoints_x[closest_wp];
    float map_y = map_waypoints_y[closest_wp];

    // angle of the segment connecting (x,y) point and the closest waypoint
	float heading = atan2((map_y-y),(map_x-x));

    // difference between yaw angle and and the angle to the closest waypoint
	float angle = fabs(theta-heading);

	// normalize the different to be in [0,pi] range
    angle = min(2*pi() - angle, angle);

    // check if the closest waypoint is behind the car. If yes then advance to the next waypoint
    if (angle > pi()/2) {
        closest_wp++;
        if (closest_wp == map_waypoints_x.size()) 
            closest_wp = 0;
    }
	
    return closest_wp;
}

/**
 *  getXY Convert Frenet coordinates to cartesian 
 *  @param s Coordinate s
 *  @param d Coordinate d
 */
vector<float> Road::getXY(float s, float d)
{
    // find previous and next waypoints
	int prev_wp = -1;
	while(s > map_waypoints_s[prev_wp+1] && (prev_wp < (int)(map_waypoints_s.size()-1))) {
		prev_wp++;
	}
	int wp2 = (prev_wp+1) % map_waypoints_x.size();

    // project (s,d) into the segment connecting previous and next waypoints
	// compute P=(x,y) coordinates of the projected point 
	float heading = atan2(map_waypoints_y[wp2]-map_waypoints_y[prev_wp], map_waypoints_x[wp2]-map_waypoints_x[prev_wp]);
	float seg_s = (s-map_waypoints_s[prev_wp]);
	float seg_x = map_waypoints_x[prev_wp]+seg_s*cos(heading);
	float seg_y = map_waypoints_y[prev_wp]+seg_s*sin(heading);

    // find direction of the axis perpendicular to the segment connecting previous and next waypoints
	float perp_heading = heading-pi()/2;

    // move P along this direction by distance d, then project into (x,y) axis
	float x = seg_x + d*cos(perp_heading);
	float y = seg_y + d*sin(perp_heading);

	return {x,y};
}