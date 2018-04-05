#include <fstream>
#include <math.h>
#include <uWS/uWS.h>
#include <chrono>
#include <iostream>
#include <thread>
#include <vector>
#include <tuple>
#include "json.hpp"
#include "spline.h"
#include "helper.h"
#include "road.h"
#include "trajectory_generation.h"
#include "behavioral_planner.h"

using namespace std;
using namespace std::chrono;

// for convenience
using json = nlohmann::json;

// Checks if the SocketIO event has JSON data.
// If there is data the JSON object in string format will be returned,
// else the empty string "" will be returned.
string hasData(string s) {
  auto found_null = s.find("null");
  auto b1 = s.find_first_of("[");
  auto b2 = s.find_first_of("}");
  if (found_null != string::npos) {
    return "";
  } else if (b1 != string::npos && b2 != string::npos) {
    return s.substr(b1, b2 - b1 + 2);
  }
  return "";
}

int main() {
  uWS::Hub h;

  // road parameters
	const int n_lanes = 3;      // number of lanes
	const int lane_width = 4;   // lane width (in meters)
	const string map_file = "../data/highway_map.csv";  // waypoint map to read from
  const float max_s = 6945.554;  // the max s value before wrapping around the track back to 0
  const float max_vel = 49.2; // maximal speed (in mph). We set is slightly less than the maximal road speed (50 mph).
  const float safety_buffer = 30; // driving straight - maximal distance to the car in front of us when we need to slow down
                                  // changing lanes - in the target lane, distance in front/behind the ego car that should be 
                                  //                  free from other cars
  const int max_turn_counter = 200; // minimal number of "KEEP LANE" states between two changes of lanes
  const int max_init = 200; // number of initial iterations before any lane change is permitted
  const int path_length = 20; // length of the generated trajectory 
   
  // start in lane 1
  int lane = 1;

  // velocity at the endof the planning segment
	float ref_vel = 0;  // mph

	Road road(n_lanes, lane_width, map_file, max_vel);                                        // object that represents the road
  BehavioralPlanner planner(road, n_lanes, lane, ref_vel, max_vel, 
                            safety_buffer, max_s, max_turn_counter, max_init, path_length);   // behavioral planner
  Car ego_car(road);                                                                        // object that represents ego car

  h.onMessage([&road,&planner,&lane,&ref_vel,&ego_car](uWS::WebSocket<uWS::SERVER> ws, char *data, size_t length, uWS::OpCode opCode) {
    // "42" at the start of the message means there's a websocket message event.
    // The 4 signifies a websocket message
    // The 2 signifies a websocket event
    
    if (length && length > 2 && data[0] == '4' && data[1] == '2') {

      auto s = hasData(data);

      if (s != "") {
        auto j = json::parse(s);
        
        string event = j[0].get<string>();
        
        if (event == "telemetry") {

            // get current time
            milliseconds current_time = duration_cast< milliseconds >(system_clock::now().time_since_epoch());

            // j[1] is the data JSON object
          
          	// ego car's localization data
          	float car_x = j[1]["x"];
          	float car_y = j[1]["y"];
          	float car_s = j[1]["s"];
          	float car_d = j[1]["d"];
          	float car_yaw = j[1]["yaw"];
          	float car_speed = j[1]["speed"];
            
            // update ego car's state
            ego_car.update(car_s, car_d, car_x, car_y, car_speed * cos(car_yaw) / 2.24, car_speed * sin(car_yaw) / 2.24, current_time);

          	// previous trajectory that was unused by simulator
            Trajectory prev_trajectory;
            auto prev_x = j[1]["previous_path_x"];
            auto prev_y = j[1]["previous_path_y"];
            for (int i = 0; i < prev_x.size(); i++) {
          	    prev_trajectory.x.push_back(prev_x[i]);
          	    prev_trajectory.y.push_back(prev_y[i]);
            }
            int prev_size = prev_trajectory.x.size();

            // Frenet coordinates of the last point of the previous trajectory that was unused by simulator
          	float end_path_s = j[1]["end_path_s"];
          	float end_path_d = j[1]["end_path_d"];

          	// sensor Fusion Data, a list of all other cars on the same side of the road.
          	auto sensor_fusion = j[1]["sensor_fusion"];
           
						// update the state of all observed cars
						for (int i = 0; i < sensor_fusion.size(); i++) {

								int id = sensor_fusion[i][0];
                float other_car_x = sensor_fusion[i][1];
								float other_car_y = sensor_fusion[i][2];
								float other_car_vx = sensor_fusion[i][3];
								float other_car_vy = sensor_fusion[i][4];
								float other_car_s = sensor_fusion[i][5];
								float other_car_d = sensor_fusion[i][6];
                Car other_car = road.updateCar(id, other_car_x, other_car_y, other_car_vx, other_car_vy, other_car_s, 
								                               other_car_d, current_time);												 																							 
						}

            // generate spline trajectory
            Trajectory next_trajectory;

            // predict location of the observed cars when previously generated trajectory ends
            vector<tuple<FrenetState,float,int>> predictions = road.predict(0.02*prev_size);

            // extend previously generated trajectory based on the locations and speed of other cars
            FrenetState end_path_frenet(end_path_s, end_path_d);
            next_trajectory = planner.update_trajectory(ego_car, predictions, prev_trajectory, end_path_frenet);

            // send new trajectory to simulator
          	json msgJson;
          	msgJson["next_x"] = next_trajectory.x;
          	msgJson["next_y"] = next_trajectory.y;

          	auto msg = "42[\"control\","+ msgJson.dump()+"]";

          	ws.send(msg.data(), msg.length(), uWS::OpCode::TEXT);
          
        }
      } else {
        // Manual driving
        std::string msg = "42[\"manual\",{}]";
        ws.send(msg.data(), msg.length(), uWS::OpCode::TEXT);
      }
    }
  });

  // We don't need this since we're not using HTTP but if it's removed the
  // program
  // doesn't compile :-(
  h.onHttpRequest([](uWS::HttpResponse *res, uWS::HttpRequest req, char *data,
                     size_t, size_t) {
    const std::string s = "<h1>Hello world!</h1>";
    if (req.getUrl().valueLength == 1) {
      res->end(s.data(), s.length());
    } else {
      // i guess this should be done more gracefully?
      res->end(nullptr, 0);
    }
  });

  h.onConnection([&h](uWS::WebSocket<uWS::SERVER> ws, uWS::HttpRequest req) {
    std::cout << "Connected!!!" << std::endl;
  });

  h.onDisconnection([&h](uWS::WebSocket<uWS::SERVER> ws, int code,
                         char *message, size_t length) {
    ws.close();
    std::cout << "Disconnected" << std::endl;
  });

  int port = 4567;
  if (h.listen(port)) {
    std::cout << "Listening to port " << port << std::endl;
  } else {
    std::cerr << "Failed to listen to port" << std::endl;
    return -1;
  }
  h.run();
}
