#include "map.h"
#include "vehicles.h"
#include "utils.h"

#include <uWS/uWS.h>
#include <fstream>
#include <iostream>
#include <string>
#include <vector>
#include "Eigen-3.3/Eigen/Core"
#include "Eigen-3.3/Eigen/QR"
#include "json.hpp"
#include <stdio.h>


// for convenience
using nlohmann::json;

// Checks if the SocketIO event has JSON data.
// If there is data the JSON object in string format will be returned,
// else the empty string "" will be returned.
std::string hasData(std::string s) {
  auto found_null = s.find("null");
  auto b1 = s.find_first_of("[");
  auto b2 = s.find_first_of("}");
  if (found_null != std::string::npos) {
    return "";
  } else if (b1 != std::string::npos && b2 != std::string::npos) {
    return s.substr(b1, b2 - b1 + 2);
  }
  return "";
}


int main() {
  uWS::Hub h;

  // Initialize the highway map class 
  std::string map_file = "../data/highway_map.csv";
  Map map(map_file);

  // Initialize the ego vehicle class
  EgoVehicle ego_vehicle(map);

  h.onMessage([&map, &ego_vehicle]
              (uWS::WebSocket<uWS::SERVER> ws, char *data, size_t length,
               uWS::OpCode opCode) {
    // "42" at the start of the message means there's a websocket message event.
    // The 4 signifies a websocket message
    // The 2 signifies a websocket event
    if (length && length > 2 && data[0] == '4' && data[1] == '2') {

      auto s = hasData(data);

      if (s != "") {
        auto j = json::parse(s);
        
        std::string event = j[0].get<std::string>();
        
        if (event == "telemetry") {
          
          // Read the incoming simulator's data
          auto data = j[1];  // j[1] is the data JSON object
          
          // Unused waypoints of the current trajectory
          auto remaining_wpts_x = data["previous_path_x"];
          auto remaining_wpts_y = data["previous_path_y"];
          
          // The ego vehicle data
          double ego_x_pos = data["x"];
          double ego_y_pos = data["y"];
          double ego_s_pos = data["s"]; 
          double ego_d_pos = data["d"];
          double ego_yaw = deg2rad(data["yaw"]);  // rad
          double ego_speed = mph2ms(data["speed"]);  // [m/s]

          // Update the states of the other traffic participants
          std::vector<Vehicle> traffic;
          auto sensor_fusion = data["sensor_fusion"];
          for (auto state: sensor_fusion) {
            double id = state[0];
            double x_pos = state[1];
            double y_pos = state[2];
            double x_vel = state[3];
            double y_vel = state[4];
            double s_pos = state[5];
            double d_pos = state[6];
            // Due to a simulator bug false sensor fusion data is delivered 
            // at the end of a lap. Ommit it.
            if (s_pos == 0) {continue;}
            // Compute the distance to the ego vehicle in the global system 
            double dist = distance(x_pos, y_pos, ego_x_pos, ego_y_pos);
            // and the longitudinal distance in Frenet s coordinates
            double s_dist = s_distance(s_pos, ego_s_pos);
            // The simple constant velocity prediction model used in this 
            // implementation assumes strongly simplifying that the vehicle 
            // perfectly follows its current lane, meaning that its lateral 
            // velocity component is always zero
            double d_vel = 0;
            // Then its longitudinal velocity component is only dependent on 
            // the distance d to the reference line and the reference line's 
            // curvature
            double curvature = map.curvature(s_pos);
            double s_vel = hypot(x_vel, y_vel) / (1 - curvature * d_pos);
            // Store the vehicle in the traffic collector
            traffic.push_back(Vehicle(id, x_pos, y_pos, x_vel, y_vel, dist,
                                      s_pos, d_pos, s_vel, d_vel, s_dist, 
                                      lane(d_pos)));
            // Since the prediction model only considers movement along the
            // Frenet s direction, lane changes of the other vehicles can not 
            // be predicted by it. As this is a hazard to the ego vehicle, 
            // possible lane changes will be considered in a different manner:
            // If the vehicle is not near its lane center, it could be about to 
            // change the lane. Conservatively, it will be treated as like 
            // also being in the nearest adjacent lane. The same vehicle will 
            // be stored twice, but with a different current lane property.
            if (possible_adjacent_lane(d_pos) != lane(d_pos)) {
              traffic.push_back(Vehicle(id, x_pos, y_pos, x_vel, y_vel, dist,
                                        s_pos, d_pos, s_vel, d_vel, s_dist, 
                                        possible_adjacent_lane(d_pos)));
            }
          }

          // Update the ego vehicle state
          ego_vehicle.update_state(ego_x_pos, ego_y_pos, ego_s_pos, ego_d_pos, 
                                   ego_yaw, ego_speed, traffic);

          // Number of current trajectory's remaining waypoints
          int num_remaining_wpts = (int)remaining_wpts_x.size();
          // Number of current trajectory's consumed waypoints
          int num_consumed_wpts = NUM_TRAJECTORY_WPTS - num_remaining_wpts;

          // Reset waypoint collectors that will be sent back to the simulator
          std::vector<double> next_wpts_x;
          std::vector<double> next_wpts_y;

          printf("Ego vehicle simulator state (x|y|s|d|yaw|speed): "
                 "%8.2f %8.2f %8.2f %6.2f %8.2f %6.2f \n", 
                 ego_vehicle.x_pos, ego_vehicle.y_pos,
                 ego_vehicle.s_pos, ego_vehicle.d_pos, 
                 rad2deg(ego_vehicle.yaw), ego_vehicle.speed);

          // Initialize the ego vehicle's trajectory at the first iteration
          if (num_remaining_wpts == 0) {
                        
            printf("\n-----------\n");
            printf("Intializing");
            printf("\n-----------\n");

            // Accelerate the ego vehicle while keeping the lane
            ego_vehicle.init_trajectory();
            next_wpts_x = ego_vehicle.trajectory.x_pos_wpts;
            next_wpts_y = ego_vehicle.trajectory.y_pos_wpts;
            
          // If enough of current trajectory's waypoints were consumed replan
          } else if (num_consumed_wpts >= REPLAN_THRESHOLD) {

            printf("\n------------------------------------------------\n");
            printf("Replaning (Remaining|Consumed waypoints): %3d|%2d",
                   num_remaining_wpts, num_consumed_wpts);
            printf("\n------------------------------------------------\n");

            ego_vehicle.update_trajectory(num_consumed_wpts);
            next_wpts_x = ego_vehicle.trajectory.x_pos_wpts;
            next_wpts_y = ego_vehicle.trajectory.y_pos_wpts;
            
          // If no replaning is required, just keep the current trajectory
          } else {
            for (int i = 0; i < num_remaining_wpts; ++i) {
              next_wpts_x.push_back(remaining_wpts_x[i]);
              next_wpts_y.push_back(remaining_wpts_y[i]);
            }
          }

          json msgJson;
          msgJson["next_x"] = next_wpts_x;
          msgJson["next_y"] = next_wpts_y;

          auto msg = "42[\"control\","+ msgJson.dump()+"]";

          ws.send(msg.data(), msg.length(), uWS::OpCode::TEXT);
        }  // end "telemetry" if
      } else {
        // Manual driving
        std::string msg = "42[\"manual\",{}]";
        ws.send(msg.data(), msg.length(), uWS::OpCode::TEXT);
      }
    }  // end websocket if
  }); // end h.onMessage

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