#ifndef VEHICLES_H
#define VEHICLES_H

#include "polynomials.h"
#include "map.h"

#include <vector>
#include <string>

using std::vector;

class Vehicle {
public:
  Vehicle(int id=-1, 
          double x_pos=0, double y_pos=0, 
          double x_vel=0, double y_vel=0, 
          double dist=0,
          double s_pos=0, double d_pos=0, 
          double s_vel=0, double d_vel=0, 
          double s_dist=0,
          int current_lane=0);
  
  ~Vehicle() {};
  
  bool operator< (const Vehicle &other) const { return dist < other.dist; }

  double s_position(double t);
  bool is_leftside_of(int lane);
  bool is_rightside_of(int lane);
  bool in_front_of(Vehicle &other);
  bool in_rear_of(Vehicle &other);

  double x_pos, y_pos, x_vel, y_vel, dist;
  double s_pos, d_pos, s_vel, d_vel, s_dist;
  double safety_dist;
  int id, current_lane;
  bool in_left_lane, in_center_lane, in_right_lane;
  bool is_present, is_followable, to_close;

};


class Trajectory {
public:
  Trajectory() {cost=1e9; is_empty=true; is_feasible=false; is_collided=true;};
  Trajectory(Polynomial s_poly, Polynomial d_poly, Map &map);
  
  ~Trajectory() {};

  bool operator< (const Trajectory &other) const { return cost < other.cost; }

  void update(Polynomial s_poly, Polynomial d_poly, 
              int num_consumed_wpts, Map &map, int scenario);
  void check_collision(vector<Vehicle> &nearby_vehicles, 
                       Map &map, int scenario);
  
  Polynomial s_poly;
  Polynomial d_poly;
  double cost;
  int target_lane;
  bool is_empty;
  bool is_feasible;
  bool is_collided;

  vector<double> s_pos_wpts;
  vector<double> s_vel_wpts;
  vector<double> s_acc_wpts;
  vector<double> d_pos_wpts;
  vector<double> d_vel_wpts;
  vector<double> d_acc_wpts;
  vector<double> x_pos_wpts;
  vector<double> y_pos_wpts;
  vector<double> yaw_wpts;
};


class Successor {
public:
  Successor(int lat=LAT::KEEP, int lon=LON::KEEP) : lat(lat), lon(lon) {};
  ~Successor() {};
  int lat, lon;
};


class EgoVehicle {
public:
  EgoVehicle(Map map, double x_pos=0, double y_pos=0, 
             double s_pos=0, double d_pos=0, 
             double yaw=0, double speed=0);

  ~EgoVehicle() {};  
  
  void update_state(double x_pos, double y_pos, 
                    double s_pos, double d_pos, 
                    double yaw, double speed, 
                    vector<Vehicle> &traffic);
  void init_trajectory();
  void update_trajectory(int num_consumed_wpts);
  vector<Successor> get_successor_states();
  Trajectory get_optimal_trajectory(vector<double> s_start, 
                                    vector<double> d_start, 
                                    vector<Successor> &successors, 
                                    int num_consumed_wpts,
                                    int scenario);
  Trajectory get_best_candidate(vector<Polynomial> s_polys, 
                                vector<Polynomial> d_polys,
                                int num_consumed_wpts,
                                int scenario);
  vector<Polynomial> get_d_polys(vector<double> d_start, 
                                 vector<double> T_set, 
                                 Successor successor, 
                                 int scenario);
  vector<Polynomial> get_s_polys(vector<double> s_start, 
                                 vector<double> T_set,
                                 Successor successor,
                                 int scenario);
  vector<Vehicle> search_leading_vehicles(vector<Vehicle> &traffic);
  vector<vector<Vehicle>> search_nearby_vehicles(vector<Vehicle> &traffic);
  vector<Vehicle> flatten(vector<vector<Vehicle>> &nearby_vehicles);
  vector<double> get_lane_cost();
  int get_lon_state(int target_lane);
  double get_lon_speed_limit();
  bool check_deadlock();
  Vehicle get_deadlock_causing_vehicle(Vehicle &cur_lead);
  bool check_opposite_lane();
  bool single_blocked_by(Vehicle &vehicle);
  bool double_blocked_by(Vehicle &vehicle);
  bool in_front_of(Vehicle &vehicle);
  bool in_rear_of(Vehicle &vehicle);
  
  double x_pos, y_pos, s_pos, d_pos, yaw, speed, s_vel_limit;
  int current_lane;
  bool to_fast, is_deadlocked, has_free_opposite;
  vector<Vehicle> leading_vehicles;
  vector<vector<Vehicle>> nearby_vehicles;
  Vehicle deadlock_causing_vehicle;
  vector<double> lane_cost;
  Trajectory trajectory;  
  Map map;
};


#endif //VEHICLES_H
